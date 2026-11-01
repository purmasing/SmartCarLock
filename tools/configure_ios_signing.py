#!/usr/bin/env python3
"""Configure Runner signing from the provisioning profile injected by GitHub Actions.
Requires a valid Apple signing certificate in the runner keychain.
"""
from pathlib import Path
import os,plistlib,re,subprocess
root=Path(__file__).resolve().parent.parent
team=os.environ.get('IOS_TEAM_ID','')
if not re.fullmatch(r'[A-Za-z0-9]{10}',team):raise ValueError('Invalid IOS_TEAM_ID')
profile=Path(os.environ['RUNNER_TEMP'])/'profile.mobileprovision'
xml=subprocess.run(['security','cms','-D','-i',str(profile)],capture_output=True,check=True).stdout
info=plistlib.loads(xml)
uuid=info['UUID']
name=info['Name']
profile_team=info['TeamIdentifier'][0]
if profile_team!=team:raise ValueError('Provisioning profile Team ID differs from IOS_TEAM_ID')
identifier=info['Entitlements']['application-identifier']
if not identifier.startswith(team+'.'):raise ValueError('Invalid App ID')
bundle=identifier[len(team)+1:]
if '*' in bundle:raise ValueError('Use an explicit, not wildcard, provisioning App ID')
provision_dir=Path.home()/'Library/MobileDevice/Provisioning Profiles'
provision_dir.mkdir(parents=True,exist_ok=True)
(provision_dir/(uuid+'.mobileprovision')).write_bytes(profile.read_bytes())
project=root/'app/ios/Runner.xcodeproj/project.pbxproj'
s=project.read_text()
s=re.sub(r'PRODUCT_BUNDLE_IDENTIFIER\s*=\s*[^;]+;',f'PRODUCT_BUNDLE_IDENTIFIER = {bundle};',s)
s=re.sub(r'DEVELOPMENT_TEAM\s*=\s*[^;]+;',f'DEVELOPMENT_TEAM = {team};',s)
s=s.replace('CODE_SIGN_STYLE = Automatic;','CODE_SIGN_STYLE = Manual;')
# Add explicit profile setting to Release configuration via all build settings (only Runner).
s=s.replace('CODE_SIGN_STYLE = Manual;',f'CODE_SIGN_STYLE = Manual;\n\t\t\t\tPROVISIONING_PROFILE_SPECIFIER = "{name}";')
project.write_text(s)
method='development' if info['Entitlements'].get('get-task-allow',False) else ('ad-hoc' if info.get('ProvisionedDevices') else 'app-store')
export={'method':method,'signingStyle':'manual','teamID':team,'provisioningProfiles':{bundle:name}}
with (root/'tools/ExportOptions.plist').open('wb') as f:plistlib.dump(export,f)
print('Configured Xcode signing with profile UUID',uuid,'method',method)
