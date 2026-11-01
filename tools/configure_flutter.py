#!/usr/bin/env python3
"""Run after `flutter create` in app directory; set local-only cleartext permissions."""
from pathlib import Path
from xml.etree import ElementTree as ET
root=Path(__file__).resolve().parent.parent/'app'
android=root/'android/app/src/main'
android.mkdir(parents=True,exist_ok=True)
manifest=android/'AndroidManifest.xml'
text=manifest.read_text()
# Replace only the application tag and insert net permissions; cloud traffic is HTTPS.
text=text.replace('<application\n', '<application\n        android:networkSecurityConfig="@xml/network_security_config"\n',1)
text=text.replace('<manifest xmlns:android="http://schemas.android.com/apk/res/android">',
  '<manifest xmlns:android="http://schemas.android.com/apk/res/android">\n    <uses-permission android:name="android.permission.INTERNET" />\n    <uses-permission android:name="android.permission.ACCESS_NETWORK_STATE" />',1)
manifest.write_text(text)
xml=android/'res/xml';xml.mkdir(parents=True,exist_ok=True)
# local_auth requires a FragmentActivity on Android.
for source in (root/'android/app/src/main/kotlin').rglob('MainActivity.kt'):
    t=source.read_text().replace('import io.flutter.embedding.android.FlutterActivity',
      'import io.flutter.embedding.android.FlutterFragmentActivity').replace(' : FlutterActivity()', ' : FlutterFragmentActivity()')
    source.write_text(t)
for source in (root/'android/app').glob('build.gradle*'):
    t=source.read_text()
    if source.suffix=='.kts':
        t=t.replace('minSdk = flutter.minSdkVersion','minSdk = 23')
    else:
        t=t.replace('minSdkVersion flutter.minSdkVersion','minSdkVersion 23')
    source.write_text(t)
(xml/'network_security_config.xml').write_text('''<?xml version="1.0" encoding="utf-8"?>
<network-security-config>
  <base-config cleartextTrafficPermitted="false" />
  <domain-config cleartextTrafficPermitted="true">
    <domain includeSubdomains="false">192.168.4.1</domain>
  </domain-config>
</network-security-config>\n''')
# Supports iOS local IP traffic while leaving cloud HTTPS enforced.
plist=root/'ios/Runner/Info.plist'
text=plist.read_text()
key='\t<key>NSLocalNetworkUsageDescription</key>\n\t<string>Connect to the vehicle ESP32 Wi-Fi for control and configuration.</string>\n'
key+='\t<key>NSFaceIDUsageDescription</key>\n\t<string>Unlock the vehicle app with Face ID.</string>\n'
key+='\t<key>NSAppTransportSecurity</key>\n\t<dict>\n\t\t<key>NSAllowsLocalNetworking</key>\n\t\t<true/>\n'
key+='\t\t<key>NSExceptionDomains</key>\n\t\t<dict>\n\t\t\t<key>192.168.4.1</key>\n\t\t\t<dict>\n'
key+='\t\t\t\t<key>NSExceptionAllowsInsecureHTTPLoads</key>\n\t\t\t\t<true/>\n\t\t\t</dict>\n\t\t</dict>\n\t</dict>\n'
text=text.replace('</dict>\n</plist>',key+'</dict>\n</plist>')
plist.write_text(text)
# Verify generated XML is still parseable.
ET.parse(manifest);ET.parse(xml/'network_security_config.xml')
print('Android/iOS local network permissions configured')
