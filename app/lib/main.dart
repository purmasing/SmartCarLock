import 'dart:async';
import 'dart:convert';
import 'dart:math';
import 'package:crypto/crypto.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_secure_storage/flutter_secure_storage.dart';
import 'package:local_auth/local_auth.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:url_launcher/url_launcher.dart';
import 'api.dart';
import 'translations.dart';

void main() { WidgetsFlutterBinding.ensureInitialized(); runApp(const SmartCarApp()); }
const vault = FlutterSecureStorage();

class SmartCarApp extends StatefulWidget {
  const SmartCarApp({super.key});
  @override
  State<SmartCarApp> createState() => _SmartCarAppState();
}
class _SmartCarAppState extends State<SmartCarApp> {
  String lang = 'fa'; bool dark = false; bool ready = false;
  @override void initState() { super.initState(); _load(); }
  Future<void> _load() async {
    final prefs = await SharedPreferences.getInstance();
    if(mounted) { setState(() { lang=prefs.getString('lang')??'fa'; dark=prefs.getBool('dark')??false;ready=true; }); }
  }
  Future<void> setLang(String s) async {
    final prefs=await SharedPreferences.getInstance();await prefs.setString('lang',s);
    if(mounted) { setState(()=>lang=s); }
  }
  Future<void> setDark(bool v) async {
    final prefs=await SharedPreferences.getInstance();await prefs.setBool('dark',v);
    if(mounted) { setState(()=>dark=v); }
  }
  @override Widget build(BuildContext context) => MaterialApp(
    debugShowCheckedModeBanner:false, title:'Smart Car Lock',
    locale:Locale(lang), supportedLocales:const [Locale('fa'),Locale('en'),Locale('ar')],
    localizationsDelegates:const [GlobalMaterialLocalizations.delegate,GlobalWidgetsLocalizations.delegate,GlobalCupertinoLocalizations.delegate],
    themeMode: dark?ThemeMode.dark:ThemeMode.light,
    theme: ThemeData(useMaterial3:true,colorSchemeSeed:const Color(0xFF2463D3),brightness:Brightness.light),
    darkTheme:ThemeData(useMaterial3:true,colorSchemeSeed:const Color(0xFF67A7FF),brightness:Brightness.dark),
    home: ready?Shell(lang:lang,dark:dark,onLang:setLang,onDark:setDark):
       const Scaffold(body:Center(child:CircularProgressIndicator())),
  );
}

enum Connection { local, cloud, offline }
class Shell extends StatefulWidget {
  const Shell({super.key,required this.lang,required this.dark,required this.onLang,required this.onDark});
  final String lang; final bool dark; final ValueChanged<String> onLang; final ValueChanged<bool> onDark;
  @override State<Shell> createState()=>_ShellState();
}
class _ShellState extends State<Shell> with WidgetsBindingObserver {
  final DeviceApi local=DeviceApi();
  String? boundDeviceId, cloudToken, cloudDeviceId;
  // Keep controllers for the entire screen lifetime. Creating them in build()
  // caused the installation code and PIN to disappear on each 8-second refresh.
  final _installationCodeController = TextEditingController();
  final _pairPinController = TextEditingController();
  final _phoneNameController = TextEditingController();
  final _unlockPinController = TextEditingController();
  bool _pairingBusy = false;
  bool _showInstallationCode = false;
  bool _showPairPin = false;

  bool unlocked=false, loading=true, paired=false, hasPin=false;
  Connection connection=Connection.offline;
  Map<String,dynamic> state={},config={};
  String? error;
  Timer? poller; bool querying=false; int tab=0;
  String t(String k)=>tr(widget.lang,k);
  @override void initState(){super.initState();WidgetsBinding.instance.addObserver(this);_bootstrap();}
  @override void dispose(){
    poller?.cancel();
    WidgetsBinding.instance.removeObserver(this);
    _installationCodeController.dispose();
    _pairPinController.dispose();
    _phoneNameController.dispose();
    _unlockPinController.dispose();
    super.dispose();
  }
  @override void didChangeAppLifecycleState(AppLifecycleState state){
    if(state==AppLifecycleState.paused||state==AppLifecycleState.inactive){if(mounted){ setState(()=>unlocked=false); }}
    if(state==AppLifecycleState.resumed && unlocked) { refresh(); }
  }
  Future<void> _bootstrap() async {
    local.token=await vault.read(key:'local_token')??'';
    boundDeviceId=await vault.read(key:'local_device_id');
    cloudToken=await vault.read(key:'cloud_token');
    cloudDeviceId=await vault.read(key:'cloud_device_id');
    hasPin=(await vault.read(key:'pin_hash'))!=null;
    paired=local.token.isNotEmpty;
    if(mounted){ setState(()=>loading=false); }
    if(!hasPin) { if(mounted){ setState(()=>unlocked=true); } }
    await refresh();
    poller=Timer.periodic(const Duration(seconds:8),(_){if(unlocked){ refresh(); }});
  }
  // Accept ASCII, Persian and Arabic numerals from different phone keyboards.
  String normalizePin(String value) => value.replaceAllMapped(
      RegExp(r'[۰-۹٠-٩]'), (match) {
        final code = match.group(0)!.runes.first;
        final digit = code >= 0x06F0 ? code - 0x06F0 : code - 0x0660;
        return '$digit';
      });
  String hashPin(String pin,String salt)=>sha256.convert(utf8.encode('$salt:${normalizePin(pin)}')).toString();
  Future<void> setPin(String pin) async {
    final random=Random.secure();
    final salt=List<int>.generate(16,(_)=>random.nextInt(256)).map((x)=>x.toRadixString(16).padLeft(2,'0')).join();
    await vault.write(key:'pin_salt',value:salt);
    await vault.write(key:'pin_hash',value:hashPin(pin,salt));
    if(mounted){ setState(()=>hasPin=true); }
  }
  Future<bool> verifyPin(String pin) async {
    final salt=await vault.read(key:'pin_salt')??'';
    final saved=await vault.read(key:'pin_hash');
    return salt.isNotEmpty&&saved==hashPin(pin,salt);
  }
  Future<void> biometrics() async {
    try {
      final auth=LocalAuthentication();
      final ok=await auth.authenticate(localizedReason:t('unlock'),options:const AuthenticationOptions(biometricOnly:false,stickyAuth:true));
      if(ok && mounted){ setState(()=>unlocked=true); }
    }catch(_){_message('Biometric authentication unavailable');}
  }
  Future<void> refresh() async {
    if(querying){ return; }
    querying=true;
    try {
      // Auto mode: only send token if device ID matches the one paired during provisioning.
      final ping=await local.ping();
      final id=ping['deviceId']?.toString();
      if(local.token.isNotEmpty && id!=null && id==boundDeviceId) {
        final data=await local.state();
        if(mounted){ setState((){connection=Connection.local;state=data;error=null;}); }
        querying=false;return;
      }
      if(local.token.isEmpty && mounted){ setState(()=>connection=Connection.offline); }
    } catch (_) { /* Not connected to AP: switch to HTTPS cloud. */ }
    if(cloudToken?.isNotEmpty==true && cloudDeviceId?.isNotEmpty==true) {
      try {
        final data=await CloudApi(base:'https://smartcarlock.ir',token:cloudToken!,deviceId:cloudDeviceId!).state();
        if(mounted){ setState((){connection=Connection.cloud;state=data;error=null;}); }
        querying=false;return;
      }catch(e){if(mounted){ setState(()=>error='$e'); }}
    }
    if(mounted){ setState(()=>connection=Connection.offline); }
    querying=false;
  }
  Future<void> loadConfig() async {
    if(connection!=Connection.local){ return; }
    try {final data=await local.config();if(mounted){ setState(()=>config=data); }}
    catch(e){_message('$e');}
  }
  void _message(String s){if(!mounted){ return; }ScaffoldMessenger.of(context).showSnackBar(SnackBar(content:Text(s)));}
  Future<void> execute(String action) async {
    final confirm=await showDialog<bool>(context:context,builder:(ctx)=>AlertDialog(
      title:Text(t('status')),content:Text('$action ?'),actions:[TextButton(onPressed:()=>Navigator.pop(ctx,false),child:const Text('Cancel')),
      FilledButton(onPressed:()=>Navigator.pop(ctx,true),child:const Text('OK'))]));
    if(confirm!=true){ return; }
    try {
      if(connection==Connection.local){ await local.action(action); }
      else if(connection==Connection.cloud && cloudToken!=null && cloudDeviceId!=null) {
        const mapped={'door_lock':'DOOR_LOCK','door_unlock':'DOOR_UNLOCK','trunk':'TRUNK'};
        await CloudApi(base:'https://smartcarlock.ir',token:cloudToken!,deviceId:cloudDeviceId!).command(mapped[action]!);
      }else{_message(t('offline'));return;}
      _message(connection==Connection.cloud?'Queued; wait for device ACK':t('saved'));await refresh();
    }catch(e){_message('$e');}
  }
  @override Widget build(BuildContext context) {
    if(loading){ return const Scaffold(body:Center(child:CircularProgressIndicator())); }
    if(!unlocked){ return _unlockScreen(); }
    if(!paired){ return _pairScreen(); }
    return Scaffold(
      appBar:AppBar(title:Text(t('app')),actions:[
        IconButton(tooltip:t('refresh'),icon:const Icon(Icons.refresh_rounded),onPressed:refresh),
        IconButton(tooltip:t('logout'),icon:const Icon(Icons.lock_outline_rounded),onPressed:()=>setState(()=>unlocked=false)),
      ]),
      body:IndexedStack(index:tab,children:[_dashboard(),_liveMap(),_settings()]),
      bottomNavigationBar:NavigationBar(selectedIndex:tab,onDestinationSelected:(i){setState(()=>tab=i);if(i==2){ loadConfig(); }},
        destinations:[NavigationDestination(icon:const Icon(Icons.dashboard_rounded),label:t('home')),
          NavigationDestination(icon:const Icon(Icons.map_rounded),label:t('map')),
          NavigationDestination(icon:const Icon(Icons.tune_rounded),label:t('settings'))]),
    );
  }
  Widget _unlockScreen(){
    return Scaffold(appBar:AppBar(title:Text(t('app'))),body:Center(child:ConstrainedBox(
      constraints:const BoxConstraints(maxWidth:440),child:Padding(padding:const EdgeInsets.all(24),
      child:Column(mainAxisSize:MainAxisSize.min,children:[
        const Icon(Icons.lock_person_rounded,size:72),const SizedBox(height:20),
        TextField(controller:_unlockPinController,keyboardType:TextInputType.number,
          inputFormatters:[FilteringTextInputFormatter.allow(RegExp(r'[0-9۰-۹٠-٩]'))],
          obscureText:true,maxLength:6,
          decoration:InputDecoration(labelText:t('pin'),border:const OutlineInputBorder())),
        FilledButton.icon(onPressed:() async {
          if(await verifyPin(_unlockPinController.text)){
            _unlockPinController.clear();
            if(mounted){ setState(()=>unlocked=true); }
          }else { _message(t('invalidPin')); }
        },icon:const Icon(Icons.lock_open_rounded),label:Text(t('unlock'))),
        TextButton.icon(onPressed:biometrics,icon:const Icon(Icons.fingerprint_rounded),label:Text(t('biometric'))),
      ]))),));
  }
  Widget _pairScreen(){
    return Scaffold(appBar:AppBar(title:Text(t('pair'))),body:Center(child:SingleChildScrollView(
      child:ConstrainedBox(constraints:const BoxConstraints(maxWidth:500),
      child:Padding(padding:const EdgeInsets.all(24),child:Column(children:[
        const Icon(Icons.directions_car_filled_rounded,size:76),
        const SizedBox(height:16),Text(t('pairHelp'),textAlign:TextAlign.center),
        const SizedBox(height:12),
        Text(t('installCodeHelp'),textAlign:TextAlign.center,style:Theme.of(context).textTheme.bodySmall),
        const SizedBox(height:16),
        TextField(controller:_installationCodeController,autocorrect:false,enableSuggestions:false,
          maxLength:128,obscureText:!_showInstallationCode,
          decoration:InputDecoration(labelText:t('code'),border:const OutlineInputBorder(),
            suffixIcon:IconButton(onPressed:()=>setState(()=>_showInstallationCode=!_showInstallationCode),
              icon:Icon(_showInstallationCode?Icons.visibility_off_rounded:Icons.visibility_rounded)))),
        const SizedBox(height:12),
        TextField(controller:_phoneNameController,maxLength:32,
          decoration:InputDecoration(labelText:t('phoneName'),border:const OutlineInputBorder())),
        const SizedBox(height:12),
        TextField(controller:_pairPinController,obscureText:!_showPairPin,
          keyboardType:TextInputType.number,
          inputFormatters:[FilteringTextInputFormatter.allow(RegExp(r'[0-9۰-۹٠-٩]'))],
          maxLength:6,decoration:InputDecoration(labelText:t('pin'),border:const OutlineInputBorder(),
            helperText:t('pinHelp'),
            suffixIcon:IconButton(onPressed:()=>setState(()=>_showPairPin=!_showPairPin),
              icon:Icon(_showPairPin?Icons.visibility_off_rounded:Icons.visibility_rounded)))),
        const SizedBox(height:12),
        FilledButton.icon(
          icon:_pairingBusy?const SizedBox(width:18,height:18,child:CircularProgressIndicator(strokeWidth:2)):
            const Icon(Icons.phonelink_setup_rounded),
          onPressed:_pairingBusy?null:()async{
            final code=_installationCodeController.text.trim();
            final pin=normalizePin(_pairPinController.text);
            if(code.length<12){_message(t('codeTooShort'));return;}
            if(!RegExp(r'^\d{6}$').hasMatch(pin)){_message(t('invalidPin'));return;}
            setState(()=>_pairingBusy=true);
            try{
              final ping=await local.ping();
              final id=ping['deviceId']?.toString();
              if(id==null||id.isEmpty){ throw ApiException('missing_device_id'); }
              final token=await local.pair(code,phoneName:_phoneNameController.text.trim());
              // Persist the device binding and bearer token on this phone only.
              await vault.write(key:'local_device_id',value:id);
              await vault.write(key:'local_token',value:token);
              await setPin(pin);
              local.token=token;boundDeviceId=id;
              _installationCodeController.clear();
              _pairPinController.clear();
              if(mounted){ setState(()=>paired=true); }
              await refresh();
            }catch(e){_message(_friendlyPairError(e));}
            finally{if(mounted){ setState(()=>_pairingBusy=false); }}
          },label:Text(t('pairNow'))),
      ]))))));
  }
  String _friendlyPairError(Object error){
    if(error is ApiException){
      const known={
        'pairing_not_provisioned','invalid_code','try_later',
        'phone_limit_reached','storage_failed','storage_unavailable',
        'local_unreachable','local_timeout','invalid_name',
      };
      if(known.contains(error.message)){ return t(error.message); }
    }
    return '$error';
  }
  Widget _connectionInfo(){
    final good=connection!=Connection.offline && (connection!=Connection.cloud || state['deviceOnline']==true);
    final color=good?Colors.green:Colors.orange;
    return Card(child:ListTile(leading:Icon(connection==Connection.local?Icons.wifi_rounded:connection==Connection.cloud?Icons.cloud_done_rounded:Icons.cloud_off_rounded,color:color),
      title:Text(connection==Connection.local?t('local'):connection==Connection.cloud?t('cloud'):t('offline')),
      subtitle:Text(connection==Connection.local?(boundDeviceId??''):connection==Connection.cloud?
        '${cloudDeviceId??''} • ${state['deviceOnline']==true?t('online'):t('offline')}':t('notReady')),
      trailing:IconButton(icon:const Icon(Icons.refresh_rounded),onPressed:refresh)));
  }
  Widget _signalTile(IconData icon,String name,String value,Color color){
    return Card(child:Padding(padding:const EdgeInsets.all(12),child:Column(mainAxisSize:MainAxisSize.min,children:[
      Icon(icon,size:29,color:color),const SizedBox(height:6),Text(name,style:Theme.of(context).textTheme.labelSmall,textAlign:TextAlign.center),
      Text(value,style:Theme.of(context).textTheme.titleSmall)
    ])));
  }
  Widget _dashboard(){
    final output=(state['outputs'] is List)?state['outputs'] as List:[];
    final gps=(state['gps'] is Map)?state['gps'] as Map:const {};
    final wifi=state['wifiPercent']??0,gsm=state['gsmPercent']??0;
    final internet=state['internet']==true;
    final fix=gps['fix']==true;
    bool v(int idx)=>output.length>idx && output[idx]==true;
    return RefreshIndicator(onRefresh:refresh,child:ListView(padding:const EdgeInsets.all(14),children:[
      _connectionInfo(),const SizedBox(height:10),
      GridView.count(shrinkWrap:true,physics:const NeverScrollableScrollPhysics(),crossAxisCount:4,childAspectRatio:.78,
        children:[_signalTile(Icons.wifi_rounded,t('wifi'),'$wifi%',Colors.blue),
          _signalTile(Icons.signal_cellular_alt_rounded,t('gsm'),'$gsm%',Colors.green),
          _signalTile(Icons.satellite_alt_rounded,t('gps'),fix?'Fix':'—',fix?Colors.green:Colors.grey),
          _signalTile(Icons.language_rounded,t('internet'),internet?t('online'):'—',internet?Colors.green:Colors.grey)]),
      const SizedBox(height:10),
      Wrap(spacing:9,runSpacing:9,children:[
        _actionCard(Icons.door_front_door_rounded,t('doors'),v(0)?t('locked'):t('unlocked'),
            v(0)?'door_unlock':'door_lock',v(0)?t('open'):t('close')),
        _actionCard(Icons.key_rounded,t('ignition'),v(3)?t('online'):t('offline'),null,null),
        _actionCard(Icons.local_gas_station_rounded,t('fuel'),v(2)?t('online'):t('offline'),null,null),
        _actionCard(Icons.car_repair_rounded,t('trunk'),v(1)?t('open'):t('locked'),'trunk',t('open')),
      ]),
      const SizedBox(height:14),
      Card(child:Padding(padding:const EdgeInsets.all(16),child:Column(crossAxisAlignment:CrossAxisAlignment.start,children:[
        Text(t('gps'),style:Theme.of(context).textTheme.titleLarge),const SizedBox(height:6),
        Text(fix?'${gps['lat']}, ${gps['lng']}':t('noFix')),
        const SizedBox(height:10),OutlinedButton.icon(icon:const Icon(Icons.map_rounded),label:Text(t('map')),
          onPressed:fix?()async{
            final u=Uri.parse('https://www.google.com/maps/search/?api=1&query=${gps['lat']},${gps['lng']}');
            if(!await launchUrl(u,mode:LaunchMode.externalApplication)){ _message(t('error')); }
          }:null)
      ]))),const SizedBox(height:8),
      Card(child:ListTile(leading:const Icon(Icons.shield_outlined),title:Text(t('safety')))),
      if(error!=null) Text(error!,style:const TextStyle(color:Colors.red))
    ]));
  }

  Widget _liveMap(){
    final gps=(state['gps'] is Map)?state['gps'] as Map:const {};
    final lat=double.tryParse('${gps['lat']??''}');
    final lon=double.tryParse('${gps['lng']??''}');
    final fix=gps['fix']==true && lat!=null && lon!=null && lat.abs()<=90 && lon.abs()<=180;
    if(!fix){ return Center(child:Column(mainAxisSize:MainAxisSize.min,children:[
      const Icon(Icons.satellite_alt_rounded,size:60),const SizedBox(height:12),Text(t('noFix')),
      TextButton(onPressed:refresh,child:Text(t('refresh')))])); }
    final center=LatLng(lat,lon);
    return Column(children:[
      Padding(padding:const EdgeInsets.all(8),child:_connectionInfo()),
      Expanded(child:FlutterMap(
        key:ValueKey('${lat.toStringAsFixed(4)},${lon.toStringAsFixed(4)}'),
        options:MapOptions(initialCenter:center,initialZoom:15),
        children:[
          TileLayer(urlTemplate:'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
            userAgentPackageName:'ir.smartcarlock.smart_car_lock'),
          MarkerLayer(markers:[Marker(point:center,width:56,height:56,
            child:const Icon(Icons.directions_car_filled_rounded,color:Colors.red,size:42))]),
          RichAttributionWidget(attributions:[TextSourceAttribution('OpenStreetMap contributors')]),
        ],
      )),
      Padding(padding:const EdgeInsets.all(10),child:Text('$lat, $lon   •   ${t('mapTilesInternet')}',
        style:Theme.of(context).textTheme.bodySmall)),
    ]);
  }
  Widget _actionCard(IconData icon,String title,String value,String? action,String? button){
    return SizedBox(width:MediaQuery.of(context).size.width>550?230:(MediaQuery.of(context).size.width-40)/2,
      child:Card(child:Padding(padding:const EdgeInsets.all(12),child:Column(children:[
        Icon(icon,size:32),const SizedBox(height:8),Text(title,style:Theme.of(context).textTheme.titleMedium),Text(value),
        if(action!=null)TextButton(onPressed:(connection==Connection.offline || (connection==Connection.cloud && state['deviceOnline']!=true))?null:()=>execute(action),child:Text(button??''))
      ]))));
  }
  Widget _settings(){
    final localOnly=connection!=Connection.local;
    return ListView(padding:const EdgeInsets.all(14),children:[
      _connectionInfo(),
      ListTile(leading:const Icon(Icons.language),title:Text(t('language')),
        trailing:DropdownButton<String>(value:widget.lang,items:const [DropdownMenuItem(value:'fa',child:Text('فارسی')),
          DropdownMenuItem(value:'en',child:Text('English')),DropdownMenuItem(value:'ar',child:Text('العربية'))],onChanged:(v){if(v!=null){ widget.onLang(v); }})),
      SwitchListTile(secondary:const Icon(Icons.dark_mode_outlined),title:Text(t('theme')),value:widget.dark,onChanged:widget.onDark),
      const Divider(),
      _menuTile(Icons.wifi_tethering_rounded,t('ap'),'ap',localOnly),
      _menuTile(Icons.satellite_alt_rounded,t('gpsSettings'),'gps',localOnly),
      _menuTile(Icons.sim_card_rounded,t('sim'),'sim',localOnly),
      _menuTile(Icons.hub_rounded,t('mqtt'),'mqtt',localOnly),
      _menuTile(Icons.nfc_rounded,t('rfid'),'rfid',localOnly),
      _menuTile(Icons.edit_note_rounded,t('rename'),'names',localOnly),
      ListTile(leading:const Icon(Icons.phonelink_lock_rounded),title:Text(t('registeredPhones')),
        subtitle:Text(t('managePhonesHint')),trailing:Icon(localOnly?Icons.lock_outline_rounded:Icons.chevron_right_rounded),
        onTap:localOnly?null:()=>Navigator.of(context).push(MaterialPageRoute(
          builder:(_)=>PairedPhonesPage(api:local,lang:widget.lang)))),
      const Divider(),
      ListTile(leading:const Icon(Icons.cloud_rounded),title:Text(t('cloudLogin')),
        subtitle:Text(cloudToken?.isNotEmpty==true?t('online'):t('noCloud')),
        trailing:const Icon(Icons.chevron_right),onTap:()=>_openEditor('cloud')),
      if(cloudToken?.isNotEmpty==true && cloudDeviceId!=null)ListTile(
        leading:const Icon(Icons.directions_car_filled_rounded),title:Text('${t('deviceId')}: $cloudDeviceId'),
        trailing:const Icon(Icons.swap_horiz_rounded),onTap:()async{
          try {
            final vehicles=await CloudApi(base:'https://smartcarlock.ir',token:cloudToken!,deviceId:cloudDeviceId!).vehicles();
            if(!mounted){ return; }
            final selected=await showDialog<String>(context:context,builder:(ctx)=>SimpleDialog(
              title:Text(t('deviceId')),children:vehicles.whereType<Map>().map((v)=>SimpleDialogOption(
                onPressed:()=>Navigator.pop(ctx,v['device_id'].toString()),
                child:Text('${v['name']} / ${v['plate']}'))).toList()));
            if(selected!=null){await vault.write(key:'cloud_device_id',value:selected);
              if(mounted){ setState(()=>cloudDeviceId=selected); }await refresh();}
          }catch(e){_message('$e');}
        }),
      if(localOnly)Padding(padding:const EdgeInsets.all(12),child:Text(t('localOnly'),style:const TextStyle(color:Colors.orange))),
    ]);
  }
  Widget _menuTile(IconData icon,String title,String type,bool disabled)=>ListTile(
      leading:Icon(icon),title:Text(title),trailing:Icon(disabled?Icons.lock_outline_rounded:Icons.chevron_right_rounded),
      onTap:disabled?null:()=>_openEditor(type));
  Future<void> _openEditor(String type) async {
    if(type!='cloud'){ await loadConfig(); }
    if(!mounted){ return; }
    await Navigator.of(context).push(MaterialPageRoute(builder:(ctx)=>SettingsEditor(
      type:type,lang:widget.lang,config:config,
      onSave:(data)async {
        if(type=='cloud'){
          final token=await CloudApi.login(data['username']??'',data['password']??'');
          final api=CloudApi(base:'https://smartcarlock.ir',token:token,deviceId:'');
          final vehicles=await api.vehicles();
          if(vehicles.isEmpty){ throw ApiException('no_assigned_vehicles'); }
          final id=vehicles.first['device_id'].toString();
          await vault.write(key:'cloud_token',value:token);
          await vault.write(key:'cloud_device_id',value:id);
          cloudToken=token;cloudDeviceId=id;
          return;
        }
        if(connection!=Connection.local){ throw ApiException('local_only'); }
        switch(type){
          case 'ap':await local.saveAp(data);break;
          case 'gps':await local.saveGps(data);break;
          case 'sim':await local.saveSim(data);break;
          case 'mqtt':await local.saveMqtt(data);break;
          case 'names':await local.rename(int.parse(data['pin']!),data['name']!);break;
          case 'rfid':await local.registerRfid(data);break;
        }
      })));
    if(type!='cloud'){ await loadConfig(); }await refresh();
  }
}

class SettingsEditor extends StatefulWidget {
  const SettingsEditor({super.key,required this.type,required this.lang,required this.config,required this.onSave});
  final String type,lang;final Map<String,dynamic> config;
  final Future<void> Function(Map<String,String>) onSave;
  @override State<SettingsEditor> createState()=>_SettingsEditorState();
}
class _SettingsEditorState extends State<SettingsEditor> {
  final values=<String,String>{};final flags=<String,bool>{};
  bool saving=false;String? error;
  String t(String k)=>tr(widget.lang,k);
  Map<String,dynamic> get obj => (widget.type=='ap'?widget.config['ap']:
    widget.type=='gps'?widget.config['gps']:widget.type=='sim'?widget.config['sim']:
    widget.type=='mqtt'?widget.config['mqtt']:<String,dynamic>{}) as Map<String,dynamic>? ?? {};
  @override void initState(){super.initState();
    for(final e in obj.entries){if(e.value is bool){ flags[e.key]=e.value==true; } else if(e.value is! List){ values[e.key]='${e.value??''}'; }}
    if(widget.type=='ap'){ values['password']=''; }
    if(widget.type=='mqtt'){values['password']='';values['commandKey']='';}
    if(widget.type=='cloud'){values['username']='';values['password']='';}
    if(widget.type=='names'){values['pin']='0';values['name']='';}
    if(widget.type=='rfid'){values.addAll({'slot':'0','name':'','secret':'','key720':'','uid':'','sha256':''});}
  }
  Widget field(String key,String label,{bool secret=false,TextInputType? keyboard})=>Padding(
    padding:const EdgeInsets.symmetric(vertical:7),child:TextFormField(
      initialValue:values[key]??'',obscureText:secret,keyboardType:keyboard,
      decoration:InputDecoration(labelText:label,border:const OutlineInputBorder(),isDense:true),
      onChanged:(s)=>values[key]=s));
  Widget toggle(String key,String label)=>SwitchListTile(title:Text(label),value:flags[key]??false,onChanged:(v)=>setState(()=>flags[key]=v));
  @override Widget build(BuildContext context){
    final form=<Widget>[];
    switch(widget.type){
      case 'ap':
        form.addAll([field('ssid',t('ssid')),field('password',t('password'),secret:true),field('ip',t('ip')),
          field('threshold',t('threshold'),keyboard:TextInputType.number),toggle('weakAuto',t('weakAuto'))]);break;
      case 'gps':
        form.addAll([toggle('safeZoneEnabled',t('safeZone')),field('safeLat',t('safeLat')),
          field('safeLng',t('safeLng')),field('safeRadius',t('safeRadius'),keyboard:TextInputType.number),
          toggle('gpsAntiCut',t('gpsAntiCut')),field('site','Map provider: 0 Google / 1 Neshan / 2 Balad')]);break;
      case 'sim':
        form.addAll([field('apn',t('apn')),toggle('security',t('enabled'))]);
        final phones=obj['phones'] as List? ?? [];
        for(int i=0;i<phones.length && i<5;i++){ values.putIfAbsent('ph$i',()=>phones[i]?.toString()??''); }
        for(int i=0;i<5;i++){ form.add(field('ph$i','${t('phones')} ${i+1}',keyboard:TextInputType.phone)); }
        final cmds=obj['commands'] as List? ?? [];
        for(int i=0;i<7;i++){values.putIfAbsent('c$i',()=>i<cmds.length?'${cmds[i]}':'');form.add(field('c$i','${t('commands')} ${i+1}'));}
        break;
      case 'mqtt':
        form.addAll([toggle('enabled',t('enabled')),toggle('tls',t('tls')),field('host',t('host')),
          field('port',t('port'),keyboard:TextInputType.number),field('deviceId',t('deviceId')),
          field('username',t('username')),field('password',t('password'),secret:true),
          field('commandKey',t('cmdKey'),secret:true),field('topicPrefix',t('topic')),
          field('keepAlive',t('keepAlive'),keyboard:TextInputType.number),
          field('positionInterval',t('gpsEvery'),keyboard:TextInputType.number),
          field('heartbeatInterval',t('heartbeat'),keyboard:TextInputType.number)]);break;
      case 'names':form.addAll([field('pin','Output index 0..3',keyboard:TextInputType.number),field('name',t('name'))]);break;
      case 'rfid':form.addAll([Text(t('rfidNote')),field('slot',t('slot'),keyboard:TextInputType.number),
          field('name',t('name')),field('secret',t('secret')),field('key720',t('key720')),field('uid',t('uid')),field('sha256',t('sha'))]);break;
      case 'cloud':form.addAll([Text('https://smartcarlock.ir'),field('username',t('username')),
          field('password',t('password'),secret:true)]);break;
    }
    return Scaffold(appBar:AppBar(title:Text(widget.type=='ap'?t('ap'):widget.type=='gps'?t('gpsSettings'):
      widget.type=='sim'?t('sim'):widget.type=='mqtt'?t('mqtt'):widget.type=='rfid'?t('rfid'):
      widget.type=='cloud'?t('cloudLogin'):t('rename'))),body:SafeArea(child:SingleChildScrollView(padding:const EdgeInsets.all(18),
      child:Column(crossAxisAlignment:CrossAxisAlignment.stretch,children:[
        ...form,if(error!=null)Padding(padding:const EdgeInsets.all(8),child:Text(error!,style:const TextStyle(color:Colors.red))),
        const SizedBox(height:18),FilledButton.icon(icon:saving?const SizedBox.square(dimension:18,child:CircularProgressIndicator(strokeWidth:2)):
          const Icon(Icons.save_rounded),label:Text(t('save')),onPressed:saving?null:()async{
            setState((){saving=true;error=null;});
            try {
              final data={...values};
              for(final item in flags.entries){ data[item.key]=item.value?'1':'0'; }
              if(widget.type=='sim'){
                data['sec']=flags['security']==true?'1':'0';
                data.remove('security');
              }
              if(widget.type=='gps'){
                data['safeEnabled']=flags['safeZoneEnabled']==true?'1':'0';
                data.remove('safeZoneEnabled');
                data['safeRadius']=data['safeRadius']??'100';
              }
              if(widget.type=='ap'){ data['weakAuto']=flags['weakAuto']==true?'1':'0'; }
              await widget.onSave(data);
              if(!context.mounted){ return; }
              ScaffoldMessenger.of(context).showSnackBar(SnackBar(content:Text(t('saved'))));
              Navigator.pop(context);
            }catch(e){if(mounted){ setState(()=>error='$e'); }}
            finally{if(mounted){ setState(()=>saving=false); }}
          })
      ]))));
  }
}


// Local-only phone management. Revocation removes one token without affecting others.
class PairedPhonesPage extends StatefulWidget {
  const PairedPhonesPage({super.key,required this.api,required this.lang});
  final DeviceApi api;
  final String lang;
  @override State<PairedPhonesPage> createState()=>_PairedPhonesPageState();
}
class _PairedPhonesPageState extends State<PairedPhonesPage> {
  Map<String,dynamic>? data;
  String? error;
  bool busy=false;
  String t(String key)=>tr(widget.lang,key);
  @override void initState(){super.initState();_load();}
  Future<void> _load() async {
    if(mounted){ setState((){busy=true;error=null;}); }
    try{
      final response=await widget.api.phones();
      if(mounted){ setState(()=>data=response); }
    }catch(e){if(mounted){ setState(()=>error='$e'); }}
    finally{if(mounted){ setState(()=>busy=false); }}
  }
  Future<void> _revoke(int slot,String name) async {
    final confirmed=await showDialog<bool>(context:context,builder:(ctx)=>AlertDialog(
      title:Text(t('revokePhone')),
      content:Text('${t('revokeWarning')}\n$name'),
      actions:[TextButton(onPressed:()=>Navigator.pop(ctx,false),child:Text(t('cancel'))),
        FilledButton(onPressed:()=>Navigator.pop(ctx,true),child:Text(t('revokePhone')))]));
    if(confirmed!=true){ return; }
    try {
      await widget.api.revokePhone(slot);
      await _load();
    }catch(e){if(mounted){ setState(()=>error='$e'); }}
  }
  @override Widget build(BuildContext context){
    final phones=(data?['phones'] as List?)??[];
    return Scaffold(appBar:AppBar(title:Text(t('registeredPhones')),
      actions:[IconButton(icon:const Icon(Icons.refresh),onPressed:busy?null:_load)]),
      body:ListView(padding:const EdgeInsets.all(16),children:[
        if(busy)const LinearProgressIndicator(),
        if(data!=null)Text('${phones.length} / ${data!['capacity'] ?? 5}',
          style:Theme.of(context).textTheme.titleLarge),
        const SizedBox(height:12),
        Text(t('phonesInfo')),
        const SizedBox(height:12),
        for(final item in phones)
          if(item is Map)Card(child:ListTile(
            leading:const Icon(Icons.smartphone_rounded),
            title:Text('${item['name'] ?? t('phoneName')}'),
            subtitle:Text('${t('phoneSlot')} ${(item['slot'] as num).toInt()+1}'),
            trailing:IconButton(icon:const Icon(Icons.person_remove_alt_1_rounded),
              tooltip:t('revokePhone'),
              onPressed:busy?null:()=>_revoke((item['slot'] as num).toInt(),
                  '${item['name'] ?? ''}')))),
        if(error!=null)Padding(padding:const EdgeInsets.all(12),
          child:Text(error!,style:const TextStyle(color:Colors.red))),
      ]));
  }
}
