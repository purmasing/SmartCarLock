import 'dart:async';
import 'dart:convert';
import 'package:http/http.dart' as http;

class ApiException implements Exception {
  final String message;
  final int? status;
  ApiException(this.message, [this.status]);
  @override
  String toString() => message;
}

class DeviceApi {
  DeviceApi({this.host = '192.168.4.1', this.token = ''});
  final String host;
  String token;

  Uri uri(String path) => Uri(scheme: 'http', host: host, path: '/api/v1/$path');
  Future<Map<String, dynamic>> request(String path,
      {Map<String, String>? form, bool auth = true,
      Duration timeout = const Duration(seconds: 4)}) async {
    final headers = <String, String>{};
    if (auth && token.isNotEmpty) { headers['X-SCL-Token'] = token; }
    http.Response response;
    try {
      response = await (form == null
          ? http.get(uri(path), headers: headers)
          : http.post(uri(path), headers: headers, body: form)).timeout(timeout);
    } on TimeoutException {
      throw ApiException('local_timeout');
    } catch (_) {
      throw ApiException('local_unreachable');
    }
    Map<String, dynamic> data;
    try {
      final decoded = jsonDecode(utf8.decode(response.bodyBytes));
      data = decoded is Map<String, dynamic> ? decoded : <String, dynamic>{};
    } catch (_) {
      throw ApiException('invalid_response', response.statusCode);
    }
    if (response.statusCode < 200 || response.statusCode >= 300) {
      throw ApiException('${data['error'] ?? data['message'] ?? 'http_error'}', response.statusCode);
    }
    return data;
  }

  Future<Map<String, dynamic>> ping() => request('ping', auth: false,
      timeout: const Duration(seconds: 2));
  Future<String> pair(String code, {String phoneName = ''}) async {
    final data = await request('pair', form: {'code': code, 'name': phoneName}, auth: false);
    final result = data['token'];
    if (result is! String || result.length != 64) { throw ApiException('invalid_token'); }
    return result;
  }
  Future<Map<String, dynamic>> phones() => request('phones');
  Future<Map<String, dynamic>> revokePhone(int slot) =>
      request('phones/revoke', form: {'slot': '$slot'});
  Future<Map<String, dynamic>> state() => request('state');
  Future<Map<String, dynamic>> config() => request('config');
  Future<Map<String, dynamic>> action(String action) => request('action', form: {'action': action});
  Future<Map<String, dynamic>> saveAp(Map<String, String> form) => request('config/ap', form: form);
  Future<Map<String, dynamic>> saveGps(Map<String, String> form) => request('config/gps', form: form);
  Future<Map<String, dynamic>> saveSim(Map<String, String> form) => request('config/sim', form: form);
  Future<Map<String, dynamic>> saveMqtt(Map<String, String> form) => request('config/mqtt', form: form);
  Future<Map<String, dynamic>> rename(int index, String name) =>
      request('config/output-name', form: {'pin': '$index', 'name': name});
  Future<Map<String, dynamic>> registerRfid(Map<String, String> form) =>
      request('rfid/register', form: form, timeout: const Duration(seconds: 20));
}

// Remote API uses HTTPS and a separate account token, never the ESP32 pairing token.
class CloudApi {
  CloudApi({required this.base, required this.token, required this.deviceId});
  final String base;
  final String token;
  final String deviceId;
  Uri _uri(String path) => Uri.parse('$base/api/v1/$path');
  Future<Map<String, dynamic>> _request(String path, {Map<String, String>? payload}) async {
    final headers = {'Authorization': 'Bearer $token'};
    try {
      final response = await (payload == null
        ? http.get(_uri(path), headers: headers)
        : http.post(_uri(path), headers: {...headers, 'Content-Type': 'application/json'}, body: jsonEncode(payload)))
        .timeout(const Duration(seconds: 8));
      final parsed = jsonDecode(utf8.decode(response.bodyBytes));
      if (parsed is! Map<String, dynamic> || response.statusCode >= 400) {
        throw ApiException('cloud_http_${response.statusCode}', response.statusCode);
      }
      return parsed;
    } on ApiException { rethrow; }
    catch (_) { throw ApiException('cloud_unreachable'); }
  }
  static Future<String> login(String username,String password) async {
    final u=Uri.parse('https://smartcarlock.ir/api/v1/auth/login');
    try{
      final r=await http.post(u,headers:{'Content-Type':'application/json'},
          body:jsonEncode({'username':username,'password':password})).timeout(const Duration(seconds:10));
      final obj=jsonDecode(utf8.decode(r.bodyBytes));
      if(r.statusCode>=400 || obj is! Map<String,dynamic> || obj['token'] is! String){ throw ApiException('cloud_login_failed',r.statusCode); }
      return obj['token'] as String;
    }on ApiException {rethrow;}catch(_){throw ApiException('cloud_login_unreachable');}
  }
  Future<List<dynamic>> vehicles() async {
    final result=await _request('me/vehicles');
    return result['vehicles'] is List?result['vehicles'] as List:[];
  }
  Future<Map<String, dynamic>> state() => _request('vehicles/$deviceId/state');
  Future<Map<String, dynamic>> command(String command) =>
      _request('vehicles/$deviceId/command', payload: {'command': command});
}
