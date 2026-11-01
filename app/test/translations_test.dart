import 'package:flutter_test/flutter_test.dart';
import 'package:smart_car_lock/translations.dart';
import 'package:smart_car_lock/api.dart';

void main(){
  test('three supported translations have title',(){
    for(final code in ['fa','en','ar']){
      expect(tr(code,'app'),isNotEmpty);
      expect(tr(code,'mqtt'),isNotEmpty);
    }
  });
  test('local API uses AP IP and versioned endpoints',(){
    expect(DeviceApi().uri('ping').toString(),'http://192.168.4.1/api/v1/ping');
  });
}
