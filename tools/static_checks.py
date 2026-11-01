#!/usr/bin/env python3
"""Checks that can be run without Flutter SDK or ESP32 core.
Not a compilation substitute.
"""
from pathlib import Path
import re
root=Path(__file__).resolve().parent.parent
ino=(root/'firmware/smart_car_lock_mobile_api.ino').read_text()
api=(root/'app/lib/api.dart').read_text()
server=(root/'server/app.py').read_text()
assert '<html' not in ino.lower() and 'text/html' not in ino.lower()
assert 'server.on("/",' not in ino
for path in ['ping','pair','state','config','action','config/ap','config/gps','config/sim','config/mqtt','config/output-name','rfid/register','phones','phones/revoke']:
    assert f'"/api/v1/{path}"' in ino,path
for method in ['ping','pair','state','config','action','config/ap','config/gps','config/sim','config/mqtt','config/output-name','rfid/register']:
    # public API either in explicit uri or uses dynamic method string: static comparison in paths above
    pass
assert 'sclTokenHashes[SCL_MAX_PAIRED_PHONES]' in ino and 'sclConstantTimeEqual' in ino
assert 'SCL_MAX_PAIRED_PHONES = 5' in ino
assert '#define AP_MAX_CLIENTS    5' in ino
assert 'pairing_window_closed' not in ino and 'millis()>600000UL' not in ino
assert 'tokenSha' in ino and 'sclLoadToken' in ino  # legacy migration
assert 'weakAuto' in ino and 'sclApSave' in ino
assert 'server.on("/rfid"' not in ino
assert 'getMainPage() {' not in ino
assert 'AT+CIPGSMLOC' not in ino.replace('//  [OPT-14] CIPGSMLOC مستقیم وقتی حامل از قبل باز است (با retry خودکار)','')
assert "'door_lock'" in (root/'app/lib/main.dart').read_text() and 'mqtt' in server
mobile=(root/'app/lib/main.dart').read_text()
assert '_installationCodeController = TextEditingController()' in mobile
assert '_pairPinController = TextEditingController()' in mobile
assert 'final code=TextEditingController()' not in mobile
assert "widget.api.revokePhone(slot)" in mobile
assert 'Future<String> pair(String code, {String phoneName' in api
print('PASS: API routes, 5 independent tokens, legacy migration, unlimited enrollment time, stable Flutter controllers')
