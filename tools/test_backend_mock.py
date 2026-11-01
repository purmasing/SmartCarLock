#!/usr/bin/env python3
"""Exercise REST permissions, expiration, MQTT command signing with a fake broker."""
import os,sys,tempfile,types,importlib,hmac,hashlib
from pathlib import Path

class FakeMQTT:
    CallbackAPIVersion=types.SimpleNamespace(VERSION2=2)
    MQTTv311=4
    MQTT_ERR_SUCCESS=0
    published=[]
    def __init__(self,*args,**kwargs):pass
    def username_pw_set(self,*args):pass
    def tls_set(self):pass
    def connect_async(self,*args,**kwargs):pass
    def loop_start(self):pass
    def loop_stop(self):pass
    def disconnect(self):pass
    def subscribe(self,*args):pass
    def publish(self,topic,payload,qos,retain):
        FakeMQTT.published.append((topic,payload))
        return types.SimpleNamespace(rc=0)

pkg=types.ModuleType('paho');mod=types.ModuleType('paho.mqtt');sub=types.ModuleType('paho.mqtt.client')
sub.Client=FakeMQTT;sub.CallbackAPIVersion=FakeMQTT.CallbackAPIVersion
sub.MQTTv311=FakeMQTT.MQTTv311;sub.MQTT_ERR_SUCCESS=0
pkg.mqtt=mod;mod.client=sub
sys.modules.update({'paho':pkg,'paho.mqtt':mod,'paho.mqtt.client':sub})
with tempfile.TemporaryDirectory() as tmp:
    os.environ['SCL_DATA_DIR']=tmp
    os.environ['SCL_JWT_SECRET']='e'*64
    sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'server'))
    appmod=importlib.import_module('app')
    appmod.initialize();appmod.connected=True
    with appmod.db() as c:
        c.execute('INSERT INTO users(username,pwd,admin,expires_at) VALUES(?,?,1,?)',
                  ('root',appmod.passwd_hash('correct-horse-battery-staple-2026'),appmod.iso(appmod.utcnow()+appmod.dt.timedelta(days=5))))
    from fastapi.testclient import TestClient
    with TestClient(appmod.app) as http:
        response=http.post('/api/v1/auth/login',json={'username':'root','password':'correct-horse-battery-staple-2026'})
        assert response.status_code==200,response.text
        token=response.json()['token'];auth={'Authorization':'Bearer '+token}
        bad=http.get('/api/v1/me/vehicles');assert bad.status_code==401
        user=http.post('/api/v1/admin/users',headers=auth,json={'username':'pilot','password':'another-strong-password-2026','days':30,'admin':False})
        assert user.status_code==200,user.text
        car=http.post('/api/v1/admin/vehicles',headers=auth,json={'deviceId':'TESTCAR01','ownerId':user.json()['id'],'name':'Demo car','model':'Test','plate':'xx'})
        assert car.status_code==200,car.text
        command=http.post('/api/v1/vehicles/TESTCAR01/command',headers=auth,json={'command':'LOCATION'})
        assert command.status_code==200,command.text
        assert FakeMQTT.published[0][0]=='smartcar/TESTCAR01/cmd'
        seq,op,mac=FakeMQTT.published[0][1].split('|')
        assert len(mac)==32
        expected=hmac.new(bytes.fromhex(car.json()['commandKey']),f'TESTCAR01|{seq}|{op}'.encode(),hashlib.sha256).hexdigest()[:32]
        assert mac==expected
        forbidden=http.post('/api/v1/vehicles/TESTCAR01/command',headers=auth,json={'command':'FUEL_OFF'})
        assert forbidden.status_code==422
        print('PASS: mock backend login, account, vehicle, MQTT HMAC command and forbidden action')
