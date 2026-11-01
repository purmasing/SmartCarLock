"""SmartCarLock MVP backend. Single SQLite DB, multiple users and vehicles.
Requires broker TLS + device ACLs, HTTPS reverse proxy, strong secrets before deployment.
"""
from __future__ import annotations
import base64
import datetime as dt
import hashlib
import hmac
import json
import os
from pathlib import Path
import secrets
import sqlite3
import threading
from contextlib import asynccontextmanager,contextmanager
from typing import Literal

import jwt
import paho.mqtt.client as mqtt
from fastapi import FastAPI, Depends, HTTPException, Header
from pydantic import BaseModel, Field

BASE=Path(os.environ.get('SCL_DATA_DIR','./data'))
BASE.mkdir(parents=True,exist_ok=True)
DB=BASE/'scl.db'
JWT_SECRET=os.environ.get('SCL_JWT_SECRET','')
if len(JWT_SECRET)<40:raise RuntimeError('Set SCL_JWT_SECRET to a random secret of at least 40 characters')
BROKER=os.environ.get('SCL_MQTT_HOST','127.0.0.1')
PORT=int(os.environ.get('SCL_MQTT_PORT','1883'))
BROKER_USER=os.environ.get('SCL_MQTT_USER','')
BROKER_PASS=os.environ.get('SCL_MQTT_PASSWORD','')
PREFIX=os.environ.get('SCL_TOPIC_PREFIX','smartcar')
if not PREFIX or '/' in PREFIX:raise RuntimeError('SCL_TOPIC_PREFIX must be one path segment')

lock=threading.RLock()
@contextmanager
def db():
    c=sqlite3.connect(DB,timeout=10)
    c.row_factory=sqlite3.Row
    c.execute('PRAGMA journal_mode=WAL')
    c.execute('PRAGMA foreign_keys=ON')
    try:
        with c:yield c
    finally:c.close()

def initialize():
    with db() as c:
        c.executescript('''
        CREATE TABLE IF NOT EXISTS users(
          id INTEGER PRIMARY KEY,username TEXT UNIQUE NOT NULL,
          pwd TEXT NOT NULL,admin INTEGER NOT NULL DEFAULT 0,
          expires_at TEXT NOT NULL
        );
        CREATE TABLE IF NOT EXISTS vehicles(
          id INTEGER PRIMARY KEY,device_id TEXT UNIQUE NOT NULL,
          owner_id INTEGER NOT NULL REFERENCES users(id),
          name TEXT NOT NULL,model TEXT NOT NULL,plate TEXT NOT NULL,
          command_key TEXT NOT NULL,seq INTEGER NOT NULL DEFAULT 0,
          state_json TEXT NOT NULL DEFAULT '{}',gps_json TEXT NOT NULL DEFAULT '{}',
          last_seen TEXT,ack TEXT
        );
        ''')

def utcnow():return dt.datetime.now(dt.timezone.utc)
def iso(value:dt.datetime)->str:return value.isoformat()
def passwd_hash(pw:str)->str:
    salt=secrets.token_bytes(16)
    digest=hashlib.pbkdf2_hmac('sha256',pw.encode(),salt,310000)
    return f'{salt.hex()}:{digest.hex()}'
def pw_valid(pw:str,stored:str)->bool:
    try:
        salt,h=stored.split(':')
        raw=hashlib.pbkdf2_hmac('sha256',pw.encode(),bytes.fromhex(salt),310000)
        return hmac.compare_digest(raw,bytes.fromhex(h))
    except (ValueError,TypeError):return False

client=mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,client_id='scl-api',protocol=mqtt.MQTTv311)
if BROKER_USER:client.username_pw_set(BROKER_USER,BROKER_PASS)
if PORT==8883:client.tls_set()
connected=False

def on_connect(c,u,flags,reason_code,properties):
    global connected
    connected=reason_code==0
    if connected:
        c.subscribe([(f'{PREFIX}/+/status',1),(f'{PREFIX}/+/telemetry',1),(f'{PREFIX}/+/ack',1)])

def on_disconnect(c,u,disconnect_flags,reason_code,properties):
    global connected
    connected=False

def parse_record(body:str)->dict:
    result={}
    for item in body.split(';'):
        if '=' in item:
            k,v=item.split('=',1)
            if k in {'gsm','net','doors','ignition','fuel','trunkPulse','alarm','fix','lat','lon','speed'}:
                result[k]=v
    return result

def on_message(c,u,msg):
    pieces=msg.topic.split('/')
    if len(pieces)!=3 or pieces[0]!=PREFIX:return
    device,kind=pieces[1],pieces[2]
    if kind not in {'status','telemetry','ack'}:return
    try:
        text=msg.payload.decode('ascii','strict')
        if len(text)>450:return
        with lock,db() as conn:
            if kind=='ack':
                # 123|ok; command acknowledgements deliberately do not trigger another command
                conn.execute('UPDATE vehicles SET ack=?,last_seen=? WHERE device_id=?',(text,iso(utcnow()),device))
            else:
                field='state_json' if kind=='status' else 'gps_json'
                conn.execute(f'UPDATE vehicles SET {field}=?,last_seen=? WHERE device_id=?',
                             (json.dumps(parse_record(text)),iso(utcnow()),device))
    except Exception:return

client.on_connect=on_connect
client.on_disconnect=on_disconnect
client.on_message=on_message

@asynccontextmanager
async def lifespan(app):
    initialize()
    client.connect_async(BROKER,PORT,keepalive=60)
    client.loop_start()
    yield
    client.loop_stop()
    client.disconnect()

app=FastAPI(title='Smart Car Lock API',version='0.1',lifespan=lifespan,docs_url=None,redoc_url=None)
class Login(BaseModel):username:str; password:str
class AddUser(BaseModel):
    username:str=Field(min_length=3,max_length=50)
    password:str=Field(min_length=14,max_length=256)
    days:int=Field(ge=1,le=3650)
    admin:bool=False
class AddVehicle(BaseModel):
    deviceId:str=Field(pattern=r'^[a-zA-Z0-9_-]{4,32}$')
    ownerId:int
    name:str=Field(min_length=1,max_length=90)
    model:str=Field(max_length=80)
    plate:str=Field(max_length=40)
class Command(BaseModel):
    command:Literal['STATUS','LOCATION','ALARM_OFF','DOOR_LOCK','DOOR_UNLOCK','TRUNK']

def auth(authorization:str=Header('')):
    if not authorization.startswith('Bearer '):raise HTTPException(401,'missing bearer token')
    try: claims=jwt.decode(authorization[7:],JWT_SECRET,algorithms=['HS256'])
    except jwt.PyJWTError:raise HTTPException(401,'invalid token')
    with db() as c:u=c.execute('SELECT id,username,admin,expires_at FROM users WHERE id=?',(claims['sub'],)).fetchone()
    if not u or dt.datetime.fromisoformat(u['expires_at'])<utcnow():raise HTTPException(403,'account expired')
    return dict(u)
def administrator(user=Depends(auth)):
    if not user['admin']:raise HTTPException(403,'admin required')
    return user

def access(device_id:str,user:dict):
    with db() as c: v=c.execute('SELECT * FROM vehicles WHERE device_id=?',(device_id,)).fetchone()
    if not v or (not user['admin'] and v['owner_id']!=user['id']):raise HTTPException(404,'vehicle not found')
    return dict(v)

@app.get('/api/v1/health')
def health():return {'ok':True,'broker':connected}
@app.post('/api/v1/auth/login')
def login(data:Login):
    with db() as c:u=c.execute('SELECT * FROM users WHERE username=?',(data.username,)).fetchone()
    if not u or not pw_valid(data.password,u['pwd']):raise HTTPException(401,'invalid login')
    if dt.datetime.fromisoformat(u['expires_at'])<utcnow():raise HTTPException(403,'account expired')
    expires=utcnow()+dt.timedelta(hours=12)
    token=jwt.encode({'sub':str(u['id']),'exp':expires},JWT_SECRET,algorithm='HS256')
    return {'token':token,'expiresAt':iso(expires)}
@app.get('/api/v1/me/vehicles')
def vehicles(user=Depends(auth)):
    with db() as c:
        where='' if user['admin'] else 'WHERE owner_id=?'
        rows=c.execute('SELECT device_id,name,model,plate,last_seen FROM vehicles '+where, () if user['admin'] else (user['id'],)).fetchall()
        return {'vehicles':[dict(x) for x in rows]}
@app.get('/api/v1/vehicles/{device_id}/state')
def state(device_id:str,user=Depends(auth)):
    v=access(device_id,user)
    s=json.loads(v['state_json']);gps=json.loads(v['gps_json'])
    fix=gps.get('fix')=='1'
    try:lat=float(gps['lat']);lng=float(gps['lon'])
    except (ValueError,KeyError):lat=0.0;lng=0.0;fix=False
    try:
        device_online=(utcnow()-dt.datetime.fromisoformat(v['last_seen'])).total_seconds()<360
    except (TypeError,ValueError):
        device_online=False
    return {'ok':True,'deviceId':device_id,'deviceOnline':device_online,'name':v['name'],'model':v['model'],'plate':v['plate'],
      'outputs':[s.get('doors')=='1',s.get('trunkPulse')=='1',s.get('fuel')=='1',s.get('ignition')=='1'],
      'gsmPercent':int(s.get('gsm',0)),'internet':s.get('net')=='1','wifiPercent':0,
      'gps':{'fix':fix,'lat':lat,'lng':lng,'speed':gps.get('speed')},
      'lastSeen':v['last_seen'],'ack':v['ack']}
@app.post('/api/v1/vehicles/{device_id}/command')
def command(device_id:str,data:Command,user=Depends(auth)):
    if not connected:raise HTTPException(503,'mqtt unavailable')
    with lock,db() as c:
        v=c.execute('SELECT * FROM vehicles WHERE device_id=?',(device_id,)).fetchone()
        if not v or (not user['admin'] and v['owner_id']!=user['id']):raise HTTPException(404)
        seq=v['seq']+1
        if seq>=2**32:raise HTTPException(409,'sequence exhausted')
        key=bytes.fromhex(v['command_key'])
        text=f'{device_id}|{seq}|{data.command}'.encode('ascii')
        mac=hmac.new(key,text,hashlib.sha256).hexdigest()[:32]
        payload=f'{seq}|{data.command}|{mac}'
        c.execute('UPDATE vehicles SET seq=? WHERE id=?',(seq,v['id']))
    result=client.publish(f'{PREFIX}/{device_id}/cmd',payload,qos=1,retain=False)
    if result.rc!=mqtt.MQTT_ERR_SUCCESS:raise HTTPException(503,'publish failed; sequence reserved')
    return {'accepted':True,'sequence':seq,'status':'queued_to_broker_not_yet_executed'}
@app.post('/api/v1/admin/users')
def add_user(data:AddUser,user=Depends(administrator)):
    expires=utcnow()+dt.timedelta(days=data.days)
    try:
        with db() as c:
            cursor=c.execute('INSERT INTO users(username,pwd,admin,expires_at) VALUES(?,?,?,?)',
              (data.username,passwd_hash(data.password),int(data.admin),iso(expires)))
            return {'id':cursor.lastrowid,'username':data.username,'expiresAt':iso(expires)}
    except sqlite3.IntegrityError:raise HTTPException(409,'username exists')
@app.post('/api/v1/admin/vehicles')
def add_vehicle(data:AddVehicle,user=Depends(administrator)):
    key=secrets.token_hex(32)
    with db() as c:
        if not c.execute('SELECT id FROM users WHERE id=?',(data.ownerId,)).fetchone():raise HTTPException(404,'owner not found')
        try:c.execute('INSERT INTO vehicles(device_id,owner_id,name,model,plate,command_key) VALUES(?,?,?,?,?,?)',
              (data.deviceId,data.ownerId,data.name,data.model,data.plate,key))
        except sqlite3.IntegrityError:raise HTTPException(409,'device ID exists')
    # This secret is shown ONCE to an admin; store in the ESP32 MQTT settings locally.
    return {'deviceId':data.deviceId,'commandKey':key}
