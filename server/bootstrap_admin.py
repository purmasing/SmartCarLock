import getpass
import os
import sys
from datetime import timedelta
from app import initialize,db,passwd_hash,utcnow,iso

if __name__=='__main__':
    initialize()
    username=input('Admin username: ').strip()
    password=getpass.getpass('Strong password (14+ chars): ')
    if len(username)<3 or len(password)<14:sys.exit('Username >=3 and password >=14 required')
    with db() as c:
        c.execute('INSERT INTO users(username,pwd,admin,expires_at) VALUES(?,?,1,?)',
                  (username,passwd_hash(password),iso(utcnow()+timedelta(days=3650))))
    print('Admin created; password not printed')
