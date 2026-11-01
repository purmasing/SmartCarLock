#!/usr/bin/env python3
from pathlib import Path
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parent.parent
src=(ROOT/'firmware/smart_car_lock_mobile_api.ino').read_text()
part=src[src.index('// ========= Mobile-only local API (no HTML pages) ========='):src.index('void sclState(){')]
header=r'''
#include <string>
#include <unordered_map>
#include <iostream>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <openssl/sha.h>
using namespace std;
class String:public std::string {
public:
  using std::string::string;
  String(const std::string &s):std::string(s){}
  String(int i):std::string(std::to_string(i)){}
  String(unsigned int i):std::string(std::to_string(i)){}
  bool isEmpty() const {return empty();}
  void trim(){ size_t a=find_first_not_of(" \n\t\r"), b=find_last_not_of(" \n\t\r"); if(a==npos)clear();else *this=substr(a,b-a+1); }
};
static unordered_map<string,string> nvs;
struct Preferences {
  bool begin(const char*,bool){return true;}
  void end(){}
  String getString(const char* k,const String& fallback){auto it=nvs.find(k);return it==nvs.end()?fallback:String(it->second);}
  size_t putString(const char* k,const String &v){nvs[k]=v;return v.length();}
  bool remove(const char* k){return nvs.erase(k)>0;}
};
static uint32_t mockMillis=7200000;
uint32_t millis(){return mockMillis;}
uint32_t esp_random(){static uint32_t s=0x61FE69AD; s = s * 1664525 + 1013904223;return s;}
void mbedtls_sha256(const unsigned char* buf,size_t len,unsigned char out[32],int){SHA256(buf,len,out);}
struct Server {
  unordered_map<string,string> args,headers;
  int status=0;
  string response;
  String arg(const char* key){return String(args[key]);}
  String header(const char* key){return String(headers[key]);}
  void sendHeader(const char*,const char*){}
  void send(int code,const char*,const String& data){status=code;response=data;}
}server;
struct {String deviceId="testDevice";} mqttConfig;
String jsonEscape(const String& s){return s;}
#define SCL_PAIRING_SECRET "UniqueInstallationSecretForTesting_2026"
'''
footer=r'''
int main(){
 // Migrate an existing one-phone installation and verify its original token.
 String legacyToken(64,'a');
 nvs["tokenSha"]=sclShaHex(legacyToken);
 sclLoadToken();
 assert(sclPairedCount()==1);
 assert(nvs.count("t0") && !nvs.count("tokenSha"));
 server.headers["X-SCL-Token"]=legacyToken;
 assert(isRequestAuthorized());
 // Pair four more phones, long after ten minutes from boot, with unique tokens.
 string tokens[5];tokens[0]=legacyToken;
 for(int i=1;i<5;i++){
   server.args["code"]=SCL_PAIRING_SECRET;
   server.args["name"]="My phone "+std::to_string(i);
   sclPair();
   assert(server.status==200);
   auto found=server.response.find("\"token\":\"");
   assert(found!=string::npos);
   tokens[i]=server.response.substr(found+9,64);
   assert(tokens[i].size()==64);
   assert(tokens[i]!=tokens[0]);
   server.headers["X-SCL-Token"]=tokens[i];
   assert(isRequestAuthorized());
 }
 assert(sclPairedCount()==5);
 server.args["code"]=SCL_PAIRING_SECRET;
 sclPair();assert(server.status==409);
 // No NVS reload should lose previously paired devices.
 sclLoadToken();assert(sclPairedCount()==5);
 for(auto &token:tokens){server.headers["X-SCL-Token"]=token;assert(isRequestAuthorized());}
 server.headers["X-SCL-Token"]=tokens[0];
 sclPhones();assert(server.status==200);
 server.args["slot"]="2";
 sclRevokePhone();assert(server.status==200);
 assert(sclPairedCount()==4);
 server.headers["X-SCL-Token"]=tokens[2];assert(!isRequestAuthorized());
 for(int i:{0,1,3,4}){server.headers["X-SCL-Token"]=tokens[i];assert(isRequestAuthorized());}
 sclLoadToken();assert(sclPairedCount()==4);
 // Refill freed slot; disabled token remains invalid.
 server.args["code"]=SCL_PAIRING_SECRET;
 sclPair();assert(server.status==200);
 assert(sclPairedCount()==5);
 // Incorrect codes are not accepted, even if there is a free slot.
 server.headers["X-SCL-Token"]=tokens[0];
 server.args["slot"]="4";sclRevokePhone();assert(server.status==200);
 server.args["code"]="wrong code";sclPair();assert(server.status==403);
 assert(sclPairedCount()==4);
 std::cout<<"PASS: migration, 5 unique tokens, persistence, access checks, revoke, refill, wrong-code rejection at 2h uptime\n";
}
'''
with tempfile.TemporaryDirectory(prefix='scl-pair-test-') as temp:
    test=Path(temp)/'host_pairing.cpp'
    program=Path(temp)/'host_pairing'
    test.write_text(header+part+footer)
    r=subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror',str(test),'-lcrypto','-o',str(program)],capture_output=True,text=True)
    if r.returncode != 0:
        print(r.stdout,r.stderr)
        raise SystemExit('Host compile failed; install a C++ compiler and libssl-dev')
    r=subprocess.run([str(program)],capture_output=True,text=True)
    print(r.stdout,r.stderr)
    assert r.returncode==0,'host runtime test failed'
