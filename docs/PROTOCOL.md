# قرارداد شبکه Smart Car Lock v1

## Local Wi-Fi API (ESP32)

آدرس نمونه: `http://192.168.4.1/api/v1/`

| روش | مسیر | دسترسی | نتیجه |
|---|---|---|---|
| GET | `ping` | آزاد | نسخه API، شناسه دستگاه، paired، تعداد pairedCount، ظرفیت pairCapacity |
| POST | `pair` | کد نصب اختصاصی و دسترسی به Wi-Fi خودرو (بدون محدودیت زمانی) | توکن ۶۴ رقم هگز مستقل برای هر گوشی، حداکثر ۵ گوشی |
| GET | `phones` | `X-SCL-Token` | فهرست جایگاه و نام گوشی‌های ثبت‌شده؛ بدون توکن |
| POST | `phones/revoke` | `X-SCL-Token` | لغو دسترسی با `slot=0..4` |
| GET | `state` | `X-SCL-Token` | وضعیت آیکون‌ها، قفل‌ها، GPS، MQTT |
| GET | `config` | `X-SCL-Token` | تنظیمات غیرمحرمانه و مقادیر مجاز |
| POST | `action` | `X-SCL-Token` | `door_lock`, `door_unlock`, `trunk` |
| POST | `config/ap` | `X-SCL-Token` | `ssid`, `password`, `ip`, `threshold`, `weakAuto` |
| POST | `config/gps` | `X-SCL-Token` | `safeEnabled`, `safeLat`, `safeLng`, `safeRadius`, `gpsAntiCut`, `site` |
| POST | `config/sim` | `X-SCL-Token` | `apn`, `sec`, `ph0..ph4`, `c0..c6` |
| POST | `config/mqtt` | `X-SCL-Token` | `enabled`, `tls`, `host`, `port`, `deviceId`, `username`, `password`, `commandKey`, `topicPrefix`, `keepAlive`, `positionInterval`, `heartbeatInterval` |
| POST | `config/output-name` | `X-SCL-Token` | `pin`, `name` |
| POST | `rfid/register` | `X-SCL-Token` | `slot`, `name`, `secret` or `key720`, `uid`, `sha256` |

POST با فرم URL-encoded انجام می‌شود. `pair` فیلدهای `code` و نام اختیاری `name` می‌پذیرد. API محلی محرمانه است، فقط به SoftAP محدود شود. پاسخ‌های HTTP هیچ صفحه HTML ندارند. توکن قدیمی هنگام ارتقا از `sclauth/tokenSha` به `sclauth/t0` مهاجرت می‌کند.

## Remote MQTT — Broker over SIM800 GPRS

Topic Prefix نمونه `smartcar`، شناسه مثال `DEVICE001`:

- `smartcar/DEVICE001/status` — `gsm=60;net=1;doors=1;ignition=0;fuel=0;trunkPulse=0;alarm=0`
- `smartcar/DEVICE001/telemetry` — `fix=1;lat=35.515068;lon=51.049798;speed=0.0`
- `smartcar/DEVICE001/cmd` — `SEQ|OP|MAC32`
- `smartcar/DEVICE001/ack` — `SEQ|ok`, `SEQ|denied`, `SEQ|not_parked`, `SEQ|duplicate`

`MAC32` = نخستین ۱۶ بایت HMAC-SHA256 به صورت ۳۲ رقم hex کوچک روی متن `deviceId|SEQ|OP`. کلید ۳۲ بایتی مستقل روی دستگاه و سرور یکسان است. `SEQ` افزایشی است و در NVS ذخیره می‌شود. `MQTT` صرفاً در صورت TLS معتبر و MQTT پشتیبانی‌شده توسط فریمور SIM800 فعال می‌شود.

## Remote REST API (FastAPI)

- `POST /api/v1/auth/login` با JSON: `{ "username": "...", "password": "..." }`
- `GET /api/v1/me/vehicles` — `Authorization: Bearer <JWT>`
- `GET /api/v1/vehicles/DEVICE001/state` — آخرین وضعیت و GPS (ممکن است کهنه باشد)
- `POST /api/v1/vehicles/DEVICE001/command` — `{ "command": "LOCATION" }` یا STATUS/ALARM_OFF/DOOR_LOCK/DOOR_UNLOCK/TRUNK
- `POST /api/v1/admin/users` — ایجاد حساب با `username,password,days,admin`
- `POST /api/v1/admin/vehicles` — ثبت خودرو با `deviceId,ownerId,name,model,plate`؛ پاسخ `commandKey` فقط یک‌بار

پاسخ `accepted` به معنی دریافت فرمان توسط دستگاه یا موفقیت اجرا **نیست**؛ ACK دستگاه باید بعداً دریافت شود. ACL بروکر باید به ازای هر دستگاه مجزا باشد. هیچ رمز MQTT/HMAC در پاسخ GET به اپلیکیشن ارائه نشود.
