# راه‌اندازی آزمایشی سرور Ubuntu با منابع 1 vCPU / 1GB RAM / 25GB

**احتیاط:** این راهنما یک نقطه شروع برای آزمایش است، نه استقرار خودکارِ تولیدی. تا وقتی TLS و ACL کامل و نسخه فریمور SIM800 تأیید نشده‌اند، پورت MQTT عمومی را فعال نکنید.

## پیش‌نیازها

- دامنه `smartcarlock.ir` برای API وب به IP سرور اشاره کند.
- زیر دامنه `mqtt.smartcarlock.ir` برای MQTT نیز به همان IP اشاره کند (در صورت انتخاب این hostname در تنظیمات دستگاه).
- پورت TCP 443 برای HTTPS و 8883 برای MQTT در فایروال، پس از آماده شدن TLS.
- گواهی‌های معتبر برای **هر نام میزبان**؛ باید با CA مورد اعتماد SIM800 سازگار باشند.
- دسترسی SSH با کلید و کاربر غیر root. سرویس‌ها روی کاربر مجزا اجرا شوند.

## بسته‌های پایه

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-venv python3-pip mosquitto mosquitto-clients certbot caddy sqlite3 ufw
sudo ufw allow OpenSSH
sudo ufw allow 80/tcp
sudo ufw allow 443/tcp
# 8883 را فعلاً بسته نگه دارید
sudo ufw enable
```

برای TLS بروکر، ابتدا DNS را تنظیم کنید و یک Certificate معتبر دریافت کنید؛ مثلاً پس از آزاد بودن TCP 80:

```bash
sudo certbot certonly --standalone -d mqtt.smartcarlock.ir
```

**قبل از این دستور سرویس اشغال‌کننده پورت 80 را متوقف کنید.** تمدید گواهی و اجازه دسترسی خواندن private key برای فرآیند Mosquitto باید جداگانه و امن تنظیم شود (بدون مجوز عمومی روی کلید خصوصی). در صورت ابهام، پورت 8883 را باز نکنید.

## فایل‌های نمونه

- `server/mosquitto.sample.conf`: فقط با `password_file`، `acl_file` و گواهی واقعی به `/etc/mosquitto/conf.d/smartcarlock.conf` منتقل شود.
- `server/mosquitto.sample.acl`: برای هر deviceID، کاربر MQTT جداگانه و مجوز Topic دقیق اضافه کنید.
- `server/Caddyfile.example`: HTTPS مربوط به REST API را با دامنه اصلی ارائه می‌کند.

سپس برای هر دستگاه و سرویس سرور، به‌صورت تعاملی رمز جداگانه بسازید:

```bash
sudo mosquitto_passwd -c /etc/mosquitto/passwd server-api
# فقط بار اول از -c استفاده کنید. بارهای بعد -c حذف شود؛ وگرنه فایل بازنویسی می‌شود.
sudo mosquitto_passwd /etc/mosquitto/passwd DEVICE001
```

پس از اعمال ACL اختصاصی هر کاربر و بررسی گواهی:

```bash
sudo mosquitto -c /etc/mosquitto/conf.d/smartcarlock.conf -v
# اگر بدون خطا و با ACL صحیح بود، سرویس اصلی را از systemd راه‌اندازی کنید.
```

این فرمان آزمایشی ممکن است با سرویس mosquitto موجود روی پورت یکسان برخورد کند؛ در این صورت اول سرویس را متوقف کنید. دستور نمونه را بدون بازبینی روی سرور تولیدی اجرا نکنید.

## اجرای Backend

پوشه `server` را روی سرور در `/opt/smartcarlock/server` قرار دهید، کاربر محدود `smartcar` بسازید و دسترسی پوشه‌ها را تنظیم کنید:

```bash
sudo useradd --system --home /opt/smartcarlock --shell /usr/sbin/nologin smartcar
sudo mkdir -p /opt/smartcarlock/{server,data}
sudo chown -R smartcar:smartcar /opt/smartcarlock
sudo -u smartcar python3 -m venv /opt/smartcarlock/venv
sudo -u smartcar /opt/smartcarlock/venv/bin/pip install -r /opt/smartcarlock/server/requirements.txt
```

فایل `.env` را با الگوی `server/.env.example` تهیه و مقادیر امن آن را جایگزین کنید:

```bash
sudo install -m 600 -o smartcar -g smartcar /opt/smartcarlock/server/.env.example /opt/smartcarlock/.env
# ویرایش: SCL_JWT_SECRET و اعتبارنامه اختصاصی broker؛ مقدار نمونه ممنوع است.
sudo nano /opt/smartcarlock/.env
```

برای ساخت ادمین، ابتدا مقادیر Environment را از فایل تنظیمات در محیط Shell بارگذاری کنید (یا موقتاً با export داده شود)، سپس:

```bash
cd /opt/smartcarlock/server
sudo -u smartcar env SCL_JWT_SECRET='REPLACE_WITH_A_RANDOM_64_CHARACTER_SECRET' \
  SCL_DATA_DIR=/opt/smartcarlock/data /opt/smartcarlock/venv/bin/python bootstrap_admin.py
```

دستور بالا باید با رمز واقعی از ENV امن اجرا شود؛ در اجرای واقعی از ثبت secret در history ترمینال خودداری کنید.

سرویس `server/smartcarlock.service` را در `/etc/systemd/system/` قرار دهید:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now smartcarlock
sudo systemctl status smartcarlock
```

با Caddy یا Reverse Proxy معتبر، `127.0.0.1:8000` فقط روی HTTPS دامنه در دسترس باشد. API نباید مستقیماً روی پورت عمومی 8000 باز شود.

## فعال‌سازی در اپلیکیشن

در صفحه ورود ابری، username/password سرور را وارد کنید. Backend پس از ورود JWT دوازده‌ساعته صادر می‌کند و برنامه خودروهای اختصاص‌یافته را نمایش می‌دهد. دستگاه از سمت دیگر با سیم‌کارت به MQTT Broker متصل می‌شود؛ credentials دستگاه با credentials کاربر متفاوت هستند.

## موردهای تکمیل‌نشده

پنل گرافیکی ادمین، بازیابی فراموشی PIN، پروتکل نهایی بکاپ رمزگذاری‌شده، نگهداری تاریخچه GPS و کنترل دقیق Retention در این بسته تکمیل نشده‌اند. پیش از نصب واقعی سامانه باید عملکرد TLS/SIM800، گواهی CA، ACL، وضعیت Offline، تغذیه ماژول و شرایط ایمنی خروجی‌ها تست شود.
