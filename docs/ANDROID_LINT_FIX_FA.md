# رفع خطای Android Lint

خطای `Missing includeSubdomains attribute` در زمان اجرای `lintVitalRelease` از تولید خودکار فایل `network_security_config.xml` می‌آید.

اصلاح انجام‌شده در فایل `tools/configure_flutter.py`:

```xml
<domain includeSubdomains="false">192.168.4.1</domain>
```

## انجام در GitHub

برای اصلاح حداقلی، فقط `tools/configure_flutter.py` این بسته را در مخزن قبلی جایگزین کنید. یا کل فایل‌های این نسخه را به ریشه مخزن آپلود کنید و فایل‌های همنام را Replace کنید. سپس commit و از Actions، گزینه Build Android and iOS / Run workflow را بزنید.

این تغییر دسترسی HTTP را به همان آدرس محلی ESP32 محدود نگه می‌دارد و تنظیم HTTPS سرور را تغییر نمی‌دهد.

**آزمون:** تولید XML و تجزیه ساختاری آن بررسی شده‌اند؛ Flutter/Gradle واقعی در محیط تولید بسته اجرا نشده است.
