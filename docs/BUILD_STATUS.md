# وضعیت آزمون‌ها در زمان تولید بسته

- ✅ تست ایستای هماهنگی Endpointها و حذف HTML (`tools/static_checks.py`)
- ✅ بررسی نحوی ماژول‌های Python با `py_compile`
- ✅ تست آزمایشی Backend با Broker شبیه‌سازی‌شده: ورود، مجوز، ایجاد حساب و خودرو، تولید دستور HMAC، رد دستور خطرناک (`tools/test_backend_mock.py`)
- ✅ فایل‌های Workflow موردنیاز Android و iOS و firmware ساخته شده‌اند
- ⚠️ **کامپایل واقعی Flutter / Arduino اینجا اجرا نشده است**؛ GitHub Actions باید اجرا و خروجی آن بررسی شود.
- ⚠️ **تست روی SIM800/GPS/RFID و خودرو واقعی اجرا نشده است**.
- ⚠️ **TLS واقعی SIM800 و اتصال به Broker روی Ubuntu تأیید نشده است**.
- ⚠️ `firmware` همچنان برخی بخش‌های مسدودکننده legacy SIM800 را حفظ می‌کند.
- ⚠️ در کد، منطق موجود رله‌های وابسته به ضعف سیگنال Wi-Fi طبق درخواست کاربر تغییر نکرده است؛ استفاده روی خودرو در حال حرکت ناایمن است.
- ⚠️ اطلاعات ثبت اولیه و Wi-Fi باید پیش از فلش برای هر دستگاه اختصاصی شوند.

نسخه فعلی: MVP توسعه‌ای برای آزمایش روی میز، نه انتشار تولیدی.

## v4 pairing update
Host C++ simulation and static checks passed. Flutter/Arduino full build is pending GitHub Actions confirmation.
