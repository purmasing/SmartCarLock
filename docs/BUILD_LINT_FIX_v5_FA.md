# رفع ۲۵ هشدار Flutter Analyze — نسخه ۵

این اصلاح بر اساس خروجی GitHub Actions شامل ۲۳ مورد `curly_braces_in_flow_control_structures` و دو مورد `unnecessary_non_null_assertion` تهیه شده است.

## فایل‌های اصلاح‌شده

- `app/lib/api.dart`: افزودن آکولاد به شرط‌های تک‌دستوری مربوط به Header و مدیریت خطای API.
- `app/lib/main.dart`: افزودن آکولاد به شرط‌ها و حلقه‌های تک‌دستوری و رفع `!` غیرضروری در ایجاد `LatLng` در صفحه نقشه.

منطق کنترل دستگاه، ثبت پنج گوشی، رمز نصب و API تغییر داده نشده است.

## اجرای مجدد در GitHub

1. فقط `app/lib/main.dart` و `app/lib/api.dart` این بسته را در همان مسیر مخزن جایگزین کنید؛ یا کل پوشه پروژه را با حفظ تنظیمات محرمانه به‌روز کنید.
2. تغییرات را Commit کنید.
3. از Actions، اجرای `Build Android and iOS` را آغاز کنید.
4. در مرحله `Dependencies and source check` باید `flutter analyze --no-fatal-infos` بدون Error یا Warning خاتمه یابد.
5. سپس منتظر نتیجه `Android APK` و در صورت تنظیم امضای اپل خروجی iOS باشید.

## وضعیت اعتبارسنجی

- بررسی ایستای محلی نشانه‌گذاری شرط‌ها، توازن پرانتز/آکولاد و نبود مورد شناخته‌شده Lint: انجام شد.
- اجرای واقعی `flutter analyze` و `flutter build apk` در این محیط ممکن نبود؛ Dart/Flutter SDK نصب نیست. نتیجه Build باید در GitHub Actions تأیید شود.
- برای رفع خطا، قوانین Lint یا کنترل‌های ایمنی Android غیرفعال نشده‌اند.
