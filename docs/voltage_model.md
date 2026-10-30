# Gerilim tabanlı motor modeli

Kontrolcü ortalama gerilim üretir; gerilim 8 bit işaretli PWM komutuna çevrilip yeniden gerilime dönüştürülerek fizik modeline uygulanır. Webots tekerlek torkunu ve zemin temasını çözer. Eski hız kontrol dünyası ayrı bir referanstır.

## Elektriksel model

I = (V - Ke * N * omega) / (R_armature + R_driver)
T = Kt * N * efficiency * I - T_friction

Varsayımlar: 12 V sabit batarya, 4.2 ohm armatür, 0.5 ohm sürücü yolu, 30:1 redüksiyon, Kt = Ke = 0.012 (motor mili, SI), yüzde 85 ileri sürüş verimi. Bunlar ölçümle kalibre edilmiş JGB37 verileri değildir. Akım sınırlaması varsayılan olarak kapalıdır; max_current yalnız ideal harici akım düzenleyicisini temsil eder.

Patinaj sınırı motor torkunu kırpmaz. Kaymayı Webots temas modeli hesaplar. 3 N*m Webots eyleyici tavanıdır; nominal motor torku veya güvenli sürekli çalışma değeri değildir. Olağan çalışma bu tavanın altında kalmalıdır.

Sıfır gerilim, modelde kısa devre frenini temsil eder; serbest dönüş değildir. Endüktans, PWM anahtarlama dalgalanması, batarya çökmesi, termal koruma ve çift yönlü redüktör verimi ayrıntılı modellenmez. Çok küçük hızda kullanılan ölü bölge, tam bir statik sürtünme çözümü değildir.

## Kontrol

Açı PD gerilim üretir. Ters EMK'nin yüzde 85'i ileri beslenir; bu katsayı gerçek motorda yeniden ayarlanmalıdır. Hız hatası kazancı 0.12 rad/(m/s), duruş hız sönümlemesi 0.15 rad/(m/s), konum tutmaya geçiş eşiği 0.02 m/s. Dürtme sonrasında başlangıç konumuna dönüş istenmez; hızın sönmesi istenir.

## Doğrulama

- `python tests/test_dc_motor_model.py`: gerilim etkisi, ters EMK/fren, yön simetrisi, PWM çözünürlüğü ve isteğe bağlı akım sınırı.
- `python tests/test_simulation_voltage.py`: 30 saniye denge, 1.5 N / 80 ms dürtme, kutu kontrolü ve son 3 saniyede hızın 0.03 m/s altında olması.

Gerilim telemetrisinin komut sütunları `cmd_left_volts` ve `cmd_right_volts` olarak adlandırılır. Bu test gerçek donanım veya gerilim modunda sürüş/dönüş kabul testi yerine geçmez.

## Optuna ile katsayı araması

`python analysis/controller/optimize_voltage_controller.py --trials 250`

Arama altı kontrol kazancını değiştirir; motorun fiziksel parametrelerini değiştirmez. Her aday doğrudan Webots'ta statik denge, iki yönlü dürtme ve ileri/geri sürüş ile frenleme senaryolarında ölçülür. Projenin bir kopyası ve her koşunun ayrı günlükleri `logs/voltage_optuna/` altında tutulur. İlk aday mevcut referans kazançlarıdır.

Amaç fonksiyonu açı hatası, hız takip hatası, gerilim büyüklüğü, gerilim değişim hızı ve son üç saniyedeki kalan hızı ağırlıklandırır. Normalize kare terimleri birden büyük olabilir. Gerilim büyüklüğü elektriksel enerji tüketimi ölçümü değildir. İtme öncesindeki konuma dönmek amaç fonksiyonuna dahil değildir.

Arama tohumu varsayılan olarak 42'dir. Seçilen aday ve referans, aramaya dahil edilmeyen 142 ve 242 tohumlarıyla yeniden karşılaştırılır. Kabul için adayın doğrulama maliyetinin düşmesi ve her doğrulama senaryosunda son üç saniyedeki mutlak hızın 0.04 m/s altında kalması gerekir. Bu eşik proje şartnamesinden değil, optimizasyon değerlendirmesinden gelir.

`report.json` karşılaştırmayı, `candidate_params.json` aday katsayılarını, `study.sqlite3` bütün denemeleri ve `manifest.json` kullanılan kaynak dosyalarının özetlerini içerir. Kontrolcü `VOLTAGE_PARAMS_FILE` ortam değişkeniyle verilen JSON dosyasını okuyabilir. Arama varsayılan kontrolcü kazançlarını otomatik değiştirmez. Sonuçlar tanımlanan aralıklar ve senaryolar için geçerlidir; küresel optimum veya donanım doğrulaması değildir.
