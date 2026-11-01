# Gerilim tabanlı motor modeli

Kontrolcü ortalama gerilim üretir; gerilim 8 bit işaretli PWM komutuna çevrilip yeniden gerilime dönüştürülerek fizik modeline uygulanır. Webots tekerlek torkunu ve zemin temasını çözer.

## Elektriksel model

I = (V - Ke * N * omega) / (R_armature + R_driver)
T = Kt * N * efficiency * I - T_friction

Varsayımlar: 12 V sabit batarya, 4.2 ohm armatür, 0.5 ohm sürücü yolu, 30:1 redüksiyon, Kt = Ke = 0.012 (motor mili, SI), yüzde 85 ileri sürüş verimi. Bunlar ölçümle kalibre edilmiş JGB37 verileri değildir. Akım sınırlaması varsayılan olarak kapalıdır; max_current yalnız ideal harici akım düzenleyicisini temsil eder.

Patinaj sınırı motor torkunu kırpmaz. Kaymayı Webots temas modeli hesaplar. 3 N*m Webots eyleyici tavanıdır; nominal motor torku veya güvenli sürekli çalışma değeri değildir. Olağan çalışma bu tavanın altında kalmalıdır.

Sıfır gerilim, modelde kısa devre frenini temsil eder; serbest dönüş değildir. Endüktans, PWM anahtarlama dalgalanması, batarya çökmesi, termal koruma ve çift yönlü redüktör verimi ayrıntılı modellenmez. Çok küçük hızda kullanılan ölü bölge, tam bir statik sürtünme çözümü değildir.

## Kontrol

Açı PD gerilim üretir. `PIDController.compute`, `update_odometry` ile alınan güncel tekerlek hızları üzerinden ters EMK'nin yüzde 85'ini ileri besler; bu katsayı gerçek motorda yeniden ayarlanmalıdır. Hız hatası kazancı 0.2486 rad/(m/s), duruş hız sönümlemesi 0.2952 rad/(m/s), konum tutmaya geçiş eşiği 0.02 m/s. Dürtme sonrasında başlangıç konumuna dönüş istenmez; hızın sönmesi istenir.

## Doğrulama

### Dönüş kontrolü

Gerilim kontrolcüsünde dönüş komutu artık sabit diferansiyel gerilim olarak uygulanmaz. Komut, iki tekerleğin yarı hız farkını hedefler; enkoderlerden ölçülen yarı hız farkı PI geri beslemesiyle izlenir. `TURN_SPEED = 1.2 rad/s`, mevcut geometride yaklaşık 0.6 rad/s (34 derece/saniye) gövde dönüş hedefidir. Geçişlerde anlık hız bu hedefi aşabilir.

Diferansiyel gerilim 2 V ile sınırlıdır; denge çıkışı doyuma yaklaşınca dönüşe ayrılan gerilim azalır. İntegral doyumda büyütülmez ve duruşta sıfırlanır. Dönüş kazançları `KP_TURN = 0.20`, `KI_TURN = 1.0` değerleridir; bunlar düz sürüş Optuna aramasından elde edilmiş kazançlar değildir.

`python tests/test_simulation_pid_turning.py` izole bir projede iki gürültü tohumu ile sol/sağ dönüşü, duruşu, kutu güvenliğini ve kaymayı kontrol eder. Bu test düz sürüş sırasında eşzamanlı dönüş veya gerçek donanım doğrulaması değildir.

- `python tests/test_dc_motor_model.py`: gerilim etkisi, ters EMK/fren, yön simetrisi, PWM çözünürlüğü ve isteğe bağlı akım sınırı.
- `python tests/test_pid_controller.py`: denge doyumunda dönüş anti-windup koruması, enkoder geri beslemesi ve tekerlek başına ters EMK ileri beslemesi.
- `python tests/test_simulation_pid.py`: 30 saniye denge, 1.5 N / 80 ms dürtme, kutu kontrolü ve son 3 saniyede hızın 0.03 m/s altında olması.

Gerilim telemetrisinin komut sütunları `cmd_left_volts` ve `cmd_right_volts` olarak adlandırılır. Bu test gerçek donanım veya gerilim modunda sürüş/dönüş kabul testi yerine geçmez.

## Katsayı optimizasyonu

Aktif giriş noktası `analysis/controllers/pid/optimize_pid_controller.py` dosyasıdır. Adaylar doğrudan Webots üzerinde ölçülür; seçim ve bağımsız son test ayrı tohumlarda yapılır. Aktif kazançlar otomatik değiştirilmez. Motor/enkoder, batarya ve IMU ölçümleri donanım hazır olduğunda yapılacaktır. Ayrıntılar için [optimizasyon kılavuzuna](pid_optimization.md) bakın.

Sensör gürültüsü yalnız Webots adaptöründe (`webots/controllers/webots_main/webots_main.py`) eklenir. `SensorFusion` verilen örnekleri filtreler; rastgele gürültü üretmez. Kontrolcü ileri besleme ve gerilim sınırından sonra PWM komutlarını döndürür; adaptör bu PWM değerlerini motor modeline uygular.
