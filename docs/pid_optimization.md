# PID ayarı: doğrudan Webots

Mevcut aktif kazançlar `self_balancing_robot/pid_controller.py` içinde korunur:
`KP_VOLT = 60.7944`, `KD_VOLT = 1.2985`, `KV_VEL = 0.2952`,
`KP_POS = 0.1000`, `KI_POS = 0.0104`, `KP_VEL_DRIVE = 0.2486`.
Dönüş kazançları `KP_TURN = 0.20`, `KI_TURN = 1.0` olarak kalır.

## Kullanım

```text
python -m pip install -e ".[test]"
python analysis/controllers/pid/optimize_pid_controller.py --trials 100
```

Webots kurulu olmalıdır; otomatik bulunamazsa `WEBOTS_EXECUTABLE` ile yolunu belirtin.
İstenirse `--output YENI_DENEY_KLASORU` ve `--port 1291` kullanılabilir.

Her aday, `analysis/controllers/pid/simulation_runner.py` üzerinden gerçek Webots fizik motorunda
30 saniye denge, iki yönlü 1.5 N / 80 ms itme ve ileri/geri sürüş-frenleme
senaryolarında ölçülür. Optuna yalnız altı kazancı önerir; puanı tahmin eden ek model yoktur.
Varsayılan bütçe, mevcut kazançlar dahil 100 adaydır; her aday iki tohumda üç senaryo çalıştırır.
Bu bütçe kesin yeterlilik veya optimum garantisi değildir; çalışma süresi Webots hızına bağlıdır.

En iyi üç farklı geçerli aday ve mevcut kazançlar 501–505 tohumlarında karşılaştırılır.
Seçim sabitlendikten sonra kazanan ve mevcut kazançlar 901–905 tohumlarında ölçülür.
Kutu/eğim, tam örnekleme ve kayma denetimleri korunur. Puan, senaryo maliyetlerinin
ortalaması + 0.25 * en kötü maliyettir; açı, hız hatası, gerilim, gerilim değişimi ve son üç saniyedeki hızı içerir.
Gerilim terimi enerji ölçümü değildir. Bu karşılaştırma istatistiksel üstünlük garantisi vermez.

Sonuçlar `logs/pid_tuning/` altında `trials.json`, `selection.json`, `report.json`,
parametreler, konsol çıktıları ve ham telemetriyle saklanır; izole kaynak kopyası `project/` altındadır.
Mevcut ve seçilen ayar son testleri geçer ve seçilen ayar daha düşük puan alırsa
`candidate_params.json` yazılır. Aktif kazançlar otomatik değiştirilmez.
Adayı uygulamadan önce denge, fiziksel mesafe ve dönüş testleri ayrıca çalıştırılmalıdır.
Arama sırasında deney klasörüne `STOP` dosyası koymak mevcut aday tamamlandıktan sonra aramayı durdurur.
Bu basit araçta eski aramayı sürdürme komutu yoktur; tamamlanan adaylar diskte korunur.

## Korunan geçmiş sonuçlar

`analysis/controllers/pid/pid_candidate_params.json` geçmiş deneyin tam hassasiyetli
kazançlarını, `analysis/controllers/pid/pid_optuna_results.json` ise geçmiş hibrit yöntemin sonuç özetini korur.
Bunlar yeni doğrudan Webots aramasının sonucu değildir. Aktif kaynak değerlerinin
yuvarlanmış hali değiştirilmemiştir. Eski ham deney kayıtları silinmemiştir.

## Donanım hazır olduğunda

Motor direnci, boşta hız/ters EMK, PWM ölü bölgesi, sürücü gerilim kaybı ve yük altındaki
akım ölçülecek. Enkoderin gerçek sayım/tur değeri ve düşük hız ölçümü, bataryanın yük
altındaki gerilimi, IMU eksenleri/ofseti, kütle ve ağırlık merkezi doğrulanacak.
Bu ölçümlerden sonra motor modeli güncellenecek ve donanımda canlı telemetriyle yeniden ayar yapılacak.
Mevcut sabit 12 V, sürekli enkoder açısı ve beyaz gürültü varsayımları donanım doğrulaması değildir.
Eşzamanlı sürüş/dönüş, otonom şarj ve gerçek donanım bu aramanın kapsamı dışındadır.
