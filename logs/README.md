# Log düzeni

Bu klasördeki CSV dosyaları test ve simülasyon telemetrisidir. Bunlar yeniden
üretilebilir olsalar da tarihli ölçüm kayıtları olduklarından saklanır.

- `telemetry/pid_balance/`: Webots kontrolcüsünün ortam değişkeniyle başka yol
  verilmediğinde yazdığı varsayılan denge telemetrisi. Dosya adı tür ve tarih-saat taşır.
- `validation/pid_balance/`: Statik denge doğrulama testinin tarihli CSV çıktısı.
- `validation/homing/YYYY-MM-DD_HH-MM-SS_microseconds/`: Her yanaşma koşusunun
  `evaluation.csv` ve `telemetry.csv` dosyaları.
- `pid_tuning/optimization/`: Tarihli Optuna aramaları ve deney raporları.
- `pid_tuning/validation/driving/`: Tarihli ileri/geri sürüş doğrulamaları.
- `pid_tuning/validation/turning/`: Tarihli PI dönüş doğrulamaları.
- Deneylerin iç dosyaları, deneme kodu tarafından kullanıldığı için ad ve yapıları korunur.
- `voltage_optuna/`: Önceki gerilim optimizasyonu ve ham deney kayıtları.

Yeni doğrulama çıktıları `validation/<test>/<tarih-saat>/` düzenine yazılır.
Loglar silinmez veya sonraki koşularla üzerine yazılmaz.
