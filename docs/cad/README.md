# Şasi Mekanik Çizimleri ve Kesim Kılavuzu (150 × 100 mm / 15 × 10 cm Dekota)

Bu dizin, İki Tekerlekli Denge ve İlaç Taşıma Robotunun 4 katmanlı şasisinin imalat, lazer kesim ve manuel işleme dosyalarını içerir.

---

## 1. Dosyalar ve Formatlar

* [pleksi_kesim_sablonu.svg](file:///c:/YTU/self-balancing-robot/docs/cad/pleksi_kesim_sablonu.svg): Güncellenmiş, renkli ve 1:1 ölçekli görsel şablon vektörü (150 × 100 mm).
* [pleksi_katmanlari.dxf](file:///c:/YTU/self-balancing-robot/docs/cad/pleksi_katmanlari.dxf): Lazer kesim tezgahları ve CAD/CAM yazılımları (AutoCAD, SolidWorks, LightBurn) için doğrudan kesim dosyası.

---

## 2. Şasi Genel Ölçüleri ve Malzeme

* **Plaka Ölçüleri:** **150.0 mm Genişlik × 100.0 mm Derinlik** (15 × 10 cm, Köşe kavisleri R=5.0 mm).
* **Malzeme:** 5.0 mm Beyaz Dekota (Forex) veya 3.0 mm Pleksi.
* **Tavsiye Edilen Tabaka:** 35 × 50 cm Dekota tabakası (4 katman sıfır fire ve rahat kesim payıyla çıkar).
* **Bağlantı Elemanları:** 4 adet kesintisiz M3 tij (gijon - 12.5 cm boy), M3 pul ve somunlar.
* **Köşe Tij Delikleri:** Kenarlardan 6.0 mm içeride, çap Ø3.4 mm (M3 tijlerin rahat geçmesi için).

---

## 3. Katman Dağılımı ve Bileşen Yerleşimi

### Katman 1 (Taban / Base Deck - Z = 0.0 mm)

* **JGB37-520 Motor L-Braketleri (2 adet):** Sol ve sağ kenarlarda, 37 mm braket delikleri (M3 × 10 mm cıvata ile). İki motor arasında 60 mm geniş açık kablo koridoru kalır.
* **TB6612FNG Çift Motor Sürücüsü:** Katman 1 tabanında; motorlara giden 12V 20 kHz PWM hatları 3 cm boyunda tutularak EMI gürültüsü tamamen hapsedilir.
* **Dikey Kablo Bacası (40 × 20 mm):** Koordinat X=75.0 mm, Y=25.0 mm (tüm katmanlarda birebir aynı hizada).
* **Şarj Temas Pedleri:** Ön kenarda 2 adet 15 × 6 mm bakır temas alanı yuvası.

### Katman 2 (Güç & Dağıtım / Power Deck - Z = +45.0 mm)

* **3S 18650 Li-ion Batarya Paketi:** Şasinin merkezinde; robotun ağırlık merkezini (CoM) ideal yükseklikte tutar.
* **KCD11 Rocker Açma-Kapama Switch (19 × 13 mm):** Sol arka köşede; bataryadan hemen sonraki ilk ana kesici.
* **3A Panel Tipi Cam Sigorta Yuvası (Ø12.2 mm):** Sağ arka köşede; ana hattı koruyan kolay değiştirilebilir panel sigortası.
* **Velcro Kayış Yarıkları (4 adet):** Bataryayı sarsıntısız sabitlemek için 16 × 8 mm yarıklar.
* **XL4016 8A 200W DC-DC Step-Down Regülatör:** Bataryadan gelen 12V'u lojik için 5.0V'a düşürür. M3 montaj delikleriyle sabitlenir.
* **Mini Güç Dağıtım Kartı (PDB - 35 × 40 mm Delikli Pertinaks):**
  * KF301 vidalı klemensler (12V, 5V ve Ortak GND rayları).
  * 100 uF 25V motor filtre elektrolitik kondansatörü.
* **Dikey Kablo Bacası:** 40 × 20 mm (Katman 1 ile birebir aynı hizada).

### Katman 3 (Beyin & Algılama / Brain Deck - Z = +85.0 mm)

* **ESP32 38-Pin Vidalı Klemens Shield:** ESP32-WROOM-32D kartı sokete takılır; tüm kablolar vidalı klemenslerle güvenle bağlanır (Turkuaz çerçeve).
* **MPU-6050 6-DOF IMU:** Tam geometrik merkezde (X=0, Y=0), motor titreşimlerinden ve manyetik alandan izole. Eksen okları tekerlek eksenine paralel gönyelenmiştir.
* **2x TSOP2238 (38 kHz) IR Alıcı Sensörleri:** Ön kenarda 85 mm diferansiyel aralıkla sola ve sağa bakan nirengi gözleri.
* **Dikey Kablo Bacası:** 40 × 20 mm (Katman 1 ve 2 ile birebir aynı hizada).

### Katman 4 (Tavan / İlaç Taşıma & Koruma - Z = +125.0 mm)

* **Standart İlaç Kutusu Gömülü Cebi (80 × 26 mm):** Aspirin veya Parol karton kutusunun düşmeden oturması için 5 mm derinlikli kaset havuzu. Katmanın tam merkezindedir.
* **Sistem / Şarj Durum LED'i:** Ø5.0 mm dairesel delik (Ön kenarda).
* **Dikey Kablo Bacası:** 40 × 20 mm (Sadece LED kablosu girişi için; üst katta yüksek akımlı güç kablosu kalmamıştır).
