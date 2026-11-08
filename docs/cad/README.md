# Şasi çizimleri ve kesim kılavuzu — 150 × 100 mm

[SVG şablonu](/C:/YTU/self-balancing-robot/docs/cad/pleksi_kesim_sablonu.svg) ve
[DXF çizimi](/C:/YTU/self-balancing-robot/docs/cad/pleksi_katmanlari.dxf),
[generate_chassis.py](/C:/YTU/self-balancing-robot/docs/cad/generate_chassis.py)
ile aynı milimetre geometrisinden üretilir. Eski 135 × 95 mm DXF değiştirilmiştir.

```text
python docs/cad/generate_chassis.py
python docs/cad/generate_chassis.py --check
```

Üretici yalnız Python standart kütüphanesini kullanır. SVG fiziksel sayfası
350 × 285 mm, viewBox birimi 1 mm'dir. Baskıda yüzde 100 ölçek kullanın ve
sayfaya sığdırmayı kapatın; kontrol çubuğu 50 mm olmalıdır. DXF birimi mm'dir.
SVG ekran gösterimindeki yakınlaştırma fiziksel baskı ölçeği değildir.

## Mekanik ölçüler

- Dört plaka: 150 × 100 mm, köşe yarıçapı 5 mm; 5 mm Dekota veya 3 mm pleksi.
- Tij delikleri: M4 (çap 4.4 mm delik), kenarlardan 6 mm; merkezleri X = +/- 69 mm, Y = +/- 44 mm.
- Dört katmanda kablo geçişi: 40 × 20 mm; sol arka köşeden merkez (75, 25) mm.
  Robot merkezine göre X = 0, Y = -25 mm; ön yön +Y'dir.
- Tekerlek çapı 85 mm, kalınlığı 35 mm, merkezden merkeze iz genişliği 190 mm.
  İç açıklık 155 mm'dir; 150 mm plaka ile her yanda 2.5 mm boşluk kalır.
  Fiziksel motor/şaft/braket yerleşimi bu iz genişliğini sağlamalıdır.
- Katman yükseklikleri taban plaka merkezine göre: 0, 35, 75 ve 115 mm.
  Webots'ta mil ekseninden plaka merkezlerine: 8, 43, 83 ve 123 mm.
- Dört plaka 350 × 285 mm çizim sayfasında yerleşir; 350 × 500 mm tabaka yeterlidir.
  Kesim payı ve kalan malzeme vardır.

## Katmanlar ve işleme ayrımı

| DXF katmanı / SVG rengi | İşlem |
| --- | --- |
| CUT_OUTLINE / mavi | Dış kontur kesimi |
| CUT_HOLES / turuncu | Geçiş, tij, kayış, switch, sigorta ve LED açıklıkları |
| REFERENCE / yeşil kesikli | Yerleşim işareti; kesilmez, DXF'te varsayılan olarak dondurulmuştur |
| POCKET / mor kesikli | Kasetin 80 × 26 mm iç sınırı; kesilmez, DXF'te dondurulmuştur |

Lazer/CAM aktarımında yalnız CUT_OUTLINE ve CUT_HOLES seçilmelidir.
Çizim kerf telafisi içermez; imalatçı kullanılan malzemeye göre uygular.

1. **Taban:** JGB37 L-braket ve TB6612FNG yerleşimi, iki adet 15 × 6 mm şarj pedi
   referansı. Kart/braket montaj delikleri fiziksel parçadan ölçülmeden kesilmez.
2. **Güç:** 19 × 13 mm switch, çap 12.2 mm sigorta, dört adet 16 × 8 mm kayış
   açıklığı. Batarya, XL4016 ve Mini PDB sınırları yerleşim referansıdır;
   ölçülen modül boyutu, soğutucu yüksekliği ve klemens erişimiyle kesinleştirilir.
3. **Kontrol:** Merkez MPU-6050, ön ESP32 shield ve 85 mm aralıklı TSOP2238
   referansları. TSOP2238 daireleri paket ayak izi veya kesim deliği değildir.
   Tutucu ve optik pencere ayrıca tasarlanır. Pinler: 1 OUT, 2 VS, 3 GND;
   besleme uygunluğu alınan parçanın veri sayfasıyla doğrulanır.
4. **Tavan:** Çap 5 mm LED deliği ve merkezde 80 × 26 mm kaset iç sınırı.
   Kaset, sağlam plaka üzerine ayrı çerçeveyle yapılır. 5 mm plakada 5 mm
   derinlikte oyma tam kesim olur ve kutunun alt desteğini kaldırır; bu sınır
   geçiş deliği olarak kesilmez. Webots çerçevesinin iç açıklığı da 80 × 26 mm'dir.

Dosyalar ölçü bakımından eşdeğerdir; elektronik kart ayak izleri, TSOP tutucusu
ve şaft montajı için fiziksel numune onayı yerine geçmez.
