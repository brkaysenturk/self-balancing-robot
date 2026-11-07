"""Generate 1:1 scale printable A4 PDF templates for manual Dekota cutting.

Each page corresponds to an exact 150mm x 100mm layer formatted on A4 (210mm x 297mm).
Features include:
- Exact 1:1 scale with 50mm calibration check ruler
- Centerlines, drill crosshairs with exact M4 hole marks (dia 4.4mm)
- Cable duct (40x20mm) through-cut outlines
- Component footprint reference outlines
- Outer boundary cut lines with R5 rounded corners
"""

from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, Circle
from matplotlib.backends.backend_pdf import PdfPages

OUTPUT_PDF = Path(__file__).resolve().parent / "dekota_kesim_sablonu_1e1_A4.pdf"


def draw_plate(ax, title, subtitle, features):
    # A4 dimensions in mm: 210 x 297
    ax.set_xlim(0, 210)
    ax.set_ylim(0, 297)
    ax.set_aspect("equal")
    ax.axis("off")

    # Center plate on A4 page
    # Plate: 150 wide, 100 high
    ox = (210 - 150) / 2  # 30 mm
    oy = (297 - 100) / 2  # 98.5 mm

    # Calibration scale (50 mm test ruler)
    ruler_x = 30
    ruler_y = 230
    ax.plot([ruler_x, ruler_x + 50], [ruler_y, ruler_y], color="black", lw=1.5)
    for tick in range(0, 51, 10):
        h = 4 if tick % 10 == 0 else 2
        ax.plot([ruler_x + tick, ruler_x + tick], [ruler_y - h, ruler_y], color="black", lw=1)
        if tick % 10 == 0:
            ax.text(ruler_x + tick, ruler_y + 2, f"{tick}", ha="center", va="bottom", fontsize=7)
    ax.text(ruler_x + 25, ruler_y - 8, "50 mm DOGRULAMA CETVELI (Yazdirirken 'Gercek Boyut / %100' secin)",
            ha="center", va="top", fontsize=7, fontweight="bold")

    # Header
    ax.text(105, 270, "YTU CDTP — IKI TEKERLEKLI ILAC TASIMA ROBOTU", ha="center", va="center",
            fontsize=12, fontweight="bold")
    ax.text(105, 263, f"{title} (150 mm x 100 mm Dekota Sablonu)", ha="center", va="center",
            fontsize=10, fontweight="bold", color="#1a56db")
    ax.text(105, 256, subtitle, ha="center", va="center", fontsize=8, color="#4b5563")

    # Outer cut outline with R5 rounded corners
    plate_box = FancyBboxPatch((ox, oy), 150, 100,
                               boxstyle="round,pad=0,rounding_size=5",
                               ec="black", fc="none", lw=1.5, ls="-")
    ax.add_patch(plate_box)

    # Plate Centerlines (dashed)
    ax.plot([ox, ox + 150], [oy + 50, oy + 50], color="#9ca3af", lw=0.5, ls="--")
    ax.plot([ox + 75, ox + 75], [oy, oy + 100], color="#9ca3af", lw=0.5, ls="--")
    ax.text(ox + 75, oy - 4, "ON / FORWARD (Y+)", ha="center", va="top", fontsize=8, fontweight="bold")
    ax.text(ox + 75, oy + 104, "ARKA / REAR (Y-)", ha="center", va="bottom", fontsize=8, color="#6b7280")

    # Draw features
    # Note: CAD coordinates origin is rear-left (y=0 rear, y=100 front)
    # We map plate local x, y directly onto ox + x, oy + y
    for f in features:
        name = f["name"]
        layer = f["layer"]
        fx = ox + f["x"]
        fy = oy + f["y"]

        if layer == "CUT_HOLES":
            if f.get("radius"):
                # Circular hole (tij, sigorta, led)
                dia = f["width"]
                r = dia / 2.0
                circle = Circle((fx, fy), r, ec="red", fc="none", lw=1.2)
                ax.add_patch(circle)
                # Drill crosshair
                ch = r + 2.5
                ax.plot([fx - ch, fx + ch], [fy, fy], color="red", lw=0.6)
                ax.plot([fx, fx], [fy - ch, fy + ch], color="red", lw=0.6)
                ax.text(fx, fy - r - 1.5, name, ha="center", va="top", fontsize=6, color="red")
            else:
                # Rectangular cut (kablo bacasi, switch, kayis)
                w, h = f["width"], f["height"]
                rect = Rectangle((fx, fy), w, h, ec="red", fc="#fee2e2", lw=1.2, ls="-")
                ax.add_patch(rect)
                ax.text(fx + w / 2, fy + h / 2, name, ha="center", va="center", fontsize=6.5,
                        fontweight="bold", color="#b91c1c")

        elif layer in ("REFERENCE", "POCKET"):
            # Placement footprint reference (dashed blue)
            w = f["width"]
            h = f.get("height", 0)
            if h == 0 and f.get("radius"):
                r = f.get("radius", f["width"] / 2.0)
                circle = Circle((fx, fy), r, ec="#2563eb", fc="#eff6ff", lw=0.8, ls="--")
                ax.add_patch(circle)
                ax.text(fx, fy, name, ha="center", va="center", fontsize=6, color="#1e40af")
            else:
                rect = Rectangle((fx, fy), w, h, ec="#2563eb", fc="#f8fafc", lw=0.8, ls="--")
                ax.add_patch(rect)
                ax.text(fx + w / 2, fy + h / 2, name, ha="center", va="center",
                        fontsize=6, color="#1e40af", wrap=True)

    # Footer note
    ax.text(105, 30, "KIRMIZI CIZGILER: KESIM VE DELIK | MAVI KESIKLI: BILESEN YERLESIM REFERANSI",
            ha="center", va="center", fontsize=7.5, fontweight="bold", color="#dc2626")
    ax.text(105, 24, "Dekotayi bu sablonun uzerine koyup kagit bantla sabitleyin; delikleri igne/biz ile isaretleyin.",
            ha="center", va="center", fontsize=7, color="#4b5563")


def get_layer_features():
    # Common features for all plates: M4 tij holes (dia 4.4mm) + Kablo bacasi (40x20mm)
    # Origin rear-left (0,0), Front is y=100. Tij centers at x=(6, 144), y=(6, 94)
    tij_holes = [
        {"name": "M4 Tij (4.4mm)", "layer": "CUT_HOLES", "x": x, "y": y, "width": 4.4, "radius": 2.2}
        for x in (6, 144) for y in (6, 94)
    ]
    cable_duct = {"name": "Kablo Bacasi (40x20)", "layer": "CUT_HOLES", "x": 55, "y": 15, "width": 40, "height": 20}

    # Layer 1: Taban (Base)
    l1 = tij_holes + [cable_duct] + [
        {"name": "Sol JGB37 Motor Braketi", "layer": "REFERENCE", "x": 12, "y": 28, "width": 18, "height": 44},
        {"name": "Sag JGB37 Motor Braketi", "layer": "REFERENCE", "x": 120, "y": 28, "width": 18, "height": 44},
        {"name": "TB6612FNG Surucu", "layer": "REFERENCE", "x": 64, "y": 39, "width": 22, "height": 22},
        {"name": "Sarj Pedi (+)", "layer": "REFERENCE", "x": 52.5, "y": 91, "width": 15, "height": 6},
        {"name": "Sarj Pedi (-)", "layer": "REFERENCE", "x": 82.5, "y": 91, "width": 15, "height": 6},
        {"name": "TCRT5000 Sensor Karti", "layer": "REFERENCE", "x": 67.5, "y": 88, "width": 15, "height": 11},
    ]

    # Layer 2: Guc (Power)
    l2 = tij_holes + [cable_duct] + [
        {"name": "KCD11 Switch (19x13)", "layer": "CUT_HOLES", "x": 10.5, "y": 11.5, "width": 19, "height": 13},
        {"name": "Sigorta (12.2mm)", "layer": "CUT_HOLES", "x": 130, "y": 18, "width": 12.2, "radius": 6.1},
        {"name": "3S 18650 Batarya Paketi", "layer": "REFERENCE", "x": 37.5, "y": 38, "width": 75, "height": 24},
        {"name": "Mini PDB Klemens", "layer": "REFERENCE", "x": 13, "y": 68, "width": 35, "height": 25},
        {"name": "XL4016 Regulator", "layer": "REFERENCE", "x": 90, "y": 68, "width": 45, "height": 25},
        {"name": "Cirt Kayis", "layer": "CUT_HOLES", "x": 17, "y": 38, "width": 16, "height": 8},
        {"name": "Cirt Kayis", "layer": "CUT_HOLES", "x": 17, "y": 54, "width": 16, "height": 8},
        {"name": "Cirt Kayis", "layer": "CUT_HOLES", "x": 117, "y": 38, "width": 16, "height": 8},
        {"name": "Cirt Kayis", "layer": "CUT_HOLES", "x": 117, "y": 54, "width": 16, "height": 8},
    ]

    # Layer 3: Kontrol (Brain)
    l3 = tij_holes + [cable_duct] + [
        {"name": "MPU-6050 (Merkez X=0, Y=0)", "layer": "REFERENCE", "x": 65, "y": 42.5, "width": 20, "height": 15},
        {"name": "ESP32 + Klemens Shield", "layer": "REFERENCE", "x": 42.5, "y": 62, "width": 65, "height": 28},
        {"name": "Sol TSOP2238", "layer": "REFERENCE", "x": 32.5, "y": 95, "width": 6, "radius": 3},
        {"name": "Sag TSOP2238", "layer": "REFERENCE", "x": 117.5, "y": 95, "width": 6, "radius": 3},
    ]

    # Layer 4: Tavan / Ilac (Top)
    l4 = tij_holes + [
        {"name": "Kablo Bacasi (20x15 Mini)", "layer": "CUT_HOLES", "x": 65, "y": 15, "width": 20, "height": 15},
        {"name": "Aspirin/Parol Ilac Yuvasi (80x26)", "layer": "POCKET", "x": 35, "y": 37, "width": 80, "height": 26},
        {"name": "Durum LED (5mm)", "layer": "CUT_HOLES", "x": 75, "y": 92, "width": 5.0, "radius": 2.5},
    ]

    return [
        ("KATMAN 1: TABAN / MOTOR VE SURUCU", "Motor braketleri, TB6612FNG surucu, sarj pedleri ve TCRT5000", l1),
        ("KATMAN 2: GUC VE ENERJI DAGITIM", "3S batarya, XL4016 regulator, Mini PDB, KCD11 switch ve sigorta", l2),
        ("KATMAN 3: KONTROL VE ALGILAMA", "ESP32 klemens shield, MPU-6050 merkez IMU ve 2x TSOP2238", l3),
        ("KATMAN 4: TAVAN VE ILAC YUVASI", "Standart 80x26 mm ilac kutusu yuvasi ve durum/sarj LED'i", l4),
    ]


def main():
    pages = get_layer_features()
    with PdfPages(OUTPUT_PDF) as pdf:
        for title, subtitle, features in pages:
            # A4 size in inches: 8.27 x 11.69 (210mm x 297mm)
            fig = plt.figure(figsize=(8.27, 11.69), dpi=300)
            ax = fig.add_axes([0, 0, 1, 1])
            draw_plate(ax, title, subtitle, features)
            pdf.savefig(fig, dpi=300)
            plt.close(fig)

    print(f"Olceklendirilmis 1:1 A4 PDF sablonu olusturuldu: {OUTPUT_PDF}")


if __name__ == "__main__":
    main()

