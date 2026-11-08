"""Generate matching millimetre SVG/DXF drawings using only the standard library.

CUT_* entities are through cuts; REFERENCE and POCKET are not through cuts.
Board footprints are placement envelopes, not measured mounting patterns.
"""

import argparse
from dataclasses import dataclass
from html import escape
from pathlib import Path


ROOT = Path(__file__).resolve().parent
SHEET_HEIGHT = 285


@dataclass(frozen=True)
class Feature:
    name: str
    layer: str
    x: float
    y: float
    width: float
    height: float = 0
    radius: float = 0


def plates():
    """Coordinates in mm: origin at rear left, front is local y=100."""
    common = [Feature("Plaka 150 x 100 / R5", "CUT_OUTLINE", 0, 0, 150, 100, 5)]
    common += [Feature("M4 tij / cap 4.4", "CUT_HOLES", x, y, 4.4)
               for x in (6, 144) for y in (6, 94)]
    common += [Feature("Kablo bacasi 40 x 20", "CUT_HOLES", 55, 15, 40, 20)]
    return [
        ("KATMAN 1 - TABAN", 10, 25, common + [
            Feature("Motor L-braket (olculecek)", "REFERENCE", 12, 28, 18, 44),
            Feature("Motor L-braket (olculecek)", "REFERENCE", 120, 28, 18, 44),
            Feature("TB6612FNG", "REFERENCE", 64, 39, 22, 22),
            Feature("Sarj pedi (+) 15 x 6", "REFERENCE", 52.5, 91, 15, 6),
            Feature("Sarj pedi (-) 15 x 6", "REFERENCE", 82.5, 91, 15, 6),
        ]),
        ("KATMAN 2 - GUC", 185, 25, common + [
            Feature("Switch 19 x 13", "CUT_HOLES", 10.5, 11.5, 19, 13),
            Feature("Sigorta / cap 12.2", "CUT_HOLES", 130, 18, 12.2),
            Feature("Batarya (yerlesim)", "REFERENCE", 37.5, 38, 75, 24),
            Feature("Mini PDB (yerlesim)", "REFERENCE", 13, 68, 35, 25),
            Feature("XL4016 (yerlesim)", "REFERENCE", 90, 68, 45, 25),
            *[Feature("Kayis 16 x 8", "CUT_HOLES", x, y, 16, 8)
              for x in (17, 117) for y in (38, 54)],
        ]),
        ("KATMAN 3 - KONTROL", 10, 150, common + [
            Feature("MPU-6050 (yerlesim)", "REFERENCE", 65, 42.5, 20, 15),
            Feature("ESP32 shield (olculecek)", "REFERENCE", 42.5, 62, 65, 28),
            Feature("TSOP2238 sol", "REFERENCE", 32.5, 100, 3),
            Feature("TSOP2238 sag", "REFERENCE", 117.5, 100, 3),
        ]),
        ("KATMAN 4 - ILAC", 185, 150, common + [
            Feature("Kaset IC SINIRI 80 x 26", "POCKET", 35, 37, 80, 26),
            Feature("LED / cap 5", "CUT_HOLES", 75, 92, 5),
        ]),
    ]


def svg_text():
    out = ['<svg xmlns="http://www.w3.org/2000/svg" width="350mm" height="285mm" '
           'viewBox="0 0 350 285" font-family="sans-serif" font-size="2.5" fill="#c9d1d9">',
           '<rect width="350" height="285" fill="#0d1117"/>',
           '<path d="' + ' '.join(f'M {x} 0 V 285' for x in range(0, 351, 5))
           + ' ' + ' '.join(f'M 0 {y} H 350' for y in range(0, 286, 5))
           + '" fill="none" stroke="#222b38" stroke-width="0.12"/>',
           '<rect x="5" y="3" width="340" height="15" rx="2" fill="#161b22" stroke="#30363d" stroke-width="0.3"/>',
           '<text x="10" y="9" fill="#58a6ff" font-size="4" font-weight="bold">'
           'YTÜ ÇDTP — İKİ TEKERLEKLİ İLAÇ TAŞIMA ROBOTU</text>',
           '<text x="10" y="14" fill="#8b949e" font-size="2.4">'
           '150 × 100 mm Dekota | 40 × 20 mm kablo bacası | 190 mm iz: fiziksel montaj teyidi gerekli</text>',
           '<text x="337" y="9" text-anchor="end" fill="#f0883e" font-size="3" font-weight="bold">TEKNİK YERLEŞİM</text>',
           '<text x="337" y="14" text-anchor="end" fill="#79c0ff">mm / 1:1 · 35 × 50 cm tabaka</text>']
    titles = ["KATMAN 1: TABAN / MOTORLAR", "KATMAN 2: GÜÇ VE DAĞITIM",
              "KATMAN 3: KONTROL VE ALGILAMA", "KATMAN 4: İLAÇ TAŞIMA"]
    for index, (title, ox, oy, features) in enumerate(plates()):
        out.append(f'<g transform="translate({ox},{oy})">')
        out.append(f'<text x="0" y="-2" fill="#f0883e" font-size="3.2" font-weight="bold">{titles[index]}</text>')
        for f in features:
            color, fill = {"CUT_OUTLINE": ("#388bfd", "#161b22"),
                           "CUT_HOLES": ("#f0883e", "#0d1117"),
                           "REFERENCE": ("#3fb950", "#14231b"),
                           "POCKET": ("#f0883e", "#261b15")}[f.layer]
            if f.name.startswith(("M4", "M3")):
                color, fill = "#f85149", "#ff7b72"
            elif f.name.startswith("Kablo"):
                color, fill = "#58a6ff", "#13233a"
            elif f.name.startswith("Mini PDB"):
                color, fill = "#bc8cff", "#1e192c"
            elif f.name.startswith("XL4016"):
                color, fill = "#d29922", "#241d14"
            elif f.name.startswith(("ESP32", "Batarya")):
                color, fill = "#14b8a6", "#132426"
            elif f.name.startswith("MPU"):
                color, fill = "#f0883e", "#261914"
            attrs = (f'id="{title.split()[1]}-{features.index(f)}" class="{f.layer}" '
                     f'fill="{fill}" stroke="{color}" stroke-width="0.4"')
            if f.layer in ("REFERENCE", "POCKET"):
                attrs += ' stroke-dasharray="1.4 0.8"'
            if f.height:
                out.append(f'<rect {attrs} x="{f.x}" y="{f.y}" width="{f.width}" '
                           f'height="{f.height}" rx="{f.radius}"><title>{escape(f.name)}</title></rect>')
            else:
                out.append(f'<circle {attrs} cx="{f.x}" cy="{f.y}" r="{f.width / 2}">'
                           f'<title>{escape(f.name)}</title></circle>')
            if not f.name.startswith(("M4", "M3", "Plaka")):
                tx = f.x + f.width / 2 if f.height else f.x
                ty = f.y + f.height / 2 if f.height else f.y - f.width / 2 - 2
                label, detail = {
                    "Kablo bacasi 40 x 20": ("KABLO BACASI", "40 × 20 mm / dikey geçiş"),
                    "Motor L-braket (olculecek)": ("JGB37", "L-braket"),
                    "TB6612FNG": ("TB6612FNG", "Motor sürücü"),
                    "Batarya (yerlesim)": ("3S 18650 Li-ion BATARYA", "Merkezde / yerleşim referansı"),
                    "Mini PDB (yerlesim)": ("MİNİ PDB", "KF301 klemensler"),
                    "XL4016 (yerlesim)": ("XL4016 REGÜLATÖR", "12 V → 5 V / yerleşim"),
                    "MPU-6050 (yerlesim)": ("MPU-6050", "Tam merkez"),
                    "ESP32 shield (olculecek)": ("ESP32 38-PIN SHIELD", "Vidalı terminal / ölçülecek"),
                    "Kaset IC SINIRI 80 x 26": ("İLAÇ KASETİ / İÇ SINIR", "80 × 26 mm / tam kesilmez"),
                    "Switch 19 x 13": ("SWITCH", "19 × 13 mm"),
                    "Kayis 16 x 8": ("KAYIŞ GEÇİŞİ", "16 × 8 mm"),
                    "Sigorta / cap 12.2": ("SİGORTA Ø12.2", ""),
                    "LED / cap 5": ("DURUM LEDİ Ø5", ""),
                    "TSOP2238 sol": ("TSOP2238 / SOL", ""),
                    "TSOP2238 sag": ("TSOP2238 / SAĞ", ""),
                    "Sarj pedi (+) 15 x 6": ("ŞARJ PEDİ (+)", "15 × 6 mm"),
                    "Sarj pedi (-) 15 x 6": ("ŞARJ PEDİ (−)", "15 × 6 mm"),
                }.get(f.name, (f.name, ""))
                size = 2.1 if f.width <= 22 else 2.8
                if f.name.startswith(("Sarj pedi", "Kayis")):
                    size = 1.8
                if not f.height:
                    size = 2.3
                out.append(f'<text x="{tx}" y="{ty - 1 if detail else ty + 0.8}" '
                           f'text-anchor="middle" fill="#e6edf3" font-weight="bold" '
                           f'font-size="{size}">{escape(label)}</text>')
                if detail:
                    out.append(f'<text x="{tx}" y="{ty + 3}" text-anchor="middle" '
                               f'fill="{color}" font-size="{min(size, 2.3)}">{escape(detail)}</text>')
        out.append('<path d="M 0 103 V 106 M 0 104.5 H 150 M 150 103 V 106" '
                   'fill="none" stroke="#58a6ff" stroke-width="0.2" stroke-dasharray="1 1"/>')
        out.append('<text x="75" y="110" text-anchor="middle" fill="#79c0ff">150 mm × 100 mm · ÖN / +Y</text></g>')
    out.append('<rect x="5" y="263" width="340" height="19" rx="2" fill="#161b22" stroke="#30363d" stroke-width="0.3"/>')
    notes = [
        "MAVİ: dış kesim  |  TURUNCU: açıklıklar  |  KIRMIZI: Ø4.4 mm M4 tij, kenardan 6 mm",
        "KESİLMEZ: kesikli kart / IR / ped referansları ve 80 × 26 mm kaset iç sınırı. Kaset sağlam tavan üzerine çerçevelenir.",
        "Kart / braket delikleri ve TSOP2238 tutucusu ölçülecek. CAM: yalnız CUT_OUTLINE + CUT_HOLES katmanlarını seçin.",
        "1:1 baskı: %100 / sayfaya sığdırma kapalı. Kontrol çubuğu: 50 mm",
    ]
    for i, note in enumerate(notes):
        out.append(f'<text x="10" y="{267 + i * 4}" fill="#c9d1d9" font-size="2.4">{escape(note)}</text>')
    out.append('<path d="M 265 279 h 50 m -50 -1 v 2 m 50 -2 v 2" fill="none" stroke="#79c0ff" stroke-width="0.2"/>')
    return "\n".join(out + ["</svg>", ""])


def dxf_text():
    """ASCII DXF R12 entities; explicit INSUNITS=4 documents mm for importers."""
    out = []

    def pairs(*values):
        out.extend(str(v) for v in values)

    pairs(0, "SECTION", 2, "HEADER", 9, chr(36) + "ACADVER", 1, "AC1009",
          9, chr(36) + "INSUNITS", 70, 4, 0, "ENDSEC",
          0, "SECTION", 2, "TABLES", 0, "TABLE", 2, "LAYER", 70, 4)
    for layer, color in (("CUT_OUTLINE", 5), ("CUT_HOLES", 1), ("REFERENCE", 3), ("POCKET", 6)):
        pairs(0, "LAYER", 2, layer, 70, 1 if layer in ("REFERENCE", "POCKET") else 0,
              62, color, 6, "CONTINUOUS")
    pairs(0, "ENDTAB", 0, "ENDSEC", 0, "SECTION", 2, "ENTITIES")

    def line(layer, x1, y1, x2, y2):
        pairs(0, "LINE", 8, layer, 10, x1, 20, y1, 30, 0, 11, x2, 21, y2, 31, 0)

    for _, ox, oy, features in plates():
        for f in features:
            x, y = ox + f.x, SHEET_HEIGHT - oy - f.y
            if not f.height:
                pairs(0, "CIRCLE", 8, f.layer, 10, x, 20, y, 30, 0, 40, f.width / 2)
                continue
            y -= f.height
            w, h, r = f.width, f.height, f.radius
            line(f.layer, x + r, y, x + w - r, y)
            line(f.layer, x + w, y + r, x + w, y + h - r)
            line(f.layer, x + w - r, y + h, x + r, y + h)
            line(f.layer, x, y + h - r, x, y + r)
            if r:
                for cx, cy, start in ((x + w - r, y + r, 270), (x + w - r, y + h - r, 0),
                                      (x + r, y + h - r, 90), (x + r, y + r, 180)):
                    pairs(0, "ARC", 8, f.layer, 10, cx, 20, cy, 30, 0, 40, r, 50, start, 51, start + 90)
    pairs(0, "ENDSEC", 0, "EOF")
    return "\n".join(out) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Fail if checked-in drawings are stale")
    args = parser.parse_args()
    for name, content in (("pleksi_kesim_sablonu.svg", svg_text()), ("pleksi_katmanlari.dxf", dxf_text())):
        path = ROOT / name
        if args.check:
            if path.read_text(encoding="utf-8") != content:
                raise SystemExit(f"Stale drawing: {path}")
        else:
            path.write_text(content, encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()
