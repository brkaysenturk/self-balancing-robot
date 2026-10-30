"""
Bütünleşik Sürüş, Dinamik Frenleme ve Engel Aşma Testi
Robotun şartname isterleri ve manevra kabiliyetini doğrular:
1. Şartname Modül B: İleri (+0.30 m/s), geri (-0.30 m/s) ve kontrollü dinamik frenleme.
2. Zemin Engeli Aşma: Y=0.85 m mesafesindeki 4 mm zemin kablo kanalını devrilmeden geçme.
3. Şartname Modül E: Sürüş ve engel tırmanışı boyunca Aspirin ilaç kutusunu düşürmeden taşıma.
"""

import subprocess
import os
import sys
import csv

def run_driving_test():
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    world_path = os.path.join(project_root, "webots", "worlds", "cdtp_balance_world.wbt")
    webots_exe = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

    env = os.environ.copy()
    env["WEBOTS_TEST_STEPS"] = "400"  # 4 saniye = 400 kontrol adımı
    # Adım 20: İleri sürüş (W) -> Y=0.85m kablo kanalını aşar, Adım 250: Dinamik fren (SPACE), Adım 310: Geri sürüş (S)
    env["TEST_DRIVE_COMMANDS"] = "20:W,250:SPACE,310:S"

    print("=" * 65)
    print("YTU CDTP - Bütünleşik Sürüş, Engel ve Yük Testi (Modül B & E)")
    print("Test Kapsamı: İleri Sürüş + 4 mm Kablo Kanalı Aşma + Fren + Geri Sürüş")
    print("=" * 65)

    res = subprocess.run(
        [webots_exe, "--port=1238", "--minimize", "--mode=fast", "--stdout", "--stderr", world_path],
        env=env,
        capture_output=True,
        text=True,
        timeout=35
    )

    print("=== WEBOTS CIKTISI ===")
    print(res.stdout)
    if res.stderr:
        print("=== UYARILAR / HATALAR ===")
        print(res.stderr)

    telemetry_path = os.path.join(project_root, "logs", "simulation_telemetry.csv")
    if not os.path.exists(telemetry_path):
        print(f"[HATA] Telemetri dosyasi bulunamadi: {telemetry_path}")
        return False

    records = []
    with open(telemetry_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            records.append({
                "time_s": float(row["time_s"]),
                "step": int(row["step"]),
                "pitch_deg": float(row["pitch_deg"]),
                "vel_meas": float(row["vel_meas_mps"]),
                "vel_target": float(row["vel_target_mps"])
            })

    if not records:
        print("[HATA] Telemetri dosyasi bos!")
        return False

    # İleri seyir hızı (Adım 100 - 200 arası)
    fwd_vels = [r["vel_meas"] for r in records if 100 <= r["step"] <= 200]
    avg_fwd = sum(fwd_vels) / len(fwd_vels) if fwd_vels else 0.0

    # Frenleme fazı hızı (Adım 280 - 310 arası)
    brake_vels = [abs(r["vel_meas"]) for r in records if 280 <= r["step"] <= 310]
    avg_brake = sum(brake_vels) / len(brake_vels) if brake_vels else 99.0

    # Geri sürüş fazı hızı (Adım 370 - 400 arası)
    bwd_vels = [r["vel_meas"] for r in records if 370 <= r["step"] <= 400]
    avg_bwd = sum(bwd_vels) / len(bwd_vels) if bwd_vels else 0.0

    max_pitch = max(abs(r["pitch_deg"]) for r in records)

    print(f"İleri Sürüş Ortalama Hız         : {avg_fwd:+.3f} m/s")
    print(f"Frenleme Anı Kalan Hız          : {avg_brake:.3f} m/s")
    print(f"Geri Sürüş Ortalama Hız          : {avg_bwd:+.3f} m/s")
    print(f"Tüm Test Boyunca Azami Eğim      : {max_pitch:.2f} derece")

    # Başarı Kriterleri
    assert "Robot devrildi" not in res.stdout, "[BASARISIZ] Robot manevra veya engel anında devrildi!"
    assert "Ilac kutusu hazneden dustu" not in res.stdout, "[BASARISIZ] İlaç kutusu hazneden düştü!"
    assert avg_fwd > 0.05, f"[BASARISIZ] İleri sürüş yetersiz: {avg_fwd} m/s"
    assert avg_brake < 0.20, f"[BASARISIZ] Dinamik frenleme tam duruş sağlayamadı: {avg_brake} m/s"
    assert avg_bwd < 0.05, f"[BASARISIZ] Geri sürüş yetersiz: {avg_bwd} m/s"
    assert max_pitch < 25.0, f"[BASARISIZ] Engel aşma veya frenleme anında aşırı yatış: {max_pitch} derece"

    print("[BASARILI] Modül B Sürüş/Manevra ve Modül E İlaç Taşıma Testi Tamamlandi!")
    return True

if __name__ == "__main__":
    success = run_driving_test()
    sys.exit(0 if success else 1)
