"""
Bütünleşik Simülasyon Doğrulama Testi
Webots simülasyonunu çalıştırarak şartname isterlerini ve dinamik performansı doğrular:
1. Şartname Modül A (Statik Denge): 5 saniye dik konum ve mikro-salınım kararlılığı.
2. Harici Bozucu Dayanıklılığı: +4.0 N ve -4.0 N itme darbelerinden sonra toparlanma.
3. Modül E (İlaç Kutusu Güvenliği): Dinamik Aspirin kutusunun düşmeden taşınması.
"""

import subprocess
import os
import sys
import csv

def run_test():
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    world_path = os.path.join(project_root, "webots", "worlds", "cdtp_balance_world.wbt")
    webots_exe = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

    env = os.environ.copy()
    env["WEBOTS_TEST_STEPS"] = "500"  # 5.0 saniye = 500 kontrol adımı
    # Adım 150 (t=1.5s): +4.0 N ileri dürtme, Adım 320 (t=3.2s): -4.0 N geri dürtme
    env["TEST_PUSH_COMMANDS"] = "150:4.0,320:-4.0"

    print("=" * 65)
    print("YTU CDTP - Statik Denge ve Bozucu Toparlanma Testi (Modül A & E)")
    print("Test Kapsamı: 5.0s Denge + 4.0 N İtme Darbeleri + İlaç Kutusu")
    print("=" * 65)

    res = subprocess.run(
        [webots_exe, "--port=1238", "--minimize", "--mode=fast", "--stdout", "--stderr", world_path],
        env=env,
        capture_output=True,
        text=True,
        timeout=40
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
                "pitch_rate": float(row["pitch_rate_dps"]),
                "vel_meas": float(row["vel_meas_mps"]),
                "cmd_left": float(row["cmd_left_rads"])
            })

    if not records:
        print("[HATA] Telemetri kaydi bos!")
        return False

    # 1. Başlangıç Statik Denge Doğrulaması (İlk 1.2 saniye, adım 20 - 120)
    init_pitches = [abs(r["pitch_deg"]) for r in records if 20 <= r["step"] <= 120]
    avg_init_pitch = sum(init_pitches) / len(init_pitches) if init_pitches else 99.0
    print(f"Statik Denge Ortalama Eğim (t=0.2s - 1.2s): {avg_init_pitch:.3f} derece")

    # 2. Dürtme Altında Azami Eğim ve Toparlanma Fazları
    max_pitch = max(abs(r["pitch_deg"]) for r in records)
    print(f"Darbe Anlarındaki Azami Eğim             : {max_pitch:.2f} derece")

    # 1. Darbe toparlanması (Adım 220 - 280)
    recov1 = [abs(r["pitch_deg"]) for r in records if 220 <= r["step"] <= 280]
    avg_recov1 = sum(recov1) / len(recov1) if recov1 else 99.0

    # 2. Darbe toparlanması (Adım 400 - 480)
    recov2 = [abs(r["pitch_deg"]) for r in records if 400 <= r["step"] <= 480]
    avg_recov2 = sum(recov2) / len(recov2) if recov2 else 99.0

    print(f"1. Darbe Sonrasi Ortalama Eğim          : {avg_recov1:.2f} derece")
    print(f"2. Darbe Sonrasi Ortalama Eğim          : {avg_recov2:.2f} derece")

    # Kontrol & Başarı Kriterleri
    assert "Robot devrildi" not in res.stdout, "[BASARISIZ] Robot devrildi!"
    assert "Ilac kutusu hazneden dustu" not in res.stdout, "[BASARISIZ] İlaç kutusu düştü!"
    assert avg_init_pitch < 0.20, f"[BASARISIZ] Statik denge salınımı yüksek: {avg_init_pitch} derece"
    assert max_pitch < 30.0, f"[BASARISIZ] Dürtme anında devrilme eşiği aşıldı: {max_pitch} derece"
    assert avg_recov1 < 2.5, f"[BASARISIZ] 1. darbe sonrası toparlanamadı: {avg_recov1} derece"
    assert avg_recov2 < 2.5, f"[BASARISIZ] 2. darbe sonrası toparlanamadı: {avg_recov2} derece"

    print("[BASARILI] Modül A Statik Denge ve Bozucu Toparlanma Testi Tamamlandi!")
    return True

if __name__ == "__main__":
    success = run_test()
    sys.exit(0 if success else 1)
