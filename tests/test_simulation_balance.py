"""
Bütünleşik Denge, Bozucu Toparlanma ve Yük Güvenliği Testi (Modül A & E)
Webots simülasyonunu çalıştırarak şartname isterlerini ve dinamik performansı doğrular:
1. Şartname Modül A (Statik Denge): Dış destek almadan kesintisiz 30 saniye dik konumda kalma.
2. Harici Bozucu Dayanıklılığı: t=15s (+4.0 N) ve t=22s (-4.0 N) itme darbelerinden sonra toparlanma.
3. Modül E (İlaç Kutusu Güvenliği): 30 saniye boyunca dinamik Aspirin kutusunun yerel eksende hazneden düşmeden taşınması.
"""

import subprocess
import os
import sys
import csv

def run_balance_test():
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    world_path = os.path.join(project_root, "webots", "worlds", "cdtp_balance_world.wbt")
    webots_exe = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

    telemetry_path = os.path.join(project_root, "logs", "simulation_telemetry.csv")
    if os.path.exists(telemetry_path):
        try:
            os.remove(telemetry_path)
        except Exception:
            pass

    env = os.environ.copy()
    # 30.0 saniye = 3000 kontrol adımı (Modül A 30s isterini tam karşılar)
    env["WEBOTS_TEST_STEPS"] = "3000"
    # t=15s (adım 1500): +4.0 N ileri dürtme, t=22s (adım 2200): -4.0 N geri dürtme
    env["TEST_PUSH_COMMANDS"] = "1500:4.0,2200:-4.0"
    env.pop("TEST_DRIVE_COMMANDS", None)

    print("=" * 65)
    print("YTU CDTP - Modül A (30s Denge + 4.0N Dürtme) & Modül E Testi")
    print("Test Kapsamı: 30s Statik Denge + 4.0N Çift Yönlü İmpuls + İlaç Güvenliği")
    print("=" * 65)

    res = subprocess.run(
        [webots_exe, "--port=1238", "--minimize", "--mode=fast", "--stdout", "--stderr", world_path],
        env=env,
        capture_output=True,
        text=True,
        timeout=60
    )

    print("=== WEBOTS CIKTISI ===")
    print(res.stdout)
    if res.stderr:
        print("=== UYARILAR / HATALAR ===")
        print(res.stderr)

    # 1. Süreç Çıkış Doğrulaması (Crash Koruması)
    assert res.returncode == 0, f"[BASARISIZ] Webots hata ile sonlandı! Çıkış kodu: {res.returncode}"

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

    # 2. Tamamlanma Doğrulaması (Erken çıkışı engeller)
    assert len(records) >= 1350, f"[BASARISIZ] 30 saniyelik telemetri eksik ({len(records)} kayıt alındı)!"
    assert records[-1]["time_s"] >= 29.8, f"[BASARISIZ] 30 saniye tamamlanamadı! Son zaman: {records[-1]['time_s']:.2f}s"
    assert records[-1]["step"] >= 2990, f"[BASARISIZ] 3000 adım tamamlanamadı! Son adım: {records[-1]['step']}"

    # 3. Başlangıç Statik Denge Fazı (İlk 12 saniye, adım 20 - 1200)
    init_pitches = [abs(r["pitch_deg"]) for r in records if 20 <= r["step"] <= 1200]
    avg_init_pitch = sum(init_pitches) / len(init_pitches) if init_pitches else 99.0
    print(f"Statik Denge Ortalama Eğim (İlk 12 sn)     : {avg_init_pitch:.3f} derece")

    # 4. Dürtme Altında Azami Eğim ve Toparlanma Fazları
    max_pitch = max(abs(r["pitch_deg"]) for r in records)
    print(f"Tüm Test Boyunca Azami Gerçek Eğim        : {max_pitch:.2f} derece")

    # 1. Darbe toparlanması (Adım 1600 - 2100)
    recov1 = [abs(r["pitch_deg"]) for r in records if 1600 <= r["step"] <= 2100]
    avg_recov1 = sum(recov1) / len(recov1) if recov1 else 99.0

    # 2. Darbe toparlanması (Adım 2300 - 2800)
    recov2 = [abs(r["pitch_deg"]) for r in records if 2300 <= r["step"] <= 2800]
    avg_recov2 = sum(recov2) / len(recov2) if recov2 else 99.0

    print(f"1. Darbe Sonrası Ortalama Eğim (Toparlanma): {avg_recov1:.2f} derece")
    print(f"2. Darbe Sonrası Ortalama Eğim (Toparlanma): {avg_recov2:.2f} derece")
    print(f"Toplam Test Süresi                        : {records[-1]['time_s']:.2f} saniye")

    # Başarı Kriterleri
    assert "Robot devrildi" not in res.stdout, "[BASARISIZ] Robot devrildi!"
    assert "Ilac kutusu hazneden dustu" not in res.stdout, "[BASARISIZ] İlaç kutusu düştü!"
    assert avg_init_pitch < 0.20, f"[BASARISIZ] Statik denge salınımı yüksek: {avg_init_pitch:.3f} derece"
    assert max_pitch < 25.0, f"[BASARISIZ] Dürtme anında aşırı yatış: {max_pitch:.2f} derece"
    assert avg_recov1 < 2.0, f"[BASARISIZ] 1. darbe sonrası toparlanamadı: {avg_recov1:.2f} derece"
    assert avg_recov2 < 2.0, f"[BASARISIZ] 2. darbe sonrası toparlanamadı: {avg_recov2:.2f} derece"

    print("[BASARILI] Modül A (30s Denge + 4.0N Bozucu) ve Modül E (İlaç Güvenliği) Doğrulandı!")
    return True

if __name__ == "__main__":
    success = run_balance_test()
    sys.exit(0 if success else 1)
