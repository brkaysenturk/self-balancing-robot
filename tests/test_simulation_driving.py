"""
Bütünleşik Sürüş, 1 Metre Mesafe, Duruş ve Geri Manevra Testi (Modül B & E)
Webots simülasyonunu çalıştırarak şartname isterlerini ve dinamik performansı doğrular:
1. Şartname Modül B (Seyir Mesafesi): Eldiven yönlendirmesiyle 0.8 m – 1.2 m mesafe kat edilmesi.
2. Kalıcı Duruş: Hedef mesafede tam duruş yapılması ve 3 saniye boyunca hareketsiz dik dengede (hız < 0.03 m/s) kalınması.
3. Geri Manevra Kabiliyeti: Geri komutuyla negatif hızda kararlı sürüş.
4. Modül E: Tüm hızlanma, fren ve manevralar boyunca Aspirin kutusunun yerel eksende hazneden düşmeden taşınması.
"""

import subprocess
import os
import sys
import csv
import shutil

def get_webots_executable() -> str:
    """Webots çalıştırılabilir dosya yolunu sistem ve ortam değişkenlerinden bulur."""
    env_path = os.environ.get("WEBOTS_EXECUTABLE") or os.environ.get("WEBOTS_HOME")
    if env_path:
        if os.path.isfile(env_path):
            return env_path
        candidate_win = os.path.join(env_path, "msys64", "mingw64", "bin", "webots.exe")
        if os.path.isfile(candidate_win):
            return candidate_win
        candidate_nix = os.path.join(env_path, "bin", "webots")
        if os.path.isfile(candidate_nix):
            return candidate_nix

    which_path = shutil.which("webots")
    if which_path and os.path.isfile(which_path):
        return which_path

    candidates = [
        r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
        "/usr/local/bin/webots",
        "/usr/bin/webots",
        "/snap/bin/webots",
        "/Applications/Webots.app/Contents/MacOS/webots",
    ]
    for c in candidates:
        if os.path.exists(c):
            return c

    raise FileNotFoundError(
        "Webots çalıştırılabilir dosyası bulunamadı! Lütfen WEBOTS_EXECUTABLE ortam değişkenini ayarlayın."
    )


def run_driving_test():
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    world_path = os.path.join(project_root, "webots", "worlds", "cdtp_balance_world.wbt")
    webots_exe = get_webots_executable()

    telemetry_path = os.path.join(project_root, "logs", "simulation_telemetry.csv")
    if os.path.exists(telemetry_path):
        try:
            os.remove(telemetry_path)
        except Exception:
            pass

    env = os.environ.copy()
    # 1750 kontrol adımı = 17.5 saniye
    # Adım 20: İleri sürüş (W) -> ~1.0 m yol alır
    # Adım 950: Dinamik fren (SPACE) -> t=11.0s civarı tam duruş (~1.00 m)
    # Adım 1100 - 1400 (t=11.0s - 14.0s, 3.0 saniye): 0.8-1.2 m bandında sabit dik duruş
    # Adım 1400 - 1750 (t=14.0s - 17.5s): Geri sürüş (S) -> negatif hız doğrulaması
    env["WEBOTS_TEST_STEPS"] = "1750"
    env["TEST_DRIVE_COMMANDS"] = "20:W,950:SPACE,1400:S"
    env.pop("TEST_PUSH_COMMANDS", None)

    print("=" * 65)
    print("Modül B (1m Mesafe + 3s Duruş + Geri Sürüş) & Modül E Testi")
    print("Test Kapsamı: 1 Metre Seyir + 3s Duruş + Geri Manevra + İlaç Güvenliği")
    print("=" * 65)

    res = subprocess.run(
        [webots_exe, "--port=1238", "--minimize", "--mode=fast", "--stdout", "--stderr", world_path],
        env=env,
        capture_output=True,
        text=True,
        timeout=55
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
                "vel_meas": float(row["vel_meas_mps"]),
                "pos_m": float(row["pos_m"])
            })

    if not records:
        print("[HATA] Telemetri kaydi bos!")
        return False

    # 2. Tamamlanma Doğrulaması (Erken çıkışı engeller)
    assert len(records) >= 800, f"[BASARISIZ] Telemetri eksik ({len(records)} kayıt alındı)!"
    assert records[-1]["time_s"] >= 17.3, f"[BASARISIZ] 17.5 saniye tamamlanamadı! Son zaman: {records[-1]['time_s']:.2f}s"
    assert records[-1]["step"] >= 1730, f"[BASARISIZ] 1750 adım tamamlanamadı! Son adım: {records[-1]['step']}"

    # 3. İleri Seyir Hızı (Adım 200 - 800 arası)
    fwd_vels = [r["vel_meas"] for r in records if 200 <= r["step"] <= 800]
    avg_fwd = sum(fwd_vels) / len(fwd_vels) if fwd_vels else 0.0

    # 4. Şartname Modül B: Duruş Öncesi Kat Edilen Mesafe (0.8 m – 1.2 m aralığı)
    stop_records = [r for r in records if 1100 <= r["step"] <= 1400]
    stop_pos = [r["pos_m"] for r in stop_records]
    avg_stop_pos = sum(stop_pos) / len(stop_pos) if stop_pos else 0.0

    # 5. Kalıcı Duruş Doğrulaması (3 saniye: t=11.0s - 14.0s, adım 1100 - 1400)
    stop_vels = [abs(r["vel_meas"]) for r in stop_records]
    avg_stop_vel = sum(stop_vels) / len(stop_vels) if stop_vels else 99.0
    max_stop_vel = max(stop_vels) if stop_vels else 99.0

    # 6. Geri Sürüş Fazı Hızı (Adım 1550 - 1750 arası)
    bwd_vels = [r["vel_meas"] for r in records if 1550 <= r["step"] <= 1750]
    avg_bwd = sum(bwd_vels) / len(bwd_vels) if bwd_vels else 0.0

    max_pitch = max(abs(r["pitch_deg"]) for r in records)

    print(f"İleri Seyir Ortalama Hız               : {avg_fwd:+.3f} m/s")
    print(f"Duruş Anındaki Net Mesafe (Hedef 0.8-1.2m): {avg_stop_pos:+.3f} metre")
    print(f"3 Saniyelik Duruş Ortalama Hız         : {avg_stop_vel:.4f} m/s")
    print(f"3 Saniyelik Duruş Azami Hız            : {max_stop_vel:.4f} m/s")
    print(f"Geri Sürüş Ortalama Hız                : {avg_bwd:+.3f} m/s")
    print(f"Tüm Test Boyunca Azami Gerçek Eğim     : {max_pitch:.2f} derece")

    # Başarı Kriterleri
    assert "Robot devrildi" not in res.stdout, "[BASARISIZ] Robot manevra anında devrildi!"
    assert "Ilac kutusu hazneden dustu" not in res.stdout, "[BASARISIZ] İlaç kutusu hazneden düştü!"
    assert 0.80 <= avg_stop_pos <= 1.20, f"[BASARISIZ] Kat edilen mesafe şartname aralığı (0.8 - 1.2 m) dışında: {avg_stop_pos:.3f} m"
    assert avg_fwd > 0.04, f"[BASARISIZ] İleri sürüş seyir hızı yetersiz: {avg_fwd:+.3f} m/s"
    assert avg_stop_vel < 0.02, f"[BASARISIZ] 3 saniyelik duruş fazında artık hız yüksek: {avg_stop_vel:.4f} m/s"
    assert max_stop_vel < 0.04, f"[BASARISIZ] 3 saniyelik duruş fazında kayma/titreme: {max_stop_vel:.4f} m/s"
    assert avg_bwd < -0.05, f"[BASARISIZ] Geri sürüş yetersiz veya yanlış yönde: {avg_bwd:+.3f} m/s"
    assert max_pitch < 15.0, f"[BASARISIZ] Manevra anında aşırı yatış: {max_pitch:.2f} derece"

    print("[BASARILI] Modül B (1m Mesafe + 3s Duruş + Geri Sürüş) ve Modül E Tam Doğrulandı!")
    return True

if __name__ == "__main__":
    success = run_driving_test()
    sys.exit(0 if success else 1)
