"""
YTU CDTP - DC Motor & TB6612 Gerilim/PWM (Tork) Modeli Doğrulama Testi
Gerçekçi DC motor modeli (Ters EMK + Direnç + Redüktör Sürtünmesi) ve
ESP32 PWM / Gerilim tabanlı kontrolcünün statik denge ve yük güvenliği performansını doğrular.
"""

import csv
import os
import shutil
import subprocess


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
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate

    raise FileNotFoundError(
        "Webots çalıştırılabilir dosyası bulunamadı! Lütfen WEBOTS_EXECUTABLE ortam değişkenini ayarlayın."
    )


def run_pid_test() -> None:
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    world_path = os.path.join(project_root, "webots", "worlds", "webots_self_balancing_robot.wbt")
    webots_exe = get_webots_executable()

    telemetry_path = os.path.join(project_root, "logs", "pid_telemetry.csv")
    if os.path.exists(telemetry_path):
        try:
            os.remove(telemetry_path)
        except Exception:
            pass

    env = os.environ.copy()
    env["PID_LOG_PATH"] = telemetry_path
    env["WEBOTS_TEST_STEPS"] = "3000"
    # t=15s (adım 1500) anında 1.5 N impuls dürtme kuvveti
    env["TEST_PUSH_COMMANDS"] = "1500:1.5"
    env.pop("TEST_DRIVE_COMMANDS", None)

    print("=" * 65)
    print("Gerilim/PWM (Tork) Modeli 30s Denge ve Yük Testi")
    print("Test Kapsamı: TB6612 Gerilim/PWM + JGB37 Ters-EMK Modeli + İlaç Güvenliği")
    print("=" * 65)

    res = subprocess.run(
        [webots_exe, "--port=1280", "--minimize", "--mode=fast", "--stdout", "--stderr", world_path],
        env=env,
        capture_output=True,
        text=True,
        timeout=75,
    )

    print("=== WEBOTS CIKTISI ===")
    print(res.stdout)
    if res.stderr:
        print("=== UYARILAR / HATALAR ===")
        print(res.stderr)

    assert res.returncode == 0, f"Simulasyon sureci hata ile sonlandi! Cikis kodu: {res.returncode}"
    assert "Robot devrildi" not in res.stdout, "Robot simülasyonda devrildi!"
    assert "Ilac kutusu hazneden dustu" not in res.stdout, "Aspirin kutusu hazneden dustu!"

    assert os.path.exists(telemetry_path), "Telemetri dosyasi olusturulamadi!"
    records = []
    with open(telemetry_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            records.append({
                "time_s": float(row["time_s"]),
                "step": int(row["step"]),
                "pitch_deg": float(row["pitch_deg"]),
                "pos_m": float(row.get("pos_m", 0.0)),
                "velocity": float(row["vel_meas_mps"]),
                "cmd_left": float(row["cmd_left_volts"]),
            })

    assert len(records) >= 1350, f"Eksik telemetri verisi! Alinan kayit: {len(records)}"
    assert records[-1]["time_s"] >= 29.8, f"Simulasyon suresi yetersiz: {records[-1]['time_s']}s"
    assert records[-1]["step"] >= 2990, f"Simulasyon adimi eksik: {records[-1]['step']}"

    # İlk 14 saniye statik denge ortalama eğimi
    static_records = [r for r in records if r["time_s"] < 14.5]
    avg_static_pitch = sum(abs(r["pitch_deg"]) for r in static_records) / len(static_records)
    print(f"\nIlk 14.5s Ortalama Eğim: {avg_static_pitch:.4f} derece")
    assert avg_static_pitch < 0.35, f"Statik denge acisi yuksek: {avg_static_pitch:.4f} deg"

    tail = [r for r in records if r["time_s"] >= 27.0]
    assert max(abs(r["velocity"]) for r in tail) < 0.03, "Durtme sonrasi hiz sonmedi"
    assert max(abs(r["pitch_deg"]) for r in records) < 15.0, "Asiri govde yatışı"

    print("\n[BASARILI] Gerilim/PWM Tabanlı DC Motor Denge Testi (30s) Eksiksiz Tamamlandı!")


if __name__ == "__main__":
    run_pid_test()
