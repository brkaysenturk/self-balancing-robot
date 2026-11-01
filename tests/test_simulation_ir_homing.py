"""
YTU CDTP - Modül C & E 38 kHz IR Alıcı-Verici İstasyona Otonom Yanaşma Doğrulama Testi
Robotun 30 cm yaklaşma konisi içerisinden (açısal ve yanal kaçıklıklı başlangıç konumundan)
2x TSOP38438 IR alıcısı ve 38 kHz IR verici fener nirengisiyle istasyona yönelip
ilaç kutusunu düşürmeden hedef mesafede otonom duruşunu Webots fizik motorunda doğrular.
"""

import csv
import math
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from self_balancing_robot import CONFIG
from tests.test_simulation_pid import get_webots_executable


def run_single_pose(pose_str: str, steps: int = 2400) -> None:
    world_path = ROOT / "webots" / "worlds" / "webots_self_balancing_robot.wbt"
    webots_exe = get_webots_executable()

    eval_path = ROOT / "logs" / "homing_evaluation.csv"
    telem_path = ROOT / "logs" / "homing_telemetry.csv"
    for p in (eval_path, telem_path):
        if p.exists():
            try:
                p.unlink()
            except OSError:
                pass

    env = os.environ.copy()
    env["PYTHONUTF8"] = "1"
    env["PID_LOG_PATH"] = str(telem_path)
    env["PID_EVALUATION_PATH"] = str(eval_path)
    env["WEBOTS_TEST_STEPS"] = str(steps)
    env["TEST_INIT_POSE"] = pose_str
    env["TEST_DRIVE_COMMANDS"] = "100:H"
    env.pop("TEST_PUSH_COMMANDS", None)

    print("=" * 70)
    print(f"Modül C & E: 38 kHz IR Alıcı-Verici Yanaşma Testi | Başlangıç: {pose_str}")
    print("=" * 70)

    res = subprocess.run(
        [webots_exe, "--port=1288", "--minimize", "--mode=fast", "--stdout", "--stderr", str(world_path)],
        cwd=str(ROOT),
        env=env,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=75,
    )

    if res.stderr:
        print(res.stderr)

    assert res.returncode == 0, f"Webots hata kodu ile kapandı: {res.returncode}"
    assert "Robot devrildi" not in res.stdout, "Robot yanaşma sırasında devrildi!"
    assert "Ilac kutusu hazneden dustu" not in res.stdout, "İlaç kutusu yanaşma sırasında düştü!"
    assert "[IR DOCKED]" in res.stdout, f"Robot ({pose_str}) hedef sürede IR istasyonuna yanaşıp duramadı!"

    with eval_path.open(encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    tail_rows = [r for r in rows if float(r["time"]) >= (steps * 0.01 - 2.5)]
    tail_vel = max(abs(float(r["velocity"])) for r in tail_rows)
    final_y = float(rows[-1]["position"])
    expected_front_y = 1.462 - CONFIG.homing.dock_stop_distance_m
    final_front_y = final_y + CONFIG.homing.receiver_forward_offset_m

    print(
        f"[{pose_str}] Son Ön Kenar Y: {final_front_y:.3f} m (Hedef: ~{expected_front_y:.3f} m) | "
        f"Son 2.5s Azami Hız: {tail_vel:.4f} m/s"
    )
    assert math.isclose(final_front_y, expected_front_y, abs_tol=0.055), (
        f"Yanaşma duruş konumu hedeften uzak: {final_front_y:.3f} m"
    )
    assert tail_vel < 0.04, f"İstasyonda duruş hızı sönümlenmedi: {tail_vel:.4f} m/s"


def run_homing_test() -> None:
    # 1. Senaryo: Sol-yakın açılı yaklaşma (X = -10 cm, Y = 1.12 m, yaw = -12.6 deg)
    run_single_pose("-0.10,1.12,-0.22", steps=2200)
    # 2. Senaryo: Sağ sınır yaklaşması (30 cm koni içi sağ sınır: X = +14 cm, Y = 1.12 m, yaw = +16 deg)
    run_single_pose("0.14,1.12,0.28", steps=2400)
    # 3. Senaryo: Yana/arkaya dönük başlangıç (yaw = +80 deg, IR görüş dışı) -> Olduğu yerde tarama (Scan) yapıp yanaşma
    run_single_pose("0.0,1.15,1.40", steps=2600)
    print("\n[BASARILI] Tüm 38 kHz IR Alıcı-Verici İstasyona Otonom Yanaşma Testleri Tamamlandı!")


if __name__ == "__main__":
    run_homing_test()
