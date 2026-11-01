"""
================================================================================
MODUL: 38 kHz DIFERANSIYEL IR NİRENGİ VE İSTASYONA YANAŞMA (IR HOMING)
================================================================================
Şartname Modül C kapsamında; istasyon üzerindeki 38 kHz kızılötesi (IR) verici
fenerin yayılım konisini ve robotun Katman 3 ön kenarında 85 mm aralıklı 2 adet
TSOP38438 IR alıcısının (sol/sağ) diferansiyel sinyal şiddeti algılamasını
modeller. Herhangi bir şarj/elektriksel kilitlenme içermeden yalnızca alıcı-verici
hizalanma ve istasyona otonom yanaşıp durma komutlarını (desired_v, desired_turn) üretir.
================================================================================
"""

from dataclasses import dataclass
import math

from .config import CONFIG, RobotConfig
from .dc_motor_model import clamp


@dataclass(frozen=True)
class IRReading:
    """Sol ve sağ TSOP38438 IR alıcı okumaları ve geometrik nirengi özeti."""

    ir_left: float          # Sol alıcı normalize sinyal şiddeti [0.0 - 1.0]
    ir_right: float         # Sağ alıcı normalize sinyal şiddeti [0.0 - 1.0]
    ir_diff: float          # Normalize fark: (ir_right - ir_left) / (ir_right + ir_left)
    ir_sum: float           # Toplam sinyal gücü: ir_left + ir_right
    distance_m: float       # Robot ön kenarından IR vericiye yatay mesafe (m)
    bearing_rad: float      # Robot ileri (+Y) ekseninden IR vericiye yönelim açısı (rad, +sağ / -sol)
    approach_bearing_rad: float  # İstasyon merkez eksenine oturtan sanal kılavuz noktasının yönelim açısı (rad)
    heading_error_rad: float     # İstasyon dik eksenine (+Y) göre gövde açı hatası (rad, +sağa dönmeli)
    lateral_error_m: float       # İstasyon merkez ekseninden yanal sapma (m)
    in_cone: bool                # İstasyonun 90 derece (yarı açı 45 derece) yaklaşma konisinde mi?
    tcrt5000_triggered: bool     # Ön tampondaki TCRT5000 yakınlık sensörü istasyon duvarını algıladı mı?


class IRHomingController:
    """
    2x TSOP38438 alıcı, 1x TCRT5000 yakınlık sensörü ve 38 kHz IR verici fener tabanlı
    otonom yanaşma kontrolcüsü.
    """

    def __init__(self, cfg: RobotConfig = CONFIG):
        self.cfg = cfg
        self.active: bool = False
        self.docked: bool = False
        self.scan_dir: float = 1.0

    def start(self) -> None:
        """IR yanaşma modunu başlatır."""
        self.active = True
        self.docked = False

    def stop(self) -> None:
        """IR yanaşma modunu devre dışı bırakır."""
        self.active = False
        self.docked = False

    def read_receivers(
        self,
        robot_xy: tuple[float, float],
        robot_yaw_rad: float,
        beacon_xy: tuple[float, float],
        beacon_normal_xy: tuple[float, float] = (0.0, -1.0),
    ) -> IRReading:
        """
        Robotun anlık konumu (X, Y) ve yönelim açısından (yaw_rad, ENU'da 0 rad = +Y ileri),
        Katman 3 ön kenarındaki 2x TSOP38438 IR alıcısının (85 mm aralıklı) gördüğü
        38 kHz IR sinyal şiddetlerini hesaplar.
        """
        hcfg = self.cfg.homing

        # ENU koordinatlarında robotun ileri (+Y_local) ve sağ (+X_local) birim vektörleri
        # Webots R2025a'da rotation=(0,0,1,psi) için:
        # ileri (+Y_body) = (-sin(psi), cos(psi)), sağ (+X_body) = (cos(psi), sin(psi))
        fwd_x = -math.sin(robot_yaw_rad)
        fwd_y = math.cos(robot_yaw_rad)
        right_x = math.cos(robot_yaw_rad)
        right_y = math.sin(robot_yaw_rad)

        # Robot ön kenar merkezi (Katman 3 ön kenar)
        front_x = robot_xy[0] + fwd_x * hcfg.receiver_forward_offset_m
        front_y = robot_xy[1] + fwd_y * hcfg.receiver_forward_offset_m

        # Ön kenardan vericiye vektör ve mesafe
        dx_front = beacon_xy[0] - front_x
        dy_front = beacon_xy[1] - front_y
        dist_front = math.hypot(dx_front, dy_front)

        # Robot ileri eksenine göre verici yönelim açısı (bearing: +sağda, -solda)
        proj_fwd = dx_front * fwd_x + dy_front * fwd_y
        proj_right = dx_front * right_x + dy_front * right_y
        bearing_rad = math.atan2(proj_right, proj_fwd)

        # Vericiden robota doğru birim vektör (yayılım konisi kontrolü için)
        if dist_front > 1e-6:
            b_to_r_x = -dx_front / dist_front
            b_to_r_y = -dy_front / dist_front
        else:
            b_to_r_x, b_to_r_y = beacon_normal_xy

        bn_norm = math.hypot(beacon_normal_xy[0], beacon_normal_xy[1]) or 1.0
        bn_x = beacon_normal_xy[0] / bn_norm
        bn_y = beacon_normal_xy[1] / bn_norm

        cos_beacon = clamp(b_to_r_x * bn_x + b_to_r_y * bn_y, -1.0, 1.0)
        beacon_angle_deg = math.degrees(math.acos(cos_beacon))
        in_cone = (
            beacon_angle_deg <= hcfg.beacon_half_angle_deg
            and dist_front <= hcfg.max_detection_range_m
        )

        # İstasyonun dik yaklaşma ekseni (verici normalinin tersi: robotun son anda bakması gereken yön)
        dock_in_x = -bn_x
        dock_in_y = -bn_y
        dock_right_x = -bn_y
        dock_right_y = bn_x

        # Robot ön kenarının istasyon merkez ekseninden yanal sapması (lateral_error_m) ve boyuna mesafesi
        rel_x = front_x - beacon_xy[0]
        rel_y = front_y - beacon_xy[1]
        longitudinal_m = max(0.0, rel_x * bn_x + rel_y * bn_y)
        lateral_error_m = rel_x * dock_right_x + rel_y * dock_right_y

        # Sanal Kılavuz Noktası (Carrot-Chasing): Robot köşeden geldiğinde doğrudan vericiye çapraz
        # dalmak yerine, önce istasyon merkez eksenine (X=0) kavis çizerek oturmasını sağlar.
        # Robot yanal olarak açıktaysa (lateral_error_m), istasyonun önünde (Y_beacon - lead_dist)
        # merkez koridor noktasını hedefler; merkeze yaklaştıkça doğrudan vericiye ilerler.
        abs_lat = abs(lateral_error_m)
        if longitudinal_m > hcfg.dock_stop_distance_m + 0.03 or abs_lat > 0.04:
            lead_dist = clamp(0.45 * longitudinal_m + 0.30 * abs_lat, 0.08, 0.24)
        else:
            lead_dist = 0.0
        carrot_x = beacon_xy[0] + bn_x * lead_dist
        carrot_y = beacon_xy[1] + bn_y * lead_dist
        dx_c = carrot_x - front_x
        dy_c = carrot_y - front_y
        dist_carrot = math.hypot(dx_c, dy_c)
        if dist_carrot > 0.03:
            approach_bearing_rad = math.atan2(
                dx_c * right_x + dy_c * right_y,
                dx_c * fwd_x + dy_c * fwd_y,
            )
        else:
            approach_bearing_rad = bearing_rad

        # İstasyon dik eksenine (dock_in) göre robotun gövde açı hatası (+ ise sağa dönmeli)
        heading_error_rad = math.atan2(
            dock_in_x * right_x + dock_in_y * right_y,
            dock_in_x * fwd_x + dock_in_y * fwd_y,
        )

        # Sol ve Sağ TSOP38438 sensör konumları (Katman 3 ön kenar, X = +/- 42.5 mm)
        half_space = 0.5 * hcfg.receiver_spacing_m
        # Sol sensöre hafif sola (-15 derece), sağ sensöre hafif sağa (+15 derece) optik bakış açısı
        cant_rad = math.radians(15.0)
        fov_half_rad = math.radians(hcfg.receiver_fov_half_deg)
        beacon_half_rad = math.radians(hcfg.beacon_half_angle_deg)

        def sensor_intensity(side_sign: float) -> float:
            sx = front_x + side_sign * half_space * right_x
            sy = front_y + side_sign * half_space * right_y
            vx = beacon_xy[0] - sx
            vy = beacon_xy[1] - sy
            d = math.hypot(vx, vy)
            if d < 1e-4 or d > hcfg.max_detection_range_m:
                return 0.0

            # Verici yayılım paterni (yakın alanda d < 15 cm huni içinde iç yansıma ile geniş açı)
            cos_emit = (-vx * bn_x - vy * bn_y) / d
            eff_beacon_half_rad = (
                max(beacon_half_rad, math.radians(68.0)) if d < 0.15 else beacon_half_rad
            )
            if cos_emit <= math.cos(eff_beacon_half_rad):
                return 0.0
            emit_gain = (cos_emit - math.cos(eff_beacon_half_rad)) / (
                1.0 - math.cos(eff_beacon_half_rad)
            )

            # Alıcı optik ekseni (side_sign = -1 sol, +1 sağ)
            look_angle = side_sign * cant_rad
            look_x = fwd_x * math.cos(look_angle) + right_x * math.sin(look_angle)
            look_y = fwd_y * math.cos(look_angle) + right_y * math.sin(look_angle)

            cos_recv = (vx * look_x + vy * look_y) / d
            if cos_recv <= math.cos(fov_half_rad):
                return 0.0
            recv_gain = (cos_recv - math.cos(fov_half_rad)) / (1.0 - math.cos(fov_half_rad))

            # Mesafe zayıflaması (ters kare + yakın alan doyumu)
            atten = 1.0 / (1.0 + 4.5 * (d ** 2))
            return float(clamp(emit_gain * recv_gain * atten, 0.0, 1.0))

        ir_left = sensor_intensity(-1.0)
        ir_right = sensor_intensity(+1.0)
        ir_sum = ir_left + ir_right
        ir_diff = (ir_right - ir_left) / (ir_sum + 1e-6) if ir_sum > 1e-4 else 0.0

        tcrt5000_triggered = (
            dist_front <= hcfg.dock_stop_distance_m
            and abs(lateral_error_m) <= 0.065
            and abs(heading_error_rad) <= math.radians(35.0)
        )

        return IRReading(
            ir_left=ir_left,
            ir_right=ir_right,
            ir_diff=ir_diff,
            ir_sum=ir_sum,
            distance_m=dist_front,
            bearing_rad=bearing_rad,
            approach_bearing_rad=approach_bearing_rad,
            heading_error_rad=heading_error_rad,
            lateral_error_m=lateral_error_m,
            in_cone=in_cone,
            tcrt5000_triggered=tcrt5000_triggered,
        )

    def compute_commands(self, reading: IRReading) -> tuple[float, float, bool]:
        """
        IR alıcı okumasından (desired_v, desired_turn, is_docked) üretir.
        - desired_v: İleri sürüş hızı komutu (m/s)
        - desired_turn: Diferansiyel dönüş komutu (+sağa dön, -sola dön, rad/s)
        - is_docked: İstasyona yanaşma/duruş mesafesine ulaşıldı mı?
        """
        hcfg = self.cfg.homing

        if not self.active:
            return 0.0, 0.0, False

        # İstasyon önünde hedef duruş mesafesine ulaşıldıysa (TCRT5000 yakınlık sensörü tetiklendiyse):
        # Eğer gövde hala çapraz duruyorsa (heading_error > tolerans), ilerlemeyi kesip yerinde tam dik hizalan!
        if reading.tcrt5000_triggered or reading.distance_m <= hcfg.dock_stop_distance_m:
            if abs(reading.heading_error_rad) > hcfg.dock_align_tolerance_rad:
                align_turn = clamp(
                    hcfg.kp_bearing_turn * reading.heading_error_rad,
                    -hcfg.max_homing_turn_rate,
                    hcfg.max_homing_turn_rate,
                )
                return 0.0, align_turn, False
            self.docked = True
            return 0.0, 0.0, True

        if self.docked and reading.distance_m <= hcfg.dock_stop_distance_m + 0.025:
            if abs(reading.heading_error_rad) > hcfg.dock_align_tolerance_rad * 1.5:
                self.docked = False
            else:
                return 0.0, 0.0, True

        self.docked = False

        # Robot +/- 45 derece IR konisinin veya menzilin tamamen dışındaysa hareket etmez.
        if not reading.in_cone:
            return 0.0, 0.0, False

        # Robot koni içinde ama yüzü başka yöne dönük olduğu için IR alıcıları ışığı görmüyorsa (ir_sum < 1e-3):
        # Olduğu yerde yavaşça kendi etrafında dönerek (Scan Modu) 38 kHz IR ışığını arar!
        if reading.ir_sum < 1e-3:
            return 0.0, self.scan_dir * hcfg.search_turn_rate, False

        if abs(reading.ir_diff) > 0.05:
            self.scan_dir = 1.0 if reading.ir_diff >= 0.0 else -1.0

        # Diferansiyel IR farkı + sanal kılavuz noktası (merkez eksenine oturtan yaklaşma açısı)
        # Son 12 cm içinde gövdeyi istasyon dik eksenine (heading_error_rad) paralel hale getir.
        near_blend = clamp((0.14 - reading.distance_m) / 0.08, 0.0, 1.0)
        steer_angle = (
            (1.0 - near_blend) * reading.approach_bearing_rad
            + near_blend * (0.65 * reading.bearing_rad + 0.35 * reading.heading_error_rad)
        )
        # Robot yanal olarak merkezden uzaktaysa (> 5 cm), doğrudan vericiye dönen ir_diff yerine
        # merkez koridora (carrot) yönlendiren steer_angle'a öncelik ver:
        ir_weight = clamp(1.0 - abs(reading.lateral_error_m) / 0.08, 0.15, 1.0)
        raw_turn = (
            ir_weight * hcfg.kp_ir_turn * reading.ir_diff
            + hcfg.kp_bearing_turn * steer_angle
        )
        desired_turn = clamp(raw_turn, -hcfg.max_homing_turn_rate, hcfg.max_homing_turn_rate)

        # Açısal hata çok büyükse (geniş köşelerde > 35 derece, koridor içinde > 52 derece)
        # önce yerinde hizalan, sonra ilerle
        turn_in_place_limit = (
            math.radians(35.0) if abs(reading.lateral_error_m) > 0.14 else math.radians(52.0)
        )
        if abs(steer_angle) > turn_in_place_limit:
            desired_v = 0.0
        elif reading.distance_m <= hcfg.slowdown_distance_m:
            # 30 cm yaklaşma konisi içinde hassas yanaşma hızı (creep speed)
            scale = clamp(
                (reading.distance_m - hcfg.dock_stop_distance_m)
                / max(1e-3, hcfg.slowdown_distance_m - hcfg.dock_stop_distance_m),
                0.45,
                1.0,
            )
            desired_v = hcfg.creep_speed_mps * scale * max(0.35, math.cos(steer_angle))
        else:
            desired_v = hcfg.cruise_speed_mps * max(0.35, math.cos(steer_angle))

        return desired_v, desired_turn, False
