"""Fabrication dimensions and wheel clearances must agree with the digital twin."""

from pathlib import Path
import re
import runpy
import xml.etree.ElementTree as ET

import pytest

from self_balancing_robot import CONFIG


ROOT = Path(__file__).resolve().parents[1]
CAD = ROOT / "docs/cad"
NS = {"s": "http://www.w3.org/2000/svg"}


def test_generated_drawings_are_current():
    generator = runpy.run_path(str(CAD / "generate_chassis.py"))
    for name, function in (("pleksi_kesim_sablonu.svg", "svg_text"),
                           ("pleksi_katmanlari.dxf", "dxf_text")):
        assert (CAD / name).read_text(encoding="utf-8") == generator[function]()


def test_svg_has_physical_scale_and_specified_cut_dimensions():
    svg = ET.parse(CAD / "pleksi_kesim_sablonu.svg").getroot()
    assert (svg.get("width"), svg.get("height"), svg.get("viewBox")) == (
        "350mm", "285mm", "0 0 350 285")
    outlines = svg.findall(".//s:rect[@class='CUT_OUTLINE']", NS)
    assert len(outlines) == 4
    assert all((p.get("width"), p.get("height"), p.get("rx")) == ("150", "100", "5") for p in outlines)
    pocket = svg.find(".//s:rect[@class='POCKET']", NS)
    assert pocket is not None
    assert tuple(float(pocket.get(k)) for k in ("x", "y", "width", "height")) == (35, 37, 80, 26)
    circles = svg.findall(".//s:circle[@class='CUT_HOLES']", NS)
    radii = [float(c.get("r")) for c in circles]
    assert radii.count(1.7) == 16
    assert radii.count(6.1) == 1
    assert radii.count(2.5) == 1


def test_dxf_circles_match_svg_and_ir_is_not_cut():
    svg = ET.parse(CAD / "pleksi_kesim_sablonu.svg").getroot()
    svg_circles = set()
    for group in svg.findall("s:g", NS):
        ox, oy = map(float, re.findall(r"[\d.]+", group.get("transform")))
        for c in group.findall("s:circle", NS):
            svg_circles.add((c.get("class"), ox + float(c.get("cx")),
                             285 - oy - float(c.get("cy")), float(c.get("r"))))
    lines = (CAD / "pleksi_katmanlari.dxf").read_text().splitlines()
    dxf_circles = set()
    entity = {}
    for code, value in zip(lines[::2], lines[1::2]):
        if code == "0":
            if entity.get("0") == "CIRCLE":
                dxf_circles.add((entity["8"], float(entity["10"]), float(entity["20"]), float(entity["40"])))
            entity = {}
        entity[code] = value
    assert dxf_circles == svg_circles
    assert sum(c[0] == "REFERENCE" for c in dxf_circles) == 2


def test_webots_wheels_clear_full_width_base_and_match_odometry():
    world = (ROOT / "webots/worlds/webots_self_balancing_robot.wbt").read_text(encoding="utf-8")
    centers = [float(v) for v in re.findall(r'name "(?:left|right)_wheel"\s+translation ([-\d.]+) 0 0', world)]
    assert len(centers) == 2
    assert centers[1] - centers[0] == pytest.approx(CONFIG.mechanics.track_width)
    assert "size 0.150 0.100 0.005" in world
    widths = [float(v) for v in re.findall(r'geometry Cylinder\s*{\s*height ([\d.]+)\s*radius 0.0425', world)]
    assert len(widths) == 2
    for center, width in zip(centers, widths):
        assert abs(center) - width / 2 - 0.150 / 2 >= 0.0025 - 1e-9
