from pathlib import Path
import xml.etree.ElementTree as ET
from robot_ia.kinematics import BASE, L1, L2

def test_visual_floor_below_elbow_and_geometry_unchanged():
    root=ET.parse(Path(__file__).resolve().parents[1]/"models"/"brazo_3d.xml").getroot()
    geoms={g.get("name"):g for g in root.findall(".//geom") if g.get("name")}
    assert float(geoms["suelo"].get("pos").split()[2]) == -0.180
    assert "columna_soporte_visual" in geoms
    assert (BASE,L1,L2) == (0.079,0.113,0.140)
