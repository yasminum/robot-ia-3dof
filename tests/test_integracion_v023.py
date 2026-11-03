import ast
from pathlib import Path
import xml.etree.ElementTree as ET

def test_codigo_compila():
    ast.parse((Path(__file__).parents[1]/'main.py').read_text(encoding='utf-8'))

def test_xml_valido():
    root=ET.parse(Path(__file__).parents[1]/'models'/'brazo_3d.xml').getroot()
    assert root.tag=='mujoco'
    assert len(root.findall('.//body/joint'))==3
    assert len(root.findall('.//actuator/motor'))==3
