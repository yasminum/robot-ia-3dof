import json
import numpy as np
import pytest
from robot_ia.ctc_evolutivo import cargar_ganancias, GANANCIAS_REFERENCIA

def test_importacion(tmp_path):
    path=tmp_path/'x.json'
    path.write_text(json.dumps({'ganancias':GANANCIAS_REFERENCIA.tolist()}))
    assert np.allclose(cargar_ganancias(path),GANANCIAS_REFERENCIA)

def test_importacion_invalida(tmp_path):
    path=tmp_path/'x.json'
    path.write_text(json.dumps({'ganancias':[1,2]}))
    with pytest.raises(ValueError): cargar_ganancias(path)
