"""CTC con ganancias personalizables, sin modificar controlador funcional v0.2.7."""
import numpy as np

LIMITES_GANANCIAS = np.array([[20., 250.]] * 3 + [[3., 55.]] * 3, float)


def validar_ganancias(theta):
    theta = np.asarray(theta, dtype=float)
    if theta.shape != (6,) or not np.all(np.isfinite(theta)):
        raise ValueError("Se requieren seis ganancias finitas [Kp1,Kp2,Kp3,Kd1,Kd2,Kd3]")
    if np.any(theta < LIMITES_GANANCIAS[:, 0]) or np.any(theta > LIMITES_GANANCIAS[:, 1]):
        raise ValueError("Ganancias fuera de rangos definidos")
    return theta


def torque_ctc(q, dq, qr, dqr, ddqr, M, bias, limites, theta):
    theta = validar_ganancias(theta)
    q, dq, qr, dqr, ddqr = (np.asarray(v, float) for v in (q, dq, qr, dqr, ddqr))
    M, bias, limites = (np.asarray(v, float) for v in (M, bias, limites))
    if not all(x.shape == (3,) for x in (q,dq,qr,dqr,ddqr,bias,limites)) or M.shape != (3,3):
        raise ValueError("Dimensiones inválidas para robot de 3 GDL")
    acel = ddqr + theta[:3] * (qr-q) + theta[3:] * (dqr-dq)
    torque = M @ acel + bias
    return np.clip(torque, -limites, limites)


GANANCIAS_REFERENCIA = np.array([169.,169.,169.,28.6,28.6,28.6])


def cargar_ganancias(ruta):
    """Importa únicamente las 6 ganancias numéricas validadas del JSON del optimizador."""
    import json
    from pathlib import Path
    datos=json.loads(Path(ruta).read_text(encoding='utf-8'))
    if not isinstance(datos,dict) or 'ganancias' not in datos:
        raise ValueError('JSON sin campo ganancias')
    return validar_ganancias(datos['ganancias'])
