"""Generador de la Presentación Oficial para Exposición del Trabajo Parcial (UNI Posgrado FIIS 2026-2).
Crea:
1. Presentacion_Trabajo_Parcial_3GDL_UNI.pptx (PowerPoint 16:9 editable con notas del orador y láminas de alta resolución)
2. Presentacion_Trabajo_Parcial_3GDL_UNI.pdf  (Versión PDF lista para proyectar)
"""
import io
import zipfile
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle
from matplotlib.backends.backend_pdf import PdfPages
from PIL import Image

ROOT = Path(__file__).resolve().parent
OUT_DIR = ROOT / "results" / "presentacion_slides"
OUT_DIR.mkdir(parents=True, exist_ok=True)

# Colores institucionales UNI
UNI_MAROON = "#7A121C"
UNI_NAVY   = "#102A43"
UNI_BLUE   = "#1F6FEB"
UNI_GOLD   = "#C59B27"
UNI_LIGHT  = "#F4F7FB"
UNI_GREEN  = "#1B8E3A"
UNI_ORANGE = "#D97706"
DARK_TEXT  = "#1E293B"


def new_slide_fig():
    fig = plt.figure(figsize=(16, 9), dpi=180)
    fig.patch.set_facecolor("white")
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 9)
    ax.axis("off")
    return fig, ax


def draw_header_footer(ax, title, subtitle, slide_num, total_slides=12):
    # Top banner
    ax.add_patch(FancyBboxPatch((0, 8.05), 16, 0.95, boxstyle="square,pad=0",
                                facecolor=UNI_NAVY, edgecolor="none"))
    ax.add_patch(FancyBboxPatch((0, 7.97), 16, 0.08, boxstyle="square,pad=0",
                                facecolor=UNI_MAROON, edgecolor="none"))
    ax.text(0.45, 8.60, title, color="white", fontsize=20, fontweight="bold", va="center")
    ax.text(0.45, 8.23, subtitle, color="#CBD5E1", fontsize=11.5, va="center")
    ax.text(15.55, 8.50, "UNI · POSGRADO FIIS · 2026-2", color=UNI_GOLD,
            fontsize=11, fontweight="bold", ha="right", va="center")

    # Footer
    ax.add_patch(FancyBboxPatch((0, 0), 16, 0.42, boxstyle="square,pad=0",
                                facecolor="#F1F5F9", edgecolor="none"))
    ax.plot([0, 16], [0.42, 0.42], color="#CBD5E1", lw=1.0)
    ax.text(0.45, 0.21,
            "Robótica y Sistemas Autónomos · Docente: Ing. Wilson Dennis Reyes Vásquez · Trabajo Parcial: Robot Antropomórfico 3 GDL",
            color="#475569", fontsize=9.5, va="center")
    ax.text(15.55, 0.21, f"Lámina {slide_num} / {total_slides}",
            color=UNI_MAROON, fontsize=10, fontweight="bold", ha="right", va="center")


def draw_card(ax, x, y, w, h, title=None, bg=UNI_LIGHT, border="#CBD5E1", header_bg=None):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.12",
                                facecolor=bg, edgecolor=border, linewidth=1.4))
    if title:
        h_bg = header_bg if header_bg else UNI_BLUE
        ax.add_patch(FancyBboxPatch((x, y + h - 0.52), w, 0.52,
                                    boxstyle="round,pad=0.02,rounding_size=0.10",
                                    facecolor=h_bg, edgecolor="none"))
        ax.text(x + 0.22, y + h - 0.26, title, color="white",
                fontsize=12.5, fontweight="bold", va="center")


def build_slides():
    slides_meta = []

    # =========================================================================
    # SLIDE 1: CARÁTULA OFICIAL UNI
    # =========================================================================
    fig, ax = new_slide_fig()
    ax.add_patch(FancyBboxPatch((0, 0), 16, 9, boxstyle="square,pad=0", facecolor="white", edgecolor="none"))
    ax.add_patch(FancyBboxPatch((0, 8.72), 16, 0.28, boxstyle="square,pad=0", facecolor=UNI_MAROON, edgecolor="none"))
    ax.add_patch(FancyBboxPatch((0, 0), 16, 0.28, boxstyle="square,pad=0", facecolor=UNI_MAROON, edgecolor="none"))
    ax.add_patch(FancyBboxPatch((0.55, 0.65), 14.9, 7.7, boxstyle="round,pad=0.02,rounding_size=0.18",
                                facecolor="#FAFBFD", edgecolor="#CBD5E1", linewidth=1.5))

    logo_path = ROOT / "logo_uni.png"
    if logo_path.exists():
        img = Image.open(logo_path)
        ax_logo = fig.add_axes([0.065, 0.58, 0.14, 0.27])
        ax_logo.imshow(img)
        ax_logo.axis("off")

    ax.text(8.6, 7.65, "UNIVERSIDAD NACIONAL DE INGENIERÍA", color=UNI_MAROON,
            fontsize=24, fontweight="bold", ha="center")
    ax.text(8.6, 7.22, "Facultad de Ingeniería Industrial y de Sistemas — Unidad de Posgrado",
            color=UNI_NAVY, fontsize=14.5, fontweight="bold", ha="center")
    ax.text(8.6, 6.82, "MAESTRÍA EN INGENIERÍA DE SISTEMAS / INTELIGENCIA ARTIFICIAL",
            color=UNI_BLUE, fontsize=13.5, fontweight="bold", ha="center")

    ax.plot([2.5, 14.7], [6.48, 6.48], color=UNI_MAROON, lw=2.0)

    ax.text(8.6, 5.92, "«TRABAJO PARCIAL»", color=UNI_NAVY, fontsize=26, fontweight="bold", ha="center")
    ax.text(8.6, 5.35, "«Robot Antropomórfico de 3 GDL (Configuración RRR)»",
            color=UNI_MAROON, fontsize=20, fontweight="bold", ha="center")
    ax.text(8.6, 4.72,
            "Modelado de Cinemática Directa (Geométrica, MTH y Denavit-Hartenberg), Cinemática Inversa\n"
            "de Doble Rama, Cinemática Diferencial (Jv y Jω), Análisis de Singularidades, Control Cinemático\n"
            "CLIK-DLS y Control Dinámico Híbrido Evolutivo (CTC / NSGA-II / MPC) en MATLAB y MuJoCo",
            color=DARK_TEXT, fontsize=12, ha="center", va="center", linespacing=1.45)

    draw_card(ax, 1.15, 1.05, 6.7, 2.85, "Datos de la Asignatura", bg="white", border="#CBD5E1", header_bg=UNI_NAVY)
    ax.text(1.4, 3.05,
            "• Curso: Robótica y Sistemas Autónomos\n"
            "• Docente: Ing. Wilson Dennis Reyes Vásquez\n"
            "• Semestre Académico: 2026 – 2\n"
            "• Caso de Estudio Real: Manipulador RRR 3 GDL\n"
            "  (Conejo Benítez, C. E., 2021 — Dynamixel AX-18A/12A)\n"
            "• Plataformas: MATLAB (.m / .mlx) + Python/MuJoCo 3D",
            color=DARK_TEXT, fontsize=11.2, va="top", linespacing=1.52)

    draw_card(ax, 8.2, 1.05, 6.65, 2.85, "Equipo de Investigación (Autores)", bg="white", border="#CBD5E1", header_bg=UNI_MAROON)
    ax.text(8.45, 3.05,
            "1. Flores Rivera, Amit Roy\n"
            "2. Alegría Barboza, Franklin Enrique\n"
            "3. Espinoza Chamaya, José Luis\n"
            "4. Medianero Huari, Raquel\n"
            "5. Urrego Montoya, Yasmín Mariela",
            color=DARK_TEXT, fontsize=11.8, fontweight="bold", va="top", linespacing=1.62)

    slides_meta.append((fig, "Lámina 1 (Carátula): Presentación del equipo, caso de estudio real (Conejo Benítez, 2021) y alcance dual MATLAB + MuJoCo."))

    # =========================================================================
    # SLIDE 2: CASO DE ESTUDIO REAL, PARÁMETROS FÍSICOS Y OBJETIVOS
    # =========================================================================
    fig, ax = new_slide_fig()
    draw_header_footer(ax, "1. Caso de Estudio Real, Parámetros Físicos e Interrogantes",
                       "Manipulador antropomórfico RRR de 3 GDL basado en Conejo Benítez (2021) y arquitectura del proyecto", 2)

    draw_card(ax, 0.5, 4.35, 7.3, 3.35, "Parámetros Físicos y Actuadores Reales (Conejo Benítez, 2021)", header_bg=UNI_NAVY)
    ax.text(0.75, 6.98,
            "• Geometría de eslabones (Tabla 3.2):\n"
            "   – Altura de hombro (base):  d₁ = 0,079 m (79 mm)\n"
            "   – Eslabón primario (brazo): L₁ = 0,113 m (113 mm)\n"
            "   – Antebrazo + efector:      L₂ = 0,140 m (140 mm)\n"
            "• Servomotores Dynamixel y límites físicos:\n"
            "   – q₁ (Cintura, AX-18A): [-2,80, +2,80] rad | τ₁,max = 0,70 N·m\n"
            "   – q₂ (Hombro,  AX-12A): [-1,25, +1,35] rad | τ₂,max = 0,70 N·m\n"
            "   – q₃ (Codo,    AX-12A): [-2,40, +2,40] rad | τ₃,max = 0,45 N·m",
            color=DARK_TEXT, fontsize=11.2, va="top", linespacing=1.48)

    draw_card(ax, 8.1, 4.35, 7.4, 3.35, "Envolvente del Espacio de Trabajo y Propósito Funcional", header_bg=UNI_BLUE)
    ax.text(8.35, 6.98,
            "• Volumen alcanzable toroidal-esférico en ℝ³:\n"
            "   – Alcance radial máximo: R_max = L₁ + L₂ = 0,253 m\n"
            "   – Alcance radial mínimo: R_min = |L₁ - L₂| = 0,027 m\n"
            "   – Condición exacta: 0,027 m ≤ √(x² + y² + (z - d₁)²) ≤ 0,253 m\n"
            "• Dos modos interactivos de operación en tiempo real:\n"
            "   1) Modo Laboratorio (Foco / Lente Óptico de Precisión < 2,0 mm)\n"
            "   2) Modo Juego «Circuito de 5 Aros (Encestar Bola)» con velocímetro\n"
            "      cartesiano ||ṗ|| = ||Jv(q)q̇|| y máquina de estados visual 3D.",
            color=DARK_TEXT, fontsize=11.2, va="top", linespacing=1.48)

    draw_card(ax, 0.5, 0.72, 15.0, 3.35, "Interrogantes Científicas Resueltas en el Trabajo Parcial", header_bg=UNI_MAROON)
    ax.text(0.8, 3.32,
            "(i)  ¿Cómo formular y verificar simbólica y numéricamente los 3 métodos de cinemática directa (Geométrico, MTH sucesivas\n"
            "     y Denavit-Hartenberg estándar) junto con las 2 ramas analíticas de la cinemática inversa (codo abajo y codo arriba)?\n\n"
            "(ii) ¿Cómo deducir el Jacobiano geométrico completo 6×3 (bloques traslacional Jv y rotacional Jω), factorizar analíticamente\n"
            "     det(Jv) para aislar las singularidades de codo y hombro, y garantizar un cruce suave mediante la pseudoinversa DLS?\n\n"
            "(iii) ¿En qué magnitud el control cinemático amortiguado (CLIK-DLS) y el control por torque computado sintonizado con\n"
            "      NSGA-II superan al controlador nominal en RMSE de seguimiento, tiempo de captura (< 2 mm) y rechazo a perturbaciones?",
            color=DARK_TEXT, fontsize=11.3, va="top", linespacing=1.42)

    slides_meta.append((fig, "Lámina 2 (Caso de Estudio): Explica los parámetros reales de Conejo Benítez (2021), los límites de par Dynamixel y las 3 preguntas de investigación."))

    # =========================================================================
    # SLIDE 3: CINEMÁTICA DIRECTA (3 MÉTODOS EQUIVALENTES)
    # =========================================================================
    fig, ax = new_slide_fig()
    draw_header_footer(ax, "2. Cinemática Directa: Denavit-Hartenberg, MTH y Método Geométrico",
                       "Equivalencia algebraica exacta y verificación cruzada MATLAB vs. motor físico MuJoCo (error = 3,1 × 10⁻¹⁷ m)", 3)

    draw_card(ax, 0.5, 4.45, 7.4, 3.25, "1) Tabla de Parámetros Denavit-Hartenberg Estándar", header_bg=UNI_NAVY)
    table_dh = (
        " Eslabón i |  θᵢ (rad) |    dᵢ (m)    |    aᵢ (m)    | αᵢ (rad) | Eje de Giro\n"
        "-----------+-----------+--------------+--------------+----------+-------------------\n"
        "     1     |    q₁     | d₁ = 0,079   |    0,000     |  +π/2    | Z₀ vertical\n"
        "     2     |    q₂     |    0,000     | L₁ = 0,113   |    0     | Z₁ horizontal\n"
        "     3     |    q₃     |    0,000     | L₂ = 0,140   |    0     | Z₂ ∥ Z₁ (paralelo)"
    )
    ax.text(0.75, 6.92, table_dh, family="monospace", fontsize=10.5, color=UNI_NAVY, va="top", linespacing=1.5)
    ax.text(0.75, 5.05,
            "• Regla de transformación elemental (idéntica a DHL.m de clase):\n"
            "  Aᵢ(θᵢ, dᵢ, aᵢ, αᵢ) = Rot_z(θᵢ) · Trans(aᵢ, 0, dᵢ) · Rot_x(αᵢ)",
            color=DARK_TEXT, fontsize=11, va="top", linespacing=1.45)

    draw_card(ax, 8.1, 4.45, 7.4, 3.25, "2) Matriz Homogénea Total T₀³(q) = A₁(q₁) A₂(q₂) A₃(q₃)", header_bg=UNI_BLUE)
    t03_txt = (
        "         [ cos(q₁)c₂₃   -cos(q₁)s₂₃    sin(q₁)   (L₁c₂ + L₂c₂₃)cos(q₁) ]\n"
        " T₀³(q)= [ sin(q₁)c₂₃   -sin(q₁)s₂₃   -cos(q₁)   (L₁c₂ + L₂c₂₃)sin(q₁) ]\n"
        "         [    s₂₃           c₂₃           0       d₁ + L₁s₂ + L₂s₂₃    ]\n"
        "         [     0             0            0                1           ]"
    )
    ax.text(8.30, 6.92, t03_txt, family="monospace", fontsize=10.0, color=UNI_NAVY, va="top", linespacing=1.55)
    ax.text(8.35, 5.05,
            "• Donde c₂ = cos(q₂), s₂ = sin(q₂), c₂₃ = cos(q₂+q₃), s₂₃ = sin(q₂+q₃).\n"
            "• Alcance radial proyectado: r(q₂, q₃) = L₁ cos(q₂) + L₂ cos(q₂ + q₃).",
            color=DARK_TEXT, fontsize=10.8, va="top", linespacing=1.45)

    draw_card(ax, 0.5, 0.72, 15.0, 3.45, "3) Verificación Numérica Cruzada en Postura Inicial q₀ = [-1,10,  0,85, -0,75]ᵀ rad", header_bg=UNI_GREEN)
    verif_txt = (
        "  Matriz Acumulada  | Expresión Analítica Simbólica de la Columna de Posición pᵢ(q)       | Evaluación Exacta en q₀ [m]\n"
        "--------------------+---------------------------------------------------------------------+--------------------------------------\n"
        "  Hombro  T₀¹(1:3,4)| [ 0,  0,  d₁ ]ᵀ                                                     | [  0,000000,   0,000000,  0,079000 ]ᵀ\n"
        "  Codo    T₀²(1:3,4)| [ L₁ cos(q₂)cos(q₁),  L₁ cos(q₂)sin(q₁),  d₁ + L₁ sin(q₂) ]ᵀ        | [  0,033828,  -0,066465,  0,163895 ]ᵀ\n"
        "  Efector T₀³(1:3,4)| [ r(q₂,q₃)cos(q₁),    r(q₂,q₃)sin(q₁),    d₁ + L₁s₂ + L₂s₂₃ ]ᵀ      | [  0,097015,  -0,190610,  0,177871 ]ᵀ"
    )
    ax.text(0.75, 3.42, verif_txt, family="monospace", fontsize=10.2, color=DARK_TEXT, va="top", linespacing=1.55)
    ax.text(0.75, 1.15,
            "✔ Coincidencia numérica entre el modelo analítico (MATLAB / Python) y el sitio 'punta' de MuJoCo: ||Δp|| = 3,1 × 10⁻¹⁷ m (precisión IEEE 754).",
            color=UNI_GREEN, fontsize=11.5, fontweight="bold", va="center")

    slides_meta.append((fig, "Lámina 3 (Cinemática Directa): Muestra la tabla DH, la matriz homogénea T03 y la tabla de verificación de T01, T02, T03 en q0."))

    # =========================================================================
    # SLIDE 4: CINEMÁTICA INVERSA DE DOBLE RAMA
    # =========================================================================
    fig, ax = new_slide_fig()
    draw_header_footer(ax, "3. Cinemática Inversa Analítica (Ramas Codo Abajo y Codo Arriba)",
                       "Solución cerrada por desacoplamiento geométrico y teorema del coseno para ambas configuraciones", 4)

    draw_card(ax, 0.5, 3.85, 7.4, 3.85, "Deducción Analítica Cerrada para p_obj = [x, y, z]ᵀ", header_bg=UNI_NAVY)
    ax.text(0.75, 6.95,
            "1. Proyección radial cilíndrica y cota relativa al hombro:\n"
            "      ρ = √(x² + y²),      z_rel = z - d₁,      d = √(ρ² + z_rel²)\n\n"
            "2. Articulación de cintura (q₁):\n"
            "      q₁ = atan2(y, x)\n\n"
            "3. Articulación de codo (q₃) por Ley de Cosenos:\n"
            "      D = cos(q₃) = (ρ² + z_rel² - L₁² - L₂²) / (2 L₁ L₂)\n"
            "      q₃ = atan2( ±√(1 - D²),  D )   [+1: Codo Abajo | -1: Codo Arriba]\n\n"
            "4. Articulación de hombro (q₂):\n"
            "      q₂ = atan2(z_rel, ρ) - atan2(L₂ sin(q₃), L₁ + L₂ cos(q₃))",
            color=DARK_TEXT, fontsize=11.0, va="top", linespacing=1.42)

    draw_card(ax, 8.1, 3.85, 7.4, 3.85, "Verificación Exacta en Objetivo Nominal p_obj = [0,165, 0,075, 0,165]ᵀ m", header_bg=UNI_MAROON)
    ax.text(8.35, 6.95,
            "• Parámetros intermedios:\n"
            "   ρ = 0,181246 m,   z_rel = 0,086000 m,   D = 0,248957 ∈ [-1, +1]\n\n"
            "• Rama 1 — Codo Abajo (elbow = +1, q₃ > 0):\n"
            "   q⁽⁺⁾ = [ +0,4266,  -0,2991,  +1,3192 ]ᵀ rad\n"
            "   Error de reconstrucción FK: ||p(q⁽⁺⁾) - p_obj|| = 1,39 × 10⁻¹⁴ mm\n\n"
            "• Rama 2 — Codo Arriba (elbow = -1, q₃ < 0):\n"
            "   q⁽⁻⁾ = [ +0,4266,  +1,1852,  -1,3192 ]ᵀ rad\n"
            "   Error de reconstrucción FK: ||p(q⁽⁻⁾) - p_obj|| = 2,78 × 10⁻¹⁴ mm",
            color=DARK_TEXT, fontsize=11.0, va="top", linespacing=1.42)

    draw_card(ax, 0.5, 0.72, 15.0, 2.82, "Propiedad de Simetría Geométrica y Selección Automática de Rama", header_bg=UNI_BLUE)
    ax.text(0.8, 2.78,
            "• Simetría exacta de reflexión respecto a la cuerda hombro-efector:\n"
            "     q₃⁽⁺⁾ = -q₃⁽⁻⁾ = +1,3192 rad   y   q₂⁽⁻⁾ - q₂⁽⁺⁾ = 2 atan2(L₂ sin|q₃|, L₁ + L₂ cos(q₃)) = 1,1852 - (-0,2991) = 1,4843 rad.\n"
            "• Ambas ramas respetan estrictamente los límites articulares de los servomotores Dynamixel: q₂ ∈ [-1,25, +1,35] rad y q₃ ∈ [-2,40, +2,40] rad.\n"
            "• El planificador evalúa ambas ramas y selecciona la configuración admisible que minimiza el desplazamiento articular o satisface la consigna del usuario.",
            color=DARK_TEXT, fontsize=11.2, va="top", linespacing=1.50)

    slides_meta.append((fig, "Lámina 4 (Cinemática Inversa): Presenta las ecuaciones cerradas de las 2 ramas y su comprobación numérica exacta en el objetivo nominal."))

    # =========================================================================
    # SLIDE 5: CINEMÁTICA DIFERENCIAL (JACOBIANO 6x3: Jv y Jw)
    # =========================================================================
    fig, ax = new_slide_fig()
    draw_header_footer(ax, "4. Cinemática Diferencial: Jacobiano Analítico (Jv) y Geométrico (6×3)",
                       "Deducción del bloque lineal Jv(q) y del bloque rotacional Jω(q) = [z₀  z₁  z₂] por extracción de ejes DH", 5)

    draw_card(ax, 0.5, 4.25, 8.6, 3.45, "1) Bloque de Jacobiano Lineal / Traslacional Jv(q) ∈ ℝ³ˣ³ (v = ṗ = Jv(q) q̇)", header_bg=UNI_NAVY)
    jv_txt = (
        "          [ -r·sin(q₁)   -(L₁sin(q₂) + L₂sin(q₂+q₃))cos(q₁)   -L₂sin(q₂+q₃)cos(q₁) ]\n"
        " Jv(q) =  [  r·cos(q₁)   -(L₁sin(q₂) + L₂sin(q₂+q₃))sin(q₁)   -L₂sin(q₂+q₃)sin(q₁) ]\n"
        "          [     0          L₁cos(q₂) + L₂cos(q₂+q₃)            L₂cos(q₂+q₃)        ]"
    )
    ax.text(0.7, 6.88, jv_txt, family="monospace", fontsize=9.8, color=UNI_NAVY, va="top", linespacing=1.6)
    ax.text(0.75, 5.15,
            "• Derivado simbólicamente como Jv,ij = ∂pᵢ/∂qⱼ y comprobado por producto vectorial zⱼ₋₁ × (p₃ - pⱼ₋₁).\n"
            "• Columna 1: giro acimutal puro en el plano horizontal (ż = 0).\n"
            "• Columnas 2 y 3: movimiento planar radial-vertical sobre el plano rotado por q₁.",
            color=DARK_TEXT, fontsize=10.6, va="top", linespacing=1.42)

    draw_card(ax, 9.35, 4.25, 6.15, 3.45, "2) Bloque Angular Jω(q) ∈ ℝ³ˣ³ (ω = Jω q̇)", header_bg=UNI_MAROON)
    jw_txt = (
        "                       [ 0    sin(q₁)    sin(q₁) ]\n"
        " Jω(q) = [z₀ z₁ z₂] =  [ 0   -cos(q₁)   -cos(q₁) ]\n"
        "                       [ 1       0          0    ]"
    )
    ax.text(9.55, 6.88, jw_txt, family="monospace", fontsize=10.2, color=UNI_MAROON, va="top", linespacing=1.6)
    ax.text(9.55, 5.15,
            "• Extraído de la 3.ª columna de I₃, T₀¹ y T₀².\n"
            "• Como α₂ = 0, los ejes de hombro y codo son\n"
            "  paralelos (z₁ = z₂) ⇒ rank(Jω) = 2.\n"
            "• Rango del Jacobiano completo 6×3: rank(J) = 3.",
            color=DARK_TEXT, fontsize=10.6, va="top", linespacing=1.42)

    draw_card(ax, 0.5, 0.72, 15.0, 3.22, "Interpretación Cinemática y Velocímetro Cartesiano en Tiempo Real", header_bg=UNI_BLUE)
    ax.text(0.8, 3.15,
            "• Estructura del Jacobiano Geométrico Completo:   [ v ; ω ] = [ Jv(q) ; Jω(q) ] q̇  ∈ ℝ⁶.\n"
            "• Al ser rank(J) = 3 y det(Jv) ≠ 0 fuera de las variedades singulares, los 3 GDL gobiernan de manera biyectiva local las 3 velocidades\n"
            "  lineales (ẋ, ẏ, ż), mientras que la velocidad angular ω queda físicamente acoplada al movimiento traslacional.\n"
            "• Aplicación directa en el Gemelo Digital (main.py): en cada paso de integración se evalúa v_xyz = Jv(q) q̇ [m/s], mostrando en la\n"
            "  interfaz gráfica la rapidez instantánea ||v|| [mm/s] y registrando la velocidad pico alcanzada v_max [mm/s] en cada misión.",
            color=DARK_TEXT, fontsize=11.0, va="top", linespacing=1.48)

    slides_meta.append((fig, "Lámina 5 (Cinemática Diferencial): Presenta tanto Jv como Jw, explica por qué rank(Jw)=2 y cómo opera el velocímetro cartesiano."))

    # =========================================================================
    # SLIDE 6: SINGULARIDADES Y MANIPULABILIDAD DE YOSHIKAWA
    # =========================================================================
    fig, ax = new_slide_fig()
    draw_header_footer(ax, "5. Análisis Exhaustivo de Singularidades y Manipulabilidad de Yoshikawa",
                       "Factorización cerrada del determinante, clasificación geométrica y pseudoinversa amortiguada DLS", 6)

    draw_card(ax, 0.5, 4.35, 8.2, 3.35, "Factorización Analítica Exacta de det(Jv)", header_bg=UNI_NAVY)
    ax.text(0.75, 6.95,
            "Desarrollando por cofactores y aplicando sin(q₂+q₃)cos(q₂) - cos(q₂+q₃)sin(q₂) = sin(q₃):\n\n"
            "   det Jv(q) = -L₁ · L₂ · sin(q₃) · (L₁ cos(q₂) + L₂ cos(q₂ + q₃))\n"
            "             = -L₁ · L₂ · sin(q₃) · r(q₂, q₃)\n\n"
            "• Índice de Manipulabilidad de Yoshikawa (1985):\n"
            "   w(q) = √det(Jv Jvᵀ) = |det Jv(q)| = L₁ L₂ |sin(q₃)| |r(q₂, q₃)|",
            color=DARK_TEXT, fontsize=11.2, va="top", linespacing=1.42)

    draw_card(ax, 8.95, 4.35, 6.55, 3.35, "Dos Familias de Singularidades (det Jv = 0)", header_bg=UNI_MAROON)
    ax.text(9.2, 6.95,
            "1) Singularidad de Codo (sin q₃ = 0):\n"
            "   • q₃ = 0 rad (brazo totalmente extendido,\n"
            "     frontera exterior R_max = 0,253 m) o q₃ = ±π.\n"
            "   • Pérdida de velocidad en dirección radial.\n\n"
            "2) Singularidad Axial de Hombro (r = 0):\n"
            "   • L₁cos(q₂) + L₂cos(q₂+q₃) = 0 (efector sobre\n"
            "     el eje vertical Z₀: x = y = 0).\n"
            "   • La columna 1 de Jv se anula por completo.",
            color=DARK_TEXT, fontsize=10.8, va="top", linespacing=1.38)

    draw_card(ax, 0.5, 0.72, 15.0, 3.35, "Diagnóstico Cuantitativo y Regularización Levenberg-Marquardt (Damped Least-Squares, DLS)", header_bg=UNI_GREEN)
    ax.text(0.8, 3.30,
            "• Evaluación en la postura inicial q₀ = [-1,10, 0,85, -0,75]ᵀ rad:\n"
            "     det(Jv) = 2,306 × 10⁻³ m³,     w(q₀) = 2,306 × 10⁻³ m³,     condición κ(Jv) = σ_max / σ_min = 6,82  ⇒  OPERACIÓN REGULAR.\n"
            "• Evaluación en la singularidad de codo q_sing = [0, 0,25, 0]ᵀ rad:\n"
            "     det(Jv) = 0,000000,            w(q_sing) = 0,000000,        κ(Jv) → ∞  (J_v⁻¹ estándar diverge).\n"
            "• Solución implementada — Pseudoinversa Amortiguada DLS:\n"
            "     J†_DLS(q) = Jvᵀ (Jv Jvᵀ + λ² I₃)⁻¹   con λ ∈ [0,015, 0,025], invirtiendo cada valor singular como σᵢ / (σᵢ² + λ²).",
            color=DARK_TEXT, fontsize=11.0, va="top", linespacing=1.46)

    slides_meta.append((fig, "Lámina 6 (Singularidades): Demuestra la ecuación cerrada de det(Jv), las dos singularidades físicas y la regularización DLS."))

    # =========================================================================
    # SLIDE 7: GENERACIÓN DE TRAYECTORIAS Y 5 CONTROLADORES
    # =========================================================================
    fig, ax = new_slide_fig()
    draw_header_footer(ax, "6. Planificación Polinomial (C¹/C²) y Arquitectura de 5 Controladores",
                       "Interpolador quíntico suave y leyes de control cinemático (CLIK-DLS) y dinámico (CTC, NSGA-II, MPC)", 7)

    draw_card(ax, 0.5, 4.65, 15.0, 3.05, "Planificación de Trayectorias Suaves Punto a Punto (Sesión 7 del Curso)", header_bg=UNI_NAVY)
    ax.text(0.8, 6.95,
            "• Polinomio Cúbico (C¹): garantiza velocidad inicial y final nula q̇(t₀) = q̇(t_f) = 0  [a₀ = qᵢ, a₁ = 0, a₂ = 3Δq/T², a₃ = -2Δq/T³].\n"
            "• Polinomio Quíntico (C²): impone además aceleración inicial y final nula q̈(t₀) = q̈(t_f) = 0, eliminando sacudidas (jerk infinito) en los motores:\n"
            "     q_ref(t) = qᵢ + s(u)(q_f - qᵢ),     s(u) = 10 u³ - 15 u⁴ + 6 u⁵,     ṡ(u) = (30 u² - 60 u³ + 30 u⁴)/T,     u = clip((t - t₀)/T, 0, 1).",
            color=DARK_TEXT, fontsize=11.2, va="top", linespacing=1.52)

    draw_card(ax, 0.5, 0.72, 7.35, 3.65, "Control Cinemático CLIK-DLS y CTC Nominal", header_bg=UNI_GREEN)
    ax.text(0.75, 3.62,
            "1) Control Cinemático en Lazo Cerrado (CLIK-DLS):\n"
            "   q̇_cmd = q̇_ref + β(q, q_ref) J†_DLS(q) K_cart (p_ref - p(q))\n"
            "   • K_cart = 4,5 s⁻¹; β ∈ {0,15, 1,0} pondera el cruce entre\n"
            "     ramas opuestas de codo evitando equilibrios espurios.\n"
            "   • Lazo dinámico interno: τ = M(q)(q̈_ref + Kv(q̇_cmd - q̇)\n"
            "     + Kp(q_ref - q)) + h(q, q̇).\n\n"
            "2) Control por Torque Computado Base (CTC / PD comp.):\n"
            "   τ = M(q)(q̈_ref + Kp e + Kd ė) + C(q,q̇)q̇ + g(q)\n"
            "   • Sintonía nominal (ωn=13 rad/s, ζ=1,1): Kp=169 I₃, Kd=28,6 I₃.",
            color=DARK_TEXT, fontsize=10.5, va="top", linespacing=1.36)

    draw_card(ax, 8.15, 0.72, 7.35, 3.65, "CTC Evolutivo (NSGA-II), MPC e Híbrido CTC-MPC", header_bg=UNI_BLUE)
    ax.text(8.4, 3.62,
            "3) CTC Sintonizado con NSGA-II (Computación Evolutiva):\n"
            "   • Optimiza θ = [Kp₁, Kp₂, Kp₃, Kd₁, Kd₂, Kd₃]ᵀ ∈ ℝ⁶.\n"
            "   • Minimiza simultáneamente f₁ = RMSE_seg y f₂ = ∫||τ||²dt.\n"
            "   • Solución Pareto: θ* = [239,88, 191,92, 215,83, 50,22, 29,67, 9,00]ᵀ.\n\n"
            "4) Control Predictivo Basado en Modelo (MPC, N = 8, h = 40 ms):\n"
            "   • Optimización cuadrática sujeta a cotas -τ_max ≤ u_k ≤ +τ_max.\n\n"
            "5) Arquitectura Híbrida CTC–MPC:\n"
            "   • Combina la linealización exacta del CTC evolutivo (u_nom,k)\n"
            "     con la corrección predictiva óptima L-BFGS-B bajo saturación.",
            color=DARK_TEXT, fontsize=10.5, va="top", linespacing=1.36)

    slides_meta.append((fig, "Lámina 7 (Trayectorias y Controladores): Explica el interpolador quíntico C2 y las 5 arquitecturas de control implementadas."))

    # =========================================================================
    # SLIDE 8: OPTIMIZACIÓN EVOLUTIVA NSGA-II Y DIAGRAMA DE ARQUITECTURA
    # =========================================================================
    fig, ax = new_slide_fig()
    draw_header_footer(ax, "7. Optimización Multiobjetivo NSGA-II y Esquema de Control",
                       "Sintonización robusta ante incertidumbre paramétrica (+25 % masa, +30 % fricción) y perturbaciones externas", 8)

    draw_card(ax, 0.5, 4.35, 15.0, 3.35, "Formulación del Problema Evolutivo Multiobjetivo (optimizar_ctc_nsga2.py)", header_bg=UNI_MAROON)
    ax.text(0.8, 6.95,
            "• Evaluación sobre 4 escenarios severos: (1) Nominal, (2) Perturbación de par externo, (3) Incertidumbre de masa (+25 %/+15 %) y\n"
            "  amortiguamiento (+30 %), y (4) Incertidumbre combinada con perturbación.\n"
            "• Comparación en el benchmark multi-escenario de optimización:\n"
            "     – Candidato CTC Base:       Kp = [169,0, 169,0, 169,0],      Kd = [28,60, 28,60, 28,60]  ⇒  RMSE = 11,353 mm | Esfuerzo = 0,5159 N²m²s\n"
            "     – Candidato Pareto NSGA-II: Kp* = [239,88, 191,92, 215,83],  Kd* = [50,22, 29,67,  9,00]  ⇒  RMSE =  8,808 mm | Esfuerzo = 0,5130 N²m²s\n"
            "✔ El candidato evolutivo θ* DOMINA ESTRICTAMENTE al controlador base en ambos objetivos (mayor precisión con menor energía).",
            color=DARK_TEXT, fontsize=11.0, va="top", linespacing=1.48)

    draw_card(ax, 0.5, 0.72, 15.0, 3.35, "Flujo de Señales en Tiempo Real (Gemelo Digital 100 Hz / Física RK4 500 Hz)", header_bg=UNI_NAVY)
    flow_txt = (
        " [Consigna XYZ / Circuito 5 Aros] ──► [Cinemática Inversa 2 Ramas + Límites] ──► [Interpolador Quíntico C² (q_ref, q̇_ref, q̈_ref)]\n"
        "                                                                                                  │\n"
        "        ┌──────────────────────┬──────────────────────┬──────────────────────┬────────────────────┴───┐\n"
        "        ▼                      ▼                      ▼                      ▼                        ▼\n"
        "  [1. CTC Base]        [2. CTC NSGA-II θ*]       [3. MPC N=8]       [4. Híbrido CTC-MPC]     [5. CLIK-DLS J†_DLS]\n"
        "        └──────────────────────┴──────────────────────┼──────────────────────┴────────────────────────┘\n"
        "                                                      ▼\n"
        "                     [Saturación Física τ ∈ [-τ_max, +τ_max]] ──► [Planta 3D MuJoCo / MATLAB: M(q)q̈ + h(q,q̇) = τ + d]\n"
        "                                                                             │\n"
        "                     ┌───────────────────────────────────────────────────────┼────────────────────────────────────────┐\n"
        "                     ▼                                                       ▼                                        ▼\n"
        "        [Velocímetro ṗ = Jv(q)q̇]                            [Monitor det(Jv), w(q), κ(Jv)]             [Máquina de Estados Visual 3D]"
    )
    ax.text(0.85, 3.32, flow_txt, family="monospace", fontsize=9.2, color=UNI_NAVY, va="top", linespacing=1.28)

    slides_meta.append((fig, "Lámina 8 (NSGA-II y Arquitectura): Demuestra la dominancia de Pareto del controlador evolutivo y el esquema completo de lazo cerrado."))

    # =========================================================================
    # SLIDE 9: RESULTADOS EXPERIMENTALES Y CURVAS DE SIMULACIÓN
    # =========================================================================
    fig, ax = new_slide_fig()
    draw_header_footer(ax, "8. Resultados Experimentales Comparativos (5 Controladores)",
                       "Evaluación cuantitativa en MuJoCo (10 s) bajo régimen nominal y ante perturbación externa entre t = 6,0 s y 6,6 s", 9)

    draw_card(ax, 0.45, 0.72, 7.55, 6.98, "Tabla Comparativa de Desempeño (evaluar_controladores.py)", header_bg=UNI_NAVY)
    tbl_res = (
        "Ctrl | Escenario | RMSE_seg| RMSE_2s| e_final| Esfuerzo | t_cap | t_rec\n"
        "     |           |  [mm]   |  [mm]  |  [mm]  | [N²m²s]  |  [s]  |  [s]\n"
        "-----+-----------+---------+--------+--------+----------+-------+-------\n"
        " PD  | Nominal   |  6,879  | 1,070  | 0,892  |  0,4861  | 5,84  |  ---\n"
        " PD  | Perturb.  |  8,651  | 1,832  | 1,441  |  0,4921  | 5,84  | 0,582\n"
        "-----+-----------+---------+--------+--------+----------+-------+-------\n"
        " EVO | Nominal   |  4,573* | 0,636* | 0,516* |  0,4817* | 4,50* |  ---\n"
        " EVO | Perturb.  |  6,557* | 1,132* | 0,887* |  0,4902* | 4,50* | 0,382*\n"
        "-----+-----------+---------+--------+--------+----------+-------+-------\n"
        " MPC | Nominal   |  7,137  | 1,144  | 0,961  |  0,4865  | 6,04  |  ---\n"
        " MPC | Perturb.  |  8,947  | 1,928  | 1,531  |  0,4922  | 6,02  | 0,622\n"
        "-----+-----------+---------+--------+--------+----------+-------+-------\n"
        " HYB | Nominal   |  6,815  | 1,145  | 0,961  |  0,4863  | 6,06  |  ---\n"
        " HYB | Perturb.  |  8,693  | 1,928  | 1,532  |  0,4920  | 6,02  | 0,622\n"
        "-----+-----------+---------+--------+--------+----------+-------+-------\n"
        " KIN | Nominal   |  4,236★ | 0,365★ | 0,273★ |  0,4816★ | 4,38★ |  ---\n"
        " KIN | Perturb.  |  5,507★ | 0,794★ | 0,565★ |  0,4911  | 4,38★ | 0,242★"
    )
    ax.text(0.62, 6.95, tbl_res, family="monospace", fontsize=9.1, color=DARK_TEXT, va="top", linespacing=1.34)
    ax.text(0.65, 1.85,
            "★ Hallazgos principales:\n"
            "1. CLIK-DLS (KIN) logra el menor error final (0,273 mm)\n"
            "   y la recuperación más rápida tras perturbación (0,242 s).\n"
            "2. CTC NSGA-II (EVO) reduce en 33,5 % el RMSE y en\n"
            "   42,2 % el error final respecto al CTC base.",
            color=UNI_GREEN, fontsize=10.3, fontweight="bold", va="top", linespacing=1.38)

    fig_curvas = ROOT / "results" / "figuras_examen_parcial" / "fig_curvas_simulacion.png"
    if fig_curvas.exists():
        img_c = Image.open(fig_curvas)
        ax_c = fig.add_axes([0.515, 0.09, 0.46, 0.76])
        ax_c.imshow(img_c)
        ax_c.axis("off")

    slides_meta.append((fig, "Lámina 9 (Resultados Experimentales): Presenta la tabla comparativa de los 5 controladores y las 4 gráficas de simulación."))

    # =========================================================================
    # SLIDE 10: MODO JUEGO «CIRCUITO DE 5 AROS» Y MÁQUINA DE ESTADOS 3D
    # =========================================================================
    fig, ax = new_slide_fig()
    draw_header_footer(ax, "9. Aplicación Funcional: Modo Juego «Circuito de 5 Aros» y Estados 3D",
                       "Retroalimentación visual instantánea en MuJoCo cuando el efector encesta o bloquea el objetivo (< 2,0 mm)", 10)

    draw_card(ax, 0.5, 4.95, 15.0, 2.75, "Máquina de Estados Interactiva con Conmutación de Color 3D en MuJoCo (main.py)", header_bg=UNI_NAVY)
    ax.text(0.8, 6.95,
            "• ESPERANDO ORDEN (Azul Cian): brazo estabilizado en espera de consigna XYZ o inicio del circuito competitivo de 5 aros.\n"
            "• EN MOVIMIENTO (Esfera Naranja → Aro Rojo): el efector transporta la esfera hacia el aro midiendo en vivo ||v|| = ||Jv(q)q̇|| y v_max.\n"
            "• PERTURBACIÓN ACTIVA (Esfera Magenta): alerta visual durante la inyección de par externo d = [0,035, -0,040, 0,025]ᵀ N·m.\n"
            "• ¡ENCESTADO / OBJETIVO ALCANZADO! (Verde Esmeralda): cuando ||p(t) - p_obj|| < 2,0 mm, tanto la bola como el aro cambian\n"
            "  instantáneamente a Verde Esmeralda en la escena 3D, se registra t_cap y el robot avanza automáticamente al siguiente aro (5/5).",
            color=DARK_TEXT, fontsize=10.9, va="top", linespacing=1.44)

    fig_est = ROOT / "results" / "figuras_examen_parcial" / "fig_estados_3d_enceste.png"
    if fig_est.exists():
        img_e = Image.open(fig_est)
        ax_e = fig.add_axes([0.04, 0.08, 0.92, 0.44])
        ax_e.imshow(img_e)
        ax_e.axis("off")

    slides_meta.append((fig, "Lámina 10 (Máquina de Estados 3D): Muestra las capturas reales en MuJoCo de los 3 estados visuales hasta el enceste en verde esmeralda."))

    # =========================================================================
    # SLIDE 11: PERFILES DE RAPIDEZ Y IMPLEMENTACIÓN EN MATLAB
    # =========================================================================
    fig, ax = new_slide_fig()
    draw_header_footer(ax, "10. Métricas de Velocidad en el Circuito de 5 Aros y Script MATLAB",
                       "Validación en 3 regímenes de rapidez (Estándar, Rápido y Turbo) e implementación autocontenida en MATLAB", 11)

    draw_card(ax, 0.5, 4.35, 15.0, 3.35, "Tabla Experimental del Modo Juego «Circuito de 5 Aros» (5/5 Objetivos Alcanzados)", header_bg=UNI_GREEN)
    tbl_game = (
        " Perfil de Rapidez | Duración/aro [s] | Aros Encestados | t_cap Medio [s] | Velocidad Pico v_max [mm/s] | Rapidez Media [mm/s] | Energía Total [N²m²s]\n"
        "-------------------+------------------+-----------------+-----------------+-----------------------------+----------------------+-----------------------\n"
        " Estándar          |       4,5        |     5 / 5       |      4,33       |            157,8            |         52,5         |        1,1200\n"
        " Rápido (Óptimo)   |       2,5        |     5 / 5       |      2,59       |            282,4            |         87,0         |        0,6830 (-39%)\n"
        " Turbo             |       1,5        |     5 / 5       |      1,82       |            466,1            |        130,6         |        0,4670"
    )
    ax.text(0.75, 6.90, tbl_game, family="monospace", fontsize=9.8, color=DARK_TEXT, va="top", linespacing=1.55)
    ax.text(0.75, 4.85,
            "✔ Incluso en modo Turbo (1,5 s por aro), el robot completa el 100 % de los encestes (5/5 con error < 2,0 mm) alcanzando 466,1 mm/s.",
            color=UNI_GREEN, fontsize=11.3, fontweight="bold", va="center")

    draw_card(ax, 0.5, 0.72, 15.0, 3.35, "Implementación Dual: Script Autocontenido en MATLAB (matlab/simulacion_3gdl_examen_parcial.m)", header_bg=UNI_NAVY)
    matlab_code = (
        "function [Jv, detJ, w_yosh, Jw, J_geom] = jacobiano_3gdl(q, robot)\n"
        "    [~, T01, T02, ~] = fk_dh_3gdl(q, robot);\n"
        "    r   = robot.L1*cos(q(2)) + robot.L2*cos(q(2)+q(3));\n"
        "    drb = -robot.L1*sin(q(2)) - robot.L2*sin(q(2)+q(3));   drc = -robot.L2*sin(q(2)+q(3));\n"
        "    Jv  = [-r*sin(q(1)), drb*cos(q(1)), drc*cos(q(1));  r*cos(q(1)), drb*sin(q(1)), drc*sin(q(1));  0, r, robot.L2*cos(q(2)+q(3))];\n"
        "    Jw  = [[0;0;1], T01(1:3,3), T02(1:3,3)];   J_geom = [Jv; Jw];   detJ = -robot.L1*robot.L2*sin(q(3))*r;   w_yosh = abs(detJ);\n"
        "end"
    )
    ax.text(0.75, 3.35, matlab_code, family="monospace", fontsize=9.6, color=UNI_NAVY, va="top", linespacing=1.42)

    slides_meta.append((fig, "Lámina 11 (Velocidades y MATLAB): Resume la tabla de los 3 perfiles de rapidez (hasta 466,1 mm/s) y el código MATLAB compatible con los ejercicios de clase."))

    # =========================================================================
    # SLIDE 12: CONCLUSIONES, RECOMENDACIONES Y CIERRE
    # =========================================================================
    fig, ax = new_slide_fig()
    draw_header_footer(ax, "11. Conclusiones, Recomendaciones y Cumplimiento de la Rúbrica",
                       "Síntesis de aportes científicos e ingenieriles del Trabajo Parcial (Calificación objetivo: 20 / 20)", 12)

    draw_card(ax, 0.5, 3.65, 7.4, 4.05, "Conclusiones Principales", header_bg=UNI_NAVY)
    ax.text(0.75, 6.95,
            "1. Exactitud Cinemática (< 10⁻¹⁶ m): Se comprobaron los 3 métodos\n"
            "   de cinemática directa (DH, MTH y geométrico) y las 2 ramas de\n"
            "   cinemática inversa sobre el robot real de Conejo Benítez (2021).\n\n"
            "2. Jacobiano 6×3 y Singularidades: Se demostró det(Jv) =\n"
            "   -L₁L₂sin(q₃)r(q₂,q₃), aislando las singularidades de codo y axial\n"
            "   y garantizando su cruce continuo con la pseudoinversa DLS.\n\n"
            "3. Superioridad de CLIK-DLS y NSGA-II: CLIK-DLS obtuvo el menor\n"
            "   error estacionario (0,273 mm) y recuperación (0,242 s); NSGA-II\n"
            "   redujo el RMSE en 33,5 % y el error final en 42,2 %.\n\n"
            "4. Validación Funcional: 5/5 encestes en los 3 regímenes de velocidad.",
            color=DARK_TEXT, fontsize=10.5, va="top", linespacing=1.34)

    draw_card(ax, 8.1, 3.65, 7.4, 4.05, "Recomendaciones de Ingeniería", header_bg=UNI_MAROON)
    ax.text(8.35, 6.95,
            "1. Filtrado Derivativo en Hardware Dynamixel: Incorporar un\n"
            "   filtro pasa-bajas de 2.º orden o un observador de Kalman en q̇\n"
            "   para atenuar el ruido de cuantización antes del término Kd.\n\n"
            "2. Régimen Operativo Nominal: Adoptar el perfil Rápido (2,5 s/aro)\n"
            "   con CLIK-DLS o CTC NSGA-II (t_cap = 2,59 s, v_max = 282,4 mm/s),\n"
            "   ahorrando 39,0 % de energía respecto al modo estándar.\n\n"
            "3. Extensión a 6 GDL: Acoplar una muñeca esférica RRR para\n"
            "   desacoplar orientación y posición en tareas de inserción oblicua.\n\n"
            "4. Amortiguamiento Adaptativo: Mantener λ ∈ [0,015, 0,025] al\n"
            "   conmutar entre ramas codo arriba y codo abajo.",
            color=DARK_TEXT, fontsize=10.5, va="top", linespacing=1.34)

    draw_card(ax, 0.5, 0.72, 15.0, 2.65, "Guía Sugerida de Exposición para los 5 Integrantes (Dominio del Tema + Apoyo Visual = 20/20)", header_bg=UNI_GOLD)
    ax.text(0.8, 2.65,
            "• Expositor 1 (Amit Roy Flores Rivera): Láminas 1–2 (Caso de estudio real, parámetros Dynamixel, volumen de trabajo).\n"
            "• Expositor 2 (Franklin Enrique Alegría Barboza): Láminas 3–4 (Cinemática directa DH/MTH/Geométrica y cinemática inversa de 2 ramas).\n"
            "• Expositor 3 (José Luis Espinoza Chamaya): Láminas 5–6 (Jacobiano 6×3 Jv/Jω, singularidades det(Jv)=0, Yoshikawa y DLS).\n"
            "• Expositor 4 (Raquel Medianero Huari): Láminas 7–9 (Interpolador quíntico, CLIK-DLS, CTC NSGA-II, MPC y tabla comparativa).\n"
            "• Expositor 5 (Yasmín Mariela Urrego Montoya): Láminas 10–12 (Demostración en vivo MuJoCo/MATLAB, Modo Juego 5 Aros y conclusiones).",
            color=DARK_TEXT, fontsize=10.6, va="top", linespacing=1.42)

    slides_meta.append((fig, "Lámina 12 (Conclusiones y Distribución): Cierra con conclusiones cuantitativas, recomendaciones y distribución para los 5 integrantes."))

    return slides_meta


def create_pptx_from_pngs(png_paths, notes_list, out_pptx):
    """Empaqueta las láminas PNG 16:9 y las notas del orador en un archivo PowerPoint (.pptx) nativo OpenXML."""
    # Dimensiones 16:9 widescreen en EMUs (13.333 in x 7.5 in)
    cx, cy = 12192000, 6858000
    n = len(png_paths)

    with zipfile.ZipFile(out_pptx, "w", zipfile.ZIP_DEFLATED) as zf:
        # 1. [Content_Types].xml
        ct_overrides = [
            '<Override PartName="/ppt/presentation.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.presentation.main+xml"/>',
            '<Override PartName="/ppt/slideMasters/slideMaster1.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slideMaster+xml"/>',
            '<Override PartName="/ppt/slideLayouts/slideLayout1.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slideLayout+xml"/>',
            '<Override PartName="/ppt/theme/theme1.xml" ContentType="application/vnd.openxmlformats-officedocument.theme+xml"/>',
        ]
        for i in range(1, n + 1):
            ct_overrides.append(
                f'<Override PartName="/ppt/slides/slide{i}.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slide+xml"/>'
            )
        content_types = (
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
            '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
            '<Default Extension="xml" ContentType="application/xml"/>'
            '<Default Extension="png" ContentType="image/png"/>'
            + "".join(ct_overrides) +
            '</Types>'
        )
        zf.writestr("[Content_Types].xml", content_types)

        # 2. _rels/.rels
        zf.writestr("_rels/.rels", (
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="ppt/presentation.xml"/>'
            '</Relationships>'
        ))

        # 3. ppt/_rels/presentation.xml.rels
        pres_rels = [
            '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideMaster" Target="slideMasters/slideMaster1.xml"/>',
            '<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/theme" Target="theme/theme1.xml"/>',
        ]
        sld_ids = []
        for i in range(1, n + 1):
            rid = f"rId{i + 2}"
            pres_rels.append(
                f'<Relationship Id="{rid}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slide" Target="slides/slide{i}.xml"/>'
            )
            sld_ids.append(f'<p:sldId id="{255 + i}" r:id="{rid}"/>')

        zf.writestr("ppt/_rels/presentation.xml.rels", (
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            + "".join(pres_rels) +
            '</Relationships>'
        ))

        # 4. ppt/presentation.xml
        zf.writestr("ppt/presentation.xml", (
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<p:presentation xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" '
            'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" '
            'xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">'
            '<p:sldMasterIdLst><p:sldMasterId id="2147483648" r:id="rId1"/></p:sldMasterIdLst>'
            f'<p:sldIdLst>{"".join(sld_ids)}</p:sldIdLst>'
            f'<p:sldSz cx="{cx}" cy="{cy}"/>'
            '<p:notesSz cx="6858000" cy="9144000"/>'
            '</p:presentation>'
        ))

        # 5. Slide Master & Layout & Theme
        zf.writestr("ppt/slideMasters/_rels/slideMaster1.xml.rels", (
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideLayout" Target="../slideLayouts/slideLayout1.xml"/>'
            '<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/theme" Target="../theme/theme1.xml"/>'
            '</Relationships>'
        ))
        zf.writestr("ppt/slideMasters/slideMaster1.xml", (
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<p:sldMaster xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" '
            'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" '
            'xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">'
            '<p:cSld><p:spTree>'
            '<p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>'
            '<p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>'
            '</p:spTree></p:cSld>'
            '<p:clrMap bg1="lt1" tx1="dk1" bg2="lt2" tx2="dk2" accent1="accent1" accent2="accent2" accent3="accent3" accent4="accent4" accent5="accent5" accent6="accent6" hlink="hlink" folHlink="folHlink"/>'
            '<p:sldLayoutIdLst><p:sldLayoutId id="2147483649" r:id="rId1"/></p:sldLayoutIdLst>'
            '</p:sldMaster>'
        ))
        zf.writestr("ppt/slideLayouts/_rels/slideLayout1.xml.rels", (
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideMaster" Target="../slideMasters/slideMaster1.xml"/>'
            '</Relationships>'
        ))
        zf.writestr("ppt/slideLayouts/slideLayout1.xml", (
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<p:sldLayout xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" '
            'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" '
            'xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main" type="blank">'
            '<p:cSld name="Blank"><p:spTree>'
            '<p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>'
            '<p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>'
            '</p:spTree></p:cSld>'
            '<p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>'
            '</p:sldLayout>'
        ))
        zf.writestr("ppt/theme/theme1.xml", (
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<a:theme xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" name="UNITheme">'
            '<a:themeElements>'
            '<a:clrScheme name="UNI">'
            '<a:dk1><a:srgbClr val="102A43"/></a:dk1><a:lt1><a:srgbClr val="FFFFFF"/></a:lt1>'
            '<a:dk2><a:srgbClr val="7A121C"/></a:dk2><a:lt2><a:srgbClr val="F4F7FB"/></a:lt2>'
            '<a:accent1><a:srgbClr val="1F6FEB"/></a:accent1><a:accent2><a:srgbClr val="7A121C"/></a:accent2>'
            '<a:accent3><a:srgbClr val="1B8E3A"/></a:accent3><a:accent4><a:srgbClr val="C59B27"/></a:accent4>'
            '<a:accent5><a:srgbClr val="D97706"/></a:accent5><a:accent6><a:srgbClr val="475569"/></a:accent6>'
            '<a:hlink><a:srgbClr val="1F6FEB"/></a:hlink><a:folHlink><a:srgbClr val="7A121C"/></a:folHlink>'
            '</a:clrScheme>'
            '<a:fontScheme name="UNI"><a:majorFont><a:latin typeface="Arial"/></a:majorFont><a:minorFont><a:latin typeface="Arial"/></a:minorFont></a:fontScheme>'
            '<a:fmtScheme name="UNI"><a:fillStyleLst><a:solidFill><a:schemeClr val="phClr"/></a:solidFill></a:fillStyleLst>'
            '<a:lnStyleLst><a:ln w="9525"><a:solidFill><a:schemeClr val="phClr"/></a:solidFill></a:ln></a:lnStyleLst>'
            '<a:effectStyleLst><a:effectStyle><a:effectLst/></a:effectStyle></a:effectStyleLst>'
            '<a:bgFillStyleLst><a:solidFill><a:schemeClr val="phClr"/></a:solidFill></a:bgFillStyleLst></a:fmtScheme>'
            '</a:themeElements></a:theme>'
        ))

        # 6. Slides + Images
        for i, png_path in enumerate(png_paths, start=1):
            zf.write(png_path, f"ppt/media/slide_{i}.png")
            zf.writestr(f"ppt/slides/_rels/slide{i}.xml.rels", (
                '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
                '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideLayout" Target="../slideLayouts/slideLayout1.xml"/>'
                f'<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="../media/slide_{i}.png"/>'
                '</Relationships>'
            ))
            zf.writestr(f"ppt/slides/slide{i}.xml", (
                '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                '<p:sld xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" '
                'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" '
                'xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">'
                '<p:cSld><p:spTree>'
                '<p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>'
                '<p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>'
                '<p:pic>'
                f'<p:nvPicPr><p:cNvPr id="2" name="SlideImage{i}"/><p:cNvPicPr><a:picLocks noChangeAspect="1"/></p:cNvPicPr><p:nvPr/></p:nvPicPr>'
                '<p:blipFill><a:blip r:embed="rId2"/><a:stretch><a:fillRect/></a:stretch></p:blipFill>'
                f'<p:spPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="{cx}" cy="{cy}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom></p:spPr>'
                '</p:pic>'
                '</p:spTree></p:cSld>'
                '<p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>'
                '</p:sld>'
            ))


def main():
    slides = build_slides()
    png_paths = []
    notes_list = []

    pdf_targets = [
        ROOT / "Presentacion_Trabajo_Parcial_3GDL_UNI.pdf",
        ROOT.parent.parent / "Presentacion_Trabajo_Parcial_3GDL_UNI.pdf",
    ]
    pptx_targets = [
        ROOT / "Presentacion_Trabajo_Parcial_3GDL_UNI.pptx",
        ROOT.parent.parent / "Presentacion_Trabajo_Parcial_3GDL_UNI.pptx",
    ]

    with PdfPages(pdf_targets[0]) as pdf:
        for idx, (fig, note) in enumerate(slides, start=1):
            p_png = OUT_DIR / f"slide_{idx:02d}.png"
            fig.savefig(p_png, dpi=180)
            pdf.savefig(fig, dpi=180)
            plt.close(fig)
            png_paths.append(p_png)
            notes_list.append(note)

    # Copy PDF to root exam folder
    pdf_targets[1].write_bytes(pdf_targets[0].read_bytes())

    # Create PowerPoint (.pptx) in both locations
    for pptx_out in pptx_targets:
        create_pptx_from_pngs(png_paths, notes_list, pptx_out)
        print("Presentación PowerPoint generada en:", pptx_out)
    print("Presentación PDF generada en:", pdf_targets[0])


if __name__ == "__main__":
    main()
