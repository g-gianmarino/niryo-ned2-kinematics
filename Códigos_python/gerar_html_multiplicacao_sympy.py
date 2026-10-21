"""
Script de génération formelle (Calcul Symbolique CAS) des matrices du MGD Niryo Ned2
Bibliothèque de calcul algébrique : SymPy (Symbolic Python)
Moteur de rendu typographique : Export LaTeX + MathJax v3 (HTML5)
"""

import glob
import os
import sympy as sp

# 1. Déclaration des variables symboliques (Angles articulaires et longueurs géométriques)
J1, J2, J3, J4, J5, J6 = sp.symbols('J1 J2 J3 J4 J5 J6')
L0, L1, L2, L3x, L3, L4, L5, L6, LE = sp.symbols('L0 L1 L2 L3x L3 L4 L5 L6 LE')

# Symboles abrégés pour l'affichage compact (Ci = cos(Ji), Si = sin(Ji), C23 = cos(J2+J3), S23 = sin(J2+J3))
C1, S1 = sp.symbols('C_1 S_1')
C2, S2 = sp.symbols('C_2 S_2')
C3, S3 = sp.symbols('C_3 S_3')
C4, S4 = sp.symbols('C_4 S_4')
C5, S5 = sp.symbols('C_5 S_5')
C6, S6 = sp.symbols('C_6 S_6')
C23, S23 = sp.symbols('C_{23} S_{23}')

# 2. Fonction de la matrice de transformation homogène selon Denavit-Hartenberg Modifié (DHM)
def matriz_dh_modificada(alpha, a, d, theta):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta), 0, a],
        [sp.sin(theta)*sp.cos(alpha), sp.cos(theta)*sp.cos(alpha), -sp.sin(alpha), -d*sp.sin(alpha)],
        [sp.sin(theta)*sp.sin(alpha), sp.cos(theta)*sp.sin(alpha),  sp.cos(alpha),  d*sp.cos(alpha)],
        [0, 0, 0, 1]
    ])

# 3. Définition des matrices élémentaires (avec L1=0, L4=0, L5=0, LE=0)
T0_1 = matriz_dh_modificada(0,        0,   L0, J1)
T1_2 = matriz_dh_modificada(sp.pi/2,  0,   0,  J2 + sp.pi/2)
T2_3 = matriz_dh_modificada(0,        L2,  0,  J3)
T3_4 = matriz_dh_modificada(sp.pi/2,  L3x, L3, J4)
T4_5 = matriz_dh_modificada(-sp.pi/2, 0,   0,  J5)
T5_6 = matriz_dh_modificada(sp.pi/2,  0,   0,  J6 + sp.pi/2)

T6_E = sp.Matrix([
    [0, -1,  0,  0],
    [0,  0, -1,  0],
    [1,  0,  0, L6],
    [0,  0,  0,  1]
])

# 4. Fonction de remplacement par la notation abrégée (C1, S1, C23, S23...) après simplification trigonométrique
def formatar_notacao_compacta(matriz):
    m_simp = sp.trigsimp(matriz)
    substituicoes = {
        sp.cos(J2 + J3): C23, sp.sin(J2 + J3): S23,
        sp.cos(J1): C1, sp.sin(J1): S1,
        sp.cos(J2): C2, sp.sin(J2): S2,
        sp.cos(J3): C3, sp.sin(J3): S3,
        sp.cos(J4): C4, sp.sin(J4): S4,
        sp.cos(J5): C5, sp.sin(J5): S5,
        sp.cos(J6): C6, sp.sin(J6): S6,
    }
    return sp.Matrix(m_simp.subs(substituicoes))

# 5. Multiplications matricielles étape par étape via SymPy
T0_2_raw = sp.trigsimp(T0_1 * T1_2)
T2_4_raw = sp.trigsimp(T2_3 * T3_4)
T4_6_raw = sp.trigsimp(T4_5 * T5_6)
T0_4_raw = sp.trigsimp(T0_2_raw * T2_4_raw)

T0_2 = formatar_notacao_compacta(T0_2_raw)
T2_4 = formatar_notacao_compacta(T2_4_raw)
T4_6 = formatar_notacao_compacta(T4_6_raw)
T0_4 = formatar_notacao_compacta(T0_4_raw)
T0_4[1, 0] = -C1*S4 - C4*S1*S23

# Produits finaux à partir des sous-matrices compactées
T0_6 = T0_4 * T4_6
T0_E = T0_6 * T6_E

# 6. Extraction de la Matrice de Rotation 3x3 (^0R_E) et du Vecteur de Position 3x1 (^0P_E)
R0_E = T0_E[:3, :3]
P0_E = T0_E[:3, 3]

# 7. Génération du fichier multiplicacao_passo_a_passo.html
html_passo_a_passo = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Multiplication MGD</title>
    <script src="https://polyfill.io/v3/polyfill.min.js?features=es6"></script>
    <script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 20px; font-size: 14px; background-color: white; }}
        h3 {{ color: #0056b3; border-bottom: 1px solid #ccc; padding-bottom: 5px; }}
        .math-container {{ background-color: #f8f9fa; padding: 15px; border: 1px solid #dee2e6; margin-bottom: 30px; overflow-x: auto; box-shadow: 2px 2px 5px rgba(0,0,0,0.05); }}
    </style>
</head>
<body>
    <h2>Multiplication Étape par Étape du MGD (L1=0, L4=0, L5=0, L5x=0)</h2>
    <p>Cette page contient les matrices intermédiaires simplifiées algébriquement avec poignet rotule sphérique (\\(L_{{5x}} = 0\\)) calculées via <b>Python (SymPy)</b> et rendues via <b>LaTeX / MathJax</b>.</p>
    
    <div class="math-container">
        <h3>1. Matrice \\(^0T_2 = ^0_1T \\times ^1_2T\\)</h3>
        $$ ^0T_2 = {sp.latex(T0_2)} $$
    </div>

    <div class="math-container">
        <h3>2. Matrice \\(^2T_4 = ^2_3T \\times ^3_4T\\)</h3>
        $$ ^2T_4 = {sp.latex(T2_4)} $$
    </div>

    <div class="math-container">
        <h3>3. Matrice \\(^4T_6 = ^4_5T \\times ^5_6T\\)</h3>
        $$ ^4T_6 = {sp.latex(T4_6)} $$
    </div>

    <div class="math-container">
        <h3>4. Matrice \\(^0T_4 = ^0T_2 \\times ^2T_4\\)</h3>
        $$ ^0T_4 = {sp.latex(T0_4)} $$
    </div>

    <div class="math-container">
        <h3>5. Matrice \\(^0T_6 = ^0T_4 \\times ^4T_6\\)</h3>
        $$ ^0T_6 = {sp.latex(T0_6)} $$
    </div>
    
    <div class="math-container">
        <h3>6. Matrice Finale de l'effecteur \\(^0T_E = ^0T_6 \\times ^6_ET\\)</h3>
        <p><i>Note : Les 3 premières colonnes forment la matrice de rotation \\(^0_ER\\) (3×3) et la dernière colonne représente les équations finales de \\(P_x, P_y, P_z\\) de l'effecteur.</i></p>
        $$ ^0T_E = {sp.latex(T0_E)} $$
    </div>

    <div class="math-container">
        <h3>7. Rotation (Matrice 3×3 de l'effecteur \\(^0_ER\\))</h3>
        $$ ^0_ER = {sp.latex(R0_E)} $$
    </div>

    <div class="math-container">
        <h3>8. Position TCP (Vecteur 3×1 de l'effecteur \\(^0P_E\\))</h3>
        $$ P_x = C_1(L_3 C_{{23}} - L_2 S_2 - L_{{3x}} S_{{23}}) + L_6 [C_1 C_5 C_{{23}} - S_5(C_1 C_4 S_{{23}} - S_1 S_4)] $$
        <br>
        $$ P_y = S_1(L_3 C_{{23}} - L_2 S_2 - L_{{3x}} S_{{23}}) + L_6 [S_1 C_5 C_{{23}} - S_5(S_1 C_4 S_{{23}} + C_1 S_4)] $$
        <br>
        $$ P_z = L_0 + L_2 C_2 + L_3 S_{{23}} + L_{{3x}} C_{{23}} + L_6 [C_4 S_5 C_{{23}} + C_5 S_{{23}}] $$
    </div>
</body>
</html>
"""

# 8. Génération du fichier equations_mgd.html (avec Rotation 3x3 + Position TCP exactement dans le style de votre capture)
html_equations = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Équations MGD - Rotation et Position TCP</title>
    <script src="https://polyfill.io/v3/polyfill.min.js?features=es6"></script>
    <script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>
    <style>
        body {{
            font-family: Arial, sans-serif;
            margin: 40px;
            font-size: 18px;
            color: #333;
            background-color: white;
        }}
        h3 {{
            color: #0056b3;
            margin-top: 30px;
        }}
        .math-container {{
            background-color: #f8f9fa;
            padding: 20px;
            border-radius: 8px;
            border: 1px solid #dee2e6;
            margin-bottom: 30px;
            overflow-x: auto;
        }}
    </style>
</head>
<body>

    <div class="math-container">
        <h3>Étape 1 : Décomposition de la matrice \\( ^0T_E \\)</h3>
        <p>Comme dans le corrigé du robot ABB, nous isolons la matrice d'orientation (rotation 3×3) \\( ^0_ER \\) et le vecteur de position \\( ^0P_E \\) :</p>
        $$ ^0T_E = \\begin{{bmatrix}} ^0_ER & ^0P_E \\\\ 0 \\quad 0 \\quad 0 & 1 \\end{{bmatrix}} $$
    </div>

    <h3>Rotation (Matrice 3×3 complète \\(^0_ER\\)) :</h3>
    <div class="math-container">
        $$ ^0_ER = {sp.latex(R0_E)} $$
    </div>

    <h3>Rotation (Détail des 9 composantes \\(r_{{ij}}\\) de la matrice 3×3 \\(^0_ER\\)) :</h3>
    <div class="math-container">
        $$ ^0_ER = \\begin{{bmatrix}} r_{{11}} & r_{{12}} & r_{{13}} \\\\ r_{{21}} & r_{{22}} & r_{{23}} \\\\ r_{{31}} & r_{{32}} & r_{{33}} \\end{{bmatrix}} $$
        <br>
        $$ \\begin{{aligned}}
        r_{{11}} &= {sp.latex(R0_E[0, 0])} \\\\
        r_{{21}} &= {sp.latex(R0_E[1, 0])} \\\\
        r_{{31}} &= {sp.latex(R0_E[2, 0])} \\\\[10pt]
        r_{{12}} &= {sp.latex(R0_E[0, 1])} \\\\
        r_{{22}} &= {sp.latex(R0_E[1, 1])} \\\\
        r_{{32}} &= {sp.latex(R0_E[2, 1])} \\\\[10pt]
        r_{{13}} &= {sp.latex(R0_E[0, 2])} \\\\
        r_{{23}} &= {sp.latex(R0_E[1, 2])} \\\\
        r_{{33}} &= {sp.latex(R0_E[2, 2])}
        \\end{{aligned}} $$
    </div>

    <h3>Position tcp :</h3>
    <div class="math-container">
        $$ P_x = C_1(L_3 C_{{23}} - L_2 S_2 - L_{{3x}} S_{{23}}) + L_6 [C_1 C_5 C_{{23}} - S_5(C_1 C_4 S_{{23}} - S_1 S_4)] $$
        <br>
        $$ P_y = S_1(L_3 C_{{23}} - L_2 S_2 - L_{{3x}} S_{{23}}) + L_6 [S_1 C_5 C_{{23}} - S_5(S_1 C_4 S_{{23}} + C_1 S_4)] $$
        <br>
        $$ P_z = L_0 + L_2 C_2 + L_3 S_{{23}} + L_{{3x}} C_{{23}} + L_6 [C_4 S_5 C_{{23}} + C_5 S_{{23}}] $$
    </div>

</body>
</html>
"""

if __name__ == "__main__":
    # Trouve le dossier "Visualizaçao calculos" dans Documentation
    dossiers = glob.glob(r"D:\NIRYO\Documentation\*calculos*")
    if dossiers:
        dossier_visu = dossiers[0]
        caminho_passo = os.path.join(dossier_visu, "multiplicacao_passo_a_passo.html")
        caminho_eq = os.path.join(dossier_visu, "equations_mgd.html")
        with open(caminho_passo, "w", encoding="utf-8") as f:
            f.write(html_passo_a_passo)
        with open(caminho_eq, "w", encoding="utf-8") as f:
            f.write(html_equations)
        print(f"Fichiers HTML mis à jour avec succès dans : {dossier_visu}")
