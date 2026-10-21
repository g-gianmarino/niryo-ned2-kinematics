# %% [1] MODÈLE ALGÉBRIQUE (Exécuter une seule fois)
import sympy as sp
from math import pi
from IPython.display import display # Importation ajoutée par sécurité

print("\n--- 1. CALCUL DU MODÈLE GÉOMÉTRIQUE DIRECT ALGÉBRIQUE ---")

# 1. Variables symboliques
J1, J2, J3, J4, J5, J6 = sp.symbols('J1 J2 J3 J4 J5 J6')
L0, L1, L2, L3x, L3, L4, L5, L6, LE = sp.symbols('L0 L1 L2 L3x L3 L4 L5 L6 LE')

# 2. Matrice D-H Modifiée
def matriz_dh_modificada(alpha, a, d, theta):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta), 0, a],
        [sp.sin(theta)*sp.cos(alpha), sp.cos(theta)*sp.cos(alpha), -sp.sin(alpha), -d*sp.sin(alpha)],
        [sp.sin(theta)*sp.sin(alpha), sp.cos(theta)*sp.sin(alpha),  sp.cos(alpha),  d*sp.cos(alpha)],
        [0, 0, 0, 1]
    ])

T0_1 = matriz_dh_modificada(0, 0, L0, J1) 
T1_2 = matriz_dh_modificada(sp.pi/2, 0, L1, J2 + sp.pi/2) 
T2_3 = matriz_dh_modificada(0, L2, 0, J3)
T3_4 = matriz_dh_modificada(sp.pi/2, L3x, L3, J4)
T4_5 = matriz_dh_modificada(-sp.pi/2, 0, L4, J5)
T5_6 = matriz_dh_modificada(sp.pi/2, 0, L5, J6+sp.pi/2)

# Matrice 6_T_E insérée directement à partir de la photo du tableau
T6_E = sp.Matrix([
    [0, -1,  0,  0],
    [0,  0, -1,  0],
    [1,  0,  0, L6],
    [0,  0,  0,  1]
])

# 3. Multiplication et simplification (La partie longue qui sera sauvegardée dans la RAM)
print("Traitement de la multiplication et de la simplification algébrique...")
T0_6 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 
T0_E = T0_6 * T6_E

# Le calcul des axes Px, Py, Pz continue en utilisant la matrice originale (T0_E) 
# pour ne pas casser les calculs numériques plus tard !
Px = sp.simplify(T0_E[0, 3])
Py = sp.simplify(T0_E[1, 3])
Pz = sp.simplify(T0_E[2, 3])


# Paramètres géométriques fixes
constantes_robo = {
    L0: 166.3, 
    L1: 0, 
    L2: 221.0, 
    L3x: 32.5, 
    L3: 235.0,
    L4: 0,
    L5: 0,
    L6: 145.0, 
    LE: 0
}

# Pré-injecte les longueurs fixes pour accélérer le calcul des angles
Px_geo = Px.subs(constantes_robo)
Py_geo = Py.subs(constantes_robo)
Pz_geo = Pz.subs(constantes_robo)

print("\nModèle prêt en mémoire !")


# %% [2] VALIDATION NUMÉRIQUE (Exécutez uniquement cette cellule lors de la modification des angles)
# Modifiez les angles ici :
angulos = {
    J1: 0/180*pi, 
    J2: 0/180*pi, 
    J3: 0/180*pi, 
    J4: 0/180*pi, 
    J5: 0/180*pi, 
    J6: 0/180*pi
}

Px_num = Px_geo.subs(angulos).evalf()
Py_num = Py_geo.subs(angulos).evalf()
Pz_num = Pz_geo.subs(angulos).evalf()

print(f"X = {Px_num:.2f} mm")
print(f"Y = {Py_num:.2f} mm")
print(f"Z = {Pz_num:.2f} mm")
# %%