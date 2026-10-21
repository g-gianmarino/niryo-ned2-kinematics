# %% [1] MODÈLE ALGÉBRIQUE 
import sympy as sp
import numpy as np
from math import pi
from IPython.display import display

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

print("Construction des matrices de transformation...")
T0_1 = matriz_dh_modificada(0, 0, L0, J1) 
T1_2 = matriz_dh_modificada(sp.pi/2, 0, L1, J2 + sp.pi/2) 
T2_3 = matriz_dh_modificada(0, L2, 0, J3)
T3_4 = matriz_dh_modificada(sp.pi/2, L3x, L3, J4)
T4_5 = matriz_dh_modificada(-sp.pi/2, 0, L4, J5)
T5_6 = matriz_dh_modificada(sp.pi/2, 0, L5, J6+sp.pi/2) 
T6_Ee = matriz_dh_modificada(0, 0, L6, 0) 
TEe_E = matriz_dh_modificada(sp.pi/2, LE, 0, sp.pi/2)

# 3. Multiplication et simplification (La partie longue qui sera sauvegardée dans la RAM)
print("Traitement de la multiplication algébrique (T0_E)...")
T0_E = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_Ee* TEe_E

print("Simplification des équations de Px, Py, Pz...")
Px = sp.simplify(T0_E[0, 3])
Py = sp.simplify(T0_E[1, 3])
Pz = sp.simplify(T0_E[2, 3])

# Paramètres géométriques fixes du Niryo Ned 2
constantes_robo = {
    L0: 166.3, 
    L1: 0, 
    L2: 221.0, 
    L3x: 32.5, 
    L3: 235.0,
    L4: 0,
    L5: 0,
    L6: 145.0, # Décalage mécanique ajusté ici dans T5_6
    LE: 0
}

# Pré-injecte les longueurs fixes pour accélérer le calcul des angles
Px_geo = Px.subs(constantes_robo)
Py_geo = Py.subs(constantes_robo)
Pz_geo = Pz.subs(constantes_robo)

print("Compilation des équations pour exécution rapide (lambdify)...")
calc_Px = sp.lambdify((J1, J2, J3, J4, J5, J6), Px_geo, "numpy")
calc_Py = sp.lambdify((J1, J2, J3, J4, J5, J6), Py_geo, "numpy")
calc_Pz = sp.lambdify((J1, J2, J3, J4, J5, J6), Pz_geo, "numpy")

print("\nModèle prêt en mémoire !")


# %% [2] VALIDATION PHYSIQUE ET NUMÉRIQUE SUR LE ROBOT
import time
from pyniryo2 import NiryoRobot

# Liste des angles avec des positions variées et extrêmes pour valider le MGD (en degrés)
testes_graus = [
    [0, -77, 53.5, 0, -70, 0],                     # 1. Position Home
]

print("\n==========================================================")
print(" 2. CONNEXION AU ROBOT ET EXÉCUTION DE LA VALIDATION PHYSIQUE")
print("==========================================================")

ip_robot = "169.254.200.200"

try:
    robot = NiryoRobot(ip_robot)
    print(f"SUCCÈS : Connecté au robot sur l'IP {ip_robot}\n")
    
    for i, angulos_graus in enumerate(testes_graus):
        print(f"---[ Point de Test {i+1} ]--------------------------------------")
        print(f"Commande des articulations vers : {angulos_graus}°")
        
        # Convertit des degrés en radians
        angulos_comando_rad = [np.radians(a) for a in angulos_graus]
        
        # 1. Commande physiquement le robot
        robot.arm.move_joints(angulos_comando_rad)
        time.sleep(1.0) # Temps pour garantir la stabilisation
        
        # 2. Lit les angles RÉELS des encodeurs
        juntas_reais = robot.arm.get_joints() 
        j1, j2, j3, j4, j5, j6 = juntas_reais
        
        # 3. Utilise le MGD pour calculer la pose théorique (avec les articulations réelles)
        x_teo_mm = calc_Px(j1, j2, j3, j4, j5, j6)
        y_teo_mm = calc_Py(j1, j2, j3, j4, j5, j6)
        z_teo_mm = calc_Pz(j1, j2, j3, j4, j5, j6)
        
        # 4. Lit la pose cartésienne que le cerveau interne du robot a calculée (via URDF)
        pose_real = robot.arm.get_pose()
        
        # Convertit de mètres en millimètres
        x_real_mm = pose_real.x * 1000
        y_real_mm = pose_real.y * 1000
        z_real_mm = pose_real.z * 1000
        
        # 5. Calcule l'écart / erreur absolue
        erro_x = abs(x_teo_mm - x_real_mm)
        erro_y = abs(y_teo_mm - y_real_mm)
        erro_z = abs(z_teo_mm - z_real_mm)
        erro_global_dist = np.sqrt(erro_x**2 + erro_y**2 + erro_z**2)
        
        print(f" > Calcul Théorique (MGD) : X= {x_teo_mm:7.2f} mm | Y= {y_teo_mm:7.2f} mm | Z= {z_teo_mm:7.2f} mm")
        print(f" > Lecture Réelle (Robot) : X= {x_real_mm:7.2f} mm | Y= {y_real_mm:7.2f} mm | Z= {z_real_mm:7.2f} mm")
        print(f" > Erreur Axe par Axe     : dX={erro_x:7.2f} mm | dY={erro_y:7.2f} mm | dZ={erro_z:7.2f} mm")
        print(f" > Écart Euclidien        : {erro_global_dist:7.2f} mm\n")

   
    
    robot.end()
    print("Tests terminés avec succès.")

except Exception as e:
    print(f"\n[!] Erreur lors de la connexion ou de l'opération du robot : {e}")
    print("Vérifiez si le robot est allumé et si le câble/réseau est ok.")
# %%
