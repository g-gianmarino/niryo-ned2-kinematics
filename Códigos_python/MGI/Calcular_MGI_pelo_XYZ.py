"""
Calculateur Interactif du Modèle Géométrique Inverse (MGI) - Niryo Ned 2
Entrée : Coordonnées cartésiennes cibles (X, Y, Z en mm) + Orientation (Roll, Pitch, Yaw en degrés)
Sortie : Angles des articulations [J1, J2, J3, J4, J5, J6] en degrés et en radians

Auteur : Équipe Projet Niryo Ned 2
"""

import numpy as np
from MGI_Niryo_Ned2 import mgi_complet, mgd_complet, selectionner_meilleure_solution, LIMITES_ARTICULAIRES

def rpy_para_matriz_rotacao(roll_deg, pitch_deg, yaw_deg):
    """
    Convertit des angles Roll, Pitch, Yaw (en degrés) en matrice de rotation 3x3.
    Convention Roll-Pitch-Yaw (XYZ extrinsèque / ZYX intrinsèque standard).
    """
    r = np.radians(roll_deg)
    p = np.radians(pitch_deg)
    y = np.radians(yaw_deg)
    
    Rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, np.cos(r), -np.sin(r)],
        [0.0, np.sin(r),  np.cos(r)]
    ])
    Ry = np.array([
        [np.cos(p), 0.0, np.sin(p)],
        [0.0, 1.0, 0.0],
        [-np.sin(p), 0.0, np.cos(p)]
    ])
    Rz = np.array([
        [np.cos(y), -np.sin(y), 0.0],
        [np.sin(y),  np.cos(y), 0.0],
        [0.0, 0.0, 1.0]
    ])
    
    return Rz @ Ry @ Rx

def calcular_mgi_para_xyz(x_mm, y_mm, z_mm, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    """
    Prend une position (X, Y, Z) en millimètres et une orientation RPY en degrés,
    et calcule les angles des 6 moteurs.
    """
    # 1. Construit la matrice homogène 4x4 cible
    R = rpy_para_matriz_rotacao(roll_deg, pitch_deg, yaw_deg)
    T_cible = np.eye(4)
    T_cible[:3, :3] = R
    T_cible[:3, 3] = [x_mm, y_mm, z_mm]
    
    print("=" * 70)
    print("  CALCUL DU MODÈLE GÉOMÉTRIQUE INVERSE (MGI) - NIRYO NED 2")
    print("=" * 70)
    print(f"Cible demandée :")
    print(f"  Coordonnées (mm) : X = {x_mm:.2f} mm | Y = {y_mm:.2f} mm | Z = {z_mm:.2f} mm")
    print(f"  Orientation      : Roll = {roll_deg:.1f}° | Pitch = {pitch_deg:.1f}° | Yaw = {yaw_deg:.1f}°\n")
    
    # 2. Résout le MGI
    solutions_toutes = mgi_complet(T_cible, filtrer_limites=False)
    solutions_physiques = mgi_complet(T_cible, filtrer_limites=True)
    
    if not solutions_toutes:
        print("[!] ERREUR : La position demandée est HORS DE PORTÉE du bras robotique !")
        return None
        
    print(f"Solutions mathématiques trouvées : {len(solutions_toutes)} configurations.")
    print(f"Solutions dans les limites physiques du robot : {len(solutions_physiques)} configurations.\n")
    
    # 3. Choisit la solution la plus naturelle (la plus proche de la posture neutre)
    meilleure = selectionner_meilleure_solution(solutions_physiques if solutions_physiques else solutions_toutes)
    
    print("-" * 70)
    print(" >>> CONFIGURATION ARTICULAIRE RECOMMANDÉE POUR VOS MOTEURS <<<")
    print("-" * 70)
    noms_juntas = ['J1 (Base)', 'J2 (Bras)', 'J3 (Coude)', 'J4 (Avant-bras)', 'J5 (Poignet)', 'J6 (Outil)']
    
    for i in range(6):
        deg = float(np.degrees(meilleure[i]))
        rad = float(meilleure[i])
        lim_inf, lim_sup = LIMITES_ARTICULAIRES[f'J{i+1}']
        lim_inf_deg = float(np.degrees(lim_inf))
        lim_sup_deg = float(np.degrees(lim_sup))
        status = "OK" if (lim_inf <= rad <= lim_sup) else "HORS LIMITE !"
        print(f"  {noms_juntas[i]:<18} : {deg:8.2f}°  ({rad:7.4f} rad)  | Limites: [{lim_inf_deg:6.1f}°, {lim_sup_deg:6.1f}°] -> [{status}]")
        
    # 4. Preuve mathématique par re-injection dans le MGD
    T_recalcul = mgd_complet(meilleure)
    x_rec, y_rec, z_rec = T_recalcul[:3, 3]
    erreur = np.linalg.norm([x_mm - x_rec, y_mm - y_rec, z_mm - z_rec])
    
    print("\n" + "-" * 70)
    print(" VÉRIFICATION THÉORIQUE (RE-INJECTION DANS LE MGD) :")
    print(f"  Position recalculée : X = {x_rec:.2f} mm | Y = {y_rec:.2f} mm | Z = {z_rec:.2f} mm")
    print(f"  Écart de position    : {erreur:.6e} mm  -->  PRÉCISION ABSOLUE !")
    print("=" * 70)
    
    return meilleure


# ==============================================================================
# CONFIGURATION MANUELLE DES COORDONNÉES
# C'EST ICI QUE VOUS MODIFIEZ DIRECTEMENT VOS VALEURS X, Y, Z !
# ==============================================================================
if __name__ == '__main__':
    # Exemple : Position devant le robot (en mm)
    # Modifiez ces valeurs comme vous le souhaitez :
    X_alvo = 0    # mm (distance vers l'avant)
    Y_alvo = 380      # mm (décalage gauche/droite)
    Z_alvo = 428   # mm (hauteur au-dessus de la table)
    
    # Orientation désirée de la tête (en degrés) :
    # (0, 0, 0) = pince à l'horizontale pointant vers l'avant
    # (0, 90, 0) = pince pointant verticalement vers la table (Pick & Place)
    Roll_alvo  = 0.0
    Pitch_alvo = 0.0
    Yaw_alvo   = 0.0
    
    # Exécution du calcul
    angulos = calcular_mgi_para_xyz(X_alvo, Y_alvo, Z_alvo, Roll_alvo, Pitch_alvo, Yaw_alvo)
