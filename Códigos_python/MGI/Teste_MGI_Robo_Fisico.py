"""
Validation Physique du Modèle Géométrique Inverse (MGI) sur le Robot Niryo Ned 2.
Même structure que vos codes de validation du MGD (Posicao_Unica.py / Comparacao_posicoes.py).

Auteur : Équipe Projet Niryo Ned 2
"""

import time
import numpy as np
from pyniryo2 import NiryoRobot
from Calcular_MGI_pelo_XYZ import calcular_mgi_para_xyz

# Adresse IP du Niryo Ned 2
IP_ROBOT = "169.254.200.200"

# ==============================================================================
# LISTE DES COORDONNÉES CARTÉSIENNES À TESTER SUR LE ROBOT PHYSIQUE :
# [X (mm), Y (mm), Z (mm), Roll (deg), Pitch (deg), Yaw (deg)]
# ==============================================================================
points_test_xyz = [
    # 1. Position devant le robot, pince vers le bas (Pick & Place classique)
    [350.0, 0.0, 150.0, 0.0, 90.0, 0.0],
    
    # 2. Position décalée sur la gauche
    [250.0, 150.0, 200.0, 0.0, 45.0, 30.0],
    
    # 3. Position décalée sur la droite
    [250.0, -150.0, 200.0, 0.0, 45.0, -30.0],
]

def executer_validation_mgi_physique():
    print("\n" + "=" * 70)
    print(" 1. CONNEXION AU ROBOT NIRYO NED 2")
    print("=" * 70)
    
    try:
        robot = NiryoRobot(IP_ROBOT)
        print(f"SUCCÈS : Connecté au robot sur l'IP {IP_ROBOT}\n")
        
        # ----------------------------------------------------------------------
        # TEST A : VALIDATION PASSIVE SANS BOUGER (Le robot lit sa pose actuelle)
        # ----------------------------------------------------------------------
        print("--- [ TEST A : VALIDATION PASSIVE DE LA POSE ACTUELLE ] ---")
        pose_actuelle = robot.arm.get_pose()
        juntas_actuelles = robot.arm.get_joints()
        
        x_reel_mm = pose_actuelle.x * 1000.0
        y_reel_mm = pose_actuelle.y * 1000.0
        z_reel_mm = pose_actuelle.z * 1000.0
        roll_reel_deg = np.degrees(pose_actuelle.roll)
        pitch_reel_deg = np.degrees(pose_actuelle.pitch)
        yaw_reel_deg = np.degrees(pose_actuelle.yaw)
        
        print(f"Pose mesurée par le robot :")
        print(f"  X = {x_reel_mm:7.2f} mm | Y = {y_reel_mm:7.2f} mm | Z = {z_reel_mm:7.2f} mm")
        print(f"  Roll = {roll_reel_deg:6.1f}° | Pitch = {pitch_reel_deg:6.1f}° | Yaw = {yaw_reel_deg:6.1f}°")
        
        # Notre MGI calcule les angles à partir de cette pose
        angulos_mgi_rad = calcular_mgi_para_xyz(
            x_reel_mm, y_reel_mm, z_reel_mm,
            roll_reel_deg, pitch_reel_deg, yaw_reel_deg
        )
        
        if angulos_mgi_rad:
            print("\nComparaison avec les encodeurs réels du robot :")
            for i in range(6):
                deg_calcule = np.degrees(angulos_mgi_rad[i])
                deg_reel = np.degrees(juntas_actuelles[i])
                diff = abs(deg_calcule - deg_reel)
                print(f"  J{i+1} : Calculé = {deg_calcule:7.2f}° | Encodeur réel = {deg_reel:7.2f}° | Écart = {diff:6.2f}°")
                
        # ----------------------------------------------------------------------
        # TEST B : COMMANDE PHYSIQUE VERS DES COORDONNÉES CARTÉSIENNES XYZ
        # ----------------------------------------------------------------------
        reponse = input("\nSouhaitez-vous déplacer physiquement le robot vers les cibles XYZ ? (o/n) : ")
        if reponse.strip().lower() in ['o', 'oui', 's', 'sim', 'y', 'yes']:
            for idx, cible in enumerate(points_test_xyz):
                x_mm, y_mm, z_mm, r_deg, p_deg, y_deg = cible
                print(f"\n--- [ Test B.{idx+1} ] Commande vers X={x_mm:.1f}, Y={y_mm:.1f}, Z={z_mm:.1f} ---")
                
                # 1. Calcul des angles par notre MGI
                q_commande_rad = calcular_mgi_para_xyz(x_mm, y_mm, z_mm, r_deg, p_deg, y_deg)
                if not q_commande_rad:
                    print("Cible ignorée (hors de portée).")
                    continue
                    
                # 2. Envoi aux moteurs
                print("Déplacement physique en cours...")
                robot.arm.move_joints(q_commande_rad)
                time.sleep(1.0)
                
                # 3. Vérification de la pose atteinte
                pose_finale = robot.arm.get_pose()
                x_atteint = pose_finale.x * 1000.0
                y_atteint = pose_finale.y * 1000.0
                z_atteint = pose_finale.z * 1000.0
                
                erreur = np.linalg.norm([x_mm - x_atteint, y_mm - y_atteint, z_mm - z_atteint])
                print(f"  Pose atteinte : X = {x_atteint:7.2f} mm | Y = {y_atteint:7.2f} mm | Z = {z_atteint:7.2f} mm")
                print(f"  Écart réel     : {erreur:7.2f} mm")
                
        robot.end()
        print("\nSession avec le robot terminée avec succès.")
        
    except Exception as e:
        print(f"\n[!] Erreur de communication avec le robot : {e}")
        print("Vérifiez l'adresse IP et la connexion réseau.")

if __name__ == '__main__':
    executer_validation_mgi_physique()
