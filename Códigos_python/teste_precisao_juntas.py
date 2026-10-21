import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as stats
import time
from pyniryo2 import NiryoRobot

# 1. Connexion au robot
ip_robot = "169.254.200.200"
robot = NiryoRobot(ip_robot)

# 2. Paramètres du test
index_articulation = 0        # 0 pour J1, 1 pour J2, ..., 5 pour J6
deplacement_deg = 10.0        # Amplitude du test en degrés (10°)
repetitions = 30              # 30 échantillons au total (15 positifs, 15 négatifs)
erreurs = []

print(f"Début des {repetitions} tests de précision pour l'axe J{index_articulation + 1}...")

for i in range(repetitions):
    # Alternance entre +10° et -10°
    signe = 1 if (i % 2 == 0) else -1
    angle_commande_deg = signe * deplacement_deg
    angle_commande_rad = np.radians(angle_commande_deg)

    # A. Retour à la position zéro (Home)
    robot.arm.move_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(1)

    # B. Déplacement vers la position cible
    position_commande = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    position_commande[index_articulation] = angle_commande_rad
    robot.arm.move_joints(position_commande)
    time.sleep(1)

    # C. Lecture des encodeurs réels
    position_reelle_rad = robot.arm.get_joints()
    angle_reel_deg = np.degrees(position_reelle_rad[index_articulation])

    # D. Normalisation (inversion du signe pour les courses négatives)
    cible_tab = angle_commande_deg * signe    # Vaut toujours +10.0°
    reel_tab = angle_reel_deg * signe         # Valeur positive en amplitude

    erreur = cible_tab - reel_tab
    erreurs.append(erreur)

    sens_str = "+10°" if signe == 1 else "-10°"
    print(f"Essai {i+1:02d}/{repetitions} [{sens_str}] | "
          f"Mesure: {angle_reel_deg:.4f}° | "
          f"Valeur normalisée: {reel_tab:.4f}° | "
          f"Écart: {erreur:.4f}°")

robot.end()

# ---------------------------------------------------------
# 3. Traitement Statistique
# ---------------------------------------------------------
erreurs = np.array(erreurs)
ecart_moyen = np.mean(erreurs)  # Moyenne de l'erreur
ecart_type = np.std(erreurs)    # Écart-type (Standard Deviation)

print("\n--- RÉSULTATS POUR LE RAPPORT ---")
print(f"Écart Moyen (mu)    : {ecart_moyen:.6f}°")
print(f"Écart-Type (sigma)  : {ecart_type:.6f}°")

# ---------------------------------------------------------
# 4. Tracé de la Distribution Gaussienne (Loi Normale)
# ---------------------------------------------------------
x = np.linspace(min(erreurs) - 3 * ecart_type, max(erreurs) + 3 * ecart_type, 100)
courbe_gauss = stats.norm.pdf(x, ecart_moyen, ecart_type)

plt.figure(figsize=(9, 5.5))

# Courbe théorique
plt.plot(x, courbe_gauss, color='blue', linewidth=2,
         label=f'Loi Normale théorique\n$\\mu$={ecart_moyen:.4f}°, $\\sigma$={ecart_type:.4f}°')

# Histogramme réel
plt.hist(erreurs, bins=8, density=True, alpha=0.5, color='orange', edgecolor='black',
         label='Échantillons expérimentaux')

# Ligne de la moyenne
plt.axvline(ecart_moyen, color='red', linestyle='dashed', linewidth=2,
            label=f'Écart moyen ($\\mu$ = {ecart_moyen:.4f}°)')

# Titres et axes en français
plt.title(f'Précision et Répétabilité du Contrôle Articulaire - Axe J{index_articulation + 1}', fontsize=12, fontweight='bold')
plt.xlabel('Écart angulaire (°)', fontsize=10)
plt.ylabel('Densité de probabilité', fontsize=10)
plt.legend(loc='upper right')
plt.grid(True, linestyle='--', alpha=0.6)

plt.tight_layout()
plt.show()