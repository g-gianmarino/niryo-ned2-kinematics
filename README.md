# 🤖 Niryo Ned 2 — 6-DOF Kinematic Modeling & Hardware Validation (MGD / MGI)

> **🇬🇧 English** | [🇧🇷 Português abaixo](#-modelagem-cinemática-e-validação-experimental--niryo-ned-2-6-gdl)

Complete analytical **Forward Kinematics (FK / MGD)** and closed-form **Inverse Kinematics (IK / MGI)** for the 6-DOF **Niryo Ned 2** collaborative robotic manipulator, developed using the **Modified Denavit-Hartenberg (DHM)** convention and experimentally validated on the physical robot via Python (`pyniryo2`).

---

## ✨ Key Engineering Highlights

* **Symbolic Forward Kinematics (CAS):** Automated derivation and trigonometric simplification of transformation matrices ${}^0T_E = {}^0T_1 \, {}^1T_2 \, {}^2T_3 \, {}^3T_4 \, {}^4T_5 \, {}^5T_6 \, {}^6T_E$ using **SymPy**, compiled to fast numerical routines via `lambdify` and exported to interactive **LaTeX / MathJax HTML** reports.
* **Closed-Form Analytical Inverse Kinematics (Pieper's Decoupling):**
  * Decouples the 6-DOF manipulator at the spherical wrist center ($O_4 = O_5 = O_6$) into a 3-DOF positioning carrier ($J_1, J_2, J_3$) and a 3-DOF spherical wrist ($J_4, J_5, J_6$).
  * Accounts for the mechanical elbow offset ($L_{3x} = 32.5\text{ mm}$, $L_3 = 235.0\text{ mm}$) via effective link geometry $L_{3\text{eff}} = \sqrt{L_3^2 + L_{3x}^2}$.
  * Computes all **8 analytical posture branches** (2 Shoulder $\times$ 2 Elbow $\times$ 2 Wrist Flip/Non-Flip), handles wrist singularities ($J_5 \in \{0, \pi\}$), filters against physical joint limits, and selects the minimum-displacement configuration in joint space.
  * Cross-validated against Forward Kinematics with machine precision ($< 10^{-12}\text{ mm}$ error).
* **Hardware-in-the-Loop Validation & Metrology:**
  * Real-time communication with the physical **Niryo Ned 2** robot over TCP/IP (`pyniryo2`).
  * Euclidean position error benchmarking comparing analytical predictions against onboard encoder/URDF telemetry.
  * Statistical repeatability and precision analysis ($N = 30$ bidirectional cycles) with Gaussian probability density fitting (`scipy.stats`, `matplotlib`).

---

## 📐 Modified Denavit-Hartenberg (DHM) Parameters

Following the Craig / Khalil Modified DH convention:

$$
{}^{i-1}T_i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i & 0 & a_i \\
\sin\theta_i\cos\alpha_i & \cos\theta_i\cos\alpha_i & -\sin\alpha_i & -d_i\sin\alpha_i \\
\sin\theta_i\sin\alpha_i & \cos\theta_i\sin\alpha_i & \cos\alpha_i & d_i\cos\alpha_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

| Link ($i$) | $\alpha_i\text{ (rad)}$ | $a_i\text{ (mm)}$ | $d_i\text{ (mm)}$ | $\theta_i\text{ (rad)}$ | Joint Limits (deg) |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **1** ($0 \to 1$) | $0$ | $0$ | $L_0 = 166.3$ | $J_1$ | $[-168.96^\circ, +168.96^\circ]$ |
| **2** ($1 \to 2$) | $+\pi/2$ | $0$ | $0$ | $J_2 + \pi/2$ | $[-104.85^\circ, +34.95^\circ]$ |
| **3** ($2 \to 3$) | $0$ | $L_2 = 221.0$ | $0$ | $J_3$ | $[-76.78^\circ, +87.00^\circ]$ |
| **4** ($3 \to 4$) | $+\pi/2$ | $L_{3x} = 32.5$ | $L_3 = 235.0$ | $J_4$ | $[-119.70^\circ, +119.70^\circ]$ |
| **5** ($4 \to 5$) | $-\pi/2$ | $0$ | $0$ | $J_5$ | $[-109.95^\circ, +110.12^\circ]$ |
| **6** ($5 \to 6$) | $+\pi/2$ | $0$ | $0$ | $J_6 + \pi/2$ | $[-144.96^\circ, +144.96^\circ]$ |

With tool transformation matrix ${}^6T_E$ incorporating the wrist-to-TCP offset $L_6 = 145.0\text{ mm}$.

---

## 📂 Repository Structure

```text
├── Códigos_python/
│   ├── MGD/                                      # Forward Kinematics (Modèle Géométrique Direct)
│   │   ├── Calculo_6TE_pronta.py                 # Symbolic & numerical FK calculation (SymPy)
│   │   ├── Comparacao_posicoes.py                # Multi-pose FK validation on the physical robot
│   │   └── Posicao_Unica.py                      # Single-pose hardware validation script
│   ├── MGI/                                      # Inverse Kinematics (Modèle Géométrique Inverse)
│   │   ├── MGI_Niryo_Ned2.py                     # Complete analytical 6-DOF IK solver + unit tests
│   │   ├── Calcular_MGI_pelo_XYZ.py              # Interactive Cartesian (X,Y,Z,R,P,Y) -> Joint solver
│   │   └── Teste_MGI_Robo_Fisico.py              # Hardware-in-the-loop IK validation on Niryo Ned 2
│   ├── gerar_html_multiplicacao_sympy.py         # Generates LaTeX/MathJax HTML symbolic derivations
│   └── teste_precisao_juntas.py                  # Statistical repeatability & Gaussian analysis
├── Documentation/
│   └── Visualização calculos/                    # Interactive MathJax HTML symbolic matrix outputs
│       ├── equations_mgd.html                    # Closed-form Rotation (3x3) & TCP Position equations
│       ├── multiplicacao_passo_a_passo.html      # Step-by-step symbolic matrix multiplication
│       └── multiplicacao_T6E_comparacao.html     # End-effector transformation comparison
├── niryo_ned2.urdf.xacro                         # Official URDF/Xacro kinematic model reference
└── requirements.txt                              # Python dependencies
```

---

## 🚀 Getting Started

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```

### 2. Run Analytical IK Validation (Offline Simulation)
Verifies all 8 analytical IK branches and singularity handling against the FK model:
```bash
python "Códigos_python/MGI/MGI_Niryo_Ned2.py"
```

### 3. Compute Joint Angles for Any Target Pose $(X, Y, Z, \text{Roll}, \text{Pitch}, \text{Yaw})$
```bash
python "Códigos_python/MGI/Calcular_MGI_pelo_XYZ.py"
```

### 4. Generate Symbolic LaTeX/MathJax Equations
```bash
python "Códigos_python/gerar_html_multiplicacao_sympy.py"
```

---
---

# 🇧🇷 Modelagem Cinemática e Validação Experimental — Niryo Ned 2 (6 GDL)

Desenvolvimento completo da **Cinemática Direta (MGD)** e **Cinemática Inversa Analítica (MGI)** para o manipulador robótico colaborativo de 6 graus de liberdade **Niryo Ned 2**, utilizando a convenção de **Denavit-Hartenberg Modificada (DHM)** e validação experimental no robô físico em Python (`pyniryo2`).

## ✨ Destaques Técnicos do Projeto

* **Cinemática Direta Simbólica (MGD):** Dedução algébrica automatizada com `SymPy`, simplificação trigonométrica da matriz homogênea ${}^0T_E$, compilação numérica rápida via `lambdify` e exportação automática das equações para **HTML interativo com LaTeX / MathJax**.
* **Cinemática Inversa Analítica Fechada (MGI — Desacoplamento de Pieper):**
  * Desacoplamento cinemático entre o posicionador regional ($J_1, J_2, J_3$) e o punho esférico ($J_4, J_5, J_6$).
  * Tratamento exato do deslocamento geométrico do cotovelo ($L_{3x} = 32,5\text{ mm}$ e $L_3 = 235,0\text{ mm}$).
  * Cálculo das **8 soluções analíticas possíveis** (Ombro Frente/Trás, Cotovelo Cima/Baixo, Punho Flip/Non-Flip), tratamento de singularidade de punho ($J_5 = 0$ ou $\pi$), filtragem pelos limites mecânicos reais das juntas e seleção automática da configuração de menor deslocamento angular.
  * Validação cruzada MGI $\leftrightarrow$ MGD com erro numérico inferior a $10^{-12}\text{ mm}$.
* **Validação no Robô Físico e Metrologia:**
  * Controle e leitura de encoders em tempo real via `pyniryo2` (TCP/IP).
  * Comparação entre pose teórica calculada pelo modelo e telemetria real do robô (erro euclidiano tridimensional).
  * Ensaio estatístico de repetibilidade e precisão articular ($N = 30$ ciclos) com ajuste de distribuição normal gaussiana (`scipy.stats` e `matplotlib`).

---

## 👤 Author / Autor

**Gustavo Gianmarino**
* 🐙 GitHub: [@g-gianmarino](https://github.com/g-gianmarino)
* 📧 Email: [gustavogianm@gmail.com](mailto:gustavogianm@gmail.com)
