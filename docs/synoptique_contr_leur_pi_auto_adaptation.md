# Synoptique du Contrôleur PI, Feed-Forward & Auto-Adaptation

Ce document résume l'architecture complète de votre régulation Ecodan telle qu'elle s'exécute sur l'ESP32 (toutes les **5 minutes** pour la boucle principale, et toutes les **30 secondes** pour les garde-fous court-cycle).

---

## 1. Schéma synoptique global (Chaîne de régulation Zone 1)

```text
                        ┌─────────────────────────────────────────────┐
                        │        ENTRÉES & CAPTEURS (Zone 1)          │
                        │  T_pièce, T_cible, T_ext, T_départ, T_retour│
                        │  État compresseur, Dégivrage, ECS, SmartGrid│
                        └──────────────────────┬──────────────────────┘
                                               │
                                               ▼
                 ┌──────────────────────────────────────────────────────────┐
                 │  ÉTAPE 0 : FILTRES DE SÉCURITÉ & VERROUILLAGE DÉGIVRAGE  │
                 │  • AA actif ? Mode Départ Fixe ? Hors ECS / Lockout ?    │
                 │  • ODIN inactif ? Thermostat virtuel ON ?                │
                 │  • Si dégivrage ou < 15 min après : T_ext & T_ret figés  │
                 └─────────────────────────────┬────────────────────────────┘
                                               │
            ┌──────────────────────────────────┴──────────────────────────────────┐
            ▼                                                                     ▼
┌───────────────────────────────────────────┐       ┌───────────────────────────────────────────┐
│   ÉTAPE 1 : CONTRÔLEUR PI (Terme I)       │       │   ÉTAPE 2 : MODÈLE FEED-FORWARD & INERTIE │
│                                           │       │                                           │
│  e_raw = T_cible - T_pièce                │       │  [Apprentissage FF (si régime stable)]    │
│  Si compresseur ON & |e_raw| <= 0.6 °C :  │       │   Collecte {T_ext, I_k} (max 1500 pts)    │
│   • Anti-windup ΔT (min/max)              │──────►│   RANSAC + Huber (24h ou 1500 pts)        │
│   • Anti-windup T_départ (min/max)        │       │   Fusion EMA adaptative + Variance -> NVS │
│   • I_k = clamp(I_{k-1} + Ki·e_raw, ±3°C) │       │                                           │
│   • Sauvegarde NVS (si Δ > 0.01 °C)       │       │  [Apprentissage Inertie τ]                │
│                                           │       │   Temps de réponse -> Filtrage IQR -> NVS │
│  Si FF actif : décroissance I_k *= 0.9952 │◄──────│                                           │
└─────────────────────┬─────────────────────┘       │  [Prédiction (si FF Control ON)]          │
                      │                             │   B_FF = a · T_ext + b                    │
                      │                             │   C_τ  = correction anticipative (τ)      │
                      │                             └─────────────────────┬─────────────────────┘
                      │                                                   │
                      └─────────────────────────┬─────────────────────────┘
                                                ▼
                 ┌──────────────────────────────────────────────────────────┐
                 │           ÉTAPE 3 : SOMMATION DU BIAIS TOTAL             │
                 │     B_total = B_manuel + I_k - D_k + B_FF + C_τ          │
                 │     T_cible_eff = T_cible + B_total                      │
                 └─────────────────────────────┬────────────────────────────┘
                                               │
                                               ▼
                 ┌──────────────────────────────────────────────────────────┐
                 │     ÉTAPE 4 : BOUCLE AUTO-ADAPTIVE (Calcul du ΔT)        │
                 │  • Facteur de froid quadratique (T_ext de 15°C à -5°C)   │
                 │  • Courbe de réponse (Linéaire ou S-Curve) + Smart Boost │
                 │  • Si consigne atteinte (e < 0) : ΔT = ΔT_base           │
                 │  • Sinon : ΔT_cible entre ΔT_min_dyn et ΔT_max           │
                 │  • Si post-dégivrage (-2 à +3°C) : rampe progressive     │
                 └─────────────────────────────┬────────────────────────────┘
                                               │
                                               ▼
                 ┌──────────────────────────────────────────────────────────┐
                 │     ÉTAPE 5 : CALCUL T_DÉPART & LIMITES PHYSIQUES        │
                 │  • T_départ_calc = T_retour + ΔT_cible                   │
                 │  • Garde court-cycle (buffer)                            │
                 │  • Limite de pas (max -1.0°C sous T_départ mesurée)      │
                 │  • Suivi post-ECS (fenêtre 8 min)                        │
                 │  • Clampage [T_min_zone, T_max_zone]                     │
                 └─────────────────────────────┬────────────────────────────┘
                                               │
                                               ▼
                 ┌──────────────────────────────────────────────────────────┐
                 │           ÉTAPE 6 : ARBITRAGE SMART GRID (SG)            │
                 │  • Si SG_OFF actif -> Blocage envoi consigne à la PAC    │
                 │  • Sinon           -> Envoi T_départ à l'Ecodan          │
                 │  • Mise à jour SG_RECO (surplus) & SG_OFF (surchauffe)   │
                 └──────────────────────────────────────────────────────────┘
```

---

## 2. Détail des blocs fonctionnels

### Bloc 1 : Contrôleur PI (Correction lente & précise)
Son rôle est d'éliminer l'erreur statique entre la température ambiante et la consigne en ajustant un **biais intégral** $I_k$ (sauvegardé en mémoire flash NVS) :

$$e_{\text{raw}} = T_{\text{cible}} - T_{\text{pièce}}$$
$$I_k = \text{clamp}\left(I_{k-1} + K_i \cdot e_{\text{raw}},\ -3.0\ ^\circ\text{C},\ +3.0\ ^\circ\text{C}\right)$$

Pour éviter tout emballement (*windup*), l'intégration est **bloquée** dans 3 cas :
1. **Grand saut de consigne** : $|e_{\text{raw}}| > 0.6\ ^\circ\text{C}$ (la boucle proportionnelle gère seule la relance).
2. **Saturation du $\Delta T$** : le $\Delta T_{\text{cible}}$ prévisionnel atteint déjà $\Delta T_{\text{max}}$ (en hausse) ou $\Delta T_{\text{min, dyn}}$ (en baisse).
3. **Saturation hydraulique** : $T_{\text{départ}}$ atteint la limite haute $T_{\text{max}}$ (ou s'en approche à $1\ ^\circ\text{C}$ selon le sens d'intégration).

---

### Bloc 2 : Auto-Adaptation Feed-Forward (Loi d'eau apprise)
Au lieu d'attendre que la pièce se refroidisse quand $T_{\text{ext}}$ chute, le modèle Feed-Forward apprend la relation linéaire entre la température extérieure et le biais nécessaire :

$$B_{\text{FF}} = a \cdot T_{\text{ext}} + b$$

* **Collecte sélective :** un point $\{T_{\text{ext}}, I_k\}$ n'est enregistré que si le chauffage est actif, que le compresseur tourne depuis plus de `minimum_compressor_on_time`, et que la pièce est stabilisée ($|e_{\text{raw}}| < 0.25\ ^\circ\text{C}$).
* **Régression robuste hybride :** déclenchée automatiquement toutes les **24 h** (dès 50 points), à **1500 points**, ou manuellement via bouton :
  1. **RANSAC** ($\text{seuil} = 0.5$) rejette les points aberrants (ensoleillement, apports internes, ouverture de fenêtres).
  2. **Huber** ($\delta = 0.3$) affine la droite $(a_{\text{new}}, b_{\text{new}})$ sur les points valides (*inliers*).
  3. **Fusion incrémentale** : combine l'ancien modèle et le nouveau via un facteur $\alpha$ adaptatif (décroissant avec le nombre de mises à jour, pondéré par la variance, et boosté $\times 3$ en cas de changement saisonnier/structurel $> 20\%$).
  4. **Fenêtre glissante** : conserve les **500 points** les plus récents pour rester réactif.
* **Transfert PI $\rightarrow$ FF :** lorsque `FF Control` est activé, l'intégrateur $I_k$ décroît doucement à chaque cycle ($\times 0.9952$) afin de laisser le modèle Feed-Forward porter le biais de fond, le PI ne gardant que le réglage fin.

---

### Bloc 3 : Inertie Thermique ($\tau$)
* **Apprentissage :** surveille les changements de consigne départ $> 1\ ^\circ\text{C}$ et mesure le temps de stabilisation de la pièce ($> 300\ \text{s}$ et $|T_{\text{pièce}} - T_{\text{cible}}| < 0.5\ ^\circ\text{C}$).
* **Filtrage :** le calcul élimine les valeurs aberrantes par méthode **IQR** ($[Q_1 - 1.5\cdot\text{IQR},\ Q_3 + 1.5\cdot\text{IQR}]$) puis applique un lissage exponentiel ($85\%$ ancien / $15\%$ nouveau) stocké en NVS.
* **Action :** ajoute une petite correction anticipative au biais FF :
  $$C_{\tau} = 0.3 \cdot \frac{e_{\text{raw}} \cdot (1 - e^{-1})}{\tau_{\text{filt}}}$$

---

### Bloc 4 : Calcul Auto-Adaptive (Conversion en $\Delta T$ et $T_{\text{départ}}$)
1. **Facteur de froid** (quadratique entre $15\ ^\circ\text{C}$ et $-5\ ^\circ\text{C}$) :
   $$c_{\text{raw}} = \frac{15 - \text{clamp}(T_{\text{ext}}, -5, 15)}{20}, \qquad \text{cold\_factor} = 1.5 \cdot c_{\text{raw}}^2$$
2. **$\Delta T_{\text{min}}$ dynamique** : remonte progressivement avec le froid, tout en respectant vos réglages HA `base_min_delta_t` et `max_delta_t`.
3. **Courbe d'erreur** : l'erreur effective $e = (T_{\text{cible}} + B_{\text{total}}) - T_{\text{pièce}}$ est normalisée sur une plage $E_{\text{eff}}$ qui se resserre par grand froid, puis transformée en courbe linéaire ou en S ($3x^2 - 2x^3$) et multipliée par le **Smart Boost** en cas de stagnation.
4. **Température départ** : $T_{\text{départ}} = T_{\text{retour}} + \Delta T_{\text{cible}}$, passée ensuite dans les protections (rampe post-dégivrage, pas max de $1.0\ ^\circ\text{C}$ pour ne pas couper le compresseur, et bornes min/max).

---

### Bloc 5 : Arbitrage Smart Grid
Exécuté en fin de cycle pour gérer le surplus photovoltaïque et éviter la surchauffe :

| Mode | Condition d'activation | Condition de relâchement | Effet sur la PAC |
| :--- | :--- | :--- | :--- |
| **SG Reco (Stockage)** | `sg_energy_available` = ON **et** $T_{\text{pièce}} < T_{\text{cible}} + \text{offset}_{\text{stockage}}$ | Surplus terminé ou $T_{\text{pièce}}$ atteinte | Active `switch_sg_mode_reco` (et force `sg_mode_off` à OFF) |
| **SG OFF (Surchauffe)** | Pas de stockage, PAC active, $T_{\text{pièce}} > T_{\text{cible}} + \text{offset}_{\text{surchauffe}}$, $T_{\text{retour}} \ge 26\ ^\circ\text{C}$ et $\Delta T \approx \Delta T_{\text{base}}$ | $T_{\text{pièce}} \le T_{\text{cible}}$ | Active `switch_sg_mode_off` et **bloque** l'envoi de nouvelles consignes départ |

---

## 3. Boucle parallèle de protection (toutes les 30 s)
Indépendamment du cycle de 5 minutes :
* **Prévention court-cycle prédictive :** surveille en continu l'écart entre $T_{\text{départ}}$ mesurée et la consigne pour empêcher la PAC de couper trop tôt.
* **Gestion du Lockout :** si le compresseur coupe avant `minimum_compressor_on_time`, verrouille temporairement le redémarrage (via *Server Control* ou *Flow Control*) jusqu'à expiration de `lockout_duration`.