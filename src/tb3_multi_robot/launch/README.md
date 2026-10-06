# Architecture des Launch Files (`tb3_multi_robot/launch`)

Ce dossier regroupe tous les scripts de lancement du système. Il distingue clairement tes **scénarios principaux** (à lancer) de la **machinerie interne Nav2/SLAM** (fichiers importés et modifiés).

---

## 1. Scénarios Métier Principaux (À lancer directement)

Ces 4 scripts situés à la racine de `launch/` constituent **tes points d'entrée applicatifs**. Ce sont eux que tu appelles dans ton terminal via `ros2 launch`.

### A. Simulations Gazebo
* **`amcl_simulation_launch.py`** :
  * Démarre Gazebo avec le monde configuré et fait spawner `robot1` et `robot2`.
  * Charge la carte statique de l'environnement (`map.yaml`).
  * Démarre la localisation **AMCL** et la pile **Nav2** pour `robot1`.
  * Démarre le nœud de suivi (`follower_node`) pour `robot2`.

* **`slam_simulation_launch.py`** :
  * Démarre Gazebo et fait spawner les deux robots.
  * Démarre la cartographie dynamique en direct (**SLAM Toolbox**) sur `robot1` au lieu d'une carte fixe.
  * Lance la navigation Nav2 et la formation multi-robot.

### B. Déploiements sur Robots Physiques
* **`reel_robot_amcl_launch.py`** :
  * Point d'entrée pour les vrais TurtleBot3 physiques.
  * N'instancie ni Gazebo ni les générateurs d'entités (spawners).
  * Configure la localisation AMCL et la navigation pour la flotte réelle.

* **`reel_robot_slam_launch.py`** :
  * Point d'entrée pour les vrais TurtleBot3 physiques en mode cartographie SLAM temps réel.

---

## 2. Machinerie Interne et Adaptateurs (`launch/nav2_bringup/`)

> **Règle :** Tu ne lances jamais ces fichiers directement à la main. Ils sont inclus dynamiquement par tes scénarios principaux.

Ce sous-dossier provient du package officiel ROS 2 `nav2_bringup`, mais **il a été forké et modifié en local par toi** pour résoudre la gestion multi-robot :

* **Pourquoi a-t-il été modifié en local ?**  
  Par défaut, Nav2 est conçu pour un robot unique écoutant sur les topics globaux (`/tf`, `/scan`). Tu as modifié ces scripts pour encapsuler chaque instance dans un **namespace dédié** (`/robot1`, `/robot2`) et isoler les transformations TF (`/tf` -> `tf`).

### Détail des composants internes :
* **`bringup_launch.py`** : Orchestre le démarrage simultané de la navigation (`navigation_launch.py`) et de la localisation/slam.
* **`localization_launch.py`** : Démarre le serveur de carte statique (`map_server`) et le filtre particulaire AMCL.
* **`navigation_launch.py`** : Démarre les contrôleurs de trajectoire, le planificateur global, les costmaps et le `bt_navigator`.
* **`rviz_launch.py`** : Lance l'interface graphique RViz configurée avec les affichages propres au robot ciblé.
* **`slam_launch.py`** : Prépare l'environnement SLAM (gestionnaire de cycle de vie, `map_saver` et remappings de topics `/tf` et `/scan`).
* **`online_sync_launch.py`** : 
  * Fichier rapatrié depuis le paquet officiel `slam_toolbox`.
  * Il contient la configuration de démarrage du nœud de cartographie en mode synchrone.
  * Il est appelé en sous-main par `slam_launch.py`.

---

## Résumé de l'Arborescence

```text
launch/
├── amcl_simulation_launch.py   <-- Scénario Simulation (Carte fixe)
├── slam_simulation_launch.py   <-- Scénario Simulation (Cartographie)
├── reel_robot_amcl_launch.py   <-- Scénario Robot Réel (Carte fixe)
├── reel_robot_slam_launch.py   <-- Scénario Robot Réel (Cartographie)
│
└── nav2_bringup/               <-- Infrastructure interne multi-robot modifiée
    ├── bringup_launch.py
    ├── localization_launch.py
    ├── navigation_launch.py
    ├── rviz_launch.py
    ├── slam_launch.py          (Appelle online_sync_launch.py avec remapping)
    └── online_sync_launch.py   (Moteur SLAM Toolbox rapatrié)
