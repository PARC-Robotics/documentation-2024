# Comment configurer votre espace de travail

Dans ce didacticiel, vous allez configurer un répertoire sur votre PC compatible ROS 2 comme espace de travail pour le développement et installer les packages ROS 2 de la compétition. Veuillez suivre attentivement les instructions ci-dessous.

!!! note
     Ceci peut être effectué UNIQUEMENT après avoir configuré votre PC (en suivant le tutoriel ici : [Configuration de votre PC](../getting-started-tutorials/setting-up-your-pc.fr.md)).

<!-- uncommment once we have docker setup -->
<!-- !!! note -->
<!--      Si vous utilisez un conteneur Docker, vous pouvez ignorer ce didacticiel et suivre les instructions de [Configuration de votre PC à l'aide de Docker](../getting-started-tutorials/setting-up-with-docker.md) à la place. -->

### Étape 1: Configurer l'espace de travail ROS 2

<!-- Premièrement, nous créons un nouveau répertoire dans votre répertoire personnel appelé `catkin_ws` avec un sous-répertoire `src`. Ensuite, nous initialisons le répertoire en tant qu'espace de travail catkin. -->

Ouvrez un nouveau terminal sur votre PC, puis copiez et collez les lignes suivantes une par une:
```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Étape 2: Cloner le dépôt

Dans le même terminal (ou dans un nouveau), copiez et collez ce qui suit:
```sh
cd ~/ros2_ws/src
git clone https://github.com/PARC-Robotics/PARC2024-Engineers-League.git .
```

### Étape 3: Installer les dépendances

Dans le même terminal (ou dans un nouveau), copiez et collez ce qui suit:
```sh
cd ~/ros2_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

### Étape 4: Compiler les packages

L'étape suivante consiste à compiler les packages installés en utilisant `colcon build`:
```sh
cd ~/ros2_ws
colcon build
```

### Étape 5: Configurer l'environnement ROS 2

La commande suivante doit être exécutée dans chaque nouveau terminal que vous ouvrez pour accéder aux commandes ROS 2:

```sh
source /opt/ros/humble/setup.bash
```

Pour éviter de rechercher le fichier d'installation de ROS à chaque fois que vous lancez un nouveau terminal, vous pouvez ajouter la commande à votre script de démarrage shell en exécutant ces lignes:

```sh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

L'espace de travail `ros2_ws` est une **superposition** au-dessus de l'installation de ROS, connue sous le nom de **sous-couche**, et de la même manière, pour utiliser les exécutables ou les bibliothèques du package dans `ros2_ws`, l'espace de travail devra provenir de chaque nouveau terminal ouvert avec cette commande:

```sh
source ~/ros2_ws/install/setup.bash
```

De même, pour éviter de rechercher manuellement l'espace de travail dans chaque terminal nouvellement lancé, la commande peut également être ajoutée au script de démarrage du shell:

```sh
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

!!! note
    Au fur et à mesure que vous développez, il est bon de définir les variables d'environnement chaque fois que vous exécutez une commande `colcon build` pour compiler les modifications apportées à vos packages. Vous pouvez le faire en:
    ```sh
    source ~/ros2_ws/install/setup.bash
    ```

### Étape 6: Installation et configuration de Gazebo Classic

Gazebo Classic, version 11, est le simulateur de robot utilisé lors de la compétition et peut être installé [ici](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install){target=_blank}.

Le champ de tomates est composé de modèles personnalisés, les plants de tomates par exemple, qui ne sont pas disponibles dans Gazebo. Ces modèles devront être copiés dans le répertoire approprié pour visualiser complètement le champ de tomates.

Accédez au dossier models dans le package `parc_robot_bringup`, puis copiez le contenu de son répertoire dans le répertoire Gazebo `models` sur votre PC:

```sh
cd ~/ros2_ws/src/parc_robot_bringup/models
cp -R . ~/.gazebo/models
```

!!! Note 
    Le visualiseur 3D pour ROS, [`RViz`](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html){target=_blank}, est automatiquement installé lorsque ROS 2 Humble a été installé sur votre PC lors du [didacticiel de configuration de votre PC](../getting-started-tutorials/setting-up-your-pc.fr.md).

### Étape 7: testez l'installation

Si vous avez terminé les tâches précédentes avec succès, vous devriez pouvoir exécuter cette commande de lancement de ROS 2 et voir le simulateur Gazebo Classic et le simulateur RViz s'ouvrir avec l'affichage suivant:

```sh
ros2 launch parc_robot_bringup task_1_launch.py
```
![Fenêtre Gazebo Simulator](assets/gazebo.png)
Gazebo Classic fenêtre du simulateur


![Fenêtre RViz](assets/rviz.png)
Fenêtre RViz


Si vous exécutez la commande suivante dans un nouveau terminal,
```
rqt_graph
```
Vous verrez un écran comme celui-ci :

![Graphique RQT](assets/rosgraph.png)

Vous devez publier/écrire dans le `sujet` `/robot_base_controller/cmd_vel_unstamped` pour déplacer le robot.
Le guide suivant vous aidera à contrôler le robot à l'aide du clavier. Une fois que vous avez testé cela, vous pouvez suivre le guide [commencer-avec-ros](../getting-started-tutorials/getting-started-with-ros.fr.md) pour écrire un programme python pour contrôler le robot.

### Étape 8: Contrôler le robot à l'aide du clavier

Tout d'abord, le package `teleop_twist_keyboard` ROS 2 est installé qui nous permettra d'utiliser le clavier pour contrôler le robot dans un terminal comme suit,

```sh
sudo apt install ros-humble-teleop-twist-keyboard
```
Exécutez ensuite la commande suivante dans un nouveau terminal,

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap \
/cmd_vel:=/robot_base_controller/cmd_vel_unstamped
```

Maintenant, en gardant ce deuxième terminal actif (ou en haut), appuyez sur, `i` pour faire avancer le robot, vous pouvez voir le robot bouger dans les fenêtres "RViz" et "Gazebo". 
Vous pouvez utiliser les touches ci-dessous pour déplacer le robot et la touche `k` pour arrêter le mouvement.

```sh
Se déplacer :
   u    i    o
   j    k    l
   m    ,    .
```
