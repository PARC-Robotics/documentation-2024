# Tâche 2 : Estimation du rendement des cultures

![Task 2 Demo](../assets/task2.gif)

## Description générale
Selon les Nations Unies, la population mondiale devrait atteindre 8,5 milliards d’habitants d’ici 2030, l’Afrique étant le continent à la croissance la plus rapide. Cette croissance démographique projetée rend nécessaire l’urgence d’assurer la sécurité alimentaire. L'estimation précise des rendements des cultures est un aspect essentiel de la quête de la sécurité alimentaire et des robots agricoles autonomes sont utilisés dans cette entreprise.

Pour cette tâche, le robot PARC est conduit de manière autonome à travers deux rangées de plants de tomates et les équipes doivent développer un logiciel permettant d'estimer le rendement du champ de tomates en utilisant les caméras RVB du robot.

## Lignes directrices sur les tâches

### Lancer la tâche

Dans un nouveau terminal, exécutez le fichier de lancement suivant pour faire apparaître le robot dans Gazebo et RViz:

```bash
ros2 launch parc_robot_bringup task2_launch.py
```

Vous devriez voir l'affichage ci-dessous dans Gazebo et RViz respectivement.

![task2_world](../assets/gazebo_on_start.png)
![task2_rviz](../assets/task2rviz.png)

Il existe trois mondes pour cette tâche, chaque monde variant en termes de nombre de plants de tomates produisant des fruits. Le monde par défaut est `world1` et similaire à la tâche 1, les options du deuxième et du troisième monde, `world2` et `world3`, peuvent être sélectionnées en passant l'argument dans la commande de `ros2 launch` ci-dessous:

```bash
# world2
ros2 launch parc_robot_bringup task2_launch.py world:=world2

# world3
ros2 launch parc_robot_bringup task2_launch.py world:=world3
```

Le robot commence à se déplacer une fois que les nœuds appelés par le fichier de lancement ont été chargés avec succès.



