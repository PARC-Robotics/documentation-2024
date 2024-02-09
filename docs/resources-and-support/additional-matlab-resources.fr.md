# Ressources supplémentaires pour MATLAB

## Publication sur le topic `/parc_robot/weed_detection`

Le topic `/parc_robot/weed_detection` attend une chaîne de caractères JSON représentant un tableau de localisations de mauvaises herbes. Le tableau doit être un tableau n x 2 de valeurs X et Y. Voici un exemple de script MATLAB pour publier sur le topic `/parc_robot/weed_detection`:

```matlab
rosshutdown
rosinit             % Créer un nœud ROS

% Créer un éditeur pour le topic /parc_robot/weed_detection
pub = rospublisher('/parc_robot/weed_detection', 'std_msgs/String');

% Définir les localisations des mauvaises herbes comme un tableau nx2 de valeurs X et Y
weed_locations = [
    0.5, 0.5;
    0.5, 0.6;
    0.5, 0.7;
    0.5, 0.8;
    -1.2, 3.25;  % Note: Ce ne sont que des valeurs d'exemple.
    1.1, 4.0;    % Votre code réel devra détecter les mauvaises herbes sur le terrain.
    ];

% Convertir les localisations des mauvaises herbes en une chaîne JSON
json_str = jsonencode(weed_locations);

% Créer un message String
msg = rosmessage('std_msgs/String');

% Définir les données du message comme la chaîne JSON
msg.Data = json_str;

% Publier le message
send(pub, msg);

% Arrêter le nœud ROS
rosshutdown
```

## Souscription au topic `/parc_robot/robot_status`

Le topic `/parc_robot/robot_status` publie l'état actuel du robot. Le type de message pour ce topic est `/std_msgs/String`, ce qui indique si le robot a commencé à se déplacer le long de l'itinéraire ou s'il a terminé l'itinéraire désigné. L'état du robot a deux valeurs possibles : "started" (démarré) et "finished" (terminé).

```matlab
rosshutdown
rosinit                % Créer un nœud ROS

% Créer un souscripteur pour le topic /parc_robot/robot_status. La fonction de rappel est appelée lorsqu'un message est reçu.
sub = rossubscriber('/parc_robot/robot_status', @callbackFcn, 'DataFormat', 'struct');

% Attendre que le robot cesse de se déplacer
msg = receive(sub, 10);

% Arrêter le nœud ROS
rosshutdown
```
