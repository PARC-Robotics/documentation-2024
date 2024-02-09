# Premiers pas avec MATLAB

MATLAB est un langage de programmation et un environnement de calcul numérique utilisé par des millions d'ingénieurs et de scientifiques dans le monde. Il fournit un ensemble d'outils puissants pour analyser les données, développer des algorithmes et créer des modèles et des simulations.
Les participants devront utiliser MATLAB pour effectuer la tâche 2 du concours.

## Présentation de MATLAB et de ses fonctionnalités

MATLAB est un langage de haut niveau et un environnement interactif qui vous permet d'effectuer des tâches de calcul intensives. Il comprend un environnement de programmation, de visualisation et de calcul numérique qui est devenu la norme pour le calcul technique dans les principales entreprises d'ingénierie et de science et le langage standard pour les mathématiques, l'informatique et la science des données.

MATLAB offre un certain nombre d'avantages aux ingénieurs et aux scientifiques, notamment :

* Outils d'analyse numérique complets
* Capacités graphiques et de visualisation faciles à utiliser
* Une grande communauté d'utilisateurs et de ressources d'assistance
* Compatibilité avec d'autres langages de programmation et outils logiciels

Pour ROS, MATLAB fournit un ensemble d'outils permettant de travailler avec des sujets, des services et des actions ROS. Ces outils vous permettent de publier et de vous abonner à des rubriques ROS, d'appeler des services ROS et d'envoyer et de recevoir des actions ROS. Vous pouvez également utiliser MATLAB pour créer et exécuter des nœuds ROS, et pour créer et exécuter des fichiers de lancement ROS.

!!! remarque "Concepts ROS"
     Pour plus d'informations sur les nœuds, les rubriques, les services et les actions ROS, consultez la documentation [Getting Started with ROS](/getting-started-tutorials/getting-started-with-ros/){:target="_blank"}.

## Commencer

### Installation de MATLAB

!!! note "Installation MATLAB"
     En tant que sponsor officiel de PARC, MathWorks a fourni une licence gratuite et complémentaire à toutes les équipes participantes. Pour demander le logiciel, veuillez visiter la [page MathWorks PARC](https://www.mathworks.com/academia/student-competitions/PARC.html){:target="_blank"}.

### Démarrage de MATLAB

Pour démarrer MATLAB, ouvrez un terminal et tapez "matlab". Cela ouvrira l'application de bureau MATLAB. Vous pouvez également démarrer MATLAB en cliquant sur l'icône MATLAB sur votre bureau ou dans votre menu Démarrer.

### Fenêtre de commande MATLAB

La fenêtre de commande MATLAB est l'endroit où vous pouvez entrer des commandes et voir les résultats. Vous pouvez également utiliser la fenêtre de commande pour afficher la valeur des variables et afficher les résultats des calculs.

## Syntaxe de base et types de données

### Syntaxe MATLAB de base

MATLAB a une syntaxe simple et intuitive, facile à apprendre et à utiliser. Voici quelques règles de syntaxe de base :

* Les instructions sont exécutées ligne par ligne.
* Un point-virgule (;) à la fin d'une instruction supprime la sortie vers la fenêtre de commande.
* Les variables sont créées en leur attribuant une valeur.
* Les espaces blancs sont ignorés par MATLAB, l'indentation n'est donc pas nécessaire.

Voici un exemple de syntaxe MATLAB de base :

``` matlab
% Ceci est un commentaire
a = 5;   % Attribuez la valeur 5 à la variable a
b = 2*a; % Affecter la valeur 10 à la variable b
disp(b); % Affiche la valeur de b dans la fenêtre de commande
```

### Types de données
MATLAB prend en charge une variété de types de données, notamment :

* Types de données numériques (entiers, nombres à virgule flottante et nombres complexes)
* Types de données de caractère et de chaîne
* Types de données logiques (valeurs vrai/faux)

Voici quelques exemples de création et d'utilisation de ces types de données dans MATLAB :

``` matlab
% Types de données numériques
x = 5;         % entier
y = 3.14159;   % nombre à virgule flottante
z = 2+3i;      % nombre complexe

% Types de données caractère et chaîne
c = 'a';                % personnage
s = 'Hello, world!';    % chaîne

% Types de données logiques
p = true;   % vraie valeur
q = false;  % fausse valeur
r = (x > y);% expression logique (renvoie vrai ou faux)

% Affichage et manipulation des données
disp(x); % valeur d'affichage de x
fprintf('La valeur de y est %f\n', y); % impression chaîne formatée
s2 = strcat(s, ' MATLAB est génial !'); % chaînes de concaténation
```

## Fonctions et outils communs

MATLAB fournit une riche collection de fonctions et d'outils intégrés qui vous permettent d'effectuer diverses tâches mathématiques et d'ingénierie. Voici quelques-unes des fonctions et outils MATLAB courants qui pourraient vous être utiles :

### 1. Tracé et visualisation

MATLAB fournit des outils puissants pour créer différents types de tracés, de graphiques et de tableaux. Vous pouvez utiliser la fonction plot pour créer des tracés linéaires 2D, la fonction surf pour créer des tracés de surface 3D, la fonction imagesc pour créer des images codées par couleur, et bien d'autres. Voici un exemple d'utilisation de la fonction plot pour créer un tracé linéaire 2D :

``` matlab
% Exemple de code pour créer un tracé linéaire simple
x = linspace(0, 10, 100);
y = sin(x);
plot(x, y)
```

### 2. Opérations matricielles

MATLAB a un support intégré pour les opérations matricielles et vectorielles. Vous pouvez effectuer des opérations élémentaires, la multiplication de matrices, l'inversion de matrices et bien d'autres. Voici quelques exemples:

``` matlab
% Exemple de code pour les opérations matricielles
A = [1 2; 3 4];
B = [5 6 ; 7 8];
C = A + B;      % d'ajout élément par élément
D = A * B;      % multiplication matricielle
E = inv(A);     % d'inversion de matrice
F = A .* B;     % multiplicat élément par élément
G = A .^ 2;     % exponentiation élément par élément
```

### 3. Traitement des images

MATLAB fournit un ensemble de fonctions et d'outils pour le traitement d'images. Vous pouvez utiliser la fonction imresize pour redimensionner une image, la fonction imrotate pour faire pivoter une image, et bien d'autres. Voici un exemple d'utilisation de la fonction imresize pour redimensionner une image :

``` matlab
% Exemple de code pour le traitement d'images
I = imread('image.jpg');    % lire une image à partir d'un fichier
J = imresize(I, 0.5);       % redimensionne l'image par un facteur de 0.5
imshow(J);                  % affiche l'image redimensionnée
```

### 4. Intégration ROS

MATLAB fournit un ensemble de fonctions et d'outils pour l'intégration avec ROS. Vous pouvez utiliser la fonction rossubscriber pour créer un abonné ROS, la fonction rospublisher pour créer un éditeur ROS, et bien d'autres. Voici un exemple d'utilisation de la fonction rossubscriber pour créer un abonné ROS et un éditeur ROS :

=== "Exemple d'éditeur ROS"

     ``` matlab
     % Exemple de code pour l'intégration ROS

     rosshutdown;                                % arrêter ROS
     rosinit;                                    % initialiser ROS
     pub = rospublisher('/chatter', 'std_msgs/String'); % créer un éditeur ROS
     count = 0;                                  % créer un compteur

     while true                                      % continuer à publier pour toujours
         msg = rosmessage(pub);                      % create a ROS message
         c = num2str(count);
         msg.Data = ['Hello, world!' ' ' c];         % attribuer une valeur aux données du message
         send(pub, msg);                             % publier le message
         count = count + 1;                          % compteur d'incrémentation
     end
     ```
=== "Exemple d'abonné ROS"
     ``` matlab
     % Exemple de code pour l'intégration ROS

     rosshutdown;                                       % redémarrer le nœud global
     rosinit;                                           % initialise ROS
     sub = rossubscriber('/chatter');                   % créer un abonné ROS
     msg = recevoir(sub, 3);                            % reçoivent un message de l'abonné
     disp(msg.Data);                                    % afficher les données du message
     rosshutdown;                                       % arrêter ROS
     ```

``` matlab
Bonjour le monde!
```

## Ressources MATLAB importantes
De nombreuses ressources sont disponibles pour apprendre MATLAB, notamment des didacticiels, des cours en ligne et de la documentation. Le site Web MathWorks fournit un ensemble complet de ressources, notamment :

<!-- * [Tutoriel Premiers pas avec MATLAB](https://www.mathworks.com/help/matlab/getting-started-with-matlab.html){:target="_blank"} -->
* [MATLAB OnRamp](https://matlabacademy.mathworks.com/details/matlab-onramp/gettingstarted) (Fortement recommandé)
* [MATLAB Answers](https://www.mathworks.com/matlabcentral/answers/){:target="_blank"} : une communauté d'utilisateurs MATLAB qui peuvent vous aider à répondre à vos questions
* [Tutoriels MATLAB et Simulink ROS](https://github.com/mathworks-robotics/matlab-and-simulink-ros-tutorials)

Grâce à ces ressources, vous pouvez vous familiariser rapidement avec MATLAB et commencer à l'utiliser pour ce concours et vos propres projets d'ingénierie et scientifiques.
