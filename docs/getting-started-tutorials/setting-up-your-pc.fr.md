# Configuration de votre PC

La **première étape** pour commencer le concours consiste à accéder à un ordinateur doté de la configuration système requise et à le configurer avec le système d'exploitation, le logiciel et l'environnement appropriés pour exécuter les packages du concours.

<!-- Ce guide vous aide à configurer votre ordinateur pour exécuter l'environnement de concurrence localement et développer le code.

Vous pouvez utiliser un ordinateur / ordinateur portable local ou une machine virtuelle dans votre ordinateur ou n'importe quelle plate-forme cloud comme [Google GCP](https://cloud.google.com/free){target=_blank}, [Amazon AWS](https://aws.amazon.com/free/){target=_blank}, [Microsoft Azure](https://azure.microsoft.com/en-us/free/){target=_blank}, [Digital Ocean](https://try.digitalocean.com/freetrialoffer/){target=_blank}, etc. (tous les fournisseurs de cloud ont un plan d'essai gratuit que vous pouvez utiliser). -->

## Configuration requise

Vous aurez besoin d'un ordinateur qui possède toutes (ou au moins la plupart) de ces spécifications :
    
- un [gpu](https://en.wikipedia.org/wiki/graphics_processing_unit){target=_blank} dédié,
     - Les cartes Nvidia ont tendance à bien fonctionner à Ubuntu
- un processeur qui est au moins un Intel i5, ou équivalent,
- au moins 4 Go d'espace disque gratuit,
- au moins 8 Go de RAM,
<!-- - [Ubuntu focal fossa](https://releases.ubuntu.com/focal/){target=_blank} installé. -->

## Système d'exploitation (SE)

Nous utiliserons **[Ubuntu Focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank}** système d'exploitation (OS) qui est une variante de [Linux](https: //en.wikipedia.org/wiki/Linux){target=_blank}. Nous savons que certains participants peuvent avoir une expérience limitée ou nulle de l'utilisation d'Ubuntu. Voici donc un guide sur les différentes manières de configurer une instance Ubuntu Focal opérationnelle pour ce concours.

Si vous avez un PC Windows (ou tout autre système d'exploitation différent d'Ubuntu Focal 20.04), voici trois (3) options à explorer :

- Option 1 (recommandée) : **Dual-Boot** : installez **[Ubuntu Focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank}** dans un démarrer à côté de votre système d'exploitation Windows.
- Option 2 : **Utiliser Docker :** Exécuter **[Ubuntu Focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank}** dans un conteneur Docker sur votre Windows natif OS.
- Option 3 : **Utiliser une machine virtuelle :** Exécuter **[Ubuntu Focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank}** dans une machine virtuelle (VM) sur votre système d'exploitation Windows natif.

<!-- !!! note
     Il est fortement recommandé d'installer [focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank} d'Ubuntu en raison de [ROS (Noetic)](http://wiki.ros.org/noetic){target=_blank} dépendance.

## Installation de ROS
Vous devez installer ROS NOetic en suivant [ce guide](http://wiki.ros.org/noetic/installation/ubuntu){target=_blank} et installer `ros-nootic-desktop-full` dans l'étape` 1.4 `du guide. -->

=== "Double démarrage (recommandé)"
     - Voici un [bon guide](https://medium.com/linuxforeveryone/how-to-install-ubuntu-20-04-and-dual-boot-alongside-windows-10-323a85271a73){target=_blank} vous pouvez suivre pour installer **[Ubuntu Focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank}** dans un double démarrage avec votre système d'exploitation Windows.

=== "Utiliser Docker"
     - Nous avons fourni un [guide détaillé](../getting-started-tutorials/setting-up-with-docker.md) sur l'installation de Docker sur votre PC et la configuration du bon conteneur Docker pour exécuter toute la compétition.
     - Si vous êtes préoccupé par l'option de double démarrage, nous vous recommandons d'envisager cette option.

=== "Utiliser une machine virtuelle"
     - Voici un [bon guide](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview){target=_blank} vous pouvez suivre pour installer VirtualBox sur votre PC et exécuter Ubuntu.
     - **REMARQUE :** Le guide ci-dessus se concentre sur Ubuntu 22.04 alors que nous avons besoin de vous pour installer Ubuntu 20.04, vous devrez modifier certaines étapes pour refléter cela.
     - Si vous préférez les didacticiels vidéo, considérez ceci : [Installation d'Ubuntu 20.04 sur VirtualBox](https://www.youtube.com/watch?v=x5MhydijWmc){target=_blank}


## Installation de ROS
!!! note
     Si vous avez suivi l'option **Utiliser Docker** ci-dessus, veuillez IGNORER cette étape.

Une fois que vous avez une nouvelle installation **[Ubuntu Focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank}**, l'étape suivante consiste à installer ROS. Nous utilisons la distribution [ROS Noetic](http://wiki.ros.org/noetic) pour ce concours. Vous pouvez installer ROS Noetic en suivant [ce guide](http://wiki.ros.org/noetic/Installation/Ubuntu){target=_blank} et installer `ros-noetic-desktop-full` à l'étape `1.4` du guide.

Si vous préférez les instructions vidéo, vous pouvez suivre cette vidéo :

- [Installation de ROS Noetic sur Ubuntu 20.04](https://www.youtube.com/watch?v=ZEfh7NxLMxA){target=_blank}