# Integration_TB3_Poppy_Vision_GRP1

Dépôt du groupe 1 concernant l'intégration finale

# Commande de démarrage

On commence par se connecter à Poppy en ssh et on vérifie que le _ROS_MASTER_URI_ pointe bien vers Poppy. On lance ensuite la commande **"roslaunch poppy_controllers control.launch"** afin de lancer le serveur de trajectoire des moteurs.

On se connecte à TB3 en ssh, on vérifie que le _ROS_MASTER_URI_ pointe vers Poppy et on lance **"roslaunch turtlebot3_bringup turtlebot3_robot.launch"**

On vérifie le _ROS_MASTER_URI_ et on lance _MoveIt_ grâce à "**roslaunch poppy_ergo_jr_moveit_config demo.launch fake_execution:=false gripper:=true"**.

On lance la localisation et la navigation autonome grâce à **"roslaunch turtlebot3_navigation turtlebot3_navigation.launch"**

On lance le package **"usb_cam"** afin de récupérer l'image de notre caméra usb grâce à **"roslaunch usb_cam usb_cam-test.launch"**

On peut maintenant lancer les 3 nodes.
On lance en premier _manipulate.py_ qui se charge d'initialiser le paramètre **"robotReady"**, **"label"**, **"targetLabel"** et **"takeImage"**.
On lance en deuxième _nn.py_ qui se charge d'identifier les différents label et leurs ordres de placement sur les cubes.
On lance en dernier _simple_navigation.py_ qui se charge de la navigation vers les points de déchargement et du retour du robot vers le point de chargement.

# Paramètres utilisés et leurs rôles dans le système

Nous avons utilisés les 4 paramètres suivant :
 - Le paramètre **"robotReady"** informe les autres nodes que TB3 est présent ou non dans la zone de chargement et déclenche la mise à jour du du paramètre **"label"** par _nn.py_.
 - Le paramètre **"label"** informe le node de manipulation quel label est inscrit sur le cube à charger et sa mise à jour déclenche le chargement du cube sur TB3.
 - Le paramètre **"targetLabel"** informe le node de navigation vers quelle zone de déchargement TB3 doit se diriger.
 - Le paramètre **"takeImage"** déclenche la capture d'une image et son traitement par _nn.py_.

# Compte-rendu intégration

La programmation des différents nodes ainsi que leur synchronisation s'est faite en suivant le diagramme de séquence et s'est déroulée sans encombre.
Nous avons eu quelques problèmes pour record et rejouer les différents mouvements de Poppy lundi à cause d'erreurs liés au réseau.
Toutefois, le problème majeur que nous avons rencontré est lié aux erreurs de positionnement que rencontre TB3 et sa navigation tout au long de la démonstration. Nous avons dû charger au moins une fois le cube à la main durant chaque démonstration car TB3 se décale au niveau de la zone de chargement au fur et à mesure des aller-retour. D'après ce que nous avons compris, ces erreurs sont liées au patinage et sont donc renvoyées par l'odométrie. Elles sont aussi liées aux erreurs de prédictions de localisation générées par les données du lidar. On pensait donc pouvoir les réduire en augmentant le refresh rate du lidar et diminuer la vitesse linéaire et rotationnelle de TB3. Malheuresement, nous n'avons pas eu le temps/les connaissances pour faire les modifications et nous avons préféré ajouter des points de passages à proximité de la zone de chargement afin d'améliorer la précision de TB3.

# Livrables

Nous vous avons envoyé une vidéo montrant le bon déroulement du chargement de TB3 lorsqu'il est immobile ainsi qu'une vidéo montrant l'intégration finale. Nous avons préféré vous envoyer celle-ci car nous avons eu à charger TB3 qu'une seule fois à la main. Toutefois, elle se termine par une erreur de liée au local planner. Nous avons fait d'autres tests sans erreur de navigation mais avec des erreurs de positionnement dans la zone de chargement qui nécessite notre intervention pour le chargement.

Vous trouverez aussi une vidéo montrant l'execution du code à travers les terminaux. Elle se déroule en condition réelle, seule la navigation du robot a été désactivée pour diminuer la longueur de la vidéo.
Sur la première ligne des terminaux se trouvent les différents roslaunch liés aux moteurs de Poppy et TB3 ainsi que la navigation et MoveIt. Sur la deuxième ligne, de gauche à droite, se trouvent _manipulate.py_, _simple_navigation.py_ et _nn.py_.



