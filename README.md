##  TP-1 Note : Turtle Regulation Package-INFO-2
## Written by:
```
BALOOMOODY DAREN DEVAKUMAREN
BEEHARRY VIBHAKARSINGH
HASSANI BACAR
```
##### la simulation de tortue est une simulation qui est faite afin de calculer l'angle de déplacement.


## TURTLE REGUALATION REGULATION
Ce package permet de reguler le movement d'une autre tortue dans turtlesim en utilisant la regulation en cap et en distance.

# Creation du launch File
- Creez un fichier 'launch_file.launch' dans votre package
- Ouvrez le fichier 'launch_file.launch' avec un editeur de texte.
- Ajoutez le contenu suivant pour lancer tous les noeuds necessaires.


### Lancement du noeud set_way_point.py
```
<node name="set_way_point_node" pkg ="my_package" 
type="set_way_point.py" output="screen"/>
```

### lancement du noeud regulation_en_cap.py
```
<node name="regulation_en_cap_node" pkg="my_package" 
type="regulation_en_cap.py" output="screen"/>
```    
### lancement du noeud regulation_en_distance.py
```
<node name="regulation_en_distance_node" pkg="my_package" 
type="regulation_en_distance.py" output="screen"/>
```


### lancement du noeud client.py
```
<node name="client_node" pkg="my_package" type="client.py" output="screen">
```

# Installation 
- Assurez-vous d'avoir installe ROS sur votre systeme
- Clonez ce depot dans votre espace de travail ROS.
- Executerz 'catkin_made' pour compiler le package.

##### etape 1:
clonez le repository dans votre catkin workspace:
```
git clone https://github.com/hachim13/turtle_regulation.git
```

Compiler le package en utilisant `catkin build`:
```
cd catkin_ws
catkin build
```
# Utilisation 
##### lancez le noeud principal avec la command:

```
roslaunch turtle_regulation turtle_regulation.launch
```

La tortue se deplacera vers le waypoint predefini en utilisant la regulation en cap et en distance.

Vous pouvez modifier le waypoint en utilisant le service `set_waypoint_service`.Assurez-vous que la tortue ne soit pas en movement avant d'appeler le service.
```
rosservice call /set_waypoint_service "{x: 5.0, y: 5.0}"
```
