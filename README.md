# Entwicklung eines autonomen Fahrzeugs SoSe 2022
---
Dieses Projekt ist in Zusammenarbeit mit meinem Kommilitonen Sven Thomas entstanden. Es war die zu erbringende Prüfungsleistung für das Modul "Entwicklung eines autonomen Fahrzeugs", welches wir im Sommersemester 2022 absolviert haben.
Aufgabe war es in einer Simulation, mit einem virtuellen autonomen Fahrzeug einen bestimmten Parkour zu absolvieren und dabei verschiedene Aufgaben zu erfüllen. Details hier zu können Sie der beigefügten [Ausarbeitung](Ausarbeitung.pdf) entnehmen.

## Installation 
```
git clone git@github.com:mbbl33/Entwicklung_eines_autonomen_Fahrzeugs_SoSe_2022.git
```
```
docker-compose build
```

## Ausführen 
```
docker-compose up -d #start docker in background
docker-compose exec ros bash #enter docker bash
ros2 launch gazebo_simulation car.launch.py  #start gazebo simulation
```

### Starten des Lösungsansatz A
``` 
ros2 launch autonomous_driving ad.launch.py
ros2 run autonomous_driving velocity_controller
```

### Starten des Lösungsansatz B
```
ros2 launch autonomous_driving_v2 ad.launch.py
ros2 run autonomous_driving_v2 velocity_controller # not final and not working
```

# Disclaimer
Die Gazebo Simulation und das darin enthaltene "cITIcar" wurden uns im Rahmen der Veranstaltung als Grundgerüst vom Dozenten [Jakob Czekansky](https://www.thm.de/mni/jakob-czekansky "Jakob Czekansky") zur Verfügung gestellt. 
Da die simulierte Helligkeit je nach Hardware etwas schwankt, kann es vorkommen, dass auf manchen Systemen die Thresholds nicht optimal sind und somit Abweichungen zum gewünschten Fahrverhalten auftreten.