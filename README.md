# Progetto_Sapore
Questo progetto nasce nell’ambito del tirocinio e della tesi magistrale, e ha come obiettivo la progettazione e simulazione di una cella robotica dual-arm per operazioni di manipolazione collaborativa.
Il sistema è sviluppato su ROS 2 (Jazzy) e utilizza MoveIt 2 per la pianificazione e l’analisi della raggiungibilità.

L’obiettivo finale è costruire un ambiente simulato realistico in cui due manipolatori (basati su modelli KUKA iiwa) eseguono task atomici all’interno di una cella dotata di oggetti e ostacoli tridimensionali.


Il progetto è suddiviso in 6 package principali, ciascuno dedicato a una specifica funzionalità della cella robotica dual-arm:

1. build_scene_package
Gestione e generazione della scena simulata, inclusi oggetti e ostacoli.

2. iiwa_description
Modelli URDF/XACRO e configurazioni dei robot KUKA iiwa.

3. iiwa_config_moveit_config
Configurazione di MoveIt 2 per la pianificazione e il controllo dei manipolatori.

4. robot_skills
Implementazione delle skill robotiche come pick, place, pouring e gestione delle traiettorie.

5.  reachability_package
Nodi ROS2 per analisi di raggiungibilità.

6. validate_trajectory
Strumenti per la validazione e l’esecuzione delle traiettorie pianificate.

Ogni package contiene un file README dedicato che spiega nel dettaglio il funzionamento e l’utilizzo delle relative componenti.
Consulta i README specifici per approfondire le funzionalità di ciascun package.



