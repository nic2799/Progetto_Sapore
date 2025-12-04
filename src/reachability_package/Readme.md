# Reachability & Pouring Analysis Node

Questo pacchetto ROS 2 contiene il nodo `test_reach.cpp`, progettato per effettuare un'analisi approfondita delle pose raggiungibili (Reachability Analysis) e verificare la fattibilità di operazioni di "pouring" (versamento) in un ambiente simulato con MoveIt 2.

##  Descrizione del Funzionamento del test di raggiunigbilità statico

L’obiettivo di questo nodo è analizzare le pose raggiungibili dal TCP  nell’ambiente simulato, considerando i vincoli cinematici e le eventuali collisioni con gli ostacoli presenti (es. rastrelliera, robot da cucina, trasportatore).

Il flusso di lavoro per ogni punto della griglia generata è il seguente:

1.  **Generazione Griglia:** Viene generata una griglia di punti 3D basata sui parametri configurati.
2.  **Verifica IK (Cinematica Inversa):** Per ciascun punto, si verifica l’esistenza di almeno una configurazione cinematica compatibile con la posa e l'orientamento fissati. Viene utilizzato il servizio `compute_ik` (basato su `moveit_msgs/GetPositionIK`).
3.  **Rilevamento Collisioni:**
    * Il nodo utilizza la classe `PlanningSceneMonitor` e il metodo `checkCollision`.
    * Se l'IK restituisce una soluzione valida ma in collisione, il nodo distingue questo caso da una soluzione inesistente, dunque ogni volta che compute_ik restituisce un fallimento chiamiamo setFromIk metodo di Robotstate che ci permette di capire se quella particolare soluzione è in collisione o se non esiste soluzione all'inversione cinematica.
    * In caso di collisione, i link coinvolti vengono evidenziati in **ROSSO** e pubblicati su `/display_robot_state` per la visualizzazione in RViz.
4.  **Strategia di Riorientamento:** Se la posa iniziale non è raggiungibile o è in collisione, l'algoritmo tenta di ruotare l'orientamento dell'end-effector attorno all'asse target (±30° o varianti) per cercare una configurazione valida alternativa.
5.  **Test di Pouring:** Se il punto è raggiungibile e libero da collisioni, viene pianificata una traiettoria di "pouring" (rotazione dell'ultimo giunto/TCP di ~120°) per verificare che l'operazione di versamento sia eseguibile senza urtare ostacoli.
6. **Salvare dati in file YAML**
     Infine la traiettoria generata dal plan viene memorizzata in un file YAML(obstacle_name + "_plans_.yaml" ).
     Questo file ci servirà per visualizzare le traiettorie offline del package: validate_trajectory
7. **Pubblicazione marker** 
     Per ogni punto appartenente alla griglia verrà applicato un marker cui colore indica se quella particolare posa sia:
     * **Rosso** : Non esiste soluzione all'inversione cinematica
     * **Verde** : Esiste soluzione all'inversione cinematica e tale configurazione non è in collisione con nessun ostacolo, ma il plan non generare una traiettoria
     * **Blu**: Tutte le configurazioni trovate per quella determinata posizione generano collisioni con ostacoli nell'ambiente modificando anche angolo di imbardata della hand.
     * **Azzurro** : Esiste una traiettoria che porta end-effector in quella particolare posa


(È presente una variabile continue_flag_ che consente di eseguire l’analisi di raggiungibilità punto per punto sulla griglia. L’avanzamento alla posa successiva avviene tramite una richiesta al servizio dedicato, permettendo così uno studio interattivo e controllato delle configurazioni raggiungibili.)

Launch file per questa funzione è : grid_points.launch.py
Parametri in ingresso: 
1. group_name: puo essere o right_ o left_ ci indica su quale braccio vogliamo fare il test.
2. Informazioni relative sulla misura della griglia di punti (xmin, ymin ...)
3. Step, ovvero la distanza tra i punti della griglia
4. Nome del file yaml su cui salvare traiettoria

NOTA: il pto 4 e 5 viene effettuato solo per lo studio di raggiungibilità nei punti apprtenenti al robot da cucina 


Le info relative alle traiettorie trovate dal plan sono memorizzate in file yaml:
`Bimby_center_plans_.yaml` e `Rastrelliera_plans_.yaml`


# utilizzo
In un terminale avvia: 
```bash
ros2 launch iiwa_config_moveit_config demo_interactive.launch.py

#dopo di che
ros2 launch nodes grid_points.launch.py

```

* RViz2 configurato per visualizzare i topic `visualization_marker_array` e `/display_robot_state`.


## Descrizione del funzionamento del  test di raggiungibilità interattivo

I nodi necessari per utilizzare tale funzionalità sono: `interactive_marker.cpp `e `Reachability_interactive.cpp`
il primo ci serve per andare a definire una terna interattiva su rviz che variare la sua posa nello spazio, tale posa verrà
pubblicata costantemente nel topi `target_pose`.
Dopo di che il nodo `Reachability_interactive.cpp` farà da service server al servizio `std_srvs::srv::Trigger`, ogni volta che viene mandata una richiesta a questo servizio esso inizia uno studio di raggiungibilita su tale posa, nel caso in cui sia raggiungibile effettua il plan raggiungere quella posa e il plan per azione di pouring.
Launch file per questa funzione è: launch.py
Parametro in ingresso `arm_side` indica su quale braccio effettuiamo il test.

NOTA : Problemi riscontrati quando avvio il launch file mi dice 
[Reac_marker-1] [INFO] [1764770888.140465239] [reachability_tavolo.moveit.ros.current_state_monitor]: Didn't receive robot state (joint angles) with recent timestamp within 1.000000 seconds. Requested time 1764770887.140343, but latest received state has time 0.000000.
Per risolvere tale errore aggiungiamo tale parametro all'avvio del nodo: {"use_sim_time": True},



# utilizzo 
``` bash
ros2 launch iiwa_config_moveit_config demo_interactive.launch.py



ros2 launch nodes interactive_marker.launch.py
```
## Configura RVIZ
* RViz2 configurato per visualizzare i topic `visualization_marker_array` e `/display_robot_state`.

##  Utilizzo

### Avvio del Nodo
Esegui il nodo specificando i parametri desiderati (o modificando il launch file):
