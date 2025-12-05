# Robot Skills - Pick & Place con Interactive Marker

##  Funzionalità

Server ROS 2 che permette di:
1. Muovere un **interactive marker** in RViz per definire la posizione target richiede quindi di avviare prima interactive_marker.launch.py
2. Triggerare un servizio che:
   - Spawna un **collision object** (cilindro) nella posizione del marker
   - Esegue una sequenza **pick , pouring , place** completa

---

##  Utilizzo
### avvia moveit
```bash
ros2 launch iiwa_config_moveit_config demo_interactive.launch.py

```
###  Avvia il pick&place server (in un altro terminale)
```bash
ros2 launch robot_skills pick_place_demo.launch.py
```

### 3. Avvia l'interactive marker (in un altro terminale) serve per applicare la posa dove deve spawnare il collision object(cilindro)
```bash
ros2 run reachability_package interactive_reach
```

### 4. In RViz:
- **Aggiungi Interactive Marker** display (se non già presente)
- **Muovi il marker** nella posizione desiderata per l'oggetto
- Il server stamperà: ` Nuova target pose ricevuta: [x, y, z]`
- Il marker pubblica su `/reachability_marker/feedback`

### 5. Triggera il pick&place:
```bash
ros2 service call /trigger_pick_place std_srvs/srv/Trigger
```

**Cosa succede:**
1. Il server usa l'**ultima** posa ricevuta da `/reachability_marker/feedback`
2. Spawna un cubo in quella posizione
3. Esegue: approccio → grasp → solleva → place 
4. Salva la traiettoria in PickPouringLeft.yaml per braccio sinistro o PickPouringRight.yaml per il braccio destro
(questi file contengono diversi plan ognuno associato ad un ID che viene settato nella variabile plan_counter_ in pick_place_server.cpp)




