# Robot Skills - Pick & Place con Interactive Marker

## 📦 Funzionalità

Server ROS 2 che permette di:
1. Muovere un **interactive marker** in RViz per definire la posizione target richiede quindi di avviare prima interactive_marker.launch.py
2. Triggerare un servizio che:
   - Spawna un **collision object** (cubo) nella posizione del marker
   - Esegue una sequenza **pick , pouring , place** completa

---

## 🚀 Utilizzo

### 1. **IMPORTANTE: Avvia MoveIt**
```bash
ros2 launch iiwa_config_moveit_config demo_interactive.launch.py
```

### 2. Avvia il pick&place server (in un altro terminale)
```bash
cd /home/nicola/Desktop/Progetto_Sapore
source install/setup.bash
ros2 launch robot_skills pick_place_demo.launch.py
```

### 3. Avvia l'interactive marker (in un altro terminale) serve per applicare la posa dove deve spawnare il collision object(cilindro)
```bash
ros2 run nodes interactive_marker
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





### Pubblica manualmente una target pose:
```bash
ros2 topic pub /reachability_marker/feedback visualization_msgs/msg/InteractiveMarkerFeedback \
  "{header: {frame_id: 'world'}, 
    marker_name: 'target_pose_marker',
    event_type: 1,
    pose: {position: {x: 0.5, y: 0.3, z: 0.2}, 
           orientation: {w: 1.0}}}"
```

