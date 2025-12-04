In questo package sono stati implementati due nodi principali per la gestione delle mesh e della scena di simulazione:

* Acquisizione delle informazioni delle mesh:
Entrambi i nodi leggono da un file YAML(environment.yaml) che contiene la posa e le proprietà di ogni mesh da inserire nella scena.

* build_scene_mesh:
Questo eseguibile genera e pubblica un unico collision object che rappresenta l’intero ambiente. Tutte le mesh sono fuse in una singola mesh globale.

* build_scene_simple:
Questo nodo, su cui ci si concentra maggiormente, pubblica un collision object separato per ogni mesh che rappresenta un singolo ostacolo. Componendo tutti questi collision object, viene generata la scena completa, permettendo una gestione più dettagliata e flessibile degli ostacoli.

In sintesi, il package permette di creare la scena robotica sia come ambiente unico che come insieme di ostacoli distinti, a partire dalle informazioni contenute nel file YAML

NOTA: Le mesh utilizzate sono versioni semplificate, ottimizzate per migliorare l’efficienza e la qualità della pianificazione delle traiettorie con MoveIt.