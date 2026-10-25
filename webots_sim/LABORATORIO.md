# DOC: demo Webots di laboratorio — work in progress

Due contenitori dimostrativi si spostano tra quattro supporti, mantenuti verticali. DocArm5 conserva i primi cinque giunti del modello UR3e Cyberbotics R2025a: il sesto giunto e il relativo link sono rimossi. La pinza radiale ha tre cursori cinematici; il suo aspetto richiama EBG-30, senza riprodurne il meccanismo adattivo o le forze reali.

## Avvio

Dalla radice di DOC-Robotics:

```sh
pixi install
pixi run assets
pixi run webots
```

Il primo recupero degli asset richiede rete. In seguito le risorse del manifest sono locali. Premi Play in Webots, poi fai clic nella vista 3D per usare i tasti:

| Tasto | Azione |
| --- | --- |
| D | A alla postazione 1, B alla 2, poi entrambi ai supporti iniziali |
| A / B | Trasferisci il campione indicato; premi di nuovo per riportarlo |
| Spazio | Arresta la sequenza e mantieni l’eventuale campione in presa |
| R | Ripristina la scena mediante reset della simulazione |

Chiudi eventuali istanze precedenti del laboratorio: la porta comandi è UDP loopback 9999 e il feedback usa 9998. Avvia un solo client di controllo alla volta.

## Controllo ROS 2 o da terminale

La [guida rapida](../QUICKSTART.md) descrive il coordinatore ROS e i comandi espliciti. In alternativa, con il coordinatore ROS spento:

```sh
pixi run laboratorio-demo
pixi run laboratorio-demo --cycles 10
pixi run laboratorio-demo --diagnose
pixi run laboratorio-demo --abort
```

Il client verifica posizioni e orientamenti restituiti dal supervisore, oltre al ritorno HOME. Una stringa DONE non è l’unico criterio di riuscita. Dopo STOP usa R per ripristinare la scena.

Il protocollo usa richieste correlate da un identificatore, per esempio:

```json
{"id":"richiesta-1","action":"TRANSFER","params":{"sample":"campione_a","destination":"postazione_1"}}
```

Il controller rifiuta nuove operazioni durante il movimento, eccetto arresto e diagnostica. I nodi ROS verificano catalogo, scena corrente e disponibilità delle destinazioni prima di procedere.

## Ricostruzione e limiti

`pixi run laboratorio-genera` ricrea mondo, braccio, pinza, texture e traiettorie. I generatori conservano guscio esagonale della pinza e palette lime/nero del modello corrente. `pixi run test` verifica la coerenza geometrica e i protocolli.

L’inversa impone tre coordinate e la direzione verticale della pinza, lasciando libera la rotazione sull’asse del flacone. Le tratte cartesiane usano un profilo temporale quintico. I sensori Webots devono raggiungere il bersaglio entro il timeout previsto.

La presa è un vincolo cinematico del supervisore: non sono simulati attrito, deformazione, scivolamento o capacità di carico della pinza stampata. I waypoint usano la geometria UR3e derivata e **non vanno inviati alla Mega del DOC fisico**. Non è un gemello digitale calibrato dell’EB-15.

La telemetria facoltativa su UDP 10000 resta separata dal controllo; la presentazione che la consumava non è inclusa in questa pubblicazione. I test e la simulazione non sostituiscono le verifiche meccaniche ed elettriche del braccio reale. Per gli sviluppi futuri di sensori e feedback vedere la [roadmap](../ROADMAP.md).
