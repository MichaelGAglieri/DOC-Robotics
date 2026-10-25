# Avvio del laboratorio su macOS

Questa guida riguarda la simulazione. Per il braccio fisico usare Robot Console e la sua guida hardware.

1. Installa Pixi e Webots R2025a su un Mac Apple Silicon.
2. Dalla radice esegui `pixi install`, poi `pixi run assets`: le risorse Cyberbotics vengono recuperate dagli URL del manifest e verificate con SHA-256. I download mantengono le condizioni del rispettivo autore.
3. Esegui `pixi run test`. I test non richiedono il robot fisico.
4. Avvia `pixi run webots` e premi Play nella simulazione.
5. In un secondo terminale avvia `pixi run doc`.

Per aprire le finestre operative insieme, usa `pixi run finestre` oppure `AVVIA_LABORATORIO.command`, una sola volta per sessione. macOS può richiedere l’autorizzazione a controllare Terminale. Non avviare anche un secondo coordinatore ROS.

## Comandi espliciti

Da un altro terminale, nella radice:

```sh
pixi run ros topic pub --once /laboratorio/testo std_msgs/msg/String '{data: "stato"}'
pixi run ros topic pub --once /laboratorio/testo std_msgs/msg/String '{data: "sposta campione a in postazione 1"}'
pixi run ros topic pub --once /laboratorio/testo std_msgs/msg/String '{data: "riporta campione a"}'
pixi run ros topic pub --once /laboratorio/testo std_msgs/msg/String '{data: "stop"}'
```

Nella console aperta da `finestre` puoi usare `stato`, `sposta_a`, `sposta_b`, `riporta_a`, `riporta_b`, `stop_lab`. Per una missione usa `pianifica ciclo completo`, controlla il piano e poi `esegui_piano`. `annulla_piano` annulla il piano; `missione` mostra il suo stato. Il planner ricontrolla la scena prima di eseguire.

Attendi l’esito prima di inviare un’altra operazione. Dopo STOP usa R in Webots per ripristinare la scena. Per uscire ferma prima l’operazione con `stop_lab`, poi termina i nodi con Ctrl+C. Il trasporto usa loopback UDP 9999/9998 e ROS_DOMAIN_ID=42.

## Moduli opzionali

`pixi run doc-voce` abilita STT/TTS e conversazione; richiede un microfono autorizzato e un modello Ollama installato localmente. Whisper MLX può scaricare il suo modello al primo utilizzo; non sono distribuiti pesi in questa repository. La conversazione non costituisce un comando motorio generato dal modello linguistico.

## Ricostruzione della scena

`pixi run laboratorio-genera` rigenera mondo, pinza, texture e traiettorie a partire dagli script. Le pose dipendono dalla geometria UR3e derivata e non vanno trasferite al robot fisico.
