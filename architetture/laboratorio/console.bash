# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
# Shell dedicata alla console del laboratorio; non modifica la shell utente.
DOC_LAB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$DOC_LAB_ROOT" || return
PS1='DOC laboratorio > '
_doc_pub() {
    local payload
    payload="$("$DOC_LAB_ROOT/.pixi/envs/default/bin/python" -c 'import json,sys; print(json.dumps({"data":sys.argv[1]}))' "$1")" || return
    (cd "$DOC_LAB_ROOT" && pixi run ros topic pub --once /laboratorio/testo std_msgs/msg/String "$payload")
}
stato()     { _doc_pub 'stato'; }
sposta_a()  { _doc_pub 'sposta campione a in postazione 1'; }
sposta_b()  { _doc_pub 'sposta campione b in postazione 2'; }
riporta_a() { _doc_pub 'riporta campione a'; }
riporta_b() { _doc_pub 'riporta campione b'; }
stop_lab()  { _doc_pub 'stop'; }
pianifica()     { _doc_pub "pianifica ${*:-ciclo completo}"; }
esegui_piano()  { _doc_pub 'esegui piano'; }
annulla_piano() { _doc_pub 'annulla piano'; }
missione()      { _doc_pub 'stato missione'; }
ultimo_report() {
    "$DOC_LAB_ROOT/.pixi/envs/default/bin/python" - "$DOC_LAB_ROOT" <<'REPORT'
from pathlib import Path
import os,sys
root=Path(os.getenv('DOC_MISSIONS_DIR',str(Path(sys.argv[1])/'output/missioni_laboratorio')))
reports=list(root.glob('*/report.md'))
if not reports:
 print('Nessuna missione registrata.')
else:
 report=max(reports,key=lambda path:path.stat().st_mtime)
 print(str(report)+'\n')
 print(report.read_text())
REPORT
}
aiuto() {
    cat <<'HELP'
Comandi disponibili:
  stato       Mostra lo stato del laboratorio
  sposta_a    Campione A → postazione 1
  sposta_b    Campione B → postazione 2
  riporta_a   Riporta A al supporto iniziale
  riporta_b   Riporta B al supporto iniziale
  stop_lab    Arresta il trasferimento nel simulatore
  pianifica                  Anteprima del ciclo completo
  pianifica prepara          Porta i campioni alle postazioni
  pianifica riordina         Riporta i campioni ai supporti
  pianifica scambia supporti  Pianifica uno scambio con appoggio temporaneo
  esegui_piano               Esegue il piano dopo un nuovo controllo
  annulla_piano              Scarta l'anteprima
  missione                   Mostra lo stato della missione
  ultimo_report              Legge l'ultimo registro di esecuzione
  aiuto                      Mostra questi comandi

Premi Play in Webots. Esegui un comando alla volta e attendi DONE
nella finestra degli esiti. Per le missioni attendi COMPLETED nel monitor
Missioni. Dopo stop_lab premi R in Webots.
Per chiudere: arresta prima il trasferimento, poi Ctrl+C nelle finestre
dei processi; exit chiude questa console. Chiudi Webots separatamente.
HELP
}
aiuto
