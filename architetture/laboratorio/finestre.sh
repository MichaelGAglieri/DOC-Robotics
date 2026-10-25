#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
# Apre le finestre operative del laboratorio DOC su macOS.
set -euo pipefail
PROG="$(cd "$(dirname "$0")/../.." && pwd)"
CON_VOCE=0
DRY_RUN=0
for arg in "$@"; do
    case "$arg" in
        --voce) CON_VOCE=1 ;;
        --senza-voce) CON_VOCE=0 ;;
        --dry-run) DRY_RUN=1 ;;
        --help|-h)
            echo "Uso: pixi run finestre [--voce|--senza-voce] [--dry-run]"
            echo "Apre Webots, ROS, esiti, monitor missioni e console comandi. Premi Play in Webots."
            exit 0 ;;
        *) echo "Opzione sconosciuta: $arg" >&2; exit 2 ;;
    esac
done
if [ "$(uname -s)" != Darwin ]; then
    echo "Questo avvio richiede Terminale su macOS." >&2; exit 1
fi
PIXI_BIN="$(command -v pixi || true)"
if [ -z "$PIXI_BIN" ]; then
    echo "Pixi non trovato nel PATH." >&2; exit 1
fi
cd "$PROG"
DOC_DOMAIN="${ROS_DOMAIN_ID:-42}"
case "$DOC_DOMAIN" in ''|*[!0-9]*) echo "ROS_DOMAIN_ID deve essere numerico." >&2; exit 2 ;; esac
printf -v PREAMBLE 'cd %q && export PATH=%q:"$PATH" ROS_DOMAIN_ID=%q && ' \
    "$PROG" "$(dirname "$PIXI_BIN")" "$DOC_DOMAIN"

open_window() {
    local title="$1" body="$2" slot="$3" command
    command="$PREAMBLE$body"
    if [ "$DRY_RUN" -eq 1 ]; then
        printf '\n[%s]\n%s\n' "$title" "$command"
        return
    fi
    # Gli argomenti vengono passati come dati: percorsi e comandi non sono
    # interpolati nel sorgente AppleScript.
    /usr/bin/osascript - "$title" "$command" "$slot" <<'APPLESCRIPT'
on run argv
    set windowTitle to item 1 of argv
    set shellCommand to item 2 of argv
    set slotIndex to (item 3 of argv) as integer
    tell application "Terminal"
        set createdTab to do script shellCommand
        set custom title of createdTab to windowTitle
        -- Identifica la finestra dal tab restituito, senza assumere che
        -- sia ancora quella in primo piano.
        repeat with candidate in windows
            if selected tab of candidate is createdTab then
                set leftEdge to 30 + (slotIndex mod 2) * 65
                set topEdge to 50 + slotIndex * 40
                set bounds of candidate to {leftEdge, topEdge, leftEdge + 920, topEdge + 560}
                exit repeat
            end if
        end repeat
        activate
    end tell
end run
APPLESCRIPT
}

if [ "$DRY_RUN" -eq 0 ]; then
    if [ ! -x /Applications/Webots.app/Contents/MacOS/webots ]; then
        echo "Webots non trovato in /Applications/Webots.app." >&2; exit 1
    fi
    echo "Compilo il laboratorio prima di aprire i terminali..."
    "$PIXI_BIN" run build
fi

if [ "$CON_VOCE" -eq 1 ]; then
    if [ "$DRY_RUN" -eq 1 ] || ! curl -fsS --max-time 2 http://localhost:11434/api/tags >/dev/null 2>&1; then
        OLLAMA_BIN="$(command -v ollama || true)"
        if [ -z "$OLLAMA_BIN" ]; then
            echo "Ollama non trovato: avvia l'app Ollama oppure usa --senza-voce." >&2; exit 1
        fi
        printf -v OLLAMA_CMD 'exec %q serve' "$OLLAMA_BIN"
        open_window 'DOC · Ollama' "$OLLAMA_CMD" 5
    fi
fi
open_window 'DOC · Webots laboratorio — premi Play' 'pixi run webots' 0
if [ "$CON_VOCE" -eq 1 ]; then
    ROS_CMD='pixi run -- architetture/laboratorio/nodo.sh ros2 launch doc_lab laboratorio.launch.py con_voce:=true con_llm:=true'
else
    ROS_CMD='pixi run -- architetture/laboratorio/nodo.sh ros2 launch doc_lab laboratorio.launch.py'
fi
open_window 'DOC · Nodi ROS laboratorio' "$ROS_CMD" 1
open_window 'DOC · Esiti delle operazioni' 'pixi run ros topic echo /laboratorio/esito' 2
open_window 'DOC · Missioni — piano e avanzamento' 'pixi run missioni' 3
open_window 'DOC · Console comandi' 'exec /bin/bash --noprofile --rcfile architetture/laboratorio/console.bash -i' 4
printf '\nFinestre predisposte. Premi Play in Webots, poi usa la console comandi.\n'
