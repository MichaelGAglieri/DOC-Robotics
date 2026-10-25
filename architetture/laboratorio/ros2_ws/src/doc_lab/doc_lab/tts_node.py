#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
"""D.O.C. - TTS: sintesi vocale nativa macOS con isolamento acustico del microfono.

Su Apple Silicon la catena piper | aplay non e' disponibile: 'aplay' e 'pactl'
appartengono ad ALSA/PulseAudio, che su macOS non esistono. Si usa 'say', il
sintetizzatore di sistema, con le voci italiane gia' installate.

La sintesi e' bloccante (say dura quanto la frase), quindi gira su un thread
dedicato con coda: il callback ROS si limita ad accodare. Farla nel callback
congelava l'executor per tutta la durata del parlato.
"""

import queue
import re
import subprocess
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

# Voce di riserva: Alice e' la voce italiana storica di macOS, presente su
# qualunque installazione e senza omonimi in altre lingue.
VOCE_DI_RISERVA = "Alice"

# Righe di 'say -v ?': nome, locale, frase di esempio. Il nome puo' contenere
# spazi e parentesi, come "Flo (Italiano (Italia))".
RIGA_VOCE = re.compile(r'^(.*?)\s+([a-z]{2}_[A-Z]{2})\s+#')

# Le varianti ad alta qualita' vanno scaricate a mano da Impostazioni di
# Sistema, ma se ci sono vanno preferite: sono nettamente piu' naturali.
QUALITA = ('premium', 'enhanced')


class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        self.declare_parameter('voce', 'Emma')
        self.declare_parameter('lingua', 'it_IT')
        self.declare_parameter('velocita', 175)
        self.declare_parameter('hard_mute', True)

        self.lingua = self.get_parameter('lingua').value
        self.velocita = self.get_parameter('velocita').value
        self.hard_mute = self.get_parameter('hard_mute').value
        self.voce = self.risolvi_voce(self.get_parameter('voce').value)

        # Il volume del microfono va ripristinato al valore che l'utente aveva
        # impostato, non a un default arbitrario.
        self.volume_input_originale = self.leggi_volume_input()

        self.speech_sub = self.create_subscription(
            String, '/robot/speech', self.on_text_received, 10)
        self.status_pub = self.create_publisher(Bool, '/robot/tts_status', 10)

        self.coda_frasi = queue.Queue()
        self.attivo = True
        self.worker = threading.Thread(target=self.loop_sintesi, daemon=True)
        self.worker.start()

        self.get_logger().info(
            f'🎙️ D.O.C. TTS Node avviato! Voce "{self.voce}" a {self.velocita} parole/min.')

    # ------------------------------------------------------------------
    # Voce
    # ------------------------------------------------------------------
    def elenco_voci(self):
        """[(nome completo, locale)] cosi' come li dichiara macOS."""
        try:
            out = subprocess.run(["say", "-v", "?"], capture_output=True,
                                 text=True, check=False).stdout
        except Exception as e:
            self.get_logger().warn(f'⚠️ Impossibile elencare le voci ({e}).')
            return []
        voci = []
        for riga in out.splitlines():
            m = RIGA_VOCE.match(riga)
            if m:
                voci.append((m.group(1).strip(), m.group(2)))
        return voci

    def risolvi_voce(self, richiesta):
        """Risolve il nome della voce restando dentro la lingua configurata.

        Serve perche' molti nomi macOS esistono in una dozzina di lingue:
        'Reed' da solo corrisponde anche al tedesco e all'inglese, e 'say'
        sceglie il primo che trova. Il risultato e' testo italiano letto con
        accento straniero, senza alcun errore che lo segnali. Qui la ricerca
        e' sempre filtrata sul locale, e a parita' di nome vince la variante
        di qualita' superiore.
        """
        voci = self.elenco_voci()
        if not voci:
            return richiesta

        nella_lingua = [n for n, loc in voci if loc == self.lingua]

        def punteggio(nome):
            basso = nome.lower()
            for i, q in enumerate(QUALITA):
                if q in basso:
                    return len(QUALITA) - i
            return 0

        breve = richiesta.split(' (')[0].strip().lower()
        candidati = [n for n in nella_lingua
                     if n.split(' (')[0].strip().lower() == breve]

        # Se l'utente ha scritto il nome per esteso, ha scelto una variante
        # precisa: si rispetta. Se ha scritto solo "Alice", si prende la
        # migliore disponibile — cosi' scaricando la variante Premium da
        # Impostazioni di Sistema il salto di qualita' arriva da solo.
        if '(' in richiesta:
            if richiesta in nella_lingua:
                return richiesta
        elif candidati:
            scelta = max(candidati, key=punteggio)
            if scelta != richiesta:
                self.get_logger().info(f'🔤 Voce "{richiesta}" risolta in "{scelta}".')
            return scelta

        if candidati:
            scelta = max(candidati, key=punteggio)
            self.get_logger().warn(
                f'⚠️ Variante "{richiesta}" non trovata: uso "{scelta}".')
            return scelta

        # La voce esiste, ma non in questa lingua: e' l'errore da evitare.
        if any(n.split(' (')[0].strip().lower() == breve for n, _ in voci):
            self.get_logger().warn(
                f'⚠️ La voce "{richiesta}" non esiste in {self.lingua}: '
                f"leggerebbe l'italiano con accento straniero. "
                f'Uso "{VOCE_DI_RISERVA}".')
        else:
            self.get_logger().warn(
                f'⚠️ Voce "{richiesta}" non installata: uso "{VOCE_DI_RISERVA}". '
                f'Le voci si aggiungono da Impostazioni di Sistema -> '
                f'Accessibilita -> Contenuto letto -> Voce di sistema.')
        return VOCE_DI_RISERVA

    # ------------------------------------------------------------------
    # Microfono
    # ------------------------------------------------------------------
    def leggi_volume_input(self):
        try:
            r = subprocess.run(
                ["osascript", "-e", "input volume of (get volume settings)"],
                capture_output=True, text=True, check=False)
            valore = int(r.stdout.strip())
            return valore if valore >= 0 else None
        except (ValueError, Exception):
            return None

    def imposta_volume_input(self, valore):
        try:
            subprocess.run(
                ["osascript", "-e", f"set volume input volume {int(valore)}"],
                check=False, capture_output=True)
        except Exception as e:
            self.get_logger().error(f"Errore controllo hardware microfono: {e}")

    def mute_microphone(self, mute=True):
        """Azzera il volume di ingresso mentre D.O.C. parla, poi lo ripristina."""
        if not self.hard_mute or self.volume_input_originale is None:
            return
        self.imposta_volume_input(0 if mute else self.volume_input_originale)

    # ------------------------------------------------------------------
    # Sintesi
    # ------------------------------------------------------------------
    def on_text_received(self, msg):
        """Callback non bloccante: accoda e basta."""
        testo = msg.data.replace('*', '').replace('#', '').replace('_', ' ').strip()
        if testo:
            self.coda_frasi.put(testo)

    def loop_sintesi(self):
        while self.attivo:
            try:
                testo = self.coda_frasi.get(timeout=0.2)
            except queue.Empty:
                continue

            self.get_logger().info('Sintesi vocale e isolamento acustico in corso...')
            self.mute_microphone(mute=True)
            self.pubblica_stato(True)

            try:
                # Il testo passa come argomento separato, non nella shell:
                # niente da citare e nessuna interpretazione di metacaratteri.
                subprocess.run(
                    ["say", "-v", self.voce, "-r", str(self.velocita), "--", testo],
                    check=False)
            except FileNotFoundError:
                self.get_logger().error(
                    "❌ Comando 'say' non trovato: questo nodo richiede macOS.")
            except Exception as e:
                self.get_logger().error(f"Errore hardware audio: {e}")
            finally:
                self.mute_microphone(mute=False)
                self.pubblica_stato(False)
                self.coda_frasi.task_done()

    def pubblica_stato(self, sta_parlando):
        msg = Bool()
        msg.data = sta_parlando
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.attivo = False
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)
        self.mute_microphone(mute=False)  # Sicurezza in chiusura
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
