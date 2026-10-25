#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
"""D.O.C. - STT: trascrizione vocale con Whisper sulla GPU Apple via MLX.

Sostituisce faster-whisper su CUDA, che su Apple Silicon non ha equivalente:
MLX esegue Whisper sulla GPU integrata del M4.

La cattura non usa piu' SpeechRecognition, che dipende da PyAudio e non ha
build per osx-arm64: il microfono viene aperto direttamente con sounddevice
(CoreAudio) e la segmentazione del parlato e' implementata qui sotto. La
taratura riproduce quella gia' calibrata sul campo: soglia adattiva al rumore
della stanza, taglio dopo 0.6 s di silenzio, 0.4 s di preroll conservati.

Nota: al primo avvio il modello viene scaricato da Hugging Face. macOS
chiedera' il permesso di accedere al microfono al processo che lancia questo
nodo (Terminale o iTerm): va concesso in Impostazioni di Sistema -> Privacy e
sicurezza -> Microfono, altrimenti la cattura resta muta.
"""

import collections
import os
import queue
import threading
import time

import numpy as np
import rclpy
import sounddevice as sd
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

import mlx_whisper

os.environ["HF_HUB_DISABLE_SYMLINKS_WARNING"] = "1"
os.environ["HF_HUB_DISABLE_IMPLICIT_TOKEN"] = "1"

# Allucinazioni tipiche di Whisper sul silenzio o sul rumore di fondo: il
# modello e' stato addestrato anche su sottotitoli di video, e quando non c'e'
# parlato tende a produrre le frasi di chiusura che ha visto migliaia di volte.
# Il confronto e' per SOTTOSTRINGA, non per uguaglianza: la frase reale e'
# quasi sempre piu' lunga ("sottotitoli e revisione a cura di QTSS").
RUMORE_NOTO = (
    "sottotitoli", "sottotitoli e revisione", "a cura di",
    "grazie per aver guardato", "grazie per l'attenzione",
    "iscriviti al canale", "buona visione", "alla prossima",
    "amara.org", "qtss", "www.", "http",
)

# Sotto queste soglie non si trascrive nemmeno: Whisper allucina soprattutto
# quando gli si passa silenzio o un frammento troppo corto per contenere
# parlato vero.
DURATA_MINIMA = 0.35      # s
ENERGIA_MINIMA = 0.004    # RMS medio del segmento

# Sotto questo livello RMS non si considera parlato in nessun caso: evita che
# in una stanza silenziosissima la soglia adattiva scenda fino al rumore
# elettrico del preamplificatore.
SOGLIA_MINIMA = 0.006


class AscoltatoreMicrofono:
    """Cattura dal microfono con rilevamento di inizio e fine frase.

    Riproduce il comportamento di SpeechRecognition.listen(): attende che
    l'energia superi la soglia, registra finche' il parlato non si interrompe
    per 'pausa_fine_frase' secondi, e conserva un preroll di silenzio davanti
    alla frase perche' Whisper non tagli la prima sillaba.
    """

    def __init__(self, sample_rate=16000, durata_blocco=0.03,
                 pausa_fine_frase=0.6, silenzio_iniziale=0.4,
                 moltiplicatore_soglia=3.5, device=None):
        self.sample_rate = sample_rate
        self.campioni_blocco = int(sample_rate * durata_blocco)
        self.durata_blocco = self.campioni_blocco / float(sample_rate)
        self.pausa_fine_frase = pausa_fine_frase
        self.blocchi_preroll = max(1, int(silenzio_iniziale / self.durata_blocco))
        self.moltiplicatore = moltiplicatore_soglia

        self.rumore_fondo = SOGLIA_MINIMA
        self.soglia = SOGLIA_MINIMA * moltiplicatore_soglia

        self.coda = queue.Queue()
        self.stream = sd.InputStream(
            samplerate=sample_rate, channels=1, dtype='float32',
            blocksize=self.campioni_blocco, device=device,
            callback=self._callback)

    def _callback(self, indata, frames, time_info, status):
        self.coda.put(indata[:, 0].copy())

    @staticmethod
    def _rms(blocco):
        return float(np.sqrt(np.mean(np.square(blocco))))

    def avvia(self):
        self.stream.start()

    def ferma(self):
        try:
            self.stream.stop()
            self.stream.close()
        except Exception:
            pass

    def svuota(self):
        """Scarta l'audio accumulato: usato dopo che il robot ha parlato."""
        while True:
            try:
                self.coda.get_nowait()
            except queue.Empty:
                return

    def calibra(self, durata=2.0):
        """Misura il rumore di fondo della stanza e fissa la soglia iniziale."""
        self.svuota()
        letture = []
        scadenza = time.monotonic() + durata
        while time.monotonic() < scadenza:
            try:
                letture.append(self._rms(self.coda.get(timeout=0.5)))
            except queue.Empty:
                break
        if letture:
            self.rumore_fondo = float(np.median(letture))
        self.soglia = max(self.rumore_fondo * self.moltiplicatore, SOGLIA_MINIMA)
        return self.soglia

    def _aggiorna_rumore(self, rms):
        """Soglia dinamica: si adatta solo mentre nessuno parla."""
        self.rumore_fondo = 0.95 * self.rumore_fondo + 0.05 * rms
        self.soglia = max(self.rumore_fondo * self.moltiplicatore, SOGLIA_MINIMA)

    def ascolta(self, timeout=3.0, durata_massima=10.0):
        """Ritorna la frase come float32 mono, oppure None se nessuno parla."""
        preroll = collections.deque(maxlen=self.blocchi_preroll)
        scadenza = time.monotonic() + timeout
        primo_blocco = None

        # Fase 1: attesa dell'inizio del parlato.
        while time.monotonic() < scadenza:
            try:
                blocco = self.coda.get(timeout=0.2)
            except queue.Empty:
                continue
            livello = self._rms(blocco)
            if livello > self.soglia:
                primo_blocco = blocco
                break
            preroll.append(blocco)
            self._aggiorna_rumore(livello)

        if primo_blocco is None:
            return None

        # Fase 2: registrazione fino alla pausa di fine frase.
        frames = list(preroll) + [primo_blocco]
        silenzio = 0.0
        limite = time.monotonic() + durata_massima
        while time.monotonic() < limite:
            try:
                blocco = self.coda.get(timeout=0.2)
            except queue.Empty:
                continue
            frames.append(blocco)
            if self._rms(blocco) > self.soglia:
                silenzio = 0.0
            else:
                silenzio += self.durata_blocco
                if silenzio >= self.pausa_fine_frase:
                    break

        return np.concatenate(frames)


class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')
        self.get_logger().info('🧠 Inizializzazione Whisper STT su GPU Apple (MLX)...')

        self.declare_parameter('whisper_repo', 'mlx-community/whisper-small-mlx')
        self.declare_parameter('lingua', 'it')
        self.declare_parameter('coda_eco', 1.0)
        self.declare_parameter('pausa_fine_frase', 0.6)
        self.declare_parameter('silenzio_iniziale', 0.4)
        self.declare_parameter('moltiplicatore_soglia', 3.5)

        self.whisper_repo = self.get_parameter('whisper_repo').value
        self.lingua = self.get_parameter('lingua').value
        self.coda_eco = self.get_parameter('coda_eco').value

        self.text_pub = self.create_publisher(String, '/user/voice_text', 10)
        self.status_sub = self.create_subscription(
            Bool, '/robot/tts_status', self.on_tts_status, 10)

        self.is_robot_speaking = False
        # Istante prima del quale non si riprende ad ascoltare: lascia decadere
        # la coda dell'eco senza bloccare l'executor con una sleep.
        self.riprendi_ascolto_dopo = 0.0

        self.microfono = AscoltatoreMicrofono(
            sample_rate=16000,
            pausa_fine_frase=self.get_parameter('pausa_fine_frase').value,
            silenzio_iniziale=self.get_parameter('silenzio_iniziale').value,
            moltiplicatore_soglia=self.get_parameter('moltiplicatore_soglia').value)

        # Scalda il modello ora, non alla prima frase dell'utente: la prima
        # invocazione scarica i pesi e compila i kernel Metal.
        self.precarica_modello()

        self.listening = True
        self.thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.thread.start()

    def precarica_modello(self):
        try:
            silenzio = np.zeros(16000, dtype=np.float32)
            mlx_whisper.transcribe(silenzio, path_or_hf_repo=self.whisper_repo,
                                   language=self.lingua, verbose=None)
            self.get_logger().info(
                f'✅ Modello "{self.whisper_repo}" caricato sulla GPU del Mac!')
        except Exception as e:
            self.get_logger().error(f'🔥 Impossibile caricare il modello MLX: {e}')

    def on_tts_status(self, msg):
        if msg.data:
            self.is_robot_speaking = True
            self.get_logger().info('🔴 D.O.C. sta parlando, ignoro input...')
        else:
            # Niente sleep dentro il callback: bloccherebbe l'intero executor.
            # Il thread di ascolto rispetta da solo la finestra di silenzio.
            self.riprendi_ascolto_dopo = time.monotonic() + self.coda_eco
            self.is_robot_speaking = False
            self.get_logger().info('🟢 D.O.C. ha finito. In ascolto...')

    def trascrivi(self, audio_np):
        risultato = mlx_whisper.transcribe(
            audio_np,
            path_or_hf_repo=self.whisper_repo,
            language=self.lingua,
            condition_on_previous_text=False,
            # Soglie di rigetto: se il segmento non sembra parlato, o se il
            # testo prodotto e' troppo compresso (segno di ripetizioni), il
            # risultato viene scartato dal decoder stesso.
            no_speech_threshold=0.6,
            logprob_threshold=-1.0,
            compression_ratio_threshold=2.2,
            verbose=None,
        )
        return risultato.get("text", "").strip()

    def testo_utile(self, testo):
        """Scarta le allucinazioni note, ma non le risposte brevi.

        Attenzione alla lunghezza minima: le risposte piu' importanti della
        conversazione sono cortissime — "m3", "si", "no", "ok". Il vecchio
        limite di tre caratteri le buttava via tutte, ed e' il motivo per cui
        rispondere "m3" alla domanda sulla misura non produceva nulla. Il
        silenzio ora e' filtrato a monte sull'energia del segnale, non sulla
        lunghezza del testo.
        """
        if not testo or len(testo.strip()) < 2:
            return False

        pulito = testo.lower().strip(" .,!?;:'\"")
        for rumore in RUMORE_NOTO:
            if rumore in pulito:
                self.get_logger().info(f'🗑️ Scartata allucinazione: "{testo}"')
                return False

        # Una trascrizione fatta di sola punteggiatura o di un solo carattere
        # ripetuto non e' parlato.
        if not any(c.isalnum() for c in pulito):
            return False

        # Whisper, sul rumore, entra in cicli: "pennangela" ripetuto 25 volte.
        # Poche parole distinte su molte totali e' la firma di quel guasto.
        parole = pulito.split()
        if len(parole) >= 6 and len(set(parole)) / len(parole) < 0.35:
            self.get_logger().info(
                f'🗑️ Scartata ripetizione anomala: "{testo[:60]}..."')
            return False

        return True

    def listen_loop(self):
        try:
            self.microfono.avvia()
        except Exception as e:
            self.get_logger().error(
                f'🔥 Microfono non accessibile: {e}\n'
                '   Su macOS concedi il permesso in Impostazioni di Sistema -> '
                'Privacy e sicurezza -> Microfono al terminale che avvia il nodo.')
            return

        try:
            self.get_logger().info('Calibrazione rumore di fondo in corso...')
            soglia = self.microfono.calibra(durata=2.0)
            self.get_logger().info(
                f'🎙️ D.O.C. STT Node operativo (soglia {soglia:.4f}) e in ascolto!')

            while rclpy.ok() and self.listening:
                # SE È ROSSO, SI FERMA QUI E NON REGISTRA NULLA
                if self.is_robot_speaking or time.monotonic() < self.riprendi_ascolto_dopo:
                    time.sleep(0.1)
                    self.microfono.svuota()
                    continue

                try:
                    audio_np = self.microfono.ascolta(timeout=3.0, durata_massima=10.0)
                    if audio_np is None:
                        continue

                    # Controllo di sicurezza: se D.O.C. ha iniziato a parlare
                    # mentre registravi, quello che hai in mano e' la sua eco.
                    if self.is_robot_speaking:
                        self.get_logger().info("🗑️ (Trascrizione annullata: eco intercettato)")
                        continue

                    # Filtro a monte: e' inutile e dannoso far trascrivere a
                    # Whisper un frammento che non contiene parlato.
                    durata = len(audio_np) / 16000.0
                    energia = float(np.sqrt(np.mean(np.square(audio_np))))
                    if durata < DURATA_MINIMA or energia < ENERGIA_MINIMA:
                        self.get_logger().debug(
                            f"(scartato: {durata:.2f}s, energia {energia:.4f})")
                        continue

                    self.get_logger().info("⚙️ Trascrizione MLX in corso...")
                    testo = self.trascrivi(audio_np)

                    if not self.testo_utile(testo):
                        continue

                    self.get_logger().info(f'🗣️ Hai detto: "{testo}"')
                    msg = String()
                    msg.data = testo.lower()
                    self.text_pub.publish(msg)

                except Exception as e:
                    self.get_logger().error(f"🔥 ERRORE FATALE: {e}")
        finally:
            self.microfono.ferma()

    def destroy_node(self):
        self.listening = False
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        self.microfono.ferma()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
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
