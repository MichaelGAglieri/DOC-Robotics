#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
"""D.O.C. - Cervello conversazionale: risposte discorsive tramite Ollama.

Gestisce tutto cio' che non e' un comando al braccio. Il modello e' un
parametro: il progetto e' nato con Gemma, ora usa Qwen3, e il nodo non fa
assunzioni sulla famiglia.

Due accorgimenti servono a restare compatibili con i modelli "thinking"
(Qwen3, DeepSeek-R1 e simili), che Ollama tratta in modo diverso:

  1. si chiede esplicitamente 'think: false', altrimenti il modello spende i
     token a ragionare e Ollama restituisce il ragionamento nel campo
     'thinking' lasciando 'response' VUOTO — il robot resterebbe muto;
  2. se 'response' arriva comunque vuoto, si ripiega su 'thinking', cosi' una
     risposta storta e' sempre meglio del silenzio.

Il prompt di sistema viaggia nel campo 'system' invece di essere accodato al
testo dell'utente: cosi' finisce nello slot giusto del template di chat del
modello, qualunque esso sia.
"""

import os
import queue
import re
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
import requests

MODELLO_PREDEFINITO = (
    'hf.co/HauhauCS/Qwen3-4B-2507-Instruct-Uncensored-HauhauCS-Aggressive:Q4_K_M')

PERSONALITA = (
    "Sei D.O.C., un braccio robotico (desktop operations companion) intelligente, "
    "assisti un utente umano per ricerche didattiche e attivita' da scrivania. "
    "Rispondi sempre in italiano, in modo simpatico accurato e preciso ma breve e "
    "conciso (massimo quattro frasi). Non usare emoji, emoticon, faccine ne' "
    "formattazione markdown: la tua risposta viene letta ad alta voce."
)

# Alcuni modelli emettono comunque il ragionamento in linea nel testo.
BLOCCO_RAGIONAMENTO = re.compile(r'<think>.*?</think>', re.DOTALL | re.IGNORECASE)


class LLMBrainNode(Node):
    def __init__(self):
        super().__init__('llm_brain_node')

        self.declare_parameter('ollama_url',
                               os.getenv('OLLAMA_URL',
                                         'http://localhost:11434/api/generate'))
        self.declare_parameter('modello_llm',
                               os.getenv('OLLAMA_MODEL', MODELLO_PREDEFINITO))
        self.declare_parameter('timeout_llm', 120.0)
        self.declare_parameter('ragionamento', False)

        self.ollama_url = self.get_parameter('ollama_url').value
        self.modello_llm = self.get_parameter('modello_llm').value
        self.timeout_llm = self.get_parameter('timeout_llm').value
        self.ragionamento = self.get_parameter('ragionamento').value

        self.subscription = self.create_subscription(
            String,
            '/llm_prompt',
            self.prompt_callback,
            10)

        self.tts_publisher = self.create_publisher(String, '/robot/speech', 10)

        # L'inferenza puo' durare decine di secondi: gestita su un thread
        # dedicato, cosi' il callback ROS resta immediato.
        self.coda_prompt = queue.Queue()
        self.attivo = True
        self.worker = threading.Thread(target=self.loop_inferenza, daemon=True)
        self.worker.start()

        self.get_logger().info(
            f'🧠 Cervello LLM avviato ({self.modello_llm}). '
            'In ascolto silenzioso su /llm_prompt...')

    def prompt_callback(self, msg):
        if msg.data.strip():
            self.coda_prompt.put(msg.data)

    def loop_inferenza(self):
        while self.attivo:
            try:
                prompt = self.coda_prompt.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self.gestisci_conversazione(prompt)
            finally:
                self.coda_prompt.task_done()

    def estrai_risposta(self, dati):
        """Ricava il testo da dire, qualunque campo il modello abbia riempito."""
        testo = (dati.get('response') or '').strip()

        if not testo:
            # Modello thinking che ha risposto solo nel canale di ragionamento.
            testo = (dati.get('thinking') or '').strip()
            if testo:
                self.get_logger().warn(
                    "⚠️ Il modello ha risposto solo nel campo 'thinking': "
                    "uso quello. Valuta 'ragionamento:=false'.")

        return BLOCCO_RAGIONAMENTO.sub('', testo).strip()

    def gestisci_conversazione(self, prompt):
        """Interroga il modello tramite Ollama per risposte discorsive"""
        self.get_logger().info(f"💬 Domanda ricevuta. Interrogo {self.modello_llm}...")

        payload = {
            "model": self.modello_llm,
            "system": PERSONALITA,
            "prompt": prompt,
            "stream": False,   # Vogliamo la risposta tutta intera
            "think": self.ragionamento,
        }

        try:
            response = requests.post(self.ollama_url, json=payload,
                                     timeout=self.timeout_llm)

            if response.status_code == 200:
                risposta = self.estrai_risposta(response.json())
                if risposta:
                    self.get_logger().info(f"Il modello risponde: {risposta}")
                    self.parla(risposta)
                else:
                    self.get_logger().error(
                        "❌ Il modello ha restituito una risposta vuota.")
                    self.parla("Non ho una risposta da darti su questo.")
            else:
                self.get_logger().error(
                    f"Ollama ha restituito l'errore HTTP {response.status_code}: "
                    f"{response.text}")
                self.parla("Il mio modello linguistico ha risposto con un errore.")

        except requests.exceptions.ConnectionError:
            self.get_logger().error("❌ Impossibile connettersi a Ollama. Assicurati che 'ollama serve' sia in esecuzione.")
            self.parla("Non riesco a raggiungere il mio modello linguistico.")
        except requests.exceptions.Timeout:
            self.get_logger().error("❌ Timeout nell'attesa della risposta di Ollama.")
            self.parla("Ci sto mettendo troppo a pensare, riprova.")
        except Exception as e:
            self.get_logger().error(f"❌ Errore imprevisto di sistema: {e}")

    def parla(self, testo):
        """Pubblica la stringa sul topic dell'audio"""
        msg = String()
        msg.data = testo
        self.tts_publisher.publish(msg)

    def destroy_node(self):
        self.attivo = False
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = LLMBrainNode()
    executor = MultiThreadedExecutor()
    executor.add_node(nodo)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
