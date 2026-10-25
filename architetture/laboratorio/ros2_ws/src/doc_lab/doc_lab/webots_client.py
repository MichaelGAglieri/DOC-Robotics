# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
"""Protocollo del client demo esistente, reso non bloccante per i callback ROS.

Nessuna ritrasmissione automatica dei movimenti. Nessun accesso seriale.
"""
import json
import socket
import time
import uuid


class WebotsClient:
    def __init__(self, command_port=9999, feedback_port=9998):
        self.target = ('127.0.0.1', command_port)
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.rx.bind(('127.0.0.1', feedback_port))
        except BaseException:
            self.rx.close()
            raise
        self.rx.setblocking(False)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pending = {}

    def send(self, action, params, callback, timeout=5.0):
        cid = uuid.uuid4().hex
        self.tx.sendto(json.dumps({'id': cid, 'action': action, 'params': params}).encode(), self.target)
        self.pending[cid] = (time.monotonic() + timeout, callback, action)
        return cid

    def cancel_callbacks(self):
        # Non ferma Webots: serve a impedire che un preflight già in viaggio
        # accodi un trasferimento dopo una richiesta di arresto.
        self.pending.clear()

    def poll(self):
        for _ in range(100):
            try:
                raw, sender = self.rx.recvfrom(65535)
            except BlockingIOError:
                break
            if sender != self.target:
                continue
            try:
                result = json.loads(raw)
            except (ValueError, UnicodeDecodeError):
                continue
            if not isinstance(result, dict) or not isinstance(result.get('id'), str):
                continue
            item = self.pending.get(result['id'])
            if item is None or result.get('action') != item[2]:
                continue
            self.pending.pop(result['id'])
            item[1](result)
        now = time.monotonic()
        for cid, (deadline, callback, action) in list(self.pending.items()):
            if now >= deadline and cid in self.pending:
                self.pending.pop(cid)
                callback({'id': cid, 'action': action, 'status': 'UNCONFIRMED',
                          'reason': 'Timeout: esito sconosciuto. Nessun movimento ritrasmesso.'})

    def close(self):
        self.pending.clear()
        self.rx.close()
        self.tx.close()
