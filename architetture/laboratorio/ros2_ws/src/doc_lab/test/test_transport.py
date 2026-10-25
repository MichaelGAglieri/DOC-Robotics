# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
import json
import socket
import time
from doc_lab.webots_client import WebotsClient


def test_correlated_reply_unknown_id_and_timeout_without_retry():
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(('127.0.0.1', 0))
    client = WebotsClient(server.getsockname()[1], 0)
    received = []
    try:
        cid = client.send('TRANSFER', {'sample': 'campione_a', 'destination': 'postazione_1'}, received.append)
        data = json.loads(server.recvfrom(65535)[0])
        assert data['id'] == cid
        target = client.rx.getsockname()
        server.sendto(b'{bad json', target)
        server.sendto(json.dumps({'id': 'wrong', 'action': 'TRANSFER', 'status': 'DONE'}).encode(), target)
        server.sendto(json.dumps({'id': cid, 'action': 'DIAGNOSI', 'status': 'DONE'}).encode(), target)
        client.poll()
        assert received == [] and cid in client.pending
        server.sendto(json.dumps({'id': cid, 'action': 'TRANSFER', 'status': 'DONE'}).encode(), target)
        end = time.monotonic() + 1
        while not received and time.monotonic() < end:
            client.poll()
        assert received[0]['status'] == 'DONE'
        client.send('TRANSFER', {}, received.append, timeout=0)
        server.recvfrom(65535)
        client.poll()
        assert received[-1]['status'] == 'UNCONFIRMED'
        server.settimeout(.05)
        try:
            server.recvfrom(65535)
        except socket.timeout:
            pass
        else:
            raise AssertionError('Un trasferimento è stato ritrasmesso')
    finally:
        client.close()
        server.close()


def test_cancel_preflight_ignores_late_reply():
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(('127.0.0.1', 0))
    client = WebotsClient(server.getsockname()[1], 0)
    results = []
    try:
        cid = client.send('PING', None, results.append)
        server.recvfrom(65535)
        client.cancel_callbacks()
        server.sendto(json.dumps({'id': cid, 'action': 'PING', 'status': 'DONE'}).encode(), client.rx.getsockname())
        client.poll()
        assert results == [] and client.pending == {}
    finally:
        client.close()
        server.close()
