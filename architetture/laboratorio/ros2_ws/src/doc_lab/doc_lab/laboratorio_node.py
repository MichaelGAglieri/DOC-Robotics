# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
"""Coordinamento campioni; il controller Webots resta l'autorità del moto."""
import json
import os
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

from .knowledge import Knowledge
from .webots_client import WebotsClient
from .planner import MissionPlanner, positions
from .missions import MissionDirector


class LaboratorioNode(Node):
    def __init__(self):
        super().__init__('laboratorio_node')
        default = str(Path(get_package_share_directory('doc_lab')) / 'config/base_conoscenza.json')
        for name, value in [('base_conoscenza', default), ('command_port', 9999),
                            ('feedback_port', 9998), ('con_llm', False), ('timeout_movimento', 90.0),
                            ('registro_missioni', os.getenv('DOC_MISSIONS_DIR', str(Path.home() / '.local/share/doc_lab/missioni')))]:
            self.declare_parameter(name, value)
        self.knowledge = Knowledge(self.get_parameter('base_conoscenza').value)
        self.client = WebotsClient(self.get_parameter('command_port').value,
                                   self.get_parameter('feedback_port').value)
        self.active = False
        self.scene = None
        self.last_seen = 0.0
        self.speech = self.create_publisher(String, '/laboratorio/risposta', 10)
        self.result = self.create_publisher(String, '/laboratorio/esito', 10)
        self.state = self.create_publisher(String, '/laboratorio/stato', 10)
        self.llm = self.create_publisher(String, '/laboratorio/llm_prompt', 10)
        self.mission_pub = self.create_publisher(String, '/laboratorio/missione',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        protocols = Path(get_package_share_directory('doc_lab')) / 'config/protocolli.json'
        self.missions = MissionDirector(self, MissionPlanner(self.knowledge, protocols),
                                        self.get_parameter('registro_missioni').value)
        self.create_subscription(String, '/laboratorio/testo', self.on_text, 10)
        self.create_subscription(String, '/laboratorio/richiesta', self.on_request, 10)
        self.create_timer(0.02, self.client.poll)
        self.create_timer(2.0, self.refresh)
        self.get_logger().info('Laboratorio pronto: catalogo JSON, 5 assi, controller Webots su loopback.')

    def say(self, text):
        self.speech.publish(String(data=text))

    def finish(self, status, message, extra=None):
        self.active = False
        if self.missions.running:
            self.missions.step_result(status, message, extra)
            return
        data = {'status': status, 'message': message}
        if extra is not None:
            data['feedback'] = extra
        self.result.publish(String(data=json.dumps(data, ensure_ascii=False)))
        self.say(message)

    def mission_event(self, event):
        self.mission_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))
        self.result.publish(String(data=json.dumps({key: value for key, value in event.items()
            if key not in ('plan', 'step')}, ensure_ascii=False)))
        if event['status'] not in ('RUNNING', 'STEP_COMPLETED', 'PLANNING', 'VALIDATING'):
            self.say(event['message'])

    def inspect_scene(self, callback):
        if self.active:
            raise ValueError('Una richiesta è già in corso.')
        self.active = True

        def diagnosis(reply):
            self.active = False
            try:
                self.remember(reply)
            except (ValueError, KeyError, TypeError) as exc:
                callback(None, str(exc))
                return
            callback(reply, None)

        def ping(reply):
            try:
                self.knowledge.validate_scene(reply)
                if reply.get('busy') is not False:
                    raise ValueError('Controller occupato. Attendi prima di pianificare o eseguire.')
                self.client.send('DIAGNOSI', None, diagnosis)
            except (ValueError, OSError) as exc:
                self.active = False
                callback(None, str(exc))
        try:
            self.client.send('PING', None, ping)
        except OSError as exc:
            self.active = False
            callback(None, str(exc))

    def remember(self, response):
        self.knowledge.validate_scene(response)
        self.scene = response
        self.last_seen = time.monotonic()
        self.state.publish(String(data=json.dumps(response, ensure_ascii=False)))

    def refresh(self):
        if not self.active and not self.client.pending:
            self.client.send('DIAGNOSI', None, self.on_refresh)

    def on_refresh(self, response):
        try:
            self.remember(response)
        except ValueError as error:
            self.scene = None
            self.state.publish(String(data=json.dumps({'status': 'UNAVAILABLE', 'reason': str(error)})))

    def on_text(self, message):
        try:
            if self.missions.handle(message.data):
                return
            request = self.knowledge.parse(message.data)
            if request:
                self.request(request)
            elif self.get_parameter('con_llm').value:
                self.llm.publish(String(data=message.data))
            else:
                self.say('Comandi: stato; sposta campione a in postazione 1; pianifica ciclo completo; esegui piano; stop.')
        except (ValueError, TypeError) as error:
            self.reject(str(error))

    def on_request(self, message):
        try:
            self.request(json.loads(message.data))
        except (ValueError, TypeError) as error:
            self.reject(str(error))

    def reject(self, reason):
        self.result.publish(String(data=json.dumps({'status': 'REJECTED', 'message': reason})))
        self.say(reason)

    def request(self, raw, from_mission=False, expected_positions=None):
        request = self.knowledge.validate_request(raw)
        action = request['action']
        if action == 'ABORT':
            had_plan = self.missions.prepared is not None
            self.missions.invalidate()
            if had_plan and not self.missions.running:
                self.missions.emit('CANCELLED', 'Piano annullato dalla richiesta di arresto.')
            # Non inviare ABORT al controller della cassettiera per errore.
            if self.scene is None or time.monotonic() - self.last_seen > 100:
                raise ValueError('Laboratorio non identificato. Usare SPAZIO nella finestra Webots.')
            self.client.cancel_callbacks()
            self.active = True
            self.client.send('ABORT', None, self.on_abort)
            return
        if self.missions.running and not from_mission:
            raise ValueError('Missione in corso: attendere oppure usare stop.')
        if expected_positions is not None:
            request['_expected_positions'] = expected_positions
        if self.active:
            raise ValueError('Richiesta in corso: attendere oppure usare stop.')
        self.active = True
        # PING identifica scenario e stato occupato prima di qualsiasi moto.
        self.client.send('PING', None, lambda reply: self.on_ping(request, reply))

    def on_ping(self, request, reply):
        try:
            self.knowledge.validate_scene(reply)
            if request['action'] == 'TRANSFER' and reply.get('busy') is not False:
                raise ValueError('Webots occupato: attendere il completamento.')
            self.client.send('DIAGNOSI', None, lambda data: self.on_diagnosis(request, data))
        except ValueError as error:
            self.finish('REJECTED', str(error), reply)

    def on_diagnosis(self, request, reply):
        try:
            self.remember(reply)
            if request['action'] == 'DIAGNOSI':
                positions = ', '.join(f"{name}: {value['station']}" for name, value in reply['samples'].items())
                self.finish('DONE', positions, reply)
                return
            self.knowledge.validate_transfer(request, reply)
            expected = request.get('_expected_positions')
            if expected is not None and positions(reply) != expected:
                raise ValueError('Scena modificata durante la missione: piano interrotto prima del prossimo movimento.')
            self.client.send('TRANSFER', request['params'], lambda data: self.on_transfer(request, data),
                             self.get_parameter('timeout_movimento').value)
        except (ValueError, KeyError, TypeError) as error:
            self.finish('REJECTED', str(error), reply)

    def on_transfer(self, request, reply):
        if reply.get('status') != 'DONE':
            self.scene = None
            self.finish(reply.get('status', 'UNCONFIRMED'), reply.get('reason', 'Trasferimento fallito.'), reply)
            return
        try:
            self.knowledge.validate_completed(request, reply)
            self.remember(reply)
            self.finish('DONE', 'Trasferimento completato nel simulatore e ritorno in home confermato.', reply)
        except (ValueError, TypeError) as error:
            self.scene = None
            self.finish('UNCONFIRMED', str(error), reply)

    def on_abort(self, reply):
        self.scene = None
        if reply.get('status') == 'DONE':
            self.finish('STOPPED', 'Simulazione arrestata. Premi R in Webots per ripristinare la scena.', reply)
        else:
            self.finish('UNCONFIRMED', 'Arresto non confermato: usa SPAZIO nella finestra Webots.', reply)

    def destroy_node(self):
        self.missions.shutdown()
        self.client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaboratorioNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
