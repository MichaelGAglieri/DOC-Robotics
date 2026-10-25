# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
"""Monitor leggibile delle missioni: piano, avanzamento e percorso del report."""
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)
    node = Node('monitor_missioni')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def show(message):
        try:
            event = json.loads(message.data)
        except ValueError:
            return
        print(f"\n[{event.get('status', '?')}] {event.get('message', '')}", flush=True)
        plan = event.get('plan')
        if plan:
            print(f"  {plan['title']} · piano {plan['plan_id']}")
            for step in plan['steps']:
                temporary = ' [appoggio temporaneo]' if step['temporary'] else ''
                print(f"  {step['index']}. {step['sample']}: {step['source']} → {step['destination']}{temporary}")
                print(f"     {step['phase']} — {step['reason']}")
            if not plan['steps']:
                print('  Gli obiettivi risultano già raggiunti.')
        if event.get('status') in ('COMPLETED', 'FAILED', 'STOPPED', 'UNCONFIRMED', 'JOURNAL_ERROR', 'INTERRUPTED') and event.get('report_path'):
            print('  Report:', event['report_path'])
        print('', flush=True)
    subscription = node.create_subscription(String, '/laboratorio/missione', show, qos)
    print('MISSIONI DOC · pianifica nella console, verifica qui i passaggi, poi esegui_piano.', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
