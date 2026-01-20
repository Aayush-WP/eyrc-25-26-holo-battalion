#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from hb_interfaces.srv import Attach
from hb_interfaces.msg import BotCmdArray
import paho.mqtt.client as mqtt
import json
import time


class MQTTBroker(Node):
    def __init__(self):
        super().__init__('mqtt_broker')

        # ---------------- MQTT CONFIG ----------------
        self.MQTT_HOST = "192.168.10.12"
        self.MQTT_PORT = 1883
        self.TOPIC_FMT = "hb/bot{}/cmd"

        self.client = mqtt.Client(client_id="ros_mqtt_broker")
        self.client.on_disconnect = self.on_disconnect
        self.connect_mqtt()

        # ---------------- STATE ----------------
        self.attach_state = {}    # per robot
        self.last_cmd = {}        # per robot

        # ---------------- ROS ----------------
        self.create_subscription(
            BotCmdArray,
            '/bot_cmd',
            self.bot_cmd_cb,
            10
        )

        self.create_service(Attach, '/attach', self.attach_cb)

        # ---------------- PER-BOT PICK / DROP CONFIG ----------------
        self.attach_config = {
            0: {
                "pick":  {"base": 20.0,  "elbow": 10.0},
                "drop":  {"base": 20.0,  "elbow": 10.0},
            },
            2: {
                "pick":  {"base": 20.0,  "elbow": 5.0},
                "drop":  {"base": 20.0,  "elbow": 5.0},
            },
            4: {
                "pick":  {"base": 20.0, "elbow": 10.0},
                "drop":  {"base": 20.0, "elbow": 10.0},
            }
        }


        self.get_logger().info("✅ MQTT Broker Ready (Multi-Robot)")

    # =====================================================
    def connect_mqtt(self):
        while True:
            try:
                self.client.connect(self.MQTT_HOST, self.MQTT_PORT, 60)
                self.client.loop_start()
                self.get_logger().info("MQTT connected")
                break
            except Exception:
                self.get_logger().warn("MQTT unreachable, retrying...")
                time.sleep(2)

    def on_disconnect(self, client, userdata, rc):
        self.get_logger().warn("MQTT disconnected — reconnecting")
        self.connect_mqtt()

    # =====================================================
    def attach_cb(self, request, response):
        bot_id = request.bot_id
        attach = request.attach

        self.attach_state[bot_id] = attach

        bot_cfg = self.attach_config.get(bot_id)

        if bot_cfg is None:
            # Safe fallback
            base = 0.0
            elbow = 65.0
        else:
            if attach:
                base  = bot_cfg["pick"]["base"]
                elbow = bot_cfg["pick"]["elbow"]
            else:
                base  = bot_cfg["drop"]["base"]
                elbow = bot_cfg["drop"]["elbow"]

        payload = {
            "m1": 0.0,
            "m2": 0.0,
            "m3": 0.0,
            "base": base,
            "elbow": elbow,
            "attach": int(attach)
        }

        topic = self.TOPIC_FMT.format(bot_id)
        self.client.publish(topic, json.dumps(payload), qos=0)

        self.get_logger().info(
            f"MQTT [{topic}] → {'PICK' if attach else 'DROP'} | base={base}, elbow={elbow}"
        )

        response.success = True
        response.message = f"{'Pick' if attach else 'Drop'} command sent to bot {bot_id}"
        return response



    # =====================================================
    def bot_cmd_cb(self, msg: BotCmdArray):
        for cmd in msg.cmds:
            bot_id = cmd.id

            self.last_cmd[bot_id] = cmd

            if bot_id not in self.attach_state:
                self.attach_state[bot_id] = False

            self.publish_to_mqtt(bot_id)

    # =====================================================
    def publish_to_mqtt(self, bot_id):
        cmd = self.last_cmd.get(bot_id, None)

        payload = {
            "m1": cmd.m1 if cmd else 0.0,
            "m2": cmd.m2 if cmd else 0.0,
            "m3": cmd.m3 if cmd else 0.0,
            "base": cmd.base if cmd else 0.0,
            "elbow": cmd.elbow if cmd else 10.0,
            "attach": int(self.attach_state.get(bot_id, False))
        }

        topic = self.TOPIC_FMT.format(bot_id)

        self.client.publish(topic, json.dumps(payload), qos=0)

        self.get_logger().debug(f"MQTT [{topic}] → {payload}")


def main():
    rclpy.init()
    node = MQTTBroker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
