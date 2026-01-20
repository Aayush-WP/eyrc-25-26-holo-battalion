#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from hb_interfaces.msg import BotCmdArray
import paho.mqtt.client as mqtt
import json
import time


class MQTTBroker(Node):
    def __init__(self):
        super().__init__('mqtt_broker')

        # -------- MQTT --------
        self.MQTT_HOST = "192.168.10.12"
        self.MQTT_PORT = 1883
        self.MQTT_TOPIC = "hb/bot0/cmd"

        self.client = mqtt.Client(client_id="ros_mqtt_broker")
        self.client.on_disconnect = self.on_disconnect
        self.connect_mqtt()

        # -------- State --------
        self.attach_state = False
        self.last_cmd = None

        # -------- ROS --------
        self.create_subscription(
            BotCmdArray,
            '/bot_cmd',
            self.bot_cmd_cb,
            10
        )

        self.create_service(
            SetBool,
            '/attach',
            self.attach_cb
        )

        self.get_logger().info("✅ MQTT Broker Node Ready")

    # ======================================================
    def connect_mqtt(self):
        while True:
            try:
                self.client.connect(self.MQTT_HOST, self.MQTT_PORT, 60)
                self.client.loop_start()
                self.get_logger().info("Connected to MQTT broker")
                break
            except Exception:
                self.get_logger().warn("MQTT broker unreachable, retrying...")
                time.sleep(2)

    def on_disconnect(self, client, userdata, rc):
        self.get_logger().warn("MQTT disconnected, reconnecting...")
        self.connect_mqtt()

    # ======================================================
    def attach_cb(self, request, response):
        self.attach_state = request.data
        response.success = True
        response.message = "Attached" if request.data else "Detached"

        # Republish last motion + new attach state
        self.publish_to_mqtt(self.last_cmd)
        return response

    # ======================================================
    def bot_cmd_cb(self, msg: BotCmdArray):
        for cmd in msg.cmds:
            if cmd.id !=2 :     #bot id
                continue
            self.last_cmd = cmd
            self.publish_to_mqtt(cmd)

    # ======================================================
    def publish_to_mqtt(self, cmd):
        payload = {
            "m1": cmd.m1 if cmd else 0.0,
            "m2": cmd.m2 if cmd else 0.0,
            "m3": cmd.m3 if cmd else 0.0,
            "base": cmd.base if cmd else 0.0,
            "elbow": cmd.elbow if cmd else 0.0,
            "attach": int(self.attach_state)
        }

        self.client.publish(
            self.MQTT_TOPIC,
            json.dumps(payload),
            qos=0
        )

        self.get_logger().info(f"MQTT → {payload}")


def main():
    rclpy.init()
    node = MQTTBroker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
