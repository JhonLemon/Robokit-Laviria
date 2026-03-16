#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node #heredar atributos de nodos de ros
from std_msgs.msg import String
import serial
import threading #Necesario para  multihilos (leer teclado sin congelar el nodo)

class SerialNode(Node):
    def __init__(self):
        super().__init__("serial_node_ros")
        
        #Configuración Serial
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=0.1)
            self.get_logger().info("Conexión Serial Establecida /dev/ttyUSB0")
        except Exception as e:
            self.get_logger().error(f"Error Serial: {e}")

        #Subscritor para recibir comandos de otros nodos
        self.subscription = self.create_subscription(String, '/serial_control', self.write_callback, 10)
        
        #Timer para leer lo que el Arduino imprime
        self.timer = self.create_timer(0.1, self.read_serial_timer)

        #HILO SECUNDARIO: función que pide teclado
        #permite que el comando no detenga el resto del programa
        self.thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.thread.start()

        self.get_logger().info("Nodo serial listo")

    def write_callback(self, msg):
        #EnvIa datos al Arduino cuando llegan por el topico
        self.send_to_arduino(msg.data)

    def send_to_arduino(self, text):
        #FunciOn interna para codificar y enviar
        if hasattr(self, 'ser') and self.ser.is_open:
            cmd = text + '\r'
            self.ser.write(cmd.encode('utf-8'))

    def read_serial_timer(self):
        #Lee lo que el Arduino envía y lo despliega
        if hasattr(self, 'ser') and self.ser.is_open and self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                self.get_logger().info(f"[ARDUINO]: {line}")

    def keyboard_listener(self):
        #FunciOn que corre en un hilo separado para no bloquear ROS
        while rclpy.ok():
            user_input = input() #Aquí sí usar input()
            self.send_to_arduino(user_input)

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    try:
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        pass
    finally:
        serial_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()