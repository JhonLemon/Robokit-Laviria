#!/usr/bin/env python3 

import rclpy 
from rclpy.node import Node #heredar atributos de nodos de ros
import tkinter as tk #crear interfaces 
import threading #Necesario para  multihilos
import serial 

#--- CLASE PARA GESTION DE COMUNICACION CON ARDUINO
class CombinedNode(Node):

    def __init__(self):
        #llamada al constructor (Node) para asignacion de nombre (control combinado)
        super().__init__("combined_control_node")
        
        #- Configurcion de puerto serial
        try:
            #inicia la conexion (MODIFICAR SEGUN EL CASO)
            self.ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=0.1)
            #Mmensaje de aprobacion 
            self.get_logger().info("Conexion Serial ESTABLECIDA")
        
        except Exception as e:
            #Mensaje de error
            self.get_logger().error(f"Error abriendo el puerto Serial: {e}")
            self.ser = None

        #- Tmer para leer respuesta del ARDUINO
        #necesario para escuchar las instrucciones de encoders cada 0.1 s
        self.timer = self.create_timer(0.1, self.read_serial_timer)

#- Funcion de escritura al ARDUINO (directamente por cable, sin usar topicos de ros)

    def send_to_arduino(self, text):
        # Depuracion,se muestra que dato se envia por la terminal
        self.get_logger().info(f"Sending: {text}")
        
        #verificacion de: 1-que exista ek objeto SERIAL 
        #2- que el pueto este disponible
        if self.ser and self.ser.is_open:
            #retorno de carro para identificar el final de una instruccion
            cmd = text + '\r'
            # string a bytes (UTF-8) y escribe FISICAMENTE por el cable USB
            self.ser.write(cmd.encode('utf-8'))
        else:
            #advertencia si el puerto esta ocupado
            self.get_logger().warning("Error")

#- Funcion de lectura al ARDUINO 

    def read_serial_timer(self):
        #Verifica con in_waiting que haya info en el buffer
        if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
            
            try:
                #lectura hasta salto de linea, conversion de bytes a texto, errors para omitir basura, strip eliminaespacios
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                
                if line: # si no esta vacia
                    #mensaje de arduino formato ros
                    self.get_logger().info(f"[ARDUINO]: {line}")
            
            except Exception: #en caso de una DESCONEXION 
                pass


#-- CLASE INTERFAZ GRAFICA (logica similar para joysticks)
#Graphical User Interface GUI
class RobotControlGUI(tk.Tk):
    #el constructor recibe ros_node que es la instancia de CombinedNode 
    def __init__(self, ros_node):
        #inicia la ventana
        super().__init__()
        #guarda la referencia al nodo para llamarlo
        self.ros_node = ros_node
        
        #- Configuración BASICA de la ventana
        self.title("ControlCombined DEMO - ROBOT SS")
        self.geometry("400x300")
        
        #inicializacion de cualquier instruccion de velocidad a 30 RPM
        self.current_rpm = tk.IntVar(value=30)
        
        #llamada a ala funcion para widgets de la pantalla
        self.create_widgets()

#- Funcion para creacion de widgets

    def create_widgets(self):
    #--- SECCIÓN DE VELOCIDAD ---
        #recuadro, margen de 20
        speed_frame = tk.Frame(self)
        speed_frame.pack(pady=20)
        
        #Etiqueta de texto 
        tk.Label(speed_frame, text="Vel.(RPM):", font=("Arial", 12)).pack(side=tk.LEFT)
        #scale- slider de 0 a 120, con disposicion horizontal, se asocia a current_rpm``
        self.speed_slider = tk.Scale(speed_frame, from_=0, to=120, orient=tk.HORIZONTAL, variable=self.current_rpm, length=200)
        #alinear a la izquierda
        self.speed_slider.pack(side=tk.LEFT, padx=10)
        
    #--- SECCIÓN DE DIRECCIÓN (D-PAD) ---
        #ontenedor para organizar los botones en forma de cruz (joystick)
        dpad_frame = tk.Frame(self)
        dpad_frame.pack(pady=10)
        
        #- BOTON ADELANTE (Fila 0, Columna central)
        #'command=self.move_forward' para asociar a su funcion de deplaazammiento
        btn_up = tk.Button(dpad_frame, text="▲ Adelante", width=12, height=2, command=self.move_forward)
        #posicionamiento en la cuadricula
        btn_up.grid(row=0, column=1, pady=5)
        
        #- BOTON IZQUIERDA (Fila 1, Columna izquierda)
        #'command=elf.turn_left' para asociar a su funcion de deplaazammiento
        btn_left = tk.Button(dpad_frame, text="◀ Giro CCW", width=15, height=2, command=self.turn_left)
        #posicionamiento en cuadricula
        btn_left.grid(row=1, column=0, padx=5)
        
        #- BOTON STOP Fila 1, Columna central)
        #de color rojo y letras blancas
        btn_stop = tk.Button(dpad_frame, text="STOP", width=12, height=2, bg="red", fg="white", font=("Arial", 10, "bold"), command=self.stop_robot)
        #posicioamiento en cuadricula
        btn_stop.grid(row=1, column=1, padx=5)
        
        #- BOToN DERECHA (Fila 1, Columna derecha)
        #'command=self.turn_right' para para asociar a su funcion de deplaazammiento
        btn_right = tk.Button(dpad_frame, text="Giro CW ▶", width=15, height=2, command=self.turn_right)
        #Pposicionamiento en cuadricula
        btn_right.grid(row=1, column=2, padx=5)
        
        #- BOTON ATRAS (Fila 2, Columna central)
        #'command=self.move_backward' para para asociar a su funcion de deplaazammiento
        btn_down = tk.Button(dpad_frame, text="▼ Atras", width=12, height=2, command=self.move_backward)
        #posicionamiento en cuadricula
        btn_down.grid(row=2, column=1, pady=5)

    #--- LOGICA BASICA DE MOVIMIENTO ---
    #FUNCIONES PARA CONTRUIR LOS STRING QUE RECIBIRA EL ARDUINO
    
#- Forward
    def move_forward(self):
        rpm = self.current_rpm.get() #valor actual del slider
        #llamada a la funcion del nodo ros para enviar el comando 
        #ambos motores al mismo rpm
        self.ros_node.send_to_arduino(f"m {rpm} {rpm}")
#-Backward
    def move_backward(self):
        rpm = self.current_rpm.get()#valor actual del slider
        #llamada a la funcion del nodo ros para enviar el comando 
        #ambos motores al mismo rpm
        self.ros_node.send_to_arduino(f"m {-rpm} {-rpm}")
#- CCW
    def turn_left(self):
        rpm = self.current_rpm.get()#valor actual del slider(a ambos motores)
        #llamada a la funcion del nodo ros para enviar el comando 
        #ambos motores al mismo rpm pero sentidos opuestos
        self.ros_node.send_to_arduino(f"m {-rpm} {rpm}")
#- CW
    def turn_right(self):
        rpm = self.current_rpm.get()#valor actual del slider
        #llamada a la funcion del nodo ros para enviar el comando 
        #ambos motores al mismo rpm pero sentidos opuestos 
        self.ros_node.send_to_arduino(f"m {rpm} {-rpm}")
#- STOP
    def stop_robot(self):
        #comando stop establecido en ARDUINO
        self.ros_node.send_to_arduino("s")



def main(args=None):
    #inicializacion de comunicaciones internas de ROS2
    rclpy.init(args=args)
    #instancia a la clase que maneja la conexion serial y ros
    node = CombinedNode()

    #Hilo secundario para que ROS pueda leer el puerto serial en el fondo
    #mientras dibuja la interfaz en un bucle infinito
    #'daemon=True' asegura que si se cierra la GUI este hilo secundario se cierre 
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start() #Arranca el hilo de ROS2 en el fondo
    
    #nstancia la GUI y le pasa el nodo de ROS para que puedan comunicarse
    app = RobotControlGUI(node)
    #Inicio del bucle infinito de la GUI. 
    #El codigo se detiene en esta línea hasta que el usuario cierre la ventana de Tkinter
    app.mainloop()
    

    node.destroy_node()
    rclpy.shutdown()

#instruccion para asegurar la ejecucion solo si el archivo es ejecutado en terminal
if __name__ == '__main__':
    main()