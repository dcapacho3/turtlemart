#!/usr/bin/env python3
"""
Nodo de teleoperación simplificado para tutorial SARA
Control con teclado que tiene prioridad sobre navegación automática
"""

import os
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopTutorial(Node):
    def __init__(self):
        super().__init__('teleop_tutorial')
        
        # Publicador a cmd_vel_teleop (para el mux)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        
        # Velocidades
        self.linear_speed = 0.2
        self.angular_speed = 1.0
        
        # Control de teclas
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)
        
        # Timer para publicar comandos
        self.timer = self.create_timer(0.1, self.publish_cmd)
        
        # Estado actual
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.keys_pressed = set()
        
        self.print_instructions()
        self.get_logger().info('Teleoperación tutorial iniciada - SARA')

    def print_instructions(self):
        instructions = """
        =================================
        TELEOPERACIÓN TUTORIAL SARA
        =================================
        
        Controles:
              w
         a    s    d
              x
        
        w/x : avanzar/retroceder
        a/d : girar izquierda/derecha  
        s   : parar
        
        CTRL-C para salir
        =================================
        """
        print(instructions)

    def get_key(self):
        """Capturar tecla presionada"""
        if os.name == 'nt':
            import msvcrt
            if msvcrt.kbhit():
                return msvcrt.getch().decode('utf-8')
            return ''
        
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def process_key(self, key):
        """Procesar tecla presionada"""
        if key == 'w':
            self.linear_vel = self.linear_speed
            self.get_logger().info('Avanzando')
        elif key == 'x':
            self.linear_vel = -self.linear_speed
            self.get_logger().info('Retrocediendo')
        elif key == 'a':
            self.angular_vel = self.angular_speed
            self.get_logger().info('Girando izquierda')
        elif key == 'd':
            self.angular_vel = -self.angular_speed
            self.get_logger().info('Girando derecha')
        elif key == 's':
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            self.get_logger().info('Detenido')

    def publish_cmd(self):
        """Publicar comando de velocidad y capturar teclas"""
        # Capturar tecla
        key = self.get_key()
        if key:
            self.process_key(key)
        
        # Crear mensaje Twist
        twist = Twist()
        twist.linear.x = self.linear_vel
        twist.angular.z = self.angular_vel
        
        # IMPORTANTE: Solo publicar si hay movimiento (para que el mux funcione)
        if abs(self.linear_vel) > 0.01 or abs(self.angular_vel) > 0.01:
            twist.angular.y = 0.02  # Señal para twist_mux (prioridad teleoperación)
            self.cmd_vel_pub.publish(twist)
        
        # Decaimiento gradual (simular que se "suelta" la tecla)
        self.linear_vel *= 0.9
        self.angular_vel *= 0.9
        
        # Parar si muy lento
        if abs(self.linear_vel) < 0.01:
            self.linear_vel = 0.0
        if abs(self.angular_vel) < 0.01:
            self.angular_vel = 0.0

def main(args=None):
    rclpy.init(args=args)
    teleop = TeleopTutorial()
    
    try:
        rclpy.spin(teleop)
    except KeyboardInterrupt:
        print('\nTeleoperación finalizada')
    finally:
        # Restaurar configuración del terminal
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, teleop.settings)
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
