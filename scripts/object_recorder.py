#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import sqlite3
import threading
import queue
from ament_index_python.packages import get_package_share_directory
import os


class ObjectRecorder(Node):
    def __init__(self):
        super().__init__('object_recorder')
        self.subscription = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.current_position = None
        self.lock = threading.Lock()

        # Conectar a la base de datos existente
          
        bringup_dir = get_package_share_directory('turtlemart')
        db_dir = os.path.join( 'src/turtlemart/database/products.db')
        self.database_connection = sqlite3.connect(db_dir)
        self.cursor = self.database_connection.cursor()

        self.get_logger().info('Object Recorder Node Initialized.')

    def odom_callback(self, msg):
        # Actualiza la posición actual del robot cada vez que llega un mensaje de odometría
        with self.lock:
            self.current_position = msg.pose.pose.position

    def save_object_position(self, object_name):
        with self.lock:
            if self.current_position:
                x = self.current_position.x
                y = self.current_position.y
                # Inserta el nombre del objeto y su posición en la base de datos
                self.cursor.execute("INSERT INTO products (name, x, y) VALUES (?, ?, ?)", (object_name, x, y))
                self.database_connection.commit()
                message = f'Object "{object_name}" recorded at position ({x}, {y}).'
                self.get_logger().info(message)
                return x, y
            else:
                self.get_logger().warning('No position data available.')
                return None, None

    def close_database(self):
        self.database_connection.close()

def user_input_thread(request_queue, response_queue):
    try:
        while rclpy.ok():
            action = input("Presione 's' para guardar la posición del objeto, o 'n' para salir: ").lower()
            if action == 's':
                object_name = input("Ingrese nombre del objeto: ")
                request_queue.put(('save', object_name))
                while True:
                    response = response_queue.get()  # Espera la respuesta del nodo
                    if response[0] == 'saved':
                        x, y = response[1], response[2]
                        choice = input(f'El objeto "{object_name}" se ha registrado en la posición ({x}, {y}). ¿Desea agregar otro objeto? (y/n): ').lower()
                        if choice == 'y':
                            break
                        elif choice == 'n':
                            request_queue.put(('shutdown', None))
                            return
                        else:
                            print("Opción no válida. Por favor, ingrese 'y' o 'n'.")
                    elif response[0] == 'error':
                        print("No se pudo registrar la posición del objeto.")
                        break
            elif action == 'n':
                request_queue.put(('shutdown', None))
                return
            else:
                print("Opción no válida. Por favor, ingrese 's' o 'n'.")
    except KeyboardInterrupt:
        request_queue.put(('shutdown', None))

def main(args=None):
    rclpy.init(args=args)
    node = ObjectRecorder()

    request_queue = queue.Queue()
    response_queue = queue.Queue()
    input_thread = threading.Thread(target=user_input_thread, args=(request_queue, response_queue))
    input_thread.start()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            try:
                request = request_queue.get_nowait()
            except queue.Empty:
                continue

            if request[0] == 'save':
                x, y = node.save_object_position(request[1])
                if x is not None and y is not None:
                    response_queue.put(('saved', x, y))
                else:
                    response_queue.put(('error', None))
            elif request[0] == 'shutdown':
                rclpy.shutdown()
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.close_database()
        node.destroy_node()
        input_thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
