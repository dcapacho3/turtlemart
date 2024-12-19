#!/usr/bin/env python3

import os
import sqlite3
import customtkinter as ctk
import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from PIL import Image
import numpy as np
import yaml
import threading
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped 
from nav_msgs.msg import Odometry
from robot_navigator import BasicNavigator, NavigationResult

from ament_index_python.packages import get_package_share_directory
from real_guiwaypoint import AutonomousNavigator
from std_msgs.msg import String
import signal
import subprocess
import time
import math
import matplotlib.patches as patches
from matplotlib import transforms
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from PIL import ImageOps 
import matplotlib.transforms as mtransforms


from finishgui import ThanksWindow

class RealNavWindow(ctk.CTk):
    def __init__(self, product_manager):
        super().__init__()
        #self.master = master
        self.after_id = None
        
         # Configuración de la interfaz
        self.product_manager = product_manager
         
       
        self.title("Smart Autonomous Retail Assistant")
        self.geometry("%dx%d+0+0" % (self.winfo_screenwidth(), self.winfo_screenheight()))
        self.resizable(width=1, height=1)

        # Inicializar la lista de productos seleccionados
        self.selected_products = []

        self.initial_pose = None  # To store the starting x, y, and yaw
        self.current_pose = None  # To store the relative x, y, and yaw (yaw remains absolute)


        self.node = None
        self.executor = None
        self.navigator = None
        self.odom_subscriber = None
        self.is_joy_on_subscriber = None
        self.continue_nav_publisher = None
        self.current_pose = None
        self.cashier_reached = False
        
        self.launch_processes = []
        self.launch_thread = None
        
        self.ros_thread = threading.Thread(target=self.init_ros)
        self.ros_thread.daemon = True  # Daemon para que termine cuando la GUI se cierre
        self.ros_thread.start()
        
        
        self.robot_patch = None
        self.robot_width = 0.15  # in meters
        self.robot_length = 0.35  # in meters
        self.robot_image = None
        self.robot_imobj = None
        self.robot_artist = None
        self.cashier_marker = None  # New attribute to store the cashier marker
        self.fixed_cash_location = {'x': 0.1, 'y': 2.0}  # Add this line to define the cashier location
        
        # Variable para controlar el estado del botón
        self.navigation_started = False
        self.go_to_cashier= False
        self.continue_nav_published = False  # Nueva variable para controlar la publicación

        # Frame superior
        self.top_frame = ctk.CTkFrame(self, height=50, fg_color="bisque2")
        self.top_frame.pack(side=ctk.TOP, fill=ctk.X, padx=10, pady=10)

        # Ejemplo de etiqueta en el frame superior
    #    self.label_superior = ctk.CTkLabel(self.top_frame, text="Información Adicional en la parte superior")
     #   self.label_superior.pack(pady=5)

        self.progress_bar = ctk.CTkProgressBar(self.top_frame, width=400)
        self.progress_bar.pack(pady=5)
        self.progress_bar.set(0)  # Initialize progress to 0

        # Add a label for status messages
        self.status_label = ctk.CTkLabel(self.top_frame, text="Por favor empiece con el proceso de compra")
        self.status_label.pack(pady=5)

        # Frame para la información de fecha, hora, etc.
        self.info_frame = ctk.CTkFrame(self, width=200, fg_color="bisque2")
        self.info_frame.pack(side=ctk.LEFT, fill=ctk.Y, padx=10, pady=10)

        # Etiquetas de reloj y fecha
        self.label_reloj = ctk.CTkLabel(self.info_frame, font=('ARIAL', 18, 'bold'))
        self.label_reloj.pack(side=ctk.TOP, padx=10, pady=10)
        self.label_fecha = ctk.CTkLabel(self.info_frame, font=('ARIAL', 18, 'bold'))
        self.label_fecha.pack(side=ctk.TOP, padx=10, pady=70)
        shop_vision_label = ctk.CTkLabel(self.info_frame, text="SARA", font=('Helvetica', 20, 'bold'))
        shop_vision_label.pack(side=ctk.BOTTOM, padx=10, pady=10)

        # Iniciar la actualización del reloj y la fecha
        self.actualizar_reloj_y_fecha()

        # Frame para el mapa
        self.map_frame = ctk.CTkFrame(self, width=800, height=600)
        self.map_frame.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True, padx=10, pady=10)

        # Frame para el gráfico y la lista de productos
        self.right_frame = ctk.CTkFrame(self, width=300,  fg_color="bisque2")
        self.right_frame.pack(side=ctk.RIGHT, fill=ctk.Y, padx=10, pady=10, expand=False)

        # Frame para la lista de productos seleccionados
        self.selected_frame = ctk.CTkFrame(self.right_frame, width=150)
        self.selected_frame.pack(side=ctk.TOP, fill=ctk.BOTH, expand=True, padx=10, pady=10)

        # Frame para el botón
        self.button_frame = ctk.CTkFrame(self.right_frame, width=120, fg_color="bisque2")
        self.button_frame.pack(side=ctk.BOTTOM, fill=ctk.X, padx=10, pady=10)

        # Crear un marco interno para el botón con más altura
        self.button_inner_frame = ctk.CTkFrame(self.button_frame, height=100, width=100, fg_color="bisque2")
        self.button_inner_frame.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True, padx=10, pady=10)
   
        self.start_calibration_button = ctk.CTkButton(
            self.button_inner_frame,
            text="Iniciar Navegacion",
            command=self.start_calibration, fg_color="blanched almond", text_color="black", hover_color="bisque2"  # Nueva función que realizarás
            
        )
        self.start_calibration_button.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True, padx=5)


        # Botón "Iniciar Navegación" con mayor altura
        self.start_navigation_button = ctk.CTkButton(
            self.button_inner_frame,
            text="Localizar productos",
            command=self.start_navigation,
            height=80, fg_color="blanched almond", text_color="black", hover_color="bisque2"
            
        )
        self.start_navigation_button.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True)
        
        self.control_frame = ctk.CTkFrame(self.info_frame, width=150)
        self.control_frame.pack(side=ctk.TOP, padx=10, pady=10)

        # Create a label for the control status text
        self.control_label = ctk.CTkLabel(self.control_frame, text="Estado de control")
        self.control_label.pack(side=ctk.LEFT, padx=5)

        # Create a canvas for the circle
        self.control_canvas = ctk.CTkCanvas(self.control_frame, width=20, height=20)
        self.control_canvas.pack(side=ctk.LEFT, padx=5)

        # Create a circle on the canvas
        self.control_circle = self.control_canvas.create_rectangle(0, 0, 20, 20, fill="red")

        # Crear y mostrar el gráfico
        self.create_plot(self.map_frame)
        
        self.update_robot_position()

        # Actualizar la lista de productos seleccionados
        self.view_selected_products()
        self.after(1000, self.view_selected_products)

    def init_ros(self):
       rclpy.init(args=None)
       self.node = rclpy.create_node('navigate_node')

       time.sleep(2.0)

       self.executor = rclpy.executors.SingleThreadedExecutor()
       self.executor.add_node(self.node)
       self.navigator = BasicNavigator()
       self.odom_subscriber = self.node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.odom_callback, 10)
       self.is_joy_on_subscriber = self.node.create_subscription(String, 'is_joy_on', self.is_joy_on_callback, 10)

       self.continue_nav_publisher = self.node.create_publisher(String, '/continue_nav', 10)
       self.cashier_publisher = self.node.create_publisher(String, '/to_do_next', 10)
       self.status_subscriber = self.node.create_subscription(String, '/navigation_status', self.status_callback, 10)
    
    # Spin the executor in a loop
       while rclpy.ok():
          self.executor.spin_once(timeout_sec=0.1)

    def odom_callback(self, msg):
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'orientation': self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        }



    def is_joy_on_callback(self, msg):
        if msg.data == "yes":
            self.control_canvas.itemconfig(self.control_circle, fill="green")
        else:
            self.control_canvas.itemconfig(self.control_circle, fill="red")       
   
    def status_callback(self, msg):
        status, waypoint_name, completion_percentage, visited_waypoints = msg.data.split('|')
        visited_waypoints = set(visited_waypoints.split(','))
        self.handle_progress_bar(status, waypoint_name, completion_percentage)
        self.update_waypoint_status(status, waypoint_name, visited_waypoints)
        self.handle_status(status, waypoint_name)
        
        if status == "REACHED" and waypoint_name == "cashier":
            self.cashier_reached = True
            self.status_label.configure(text="Status: REACHED - Waypoint: cashier")
       
    def update_waypoint_status(self, status, waypoint_name, visited_waypoints):
    # Obtener la ubicación de los waypoints desde la base de datos
        locations = self.get_product_locations()

        for loc in locations:
            if loc['name'] == waypoint_name:
                pixel_x = int((loc['x'] - self.origin[0]) / self.resolution)
                pixel_y = int((loc['y'] - self.origin[1]) / self.resolution)

                if status == "REACHED":
                    # El waypoint ya ha sido visitado, cambiar a verde
                    self.update_marker_color(pixel_x, pixel_y, 'green')
                    self.update_text_color(waypoint_name, 'green')
                elif status == "NAVIGATING":
                    # El waypoint está en navegación, cambiar a amarillo
                    self.update_marker_color(pixel_x, pixel_y, 'yellow')
                    self.update_text_color(waypoint_name, 'yellow')

    def update_marker_color(self, x, y, color):
    # Actualizar el color del marcador en el gráfico
        for marker in self.ax.lines:
            if marker.get_xdata() == x and marker.get_ydata() == y:
                marker.set_color(color)
        self.canvas.draw()

    def update_text_color(self, name, color):
        # Actualizar el color del texto en el panel de productos
        for widget in self.selected_frame.winfo_children():
            if isinstance(widget, ctk.CTkLabel) and widget.cget("text") == name:
                widget.configure(text_color=color)
      
    def get_yaw_from_quaternion(self, quaternion):
        # Convert quaternion to Euler angles
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return yaw
      
    def spin_ros_node(self):
        rclpy.spin(self.node)
    
    def handle_status(self, status, waypoint_name):
        if status == "READY":
            #self.show_info("Ahora puede empezar a dirigirse a sus productos, por favor dar click a siguiente producto")
            self.status_label.configure(text="Ahora puede empezar a dirigirse a sus productos, por favor dar click a siguiente producto")
            self.start_navigation_button.configure(state="enabled" , text= "Siguiente producto")
        elif status == "WAITING":
            #self.show_info("Ha llegado a su destino, cuando este listo dar click a siguiente producto") 
            self.status_label.configure(text="Ha llegado a su destino, cuando este listo dar click a siguiente producto")  
            self.start_navigation_button.configure(state="enabled")
        elif status == "FINISHED":
            self.status_label.configure(text="Ha finalizado su proceso de compra cuando este listo dar click a Ir a caja") 
            self.start_navigation_button.configure(text= "Ir a caja", state="enabled")
            self.go_to_cashier = True
            
        elif status == "SHOPPING_AGAIN":
            self.destroy()
                 
    def handle_progress_bar(self, status, waypoint_name, completion_percentage):
         
        if completion_percentage:
            progress = float(completion_percentage) / 100
            self.progress_bar.set(progress)
        else:
            self.progress_bar.set(0)
            
        if status == "REACHED":
           self.progress_bar.set(1)
        
        status_text = f"Status: {status}"
        if waypoint_name:
            status_text += f" - Waypoint: {waypoint_name}"
        
        self.status_label.configure(text=status_text)

    def show_popup(self):
        popup_window = ctk.CTkToplevel()
        popup_window.title("Acciones")
        popup_window.geometry("300x150")
        
        label = ctk.CTkLabel(popup_window, text="Has terminado con tu compra, cuando estes listo presiona ir a caja", padx=20, pady=20)
        label.pack(expand=True)
        
        # Botón para ir a caja
        go_to_checkout_button = ctk.CTkButton(popup_window, text="Ir a Caja", command=self.go_to_checkout)
        go_to_checkout_button.pack(side="left", padx=20, pady=10)
            
    def go_to_checkout(self):
        self.add_cashier_marker()
        self.publish_cashier()
        self.cashier_reached = False
        self.wait_for_cashier_reached()

    def wait_for_cashier_reached(self):
        if self.cashier_reached:
            self.finish_shopping()
        else:
            self.after(100, self.wait_for_cashier_reached)
       
    def finish_shopping(self):
        
        if self.after_id is not None:
            self.after_cancel(self.after_id) 
        
        self.new_window = ThanksWindow(self)  # Crea una nueva ventana
        
        self.withdraw()
        self.new_window.mainloop()
        
    def show_main_window(self):
        self.product_manager.deiconify()  # Show the ProductManager window
        self.destroy()  # Close the NavigationWindow
    
    def add_cashier_marker(self):
        if self.cashier_marker:
            self.cashier_marker.remove()

        pixel_x = int((self.fixed_cash_location['x'] - self.origin[0]) / self.resolution)
        pixel_y = int((self.fixed_cash_location['y'] - self.origin[1]) / self.resolution)

        self.cashier_marker, = self.ax.plot(pixel_x, pixel_y, 'g*', markersize=15, label='Cashier')
        self.ax.legend()
        self.canvas.draw()
        
    def stop_ros_processes(self):

        for process in self.launch_processes:
            if process.poll() is None:  # Verificar si el proceso sigue ejecutándose
                process.terminate()  # Terminar el proceso
            process.wait()  # Esperar a que el proceso termine

    # Detener el nodo ROS y el ejecutor
        if self.executor:
            self.executor.shutdown()
        if self.node:
            self.node.destroy_node()

    # Finalizar el cliente ROS 2
        if rclpy.ok():
            rclpy.shutdown()

    # Reinicializar los atributos
        self.node = None
        self.executor = None
        self.navigator = None
        self.odom_subscriber = None
        self.continue_nav_publisher = None
        self.launch_processes = []  # Limpiar la lista de procesos
   
    def show_info(self, message, title="Info"):
        info_window = ctk.CTkToplevel()
        info_window.title(title)
        info_window.geometry("300x150")
        label = ctk.CTkLabel(info_window, text=message, padx=20, pady=20)
        label.pack(expand=True)
        ok_button = ctk.CTkButton(info_window, text="OK", command=info_window.destroy)
        ok_button.pack(pady=10)
          
    def update_robot_position(self):
        if self.current_pose:
            # Convert map coordinates to pixel coordinates
            pixel_x = int((self.current_pose['x'] - self.origin[0]) / self.resolution)
            pixel_y = int((self.current_pose['y'] - self.origin[1]) / self.resolution)
           # print(f"Robot position: pixel_x={pixel_x}, pixel_y={pixel_y}")  # Debug print

            orientation = self.current_pose.get('orientation', 0)  # in radians

                      # Remove the previous robot image if it exists
            robot_width_pixels = self.robot_width / self.resolution
            robot_length_pixels = self.robot_length / self.resolution
            
            # Update the position and rotation of the image
            tr = mtransforms.Affine2D().rotate(orientation -math.pi/2).translate(pixel_x, pixel_y)
            self.robot_imobj.set_transform(tr + self.ax.transData)
            
            # Update the extent of the image
            extent = [-robot_width_pixels, robot_width_pixels,
                      -robot_length_pixels, robot_length_pixels]
            self.robot_imobj.set_extent(extent)

            self.canvas.draw()
        else:
            print("No current pose available")
        
        self.after(100, self.update_robot_position)

    def actualizar_reloj_y_fecha(self):
        now = datetime.datetime.now()
        self.label_reloj.configure(text=now.strftime("%H:%M:%S"))
        self.label_fecha.configure(text=now.strftime("%Y-%m-%d"))
        self.after(1000, self.actualizar_reloj_y_fecha)
  
    def create_plot(self, frame):
        self.fig, self.ax = plt.subplots(figsize=(6, 4), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().pack(fill=ctk.BOTH, expand=True)

        self.update_map_plot()
        
        pkg_dir = get_package_share_directory('turtlemart')
        robot_image_path = os.path.join(pkg_dir, 'images/shoppingcart.png')
        self.robot_image = plt.imread(robot_image_path)
        robot_width_data = self.robot_width / self.resolution
        robot_height_data = self.robot_length / self.resolution
        
        # Create the image object on the axes, swapping width and height in extent
        self.robot_imobj = self.ax.imshow(self.robot_image, 
                                          extent=[-robot_height_data/2, robot_height_data/2, 
                                                  -robot_width_data/2, robot_width_data/2],
                                          zorder=2,  # Ensure the robot is drawn on top of the map
                                          alpha=0.8)  # Slight transparency to see the map underneath

        # Set the initial position off-screen
        self.robot_imobj.set_transform(mtransforms.Affine2D().translate(-1000, -1000) + self.ax.transData)

        self.ax.legend()
      

    def update_map_plot(self):
        self.ax.clear()
        # Cargar el mapa y actualizar el gráfico
        self.map_array, self.resolution, self.origin = self.load_map()
        self.ax.imshow(np.flipud(self.map_array), cmap='gray', origin='lower')
        map_height, map_width = self.map_array.shape
        self.ax.set_xlim(0, map_width)
        self.ax.set_ylim(0, map_height)
        # Obtener y plotear ubicaciones de productos
        self.plot_product_locations()
        
        #self.robot_position, = self.ax.plot([], [], 'bo', markersize=10, label='Robot')
        self.ax.legend()
        
    def load_map(self):
        bringup_dir = get_package_share_directory('turtlemart')
        #map_yaml_path = os.path.join(bringup_dir, 'maps/supermarket_map.yaml')
        #map_yaml_path = os.path.join(bringup_dir, 'maps/labrobsuper_map.yaml')
        map_yaml_path = os.path.join(bringup_dir, 'maps/labrobfinal_mask.yaml')
        #map_yaml_path = os.path.join(bringup_dir, 'maps/labmap2_mask.yaml')
        with open(map_yaml_path, 'r') as f:
            yaml_content = yaml.safe_load(f)

        resolution = yaml_content['resolution']
        origin = yaml_content['origin']
        map_image_path = yaml_content['image']

        if not os.path.isabs(map_image_path):
            map_image_path = os.path.join(os.path.dirname(map_yaml_path), map_image_path)

        map_image = Image.open(map_image_path)
        map_array = np.array(map_image)

        return map_array, resolution, origin

    def plot_product_locations(self):
        # Obtener ubicaciones de la base de datos
        locations = self.get_product_locations()

        for loc in locations:
            # Convertir las coordenadas de mapa a píxeles
            pixel_x = int((loc['x'] - self.origin[0]) / self.resolution)
            pixel_y = int((loc['y'] - self.origin[1]) / self.resolution)
            # Plottear las ubicaciones en el mapa
            self.ax.plot(pixel_x, pixel_y, 'ro')  # 'ro' indica puntos rojos

        self.canvas.draw()

    def get_product_locations(self):
        db_dir = os.path.join('src/turtlemart/database/products.db')
        conn = sqlite3.connect(db_dir)
        cursor = conn.cursor()
        cursor.execute('SELECT name, x, y FROM selected_products')
        locations = [{'name': row[0], 'x': row[1], 'y': row[2]} for row in cursor.fetchall()]
        conn.close()
        return locations

    def start_navigation(self):
        if not self.navigation_started:
            print("Iniciando navegación...")
            print("Iniciando lanzamiento de archivos y navegación...")
            

            navigation_thread = threading.Thread(target=self.run_navigation)
            navigation_thread.start()
            self.navigation_started = True  # Marcar que la navegación ha comenzado
            self.start_navigation_button.configure(state="disabled")
        elif self.go_to_cashier:
            self.start_navigation_button.configure( state="disabled")
            self.go_to_checkout()
        else:
            print("La navegación ya ha comenzado. Ejecutando otra acción...")
            self.perform_alternate_action()
                
    def launch_ros2_files(self):
        launch_commands = [
            "ros2 launch turtlemart mux.launch.py",
            "ros2 launch turtlemart real_nav.launch.py"
            #"ros2 launch turtlemart navagv.launch.py"
   
        ]
        
        
        devnull = open(os.devnull, 'w')
        for cmd in launch_commands:
            process = subprocess.Popen(
                cmd, 
                shell=True, 
                stdout=devnull, 
                stderr=subprocess.STDOUT
            )
            self.launch_processes.append(process)
        
        
        print("Launch files started successfully (output suppressed).")
        basic_control_thread = threading.Thread(target=self.launch_basic_control)
        basic_control_thread.start()
        
    def launch_basic_control(self):
        cmd = "ros2 launch turtlemart basic_control.launch.py"
        process = subprocess.Popen(
            cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True
        )
        self.launch_processes.append(process)
    
        # Monitor the process output
        for line in process.stdout:
            print("Basic Control:", line.strip())
     
        process.wait()
        print("Basic Control launch file completed.")
        
    def is_process_running(self, process_name):
        """Verifica si un proceso con el nombre especificado ya está en ejecución"""
        try:
            # `pgrep` busca procesos que coincidan con el nombre
            output = subprocess.check_output(["pgrep", "-f", process_name])
            return bool(output.strip())  # Si hay salida, el proceso está corriendo
        except subprocess.CalledProcessError:
            return False  # Si no se encuentra el proceso, retorna False

    def run_navigation(self):
        # Esta función será ejecutada en un hilo separado
        navigator = AutonomousNavigator()
        waypoints_reached = navigator.navigate()
        print("Navegación completa")
        # Publicar el mensaje al finalizar la navegación
        self.publish_continue_nav()   
        
    def __del__(self):
        # Terminate launch file processes when the window is closed
        for process in self.launch_processes:
            process.terminate()
        
        if self.node:
            self.node.destroy_node()
        rclpy.shutdown()

    def perform_alternate_action(self):
            self.start_navigation_button.configure(state="disabled")
            self.publish_continue_nav()
            self.continue_nav_published = True  # Marcastartr que el mensaje ha sido publicado
            self.publish_stop()

    def publish_continue_nav(self):
        msg = String()
        msg.data = "continue"
        self.continue_nav_publisher.publish(msg)
        
    def publish_stop(self):
        msg = String()
        msg.data = "stop"
        self.continue_nav_publisher.publish(msg)
        
    def publish_cashier(self):
        msg = String()
        msg.data = "cash"
        self.cashier_publisher.publish(msg)

    def publish_shop_again(self):
        msg = String()
        msg.data = "shop_again"
        self.cashier_publisher.publish(msg)
    
# Nueva función que manejará la acción del nuevo botón
    def start_calibration(self):
        if not self.launch_thread or not self.launch_thread.is_alive():
            self.launch_thread = threading.Thread(target=self.launch_ros2_files)
            self.launch_thread.start()
            self.start_calibration_button.configure(state="disabled")
            print("Launching ROS 2 files in background...")
        else:
            print("ROS 2 launch files are already running.")


    def view_selected_products(self):
        self.selected_frame.destroy()  # Destruir el marco anterior
        self.selected_frame = ctk.CTkFrame(self.right_frame, width=150, fg_color="peachpuff")
        self.selected_frame.pack(side=ctk.TOP, fill=ctk.BOTH, expand=True, padx=10, pady=10)

        # Re-crear la lista de productos seleccionados
        conn = sqlite3.connect('src/turtlemart/database/products.db')
        cursor = conn.cursor()
        cursor.execute('SELECT name FROM selected_products')
        rows = cursor.fetchall()
        conn.close()

        for row in rows:
            product_name = row[0]
            label = ctk.CTkLabel(self.selected_frame, text=product_name)
            label.pack(pady=5)

if __name__ == "__main__":
    app = RealNavWindow()
    app.mainloop()
