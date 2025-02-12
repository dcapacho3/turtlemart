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
from real_guiwaypoint import AutonomousNavigator as RealNavigator
from guiwaypoint import AutonomousNavigator as SimNavigator
from std_msgs.msg import String, Bool
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



class UnifiedNavigationWindow(ctk.CTk):
    def __init__(self, product_manager,navigation_mode):
        super().__init__()
        #self.master = master

        self.product_colors = {}
        self.product_labels = {}


        self.STATUS_FONT = ('Arial', 25, 'bold')
        self.PRODUCT_FONT = ('Arial', 20, 'bold')
        self.PRODUCT_TITLE_FONT = ('Arial', 30, 'bold')      
        self.CLOCK_FONT = ('ARIAL', 25, 'bold')  # Para el reloj también


        self.colors = {
        # Fondos
        'primary_bg': "#FEF2F2",  # Rojo claro suave
        'secondary_bg': "#FEE2E2",  # Rosa coral vibrante
        'accent_bg': "#FFFFFF",  
        'list_bg': "#F8FAFC",  
        'frame_bg': "#FEE2E2",  
        'scrollable_frame_bg': "#FFFFFF",  
        # Texto
        'text_primary': "#7F1D1D",  # Rojo vino
        'text_secondary': "#DC2626",  # Rojo fuego
        'label_text': "#B91C1C",  # Rojo intenso
        # Elementos interactivos
        'button_bg': "#DC2626",  # Rojo llamativo
        'button_hover': "#991B1B",  # Rojo oscuro
        'button_text': "#FFFFFF",  
        'entry_bg': "#F8FAFC",  
        'checkbox_bg': "#DC2626",  
        'checkbox_hover': "#991B1B",  
        # Menú
        'optionmenu_bg': "#F8FAFC",  
        'optionmenu_button': "#DC2626",  
        'optionmenu_hover': "#991B1B",  
        'optionmenu_text': "#7F1D1D", 
        }

        self.configure(fg_color=self.colors['primary_bg'])
        self.after_id = None
        
        self.is_closing = False
        self.ros_initialized = False
        self.threads = []
        self.lock = threading.Lock()
        
        # Configuración de la interfaz
        self.product_manager = product_manager
        self.nav_mode = navigation_mode 
         
       
        self.title("Smart Autonomous Retail Assistant")
        self.geometry("%dx%d+0+0" % (self.winfo_screenwidth(), self.winfo_screenheight()))

        #self.attributes('-fullscreen', True) 
        #self.overrideredirect(True)  

        self.resizable(width=1, height=1)

        # Inicializar la lista de productos seleccionados
        self.selected_products = []

        self.initial_pose = None  # To store the starting x, y, and yaw
        self.current_pose = None  # To store the relative x, y, and yaw (yaw remains absolute)
        self.pose_message_printed = False  # Solo agregar esta variable

        self.node = None
        self.executor = None
        self.navigator = None
        self.odom_subscriber = None
        self.is_joy_on_subscriber = None
        self.continue_nav_publisher = None
        self.current_pose = None
        self.cashier_reached = False
        self.lock_all_active = False  # Agregar esta variable de estado
        self.should_show_popup = True
        self.current_popup = None  # Mantener referencia al popup actual


        self.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.calibration_complete = False 
        
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
        self.fixed_cash_location = {'x': 0.1, 'y': 2.0} if self.nav_mode == "Real" \
            else {'x': -1.0, 'y': -2.0}

        # Variable para controlar el estado del botón
        self.navigation_started = False
        self.go_to_cashier= False
        self.continue_nav_published = False  # Nueva variable para controlar la publicación

        # Frame superior
        self.top_frame = ctk.CTkFrame(self, height=100, fg_color=self.colors['secondary_bg'])
        self.top_frame.pack(side=ctk.TOP, fill=ctk.X, padx=10, pady=10)
        self.top_frame.pack_propagate(False)

        clock_frame = ctk.CTkFrame(self.top_frame, fg_color=self.colors['secondary_bg'])
        clock_frame.pack(side=ctk.RIGHT, padx=20)

        status_center_frame = ctk.CTkFrame(self.top_frame, fg_color=self.colors['secondary_bg'])
        status_center_frame.pack(side=ctk.LEFT, expand=True, fill=ctk.BOTH, padx=20)
        status_center_frame.pack_propagate(False) 
        
        self.status_label = ctk.CTkLabel(
            status_center_frame, 
            text="Por favor empiece con el proceso de compra",
            font=self.STATUS_FONT,
            text_color=self.colors['text_primary'],
            wraplength=1400
        )
        self.status_label.pack(pady=(10, 5))

        self.progress_bar = ctk.CTkProgressBar(status_center_frame, width=700, progress_color=self.colors['button_bg'], fg_color=self.colors['secondary_bg'])
        self.progress_bar.pack(pady=(10, 20))
        self.progress_bar.set(0)



        # Frame para la información de fecha, hora, etc.
        self.info_frame = ctk.CTkFrame(self, width=200, fg_color=self.colors['secondary_bg'])
        self.info_frame.pack(side=ctk.LEFT, fill=ctk.Y, padx=10, pady=10)

        # Etiquetas de reloj y fecha
        self.label_fecha = ctk.CTkLabel(
            clock_frame, 
            text="",  # Se actualizará con actualizar_reloj_y_fecha
            font=self.CLOCK_FONT,
            text_color=self.colors['text_primary'],
        )
        self.label_fecha.pack(side=ctk.TOP, pady=5)

        self.label_reloj = ctk.CTkLabel(
            clock_frame, 
            text="",
            font=self.CLOCK_FONT,
            text_color=self.colors['text_primary']
        )
        self.label_reloj.pack(side=ctk.TOP, pady=5)

        shop_vision_label = ctk.CTkLabel(self.info_frame, text="SARA", font=('Helvetica', 35, 'bold'), text_color=self.colors['text_primary'],)
        shop_vision_label.pack(side=ctk.BOTTOM, padx=10, pady=10)

        # Iniciar la actualización del reloj y la fecha
        self.actualizar_reloj_y_fecha()

        # Frame para el mapa
        self.map_frame = ctk.CTkFrame(self, width=800, height=600, fg_color=self.colors['primary_bg'])
        self.map_frame.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True, padx=10, pady=10)

        # Frame para el gráfico y la lista de productos
        self.right_frame = ctk.CTkFrame(self, width=450,  fg_color=self.colors['secondary_bg'])
        self.right_frame.pack(side=ctk.RIGHT, fill=ctk.Y, padx=10, pady=10, expand=False)
        self.right_frame.pack_propagate(False) 

        # Frame para la lista de productos seleccionados
        self.selected_frame = ctk.CTkFrame(self.right_frame, width=350, fg_color=self.colors['list_bg'])
        self.selected_frame.pack(side=ctk.TOP, fill=ctk.BOTH, expand=True, padx=10, pady=10)
 # Prevent frame from shrinking
        # Frame para el botón
        self.button_frame = ctk.CTkFrame(self.right_frame, width=120, fg_color=self.colors['secondary_bg'])
        self.button_frame.pack(side=ctk.BOTTOM, fill=ctk.X, padx=10, pady=10)

        # Crear un marco interno para el botón con más altura
        self.button_inner_frame = ctk.CTkFrame(self.button_frame, height=100, width=100, fg_color=self.colors['secondary_bg'])
        self.button_inner_frame.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True, padx=10, pady=10)
   
        self.navigation_button = ctk.CTkButton(
            self.button_inner_frame,
            text="Iniciar",
            command=self.handle_navigation_button,        
            height=90,  # Mantenemos la altura del botón original
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover'],
            font=self.PRODUCT_FONT
        )
        self.navigation_button.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True, padx=5)

        self.control_frame = ctk.CTkFrame(self.info_frame, width=200, height=150, fg_color="transparent")
        self.control_frame.pack(side=ctk.TOP, padx=10, pady=20, expand=True)
        self.control_frame.pack_propagate(False)
        container = ctk.CTkFrame(self.control_frame, fg_color="transparent")
        container.place(relx=0.5, rely=0.5, anchor="center")



        # Create a label for the control status text
        self.control_label = ctk.CTkLabel(
            container, 
            text="Estado de Control Remoto",
            font=self.PRODUCT_FONT,
            text_color=self.colors['text_primary'],
            fg_color="transparent",
            wraplength=200
        )
        self.control_label.pack(pady=(0, 10))


        # Create a canvas for the circle
        self.control_canvas = ctk.CTkCanvas(
            container, 
            width=40, 
            height=40, 
            highlightthickness=0,
            bg=self.colors['secondary_bg']
        )
        self.control_canvas.pack(pady=(5, 0))

        # Create a circle on the canvas

        # Create the circle with padding
        padding = 5
        self.control_circle = self.control_canvas.create_oval(
            padding, 
            padding, 
            40 - padding, 
            40 - padding, 
            fill="red", 
            outline=""
        )
        self.control_status_label = ctk.CTkLabel(
            container,
            text="Deshabilitado",  # Texto inicial
            font=self.PRODUCT_FONT,
            text_color=self.colors['text_primary'],
            fg_color="transparent"
        )
        self.control_status_label.pack(pady=(5, 0))


        # Crear y mostrar el gráfico
        self.create_plot(self.map_frame)
        
        self.update_robot_position()

        # Actualizar la lista de productos seleccionados
        self.view_selected_products()
        self.after(1000, self.view_selected_products)

    def init_ros(self):
        try:
            rclpy.init(args=None)
            self.ros_initialized = True
            self.node = rclpy.create_node('navigate_node')

            time.sleep(2.0)

            self.executor = rclpy.executors.SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.navigator = BasicNavigator()

            if self.nav_mode == "Real":
                self.odom_subscriber = self.node.create_subscription(
                    PoseWithCovarianceStamped, 'amcl_pose', self.odom_callback, 10)
                self.lock_all_subscriber = self.node.create_subscription(
                    Bool, 'lock_all', self.lock_all_callback, 10)
            else:
                self.odom_subscriber = self.node.create_subscription(
                    Odometry, 'odom', self.odom_callback, 10)

            self.is_joy_on_subscriber = self.node.create_subscription(String, 'is_joy_on', self.is_joy_on_callback, 10) 

            self.continue_nav_publisher = self.node.create_publisher(String, '/continue_nav', 10)
            self.cashier_publisher = self.node.create_publisher(String, '/to_do_next', 10)
            self.status_subscriber = self.node.create_subscription(String, '/navigation_status', self.status_callback, 10)
        
        # Spin the executor in a loop
            while rclpy.ok() and not self.is_closing:
                try:
                    self.executor.spin_once(timeout_sec=0.1)
                except rclpy.executors.ExternalShutdownException:
                    break
                except Exception as e:
                    print(f"Error in ROS loop: {e}")
                    if self.is_closing:
                        break
        except Exception as e:
            print(f"Error initializing ROS: {e}")
        finally:
            if self.is_closing:
                self.cleanup_ros()

    def odom_callback(self, msg):
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'orientation': self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        }

    def is_joy_on_callback(self, msg):
        try:
            if msg.data == "yes":
                self.control_canvas.itemconfig(
                    self.control_circle, 
                    fill="green", 
                    outline=""
                )
                self.control_status_label.configure(text="Habilitado")
            else:
                self.control_canvas.itemconfig(
                    self.control_circle, 
                    fill="red", 
                    outline=""
                )
                self.control_status_label.configure(text="Deshabilitado")
            # Force a redraw of the canvas to ensure immediate update
            self.control_canvas.update_idletasks()
        except Exception as e:
            print(f"Error updating control circle color: {e}")

    def lock_all_callback(self, msg):
        if msg.data and self.should_show_popup:  # Si es True y debemos mostrar el popup
            self.show_info("Peso máximo de plataforma excedida, por favor retire el último producto agregado", "¡Atención!")
            self.should_show_popup = False
        elif not msg.data:  # Si es False
            if self.current_popup is not None:
                self.current_popup.destroy()  # Cerrar el popup si existe
                self.current_popup = None
            self.should_show_popup = True  # Permitir que se muestre el próximo True
        
        self.lock_all_active = msg.data

    def status_callback(self, msg):

        status, waypoint_name, completion_percentage, visited_waypoints = msg.data.split('|')
        visited_waypoints = set(visited_waypoints.split(','))
        self.handle_progress_bar(status, waypoint_name, completion_percentage)
        self.update_waypoint_status(status, waypoint_name, visited_waypoints)
        self.handle_status(status, waypoint_name)
        
        # Actualizar el texto del botón según el estado
        if status == "READY":
            self.navigation_button.configure(
                text="Siguiente producto",
                state="normal"
            )
        elif status == "WAITING":
            self.navigation_button.configure(
                state="normal"
            )
        elif status == "FINISHED":
            self.navigation_button.configure(
                text="Ir a caja",
                state="normal"
            )
            self.go_to_cashier = True
            
        if status == "REACHED" and waypoint_name == "cashier":
            self.cashier_reached = True
            self.status_label.configure(text="Status: REACHED - Waypoint: cashier")

            
    def update_waypoint_status(self, status, waypoint_name, visited_waypoints):
        locations = self.get_product_locations()
        
        for loc in locations:
            if loc['name'] == waypoint_name:
                pixel_x = int((loc['x'] - self.origin[0]) / self.resolution)
                pixel_y = int((loc['y'] - self.origin[1]) / self.resolution)
                
                if status == "REACHED":
                    self.update_marker_color(pixel_x, pixel_y, 'green')
                    self.update_product_color(waypoint_name, "#00FF00")  # Verde brillante
                elif status == "NAVIGATING":
                    self.update_marker_color(pixel_x, pixel_y, 'yellow')
                    self.update_product_color(waypoint_name, "#FFD700")  # Amarillo

    def update_product_color(self, product_name, color):
        """Actualiza el color de un producto específico sin recrear la lista"""
        self.product_colors[product_name] = color
        if product_name in self.product_labels:
            label = self.product_labels[product_name]
            label.configure(text_color=color)

    def update_marker_color(self, x, y, color):
    # Actualizar el color del marcador en el gráfico
        for marker in self.ax.lines:
            if marker.get_xdata() == x and marker.get_ydata() == y:
                marker.set_color(color)
        self.canvas.draw()

    def update_text_color(self, name, color):

        self.product_colors[name] = color
        # Actualizar el color del texto en el panel de productos
        scrollable_frame = None
        for widget in self.selected_frame.winfo_children():
            if isinstance(widget, ctk.CTkScrollableFrame):
                scrollable_frame = widget
                break
        if scrollable_frame:
            # Buscar en los contenedores de productos dentro del ScrollableFrame
            for product_container in scrollable_frame.winfo_children():
                if isinstance(product_container, ctk.CTkFrame):
                    # Buscar la etiqueta dentro del contenedor del producto
                    for label in product_container.winfo_children():
                        if isinstance(label, ctk.CTkLabel) and label.cget("text") == name:
                            label.configure(text_color=color)
                            return
      
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
            self.status_label.configure(text="Ahora puede empezar a dirigirse a sus productos, por favor dar click a siguiente producto")
            self.navigation_button.configure(state="enabled", text="Siguiente producto")
        elif status == "WAITING":
            self.status_label.configure(text="Ha llegado a su destino, cuando este listo dar click a siguiente producto")  
            self.navigation_button.configure(state="enabled")
        elif status == "FINISHED":
            self.status_label.configure(text="Ha finalizado su proceso de compra cuando este listo dar click a Ir a caja") 
            self.navigation_button.configure(text="Ir a caja", state="enabled")
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
        popup_window.configure(fg_color=self.colors['primary_bg'])
        popup_window.title("Acciones")
        popup_window.geometry("300x150")
        
        label = ctk.CTkLabel(popup_window, text="Has terminado con tu compra, cuando estes listo presiona ir a caja", padx=20, pady=20)
        label.pack(expand=True)
        
        # Botón para ir a caja
        go_to_checkout_button = ctk.CTkButton(popup_window, text="Ir a Caja", command=self.go_to_checkout, fg_color=self.colors['button_bg'], text_color=self.colors['button_text'], hover_color=self.colors['button_hover'],)
        go_to_checkout_button.pack(side="left", padx=20, pady=10)
            
    def go_to_checkout(self):
        self.navigation_button.configure(state="disabled")  # Añadir esta línea
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
        
        self.navigation_button.configure(state="disabled")  # Añadir esta línea
        self.new_window = ThanksWindow(self)
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

        self.cashier_marker, = self.ax.plot(pixel_x, pixel_y, 'g*', markersize=15)
        #self.ax.legend()
        self.canvas.draw()
        
    def stop_ros_processes(self):
        print("Stopping ROS processes...")
        
        # Stop all launch processes
        for process in self.launch_processes:
            try:
                if process and process.poll() is None:
                    process.terminate()
                    process.wait(timeout=5)  # Wait up to 5 seconds for process to terminate
            except Exception as e:
                print(f"Error terminating process: {e}")
                try:
                    process.kill()  # Force kill if terminate fails
                except:
                    pass

        # Cleanup ROS node and executor
        if self.executor:
            try:
                self.executor.shutdown()
            except Exception as e:
                print(f"Error shutting down executor: {e}")

        if self.node:
            try:
                self.node.destroy_node()
            except Exception as e:
                print(f"Error destroying node: {e}")

        # Shutdown ROS context if it's still running
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as e:
                print(f"Error shutting down ROS: {e}")

        self.node = None
        self.executor = None
        self.navigator = None
        self.odom_subscriber = None
        self.continue_nav_publisher = None
        self.launch_processes = []
   
    def show_info(self, message, title="Info"):
        if self.current_popup is not None:
            self.current_popup.destroy()
        
        info_window = ctk.CTkToplevel()
        info_window.configure(fg_color=self.colors['primary_bg'])
        info_window.title(title)
        info_window.geometry("500x200")  # Ventana más grande
        
        # Centrar la ventana en la pantalla
        screen_width = info_window.winfo_screenwidth()
        screen_height = info_window.winfo_screenheight()
        x = (screen_width - 500) // 2  # 500 es el ancho de la ventana
        y = (screen_height - 200) // 2  # 200 es el alto de la ventana
        info_window.geometry(f"500x200+{x}+{y}")
        
        # Frame principal para organizar el contenido
        main_frame = ctk.CTkFrame(info_window, fg_color=self.colors['secondary_bg'])
        main_frame.pack(expand=True, fill="both", padx=20, pady=20)
        
        # Label con texto mejorado
        label = ctk.CTkLabel(
            main_frame, 
            text=message,
            font=('Arial', 16),  # Fuente más grande
            wraplength=400,  # Ajuste de texto
            justify="center"  # Centrar el texto
        )
        label.pack(expand=True, padx=20, pady=20)
        
        # Botón OK mejorado
        ok_button = ctk.CTkButton(
            main_frame,
            text="OK",
            command=info_window.destroy,
            width=100,
            height=35,
            font=('Arial', 14),
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover']
        )
        ok_button.pack(pady=(0, 10))
        
        self.current_popup = info_window 
          
    def update_robot_position(self):
        if not self.is_closing:
            try:
                if self.current_pose:
                    with self.lock:
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

                        if not self.is_closing:

                            self.canvas.draw()

                        if not self.pose_message_printed:  # Solo imprime si no se ha impreso antes
                                    print("No current pose available")
                                    self.pose_message_printed = True

                if not self.is_closing:
                    self.after(100, self.update_robot_position)

            except Exception as e:
                print(f"Error in update_robot_position: {e}")
                if not self.is_closing:
                    self.after(100, self.update_robot_position)


    def actualizar_reloj_y_fecha(self):
        now = datetime.datetime.now()
        self.label_reloj.configure(text=now.strftime("%H:%M:%S"))
        self.label_fecha.configure(text=now.strftime("%Y-%m-%d"))
        self.after(1000, self.actualizar_reloj_y_fecha)
  
    def create_plot(self, frame):
        self.fig, self.ax = plt.subplots(figsize=(6, 4), dpi=100)
        # Ocultar los ejes y números
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        # Opcional: también puedes ocultar los bordes del gráfico
        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        self.ax.spines['bottom'].set_visible(False)
        self.ax.spines['left'].set_visible(False)
        
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

        
        #self.ax.legend()
      

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
        #self.ax.legend()
        
    def load_map(self):
        bringup_dir = get_package_share_directory('turtlemart')
        if self.nav_mode == "Real":
            map_yaml_path = os.path.join(bringup_dir, 'maps/labrobfinal_mask.yaml')
        else:
            map_yaml_path = os.path.join(bringup_dir, 'maps/supermarket_map.yaml')

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
        db_dir = get_source_db_path('turtlemart', 'products.db')
        conn = sqlite3.connect(db_dir)
        cursor = conn.cursor()
        cursor.execute('SELECT name, x, y FROM selected_products')
        locations = [{'name': row[0], 'x': row[1], 'y': row[2]} for row in cursor.fetchall()]
        conn.close()
        return locations

    def handle_navigation_button(self):
        if not self.calibration_complete:
            # Primera fase: iniciar calibración
            if not self.launch_thread or not self.launch_thread.is_alive():
                self.launch_thread = self.start_thread(self.launch_and_update)
                self.navigation_button.configure(state="disabled")
                print("Launching ROS 2 files in background...")
        else:
            # Segunda fase: navegación
            if not self.navigation_started:
                print("Iniciando navegación...")
                self.start_thread(self.run_navigation)
                self.navigation_started = True
                self.navigation_button.configure(state="disabled")
            elif self.go_to_cashier:
                self.navigation_button.configure(state="disabled")
                self.go_to_checkout()
            else:
                print("La navegación ya ha comenzado. Ejecutando otra acción...")
                self.perform_alternate_action()

    def launch_and_update(self):
        # Lanzar los archivos ROS2
        self.launch_ros2_files()
        # Esperar un tiempo prudencial para que todo se inicie
        time.sleep(10)  # Ajusta este tiempo según sea necesario
        # Actualizar el botón
        self.calibration_complete = True
        self.after(0, lambda: self.navigation_button.configure(
            text="Buscar productos",
            state="normal"
        ))

                
    def launch_ros2_files(self):
        launch_commands = [
            "ros2 launch turtlemart mux.launch.py",
        ]

        if self.nav_mode == "Real":
            launch_commands.append("ros2 launch turtlemart real_nav.launch.py")
        else:
            launch_commands.append("ros2 launch turtlemart navagv.launch.py")
        
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
       # basic_control_thread = threading.Thread(target=self.launch_basic_control)
        #basic_control_thread.start()
    
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
        try:
            navigator_class = RealNavigator if self.nav_mode == "Real" else SimNavigator
            navigator = navigator_class()
            waypoints_reached = navigator.navigate()
            print("Navegación completa")
            if not self.is_closing:
                self.publish_continue_nav()
                # Use after_idle for thread-safe GUI updates
                self.after_idle(lambda: self.navigation_button.configure(state="normal"))

        except rclpy.executors.ExternalShutdownException:
            print("ROS shutdown during navigation")
        except Exception as e:
            print(f"Error en la navegación: {e}")
            if not self.is_closing:
                self.after_idle(lambda: self.navigation_button.configure(state="normal"))
            
    def __del__(self):
        # Terminate launch file processes when the window is closed
        for process in self.launch_processes:
            process.terminate()
        
        if self.launch_thread and self.launch_thread.is_alive():
            self.launch_thread.join(timeout=1)  # Dar tiempo para que termine
            
        if self.node:
            self.node.destroy_node()
        rclpy.shutdown()

    def perform_alternate_action(self):
        self.navigation_button.configure(state="disabled")
        self.publish_continue_nav()
        self.continue_nav_published = True
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
    


    def cleanup_ros(self):
        print("Cleaning up ROS...")
        try:
            if self.executor:
                self.executor.shutdown()
            if self.node:
                self.node.destroy_node()
            if rclpy.ok() and self.ros_initialized:
                rclpy.shutdown()
        except Exception as e:
            print(f"Error during ROS cleanup: {e}")


    def on_closing(self):
        if self.is_closing:  # Prevent multiple closes
            return

        print("Initiating clean shutdown...")
        self.is_closing = True  # Set closing flag
        
        # Cancel all pending after callbacks
        for after_id in self.tk.eval('after info').split():
            try:
                self.after_cancel(after_id)
            except Exception as e:
                print(f"Error canceling after callback: {e}")
        
        # Stop ROS processes
        try:
            self.stop_ros_processes()
            self.cleanup_ros()
            for thread in self.threads:
                if thread.is_alive():
                    thread.join(timeout=2.0)

        except Exception as e:
            print(f"Error during thread cleanup: {e}")
        
        # Close matplotlib figure
        try:
            plt.close(self.fig)
        except Exception as e:
            print(f"Error closing matplotlib figure: {e}")
        
        # Destroy the window
        try:
            self.quit()
            self.destroy()
        except Exception as e:
            print(f"Error destroying window: {e}")

# Nueva función que manejará la acción del nuevo botón
    def start_thread(self, target, *args, **kwargs):
        """Helper method to start and track threads"""
        thread = threading.Thread(target=target, args=args, kwargs=kwargs)
        thread.daemon = True
        self.threads.append(thread)
        thread.start()
        return thread

    def view_selected_products(self):
        self.selected_frame.destroy()
        self.selected_frame = ctk.CTkFrame(self.right_frame, width=250, fg_color=self.colors['list_bg'])
        self.selected_frame.pack(side=ctk.TOP, fill=ctk.BOTH, expand=True, padx=10, pady=10)

        # Título con fuente constante
        title_label = ctk.CTkLabel(
            self.selected_frame, 
            text="Lista de productos",
            text_color=self.colors['text_primary'],
            font=self.PRODUCT_TITLE_FONT
        )
        title_label.pack(pady=(10, 15))
        db_dir = get_source_db_path('turtlemart', 'products.db')
        conn = sqlite3.connect(db_dir)
        cursor = conn.cursor()
        cursor.execute('SELECT name FROM selected_products')
        rows = cursor.fetchall()
        conn.close()

        products_container = ctk.CTkScrollableFrame(
            self.selected_frame,
            fg_color="transparent",
            height=400
        )
        products_container.pack(fill=ctk.BOTH, expand=True, padx=5, pady=5)

        # Limpiar el diccionario de labels antiguos
        self.product_labels.clear()

        for row in rows:
            product_name = row[0]
            product_container = ctk.CTkFrame(products_container, fg_color=self.colors['list_bg'], height=50)
            product_container.pack(fill=ctk.X, pady=10, padx=15)
            product_container.pack_propagate(False)

            # Usar el color guardado en el diccionario si existe, si no usar el color por defecto
            text_color = self.product_colors.get(product_name, self.colors['text_primary'])
            
            label = ctk.CTkLabel(
                product_container, 
                text=product_name,
                text_color=text_color,
                font=self.PRODUCT_FONT
            )
            label.pack(expand=True)
            self.product_labels[product_name] = label


    def refresh_product_list(self):
        # Llamar a view_selected_products para actualizar la lista completa
        self.view_selected_products()

def get_source_db_path(package_name, db_filename):
    """
    Obtiene la ruta a la base de datos en el directorio src del paquete
    """
    # Obtener el directorio share del paquete
    share_dir = get_package_share_directory(package_name)
    
    # Navegar hasta la raíz del workspace (subir 4 niveles: share/package/install/workspace)
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(share_dir))))
    
    # Construir la ruta a la base de datos en src
    db_path = os.path.join(workspace_root, 'src', package_name, 'database', db_filename)
    
    #print(f"Trying to access database at: {db_path}")
    
    return db_path

if __name__ == "__main__":
    app = RealNavWindow()
    app.mainloop()