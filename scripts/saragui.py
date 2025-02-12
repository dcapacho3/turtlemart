#!/usr/bin/env python3

import sqlite3
import customtkinter as ctk
from tkinter import simpledialog
import os
import datetime
from navigationgui import NavigationWindow
import signal
from base_navgui import UnifiedNavigationWindow
from PIL import Image
from ament_index_python.packages import get_package_share_directory

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
        

class ProductManager:
    def __init__(self, root):
        self.show_mode_selector = True 
        #self.show_mode_selector = False 
        self.root = root
        self.checkbox_vars = {}
        self.after_id = None
        self.navigation_window = None
        

        # Definición de la paleta de colores
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

        # Configuración de la interfaz
        self.setup_ui()
        signal.signal(signal.SIGINT, self.signal_handler)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)


    def setup_ui(self):
        ctk.set_appearance_mode("light")
        self.root.configure(fg_color=self.colors['primary_bg'])
        self.root.title("Smart Autonomous Retail Assistant")
        self.root.geometry("%dx%d+0+0" % (self.root.winfo_screenwidth(), self.root.winfo_screenheight()))
        self.root.resizable(width=1, height=1)

        # Frame izquierdo para imagen y controles
        left_frame = ctk.CTkFrame(self.root, width=400, fg_color=self.colors['secondary_bg'])
        left_frame.pack(side=ctk.LEFT, fill=ctk.Y)
        left_frame.pack_propagate(False)

        # Frame derecho contenedor
        right_container = ctk.CTkFrame(self.root, fg_color=self.colors['primary_bg'])
        right_container.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True)

        # Frame superior con reloj/fecha
        top_frame = ctk.CTkFrame(right_container, height=100, fg_color=self.colors['secondary_bg'])
        top_frame.pack(fill=ctk.X, padx=10, pady=10)

        # Frame para reloj y fecha
        clock_frame = ctk.CTkFrame(top_frame, fg_color=self.colors['secondary_bg'])
        clock_frame.pack(side=ctk.RIGHT, padx=20)
        
        now = datetime.datetime.now()

        self.label_fecha = ctk.CTkLabel(
            clock_frame, 
            text=now.strftime("%Y-%m-%d"),
            font=('ARIAL', 25, 'bold'),
            text_color=self.colors['text_primary']
        )
        self.label_fecha.pack(side=ctk.TOP, pady=5)
        
        self.label_reloj = ctk.CTkLabel(
            clock_frame, 
            text=now.strftime("%H:%M:%S"),
            font=('ARIAL', 25, 'bold'),
            text_color=self.colors['text_primary']
        )
        self.label_reloj.pack(side=ctk.TOP, pady=5)



        # Imagen de usuario
        pkg_dir = get_package_share_directory('turtlemart')
        image_path = os.path.join(pkg_dir, 'images/userlogo.png')
        img = Image.open(image_path)
        size = 300

        image_frame = ctk.CTkFrame(left_frame, width=size, height=size, fg_color=self.colors['secondary_bg'])
        image_frame.pack(padx=50, pady=30)
        image_frame.pack_propagate(False)  # Mantener el tamaño del frame

        img = img.resize((size, size), Image.LANCZOS)
        
        ctk_image = ctk.CTkImage(light_image=img, size=(size, size))
        label = ctk.CTkLabel(image_frame, image=ctk_image, text="")
        label.pack(expand=True, fill="both")


        # Configuración del selector de modo 
        option_frame = ctk.CTkFrame(left_frame, fg_color=self.colors['secondary_bg'])
        option_frame.pack(expand=True, fill=ctk.BOTH, padx=20, pady=5)

        option_center_frame = ctk.CTkFrame(option_frame, fg_color=self.colors['secondary_bg'])
        option_center_frame.pack(expand=True)

        welcome_label = ctk.CTkLabel(
            option_center_frame, 
            text="¡Bienvenido!", 
            font=("Arial", 36, "italic", "bold"),
            text_color=self.colors['text_primary']
        )
        welcome_label.pack(pady=(0, 30))

        self.navigation_mode = ctk.StringVar()
        self.navigation_mode.set("Real")
            

        if self.show_mode_selector:
            mode_label = ctk.CTkLabel(
                option_center_frame, 
                text="Que modo desea usar?", 
                font=("Arial", 28, "bold"),
                text_color=self.colors['text_primary'],
                wraplength=300
            )
            mode_label.pack(pady=(0, 20))


            option_menu = ctk.CTkOptionMenu(
                option_center_frame, 
                values=["Real", "Simulacion"], 
                variable=self.navigation_mode, 
                fg_color=self.colors['optionmenu_bg'],
                button_color=self.colors['optionmenu_button'],
                button_hover_color=self.colors['optionmenu_hover'],
                text_color=self.colors['optionmenu_text'],
                font=("Arial", 26),
                width=200,  # Añadido ancho específico
                height=40   # Añadido alto específico
            )
            option_menu.pack()

        # Texto SARA
        shop_vision_label = ctk.CTkLabel(
            left_frame, 
            text="SARA", 
            font=('Helvetica', 48, 'bold'),
            text_color=self.colors['text_primary']
        )
        shop_vision_label.pack(side=ctk.BOTTOM, padx=20, pady=20)

        # Frame principal de contenido
        main_content = ctk.CTkFrame(right_container, fg_color=self.colors['primary_bg'])
        main_content.pack(fill=ctk.BOTH, expand=True, padx=10, pady=10)

        # Frame de búsqueda
        top_search_frame = ctk.CTkFrame(main_content, fg_color=self.colors['primary_bg'], height=40)
        top_search_frame.pack(fill=ctk.X, padx=20, pady=(0, 10))
        top_search_frame.pack_propagate(False)

        search_container = ctk.CTkFrame(top_search_frame, fg_color=self.colors['primary_bg'])
        search_container.pack(expand=True)

        self.search_frame = ctk.CTkFrame(search_container, fg_color=self.colors['primary_bg'])
        self.search_frame.pack()

        # Entry de búsqueda y botón buscar
        self.search_entry = ctk.CTkEntry(
            self.search_frame, 
            font=("Arial", 20),
            width=550,
            height=40,
            fg_color=self.colors['entry_bg']
        )
        self.search_entry.pack(side=ctk.LEFT, padx=(0, 10))

        self.search_button = ctk.CTkButton(
            self.search_frame,
            text="Buscar",
            command=self.perform_search,
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover'],
            font=("Arial", 20, "bold"),
            width=200,
            height=40
        )
        self.search_button.pack(side=ctk.LEFT)

        # Bindear evento Enter para buscar
        self.search_entry.bind('<Return>', lambda e: self.perform_search())
        self.search_frame.bind('<Button-1>', lambda e: 'break')

        # Frame para lista de productos
        frame1 = ctk.CTkFrame(main_content, fg_color=self.colors['frame_bg'])
        frame1.pack(fill=ctk.BOTH, padx=20, pady=(0, 10), expand=True)

        products_label = ctk.CTkLabel(
            frame1, 
            text="Productos disponibles", 
            font=("Helvetica", 24, "bold"),
            text_color=self.colors['text_primary']
        )
        products_label.pack(pady=10)

        self.treeview_frame = ctk.CTkScrollableFrame(
            frame1, 
            width=300, 
            height=200, 
            fg_color=self.colors['scrollable_frame_bg']
        )
        self.treeview_frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=10)

        select_button = ctk.CTkButton(
            frame1, 
            text="Seleccionar productos", 
            command=self.select_products, 
            width=200, 
            height=50, 
            font=("Helvetica", 16), 
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover']
        )
        select_button.pack(pady=10)

        # Frame para productos seleccionados
        frame2 = ctk.CTkFrame(main_content, fg_color=self.colors['frame_bg'])
        frame2.pack(fill=ctk.BOTH, padx=20, pady=(10, 20), expand=True)

        selected_label = ctk.CTkLabel(
            frame2, 
            text="Productos Seleccionados", 
            font=("Helvetica", 24, "bold"),
            text_color=self.colors['text_primary']
        )
        selected_label.pack(pady=10)

        self.selected_frame = ctk.CTkScrollableFrame(
            frame2, 
            width=300, 
            height=200, 
            fg_color=self.colors['scrollable_frame_bg']
        )
        self.selected_frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=10)

        go_to_products_button = ctk.CTkButton(
            frame2, 
            text="Dirigirse a productos", 
            command=self.open_navigation_window, 
            width=200, 
            height=50, 
            font=("Helvetica", 16), 
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover']
        )
        go_to_products_button.pack(pady=10)

        self.actualizar_reloj_y_fecha()
        self.refresh_treeview()

    def show_info(self, message, title="Info"):
        info_window = ctk.CTkToplevel()
        info_window.title(title)
        info_window.geometry("300x150")
        
        label = ctk.CTkLabel(
            info_window, 
            text=message, 
            text_color=self.colors['text_primary'],
            padx=20, 
            pady=20
        )
        label.pack(expand=True)
        
        ok_button = ctk.CTkButton(
            info_window, 
            text="OK", 
            command=info_window.destroy, 
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover']
        )
        ok_button.pack(pady=10)
    



    def refresh_treeview(self, search_term=""):
        # Limpiar la vista actual
        for widget in self.treeview_frame.winfo_children():
            widget.destroy()

        try:
            db_dir = get_source_db_path('turtlemart', 'products.db')
            connection = sqlite3.connect(db_dir)
            cursor = connection.cursor()

            # Añadir logging para debug
            print(f"Searching for: '{search_term}'")
            
            # Modificar la consulta para ser case-insensitive
            cursor.execute("SELECT * FROM products WHERE LOWER(name) LIKE LOWER(?)", 
                        ('%' + search_term + '%',))
            
            results = cursor.fetchall()
            print(f"Found {len(results)} results")

            if not results:
                # Mostrar mensaje cuando no hay resultados
                no_results_label = ctk.CTkLabel(
                    self.treeview_frame,
                    text="No se encontraron productos",
                    font=('Helvetica', 20),
                    text_color=self.colors['text_primary']
                )
                no_results_label.pack(pady=20)
            
            for row in results:
                if row[0] not in self.checkbox_vars:
                    self.checkbox_vars[row[0]] = ctk.IntVar()
                
                checkbox = ctk.CTkCheckBox(
                    self.treeview_frame,
                    font=('Helvetica', 26, 'bold'),
                    text=f"{row[1]}", 
                    variable=self.checkbox_vars[row[0]],
                    fg_color=self.colors['checkbox_bg'],
                    hover_color=self.colors['checkbox_hover']
                )
                checkbox.pack(anchor="w", padx=10, pady=5)

                if self.checkbox_vars[row[0]].get():
                    checkbox.select()
                else:
                    checkbox.deselect()

        except sqlite3.Error as e:
            print(f"Database error: {e}")
            error_label = ctk.CTkLabel(
                self.treeview_frame,
                text=f"Error de base de datos: {str(e)}",
                font=('Helvetica', 20),
                text_color='red'
            )
            error_label.pack(pady=20)
        except Exception as e:
            print(f"Error: {e}")
            error_label = ctk.CTkLabel(
                self.treeview_frame,
                text=f"Error: {str(e)}",
                font=('Helvetica', 20),
                text_color='red'
            )
            error_label.pack(pady=20)
        finally:
            if 'connection' in locals():
                connection.close()


    def select_products(self):
        selected_items = [key for key, var in self.checkbox_vars.items() if var.get()]
        if selected_items:
            db_dir = get_source_db_path('turtlemart', 'products.db')
            connection = sqlite3.connect(db_dir)
            cursor = connection.cursor()
            
            cursor.execute("DELETE FROM selected_products")
            
            for product_id in selected_items:
                cursor.execute("SELECT name, x, y FROM products WHERE id = ?", (product_id,))
                product_info = cursor.fetchone()
                cursor.execute("INSERT INTO selected_products (name, x, y) VALUES (?, ?, ?)",
                               (product_info[0], product_info[1], product_info[2]))
            
            connection.commit()
            connection.close()
            self.view_selected_products()
        else:
            self.show_info("No items selected.", title="Error")

    def view_selected_products(self):
        for widget in self.selected_frame.winfo_children():
            widget.destroy()

        db_dir = get_source_db_path('turtlemart', 'products.db')
        connection = sqlite3.connect(db_dir)
        cursor = connection.cursor()
        cursor.execute("SELECT * FROM selected_products")
        
        for row in cursor.fetchall():
            ctk.CTkLabel(
                self.selected_frame, 
                text=f"{row[0]}", 
                font=('Helvetica', 26, 'bold'),
                text_color=self.colors['text_primary']
            ).pack(anchor="c", padx=10, pady=5)
        connection.close()

    def perform_search(self):
        """Realiza la búsqueda basada en el texto ingresado"""
        search_term = self.search_entry.get().strip()  # Eliminar espacios en blanco
        print(f"Performing search with term: '{search_term}'")  # Debug log
        self.refresh_treeview(search_term)

    def actualizar_reloj_y_fecha(self):
        now = datetime.datetime.now()
        self.label_reloj.configure(text=now.strftime("%H:%M:%S"))
        self.label_fecha.configure(text=now.strftime("%Y-%m-%d"))
        self.after_id = self.root.after(1000, self.actualizar_reloj_y_fecha)

    def open_navigation_window(self):
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
            self.navigation_window = UnifiedNavigationWindow(self, self.navigation_mode.get())
            self.root.withdraw()
            self.navigation_window.mainloop()

    def signal_handler(self, sig, frame):
        print("Ctrl+C detectado, cerrando la aplicación...")
        self.on_closing()

    def on_closing(self):
        """Método para manejar el cierre controlado de la ventana."""
        print("Cerrando la ventana correctamente...")
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
        if self.navigation_window:
            self.navigation_window.destroy()
        self.root.quit()
        self.root.destroy()
        
    def deiconify(self):
        self.root.deiconify()

if __name__ == '__main__':
    root = ctk.CTk()
    app = ProductManager(root)
    root.mainloop()