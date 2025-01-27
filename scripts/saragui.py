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


class ProductManager:
    def __init__(self, root):
        self.root = root
        self.checkbox_vars = {}
        self.after_id = None
        self.navigation_window = None

        # Configuración de la interfaz
        self.setup_ui()
        signal.signal(signal.SIGINT, self.signal_handler)

        # Configurar el cierre de ventana con la "X"
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)


    def setup_ui(self):
        ctk.set_appearance_mode("light")
        #ctk.set_default_color_theme("blue")
        self.root.configure(fg_color="blanched almond")


        self.root.title("Smart Autonomous Retail Assistant")
        self.root.geometry("%dx%d+0+0" % (self.root.winfo_screenwidth(), self.root.winfo_screenheight()))
        self.root.resizable(width=1, height=1)

        # Frame en la parte superior
        top_frame = ctk.CTkFrame(self.root, height=200,fg_color="bisque2")
        top_frame.pack(side=ctk.TOP, fill=ctk.X)
        title_label = ctk.CTkLabel(top_frame, text="Bienvenido, escoge el producto que desee buscar", font=("Arial", 35, "italic", "bold"))
        title_label.pack(padx=20, pady=10)

        # Frame a la izquierda
        left_frame = ctk.CTkFrame(self.root, width=200, fg_color="bisque2")
        left_frame.pack(side=ctk.LEFT, fill=ctk.Y)
        
        pkg_dir = get_package_share_directory('turtlemart')
        image_path = os.path.join(pkg_dir, 'images/Usuario.png')
        img = Image.open(image_path)
        img = img.resize((120, 150))
        ctk_image = ctk.CTkImage(light_image=img, size=(120, 150))
        label = ctk.CTkLabel(left_frame, image=ctk_image, text="")
        label.pack(padx=30, pady=30)

        self.label_fecha = ctk.CTkLabel(left_frame, font=('ARIAL', 25, 'bold'))
        self.label_fecha.pack(side=ctk.TOP, padx=10, pady=10)

        # Reloj
        self.label_reloj = ctk.CTkLabel(left_frame, font=('ARIAL', 25, 'bold'))
        self.label_reloj.pack(side=ctk.TOP, padx=10, pady=70)

        # Fecha

        # Actualizar reloj y fecha
        self.actualizar_reloj_y_fecha()

        # Texto SARA
        shop_vision_label = ctk.CTkLabel(left_frame, text="SARA", font=('Helvetica', 35, 'bold'))
        shop_vision_label.pack(side=ctk.BOTTOM, padx=10, pady=10)

        # Frame de búsqueda dentro del frame izquierdo
        search_frame = ctk.CTkFrame(left_frame, fg_color="bisque2")
        search_frame.pack(side=ctk.TOP, fill=ctk.X, padx=10, pady=10)

        # Campo de entrada para búsqueda
        self.search_entry = ctk.CTkEntry(search_frame, font=("Arial", 30))
        self.search_entry.pack(side=ctk.TOP, padx=10, pady=5)

        # Botón de búsqueda
        search_button = ctk.CTkButton(search_frame, text="Buscar", command=self.perform_search, fg_color="blanched almond", text_color="black", hover_color="bisque2", font=("Arial", 20))
        search_button.pack(side=ctk.TOP, padx=10, pady=5)

        # Frame para lista de productos
        frame1 = ctk.CTkFrame(self.root, width=350, fg_color="peachpuff")
        frame1.pack(side=ctk.LEFT, fill=ctk.BOTH, padx=20, pady=20, expand=True)

        # Frame para productos seleccionados
        frame2 = ctk.CTkFrame(self.root, width=350, fg_color="peachpuff")
        frame2.pack(side=ctk.RIGHT, fill=ctk.BOTH, padx=20, pady=20, expand=True)

        # Título para la lista de productos
        products_label = ctk.CTkLabel(frame1, text="Productos", font=("Helvetica", 24, "bold"))
        products_label.pack(pady=10)

        # Scrollable Frame para la lista de productos
        self.treeview_frame = ctk.CTkScrollableFrame(frame1, width=300, height=400, fg_color="Ivory")
        self.treeview_frame.pack(fill=ctk.BOTH, expand=True, pady=10)

        select_button = ctk.CTkButton(frame1, text="Seleccionar productos", command=self.select_products, width=200, height=50, font=("Helvetica", 16), fg_color="blanched almond", text_color="black", hover_color="bisque2")
        select_button.pack(pady=10)

        # Título para productos seleccionados
        selected_label = ctk.CTkLabel(frame2, text="Productos Seleccionados", font=("Helvetica", 24, "bold"))
        selected_label.pack(pady=10)

        self.selected_frame = ctk.CTkScrollableFrame(frame2, width=300, height=400, fg_color="Ivory")
        self.selected_frame.pack(fill=ctk.BOTH, expand=True, pady=10)

    # Frame for option menu
        option_frame = ctk.CTkFrame(left_frame, fg_color="bisque2" )
        option_frame.pack(side=ctk.TOP, fill=ctk.X, padx=10, pady=5)

    # Create a variable to store the selected option
        self.navigation_mode = ctk.StringVar()
        self.navigation_mode.set("Simulacion")  # Default value

    # Create an option menu
        option_menu = ctk.CTkOptionMenu(option_frame, values=["Simulacion", "Real"], variable=self.navigation_mode, fg_color="white", button_hover_color="bisque2", button_color="bisque2", text_color="black", font=("Arial", 20))
        option_menu.pack(side=ctk.LEFT, padx=10, pady=5)


        go_to_products_button = ctk.CTkButton(frame2, text="Dirigirse a productos", command=self.open_navigation_window, width=200, height=50, font=("Helvetica", 16), fg_color="blanched almond", text_color='black', hover_color="bisque2")
        go_to_products_button.pack(pady=10)

        self.refresh_treeview()

    def show_info(self, message, title="Info"):
        info_window = ctk.CTkToplevel()
        info_window.title(title)
        info_window.geometry("300x150")
        label = ctk.CTkLabel(info_window, text=message, padx=20, pady=20)
        label.pack(expand=True)
        ok_button = ctk.CTkButton(info_window, text="OK", command=info_window.destroy, fg_color="bisque2", text_color='black')
        ok_button.pack(pady=10)

    def refresh_treeview(self, search_term=""):
        # Limpiar el frame de productos
        for widget in self.treeview_frame.winfo_children():
            widget.destroy()

        db_dir = os.path.join('src/turtlemart/database/products.db')
        connection = sqlite3.connect(db_dir)
        cursor = connection.cursor()

        # Modificar la consulta para filtrar por el término de búsqueda
        cursor.execute("SELECT * FROM products WHERE name LIKE ?", ('%' + search_term + '%',))

        for row in cursor.fetchall():
            if row[0] not in self.checkbox_vars:
                self.checkbox_vars[row[0]] = ctk.IntVar()
            checkbox = ctk.CTkCheckBox(self.treeview_frame,font=('Helvetica', 26, 'bold'), text=f"{row[1]}", variable=self.checkbox_vars[row[0]])
            checkbox.pack(anchor="w", padx=10, pady=5)
            # Restaurar el estado del checkbox
            if self.checkbox_vars[row[0]].get():
                checkbox.select()
                checkbox.configure(fg_color="bisque2")
                
            else:
                checkbox.deselect()
                checkbox.configure(fg_color="bisque2")

        connection.close()
        
    def select_products(self):
        selected_items = [key for key, var in self.checkbox_vars.items() if var.get()]
        if selected_items:
            db_dir = os.path.join('src/turtlemart/database/products.db')
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
            self.show_info("Products selected successfully.")
            self.view_selected_products()
        else:
            self.show_info("No items selected.", title="Error")

    def view_selected_products(self):
        for widget in self.selected_frame.winfo_children():
            widget.destroy()

        db_dir = os.path.join('src/turtlemart/database/products.db')
        connection = sqlite3.connect(db_dir)
        cursor = connection.cursor()
        cursor.execute("SELECT * FROM selected_products")
        for row in cursor.fetchall():
            ctk.CTkLabel(self.selected_frame, font=('Helvetica', 26, 'bold'), text=f"{row[0]}").pack(anchor="c", padx=10, pady=5)
        connection.close()

    def perform_search(self):
        search_term = self.search_entry.get()
        self.refresh_treeview(search_term)

    def actualizar_reloj_y_fecha(self):
        now = datetime.datetime.now()
        self.label_reloj.configure(text=now.strftime("%H:%M:%S"))
        self.label_fecha.configure(text=now.strftime("%Y-%m-%d"))
        self.after_id = self.root.after(1000, self.actualizar_reloj_y_fecha)  # Guarda el ID del after

    def open_navigation_window(self):
        if self.after_id is not None:
            self.root.after_cancel(self.after_id) 
            # En lugar de crear diferentes ventanas, usa la ventana unificada
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

