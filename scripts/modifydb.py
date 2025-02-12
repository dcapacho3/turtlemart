#!/usr/bin/env python3

import sqlite3
import customtkinter as ctk
from tkinter import messagebox
import os
import datetime
import signal
import sys
from PIL import Image
from ament_index_python.packages import get_package_share_directory
from object_recorder_gui import SecondWindow  



def get_source_db_path(package_name, db_filename):
    share_dir = get_package_share_directory(package_name)
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(share_dir))))
    db_path = os.path.join(workspace_root, 'src', package_name, 'database', db_filename)
    return db_path

def show_virtual_keyboard(entry_widget, parent, colors):
    keyboard = CTkVirtualKeyboard(parent, entry_widget, colors)
    parent.wait_window(keyboard)

class CTkVirtualKeyboard(ctk.CTkToplevel):
    def __init__(self, parent, entry_widget, colors):
        super().__init__(parent)
        
        self.entry_widget = entry_widget
        self.colors = colors
        
        # Configuración de la ventana
        self.title("Teclado Virtual")
        self.configure(fg_color=colors['secondary_bg'])
        
        # Centrar la ventana
        window_width = 800
        window_height = 380
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        center_x = int(screen_width/2 - window_width/2)
        center_y = int(screen_height/2 - window_height/2)
        self.geometry(f'{window_width}x{window_height}+{center_x}+{center_y}')
        
        # Frame principal con padding
        self.main_frame = ctk.CTkFrame(self, fg_color=colors['secondary_bg'])
        self.main_frame.pack(expand=True, fill='both', padx=20, pady=20)
        
        # Mostrar el texto actual
        self.create_display()
        
        # Crear el teclado
        self.create_keyboard()
        
        # Hacer la ventana modal
        self.transient(parent)
        self.grab_set()
        
    def create_display(self):
        # Frame para el display
        display_frame = ctk.CTkFrame(self.main_frame, fg_color=self.colors['primary_bg'])
        display_frame.pack(fill='x', pady=(0, 10))
        
        # Entry para mostrar el texto
        self.display = ctk.CTkEntry(
            display_frame,
            fg_color=self.colors['entry_bg'],
            text_color=self.colors['text_primary'],
            height=40,
            font=("Helvetica", 18)
        )
        self.display.pack(fill='x', padx=10, pady=10)
        self.display.insert(0, self.entry_widget.get())
        
    def create_keyboard(self):
        # Layout del teclado
        layouts = {
            'default': [
                ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0'],
                ['Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P'],
                ['A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', 'Ñ'],
                ['Z', 'X', 'C', 'V', 'B', 'N', 'M', '.', '-', '_']
            ],
            'symbols': [
                ['!', '@', '#', '$', '%', '^', '&', '*', '(', ')'],
                ['+', '=', '{', '}', '[', ']', '|', '\\', ':', ';'],
                ['<', '>', '?', '/', '"', "'", ',', '.', '~', '`'],
                ['¡', '¿', '°', '¬', '€', '£', '¥', '§', '¶', '©']
            ]
        }
        
        self.current_layout = 'default'
        
        # Frame para el teclado
        keyboard_frame = ctk.CTkFrame(self.main_frame, fg_color=self.colors['secondary_bg'])
        keyboard_frame.pack(expand=True, fill='both')
        
        # Crear filas para el teclado
        for row in layouts['default']:
            row_frame = ctk.CTkFrame(keyboard_frame, fg_color=self.colors['secondary_bg'])
            row_frame.pack(expand=True, fill='x', pady=2)
            
            for char in row:
                btn = ctk.CTkButton(
                    row_frame,
                    text=char,
                    command=lambda c=char: self.add_character(c),
                    width=60,
                    height=45,
                    fg_color=self.colors['button_bg'],
                    text_color=self.colors['button_text'],
                    hover_color=self.colors['button_hover'],
                    font=("Helvetica", 16)
                )
                btn.pack(side='left', padx=2, expand=True)
        
        # Frame para botones especiales
        special_frame = ctk.CTkFrame(keyboard_frame, fg_color=self.colors['secondary_bg'])
        special_frame.pack(expand=True, fill='x', pady=2)
        
        # Botones especiales
        special_buttons = [
            ("⌫ Borrar", self.backspace, 120),
            ("Espacio", lambda: self.add_character(' '), 300),
            ("123/#!", self.toggle_layout, 120),
            ("✓ Aceptar", self.accept, 120)
        ]
        
        for text, command, width in special_buttons:
            btn = ctk.CTkButton(
                special_frame,
                text=text,
                command=command,
                width=width,
                height=45,
                fg_color=self.colors['button_bg'],
                text_color=self.colors['button_text'],
                hover_color=self.colors['button_hover'],
                font=("Helvetica", 16)
            )
            btn.pack(side='left', padx=2, expand=True if text == "Espacio" else False)
            
    def add_character(self, char):
        current_pos = self.display.index(ctk.INSERT)
        self.display.insert(current_pos, char)
        
    def backspace(self):
        current_pos = self.display.index(ctk.INSERT)
        if current_pos > 0:
            self.display.delete(current_pos - 1)
            
    def toggle_layout(self):
        # Implementar el cambio entre layouts si se desea
        pass
        
    def accept(self):
        text = self.display.get()
        self.entry_widget.delete(0, 'end')
        self.entry_widget.insert(0, text)
        self.destroy()
        
    def cancel(self):
        self.destroy()



class ProductInputDialog(ctk.CTkToplevel):
    def __init__(self, parent, title, colors, product_data=None):
        self.is_closing = False
        self.root = root
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)


        self.second_window = None
        super().__init__(parent)
        self.colors = colors
        self.title(title)
        self.result = None
        self.product_data = product_data
        
        window_width = 400
        window_height = 350
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        center_x = int(screen_width/2 - window_width/2)
        center_y = int(screen_height/2 - window_height/2)
        self.geometry(f'{window_width}x{window_height}+{center_x}+{center_y}')
        
        self.configure(fg_color=self.colors['secondary_bg'])
        self.transient(parent)
        
        self.create_widgets()
        
        if self.product_data:
            self.name_entry.insert(0, self.product_data[1])
            self.x_entry.insert(0, str(self.product_data[2]))
            self.y_entry.insert(0, str(self.product_data[3]))
            
        self.wait_visibility()
        self.grab_set()
        
    
    def create_entry_with_keyboard(self, parent, placeholder="", width=200):
        """Helper function to create an entry with a keyboard button"""
        frame = ctk.CTkFrame(parent, fg_color="transparent")
        frame.pack(pady=(0, 15))
        
        entry = ctk.CTkEntry(
            frame,
            font=("Helvetica", 16),
            width=width,
            fg_color=self.colors['entry_bg'],
            placeholder_text=placeholder
        )
        entry.pack(side='left', padx=(0, 5))
        
        keyboard_btn = ctk.CTkButton(
            frame,
            text="⌨",
            command=lambda: show_virtual_keyboard(entry, self, self.colors),
            width=40,
            height=32,
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover']
        )
        keyboard_btn.pack(side='left')
        
        return entry


        
    def create_widgets(self):
        # Product name entry
        name_label = ctk.CTkLabel(
            self,
            text="Nombre del producto:",
            font=("Helvetica", 16),
            text_color=self.colors['text_primary']
        )
        name_label.pack(pady=(20, 5))
        
        self.name_entry = self.create_entry_with_keyboard(self, "Ingrese el nombre del producto")
        
        # X coordinate entry
        x_label = ctk.CTkLabel(
            self,
            text="Coordenada X:",
            font=("Helvetica", 16),
            text_color=self.colors['text_primary']
        )
        x_label.pack(pady=(5, 5))
        
        self.x_entry = self.create_entry_with_keyboard(self, "Ingrese la coordenada X")
        
        # Y coordinate entry
        y_label = ctk.CTkLabel(
            self,
            text="Coordenada Y:",
            font=("Helvetica", 16),
            text_color=self.colors['text_primary']
        )
        y_label.pack(pady=(5, 5))
        
        self.y_entry = self.create_entry_with_keyboard(self, "Ingrese la coordenada Y")
        
        # Buttons frame
        button_frame = ctk.CTkFrame(self, fg_color=self.colors['secondary_bg'])
        button_frame.pack(pady=20)
        
        # Accept button
        accept_button = ctk.CTkButton(
            button_frame,
            text="Aceptar",
            command=self.accept,
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover'],
            width=100
        )
        accept_button.pack(side=ctk.LEFT, padx=10)
        
        # Cancel button
        cancel_button = ctk.CTkButton(
            button_frame,
            text="Cancelar",
            command=self.cancel,
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover'],
            width=100
        )
        cancel_button.pack(side=ctk.LEFT, padx=10)

        
    def accept(self):
        try:
            name = self.name_entry.get().strip()
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            
            if not name:
                raise ValueError("El nombre no puede estar vacío")
                
            self.result = (name, x, y)
            self.destroy()
        except ValueError as e:
            messagebox.showerror("Error", str(e))
            
    def on_closing(self):
        """Maneja el cierre de la ventana con la X"""
        self.cancel()
    def cancel(self):
        self.result = None
        self.destroy()

class ModernDialog(ctk.CTkToplevel):
    def __init__(self, parent, title, message, type_="info", colors=None):
        super().__init__(parent)
        self.colors = colors
        self.result = False
        
        self.title(title)
        self.geometry("400x200")
        self.configure(fg_color=self.colors['secondary_bg'])
        
        window_width = 400
        window_height = 200
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        center_x = int(screen_width/2 - window_width/2)
        center_y = int(screen_height/2 - window_height/2)
        self.geometry(f'{window_width}x{window_height}+{center_x}+{center_y}')
        
        self.transient(parent)
        
        message_label = ctk.CTkLabel(
            self,
            text=message,
            font=("Helvetica", 16),
            text_color=self.colors['text_primary'],
            wraplength=350
        )
        message_label.pack(pady=(30, 20))
        
        button_frame = ctk.CTkFrame(self, fg_color=self.colors['secondary_bg'])
        button_frame.pack(pady=20)
        
        if type_ == "confirm":
            accept_button = ctk.CTkButton(
                button_frame,
                text="Aceptar",
                command=self.on_accept,
                fg_color=self.colors['button_bg'],
                text_color=self.colors['button_text'],
                hover_color=self.colors['button_hover'],
                width=100
            )
            accept_button.pack(side=ctk.LEFT, padx=10)
            
            cancel_button = ctk.CTkButton(
                button_frame,
                text="Cancelar",
                command=self.on_cancel,
                fg_color=self.colors['button_bg'],
                text_color=self.colors['button_text'],
                hover_color=self.colors['button_hover'],
                width=100
            )
            cancel_button.pack(side=ctk.LEFT, padx=10)
        else:
            ok_button = ctk.CTkButton(
                button_frame,
                text="Aceptar",
                command=self.on_accept,
                fg_color=self.colors['button_bg'],
                text_color=self.colors['button_text'],
                hover_color=self.colors['button_hover'],
                width=100
            )
            ok_button.pack(padx=10)
        
        self.wait_visibility()
        self.grab_set()
        self.focus_set()
    
    def on_accept(self):
        self.result = True
        self.destroy()
    
    def on_cancel(self):
        self.result = False
        self.destroy()

class ModernProductManager:
    def __init__(self, root):
        self.root = root
        self.is_closing = False
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.second_window = None
        
        self.search_var = ctk.StringVar()
        self.search_var.trace('w', self.on_search_change)
        # Variable para controlar la visibilidad del selector de modo
        self.show_mode_selector = True
        
        self.root = root
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Configurar el manejo de señales
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Configurar llamada periódica para procesar señales
        self.root.after(200, self.check_signal)

        # Color scheme
        self.colors = {
            'primary_bg': "#FEF2F2",
            'secondary_bg': "#FEE2E2",
            'accent_bg': "#FFFFFF",
            'list_bg': "#F8FAFC",
            'frame_bg': "#FEE2E2",
            'scrollable_frame_bg': "#FFFFFF",
            'text_primary': "#7F1D1D",
            'text_secondary': "#DC2626",
            'label_text': "#B91C1C",
            'button_bg': "#DC2626",
            'button_hover': "#991B1B",
            'button_text': "#FFFFFF",
            'entry_bg': "#F8FAFC",
            'checkbox_bg': "#DC2626",
            'checkbox_hover': "#991B1B",
            'optionmenu_bg': "#F8FAFC",
            'optionmenu_button': "#DC2626",
            'optionmenu_hover': "#991B1B",
            'optionmenu_text': "#7F1D1D",
        }
        
        self.setup_ui()

    def check_signal(self):
        """Método para procesar señales periódicamente"""
        if not self.is_closing:
            self.root.after(200, self.check_signal)

    def signal_handler(self, signum, frame):
        """Manejador de señales para SIGINT y SIGTERM"""
        if not self.is_closing:
            print("\nRecibida señal de terminación. Cerrando aplicación...")
            self.cleanup_and_close()


    def cleanup_and_close(self):
        """Realiza la limpieza y cierre de la aplicación"""
        try:
            self.is_closing = True
            
            # Cerrar la ventana secundaria si está abierta
            if self.second_window and self.second_window.winfo_exists():
                self.second_window.destroy()
            
            # Cerrar cualquier conexión a la base de datos que esté abierta
            if hasattr(self, 'connection') and self.connection:
                self.connection.close()
            
            # Destruir la ventana principal
            self.root.quit()
            self.root.destroy()
            
            # Salir del programa
            sys.exit(0)
            
        except Exception as e:
            print(f"Error durante el cierre: {e}")
            sys.exit(1)

        
    def setup_ui(self):
        ctk.set_appearance_mode("light")
        self.root.configure(fg_color=self.colors['primary_bg'])
        self.root.title("Gestor de Base de Datos de Productos")
        self.root.geometry("%dx%d+0+0" % (self.root.winfo_screenwidth(), self.root.winfo_screenheight()))
        
        # Left frame with logo and title
        left_frame = ctk.CTkFrame(self.root, width=400, fg_color=self.colors['secondary_bg'])
        left_frame.pack(side=ctk.LEFT, fill=ctk.Y)
        left_frame.pack_propagate(False)

        # Logo setup
        pkg_dir = get_package_share_directory('turtlemart')
        image_path = os.path.join(pkg_dir, 'images/userlogo.png')
        if os.path.exists(image_path):
            img = Image.open(image_path)
            size = 300
            img = img.resize((size, size), Image.LANCZOS)
            ctk_image = ctk.CTkImage(light_image=img, size=(size, size))
            logo_label = ctk.CTkLabel(left_frame, image=ctk_image, text="")
            logo_label.pack(pady=30)

        # Selector de modo (si está habilitado)
        if self.show_mode_selector:
            mode_label = ctk.CTkLabel(
                left_frame,
                text="¿Qué modo desea usar?",
                font=("Arial", 24, "bold"),
                text_color=self.colors['text_primary']
            )
            mode_label.pack(pady=(20, 10))
            
            self.navigation_mode = ctk.StringVar(value="Real")
            mode_menu = ctk.CTkOptionMenu(
                left_frame,
                values=["Real", "Simulacion"],
                variable=self.navigation_mode,
                fg_color=self.colors['optionmenu_bg'],
                button_color=self.colors['optionmenu_button'],
                button_hover_color=self.colors['optionmenu_hover'],
                text_color=self.colors['optionmenu_text'],
                font=("Arial", 20)
            )
            mode_menu.pack(pady=(0, 20))

        # Title
        title_label = ctk.CTkLabel(
            left_frame,
            text="Donatellos Manager",
            font=('Helvetica', 48, 'bold'),
            text_color=self.colors['text_primary'],
            wraplength=350
        )
        title_label.pack(side=ctk.BOTTOM, pady=20)

        # Right container
        right_container = ctk.CTkFrame(self.root, fg_color=self.colors['primary_bg'])
        right_container.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True)

        # Products section
        products_frame = ctk.CTkFrame(right_container, fg_color=self.colors['frame_bg'])
        products_frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=10)

        products_label = ctk.CTkLabel(
            products_frame,
            text="Productos en Base de Datos",
            font=("Helvetica", 24, "bold"),
            text_color=self.colors['text_primary']
        )
        products_label.pack(pady=10)

        # Add search and refresh functionality
        self.setup_search_and_refresh(products_frame)

        # Products scrollable frame
        self.products_scroll = ctk.CTkScrollableFrame(
            products_frame,
            fg_color=self.colors['scrollable_frame_bg']
        )
        self.products_scroll.pack(fill=ctk.BOTH, expand=True, padx=20, pady=10)

        # Buttons frame with better spacing
        buttons_frame = ctk.CTkFrame(products_frame, fg_color=self.colors['frame_bg'])
        buttons_frame.pack(fill=ctk.X, padx=20, pady=20)

        # Grid layout for buttons
        buttons_frame.grid_columnconfigure((0, 1, 2, 3), weight=1)

        # Action buttons
        actions = [
            ("Añadir Producto", self.add_product),
            ("Actualizar Producto", self.update_product),
            ("Eliminar Producto", self.delete_product),
            ("Localizar Producto con Robot", self.open_second_window)
        ]

        for i, (text, command) in enumerate(actions):
            btn = ctk.CTkButton(
                buttons_frame,
                text=text,
                command=command,
                fg_color=self.colors['button_bg'],
                text_color=self.colors['button_text'],
                hover_color=self.colors['button_hover'],
                font=("Helvetica", 16),
                width=200,
                height=40
            )
            btn.grid(row=0, column=i, padx=20, pady=10)

        self.refresh_products()
        
        if text == "Localizar Producto con Robot":
            self.second_window_button = btn

    def on_closing(self):
        """Maneja el cierre desde la interfaz gráfica"""
        if not self.is_closing:
            dialog = ModernDialog(
                self.root, 
                "Salir", 
                "¿Desea salir de la aplicación?", 
                type_="confirm", 
                colors=self.colors
            )
            self.root.wait_window(dialog)
            if dialog.result:
                self.cleanup_and_close()
    def refresh_products(self):
        for widget in self.products_scroll.winfo_children():
            widget.destroy()

        try:
            db_dir = get_source_db_path('turtlemart', 'products.db')
            connection = sqlite3.connect(db_dir)
            cursor = connection.cursor()
            cursor.execute("SELECT * FROM products ORDER BY id")
            
            self.checkbox_vars = {}  # Reset checkbox variables
            
            for row in cursor.fetchall():
                product_frame = ctk.CTkFrame(
                    self.products_scroll,
                    fg_color=self.colors['scrollable_frame_bg']
                )
                product_frame.pack(fill=ctk.X, padx=5, pady=2)
                
                # Store the complete row data as a attribute of the frame
                product_frame.product_data = row
                
                # Create checkbox variable
                self.checkbox_vars[row[0]] = ctk.IntVar()
                
                # Create checkbox
                checkbox = ctk.CTkCheckBox(
                    product_frame,
                    text=f"{row[1]}",  # Show only the name
                    variable=self.checkbox_vars[row[0]],
                    font=("Helvetica", 16),
                    fg_color=self.colors['checkbox_bg'],
                    hover_color=self.colors['checkbox_hover'],
                    text_color=self.colors['text_primary']
                )
                checkbox.pack(side=ctk.LEFT, padx=10, pady=5)

        except sqlite3.Error as e:
            self.show_error(f"Error de base de datos: {str(e)}")
        finally:
            if 'connection' in locals():
                connection.close()

    def setup_search_and_refresh(self, products_frame):
        # Create a frame for search and refresh
        search_frame = ctk.CTkFrame(products_frame, fg_color=self.colors['frame_bg'])
        search_frame.pack(fill=ctk.X, padx=20, pady=(10, 0))
        
        # Configure grid weights
        search_frame.grid_columnconfigure(0, weight=1)
        search_frame.grid_columnconfigure((1, 2), weight=0)
        
        # Search box with keyboard button
        search_container = ctk.CTkFrame(search_frame, fg_color="transparent")
        search_container.grid(row=0, column=0, sticky="ew", padx=10, pady=10)
        
        search_entry = ctk.CTkEntry(
            search_container,
            placeholder_text="Buscar productos...",
            font=("Helvetica", 16),
            fg_color=self.colors['entry_bg'],
            textvariable=self.search_var
        )
        search_entry.pack(side='left', fill='x', expand=True, padx=(0, 5))
        
        keyboard_button = ctk.CTkButton(
            search_container,
            text="⌨",
            command=lambda: show_virtual_keyboard(search_entry, self.root, self.colors),
            width=40,
            height=32,
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover']
        )
        keyboard_button.pack(side='left')
        
        # Refresh button
        refresh_button = ctk.CTkButton(
            search_frame,
            text="↻ Refrescar",
            command=self.refresh_products,
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover'],
            font=("Helvetica", 16),
            width=120,
            height=32
        )
        refresh_button.grid(row=0, column=2, padx=10, pady=10)

    def on_search_change(self, *args):
        self.refresh_products()

    def refresh_products(self):
        # Clear existing products
        for widget in self.products_scroll.winfo_children():
            widget.destroy()

        try:
            db_dir = get_source_db_path('turtlemart', 'products.db')
            connection = sqlite3.connect(db_dir)
            cursor = connection.cursor()
            
            # Get search term
            search_term = self.search_var.get().strip().lower()
            
            if search_term:
                # Use LIKE query for search
                cursor.execute("""
                    SELECT * FROM products 
                    WHERE LOWER(name) LIKE ? 
                    ORDER BY id
                """, (f'%{search_term}%',))
            else:
                cursor.execute("SELECT * FROM products ORDER BY id")
            
            self.checkbox_vars = {}  # Reset checkbox variables
            
            for row in cursor.fetchall():
                product_frame = ctk.CTkFrame(
                    self.products_scroll,
                    fg_color=self.colors['scrollable_frame_bg']
                )
                product_frame.pack(fill=ctk.X, padx=5, pady=2)
                
                # Store the complete row data as a attribute of the frame
                product_frame.product_data = row
                
                # Create checkbox variable
                self.checkbox_vars[row[0]] = ctk.IntVar()
                
                # Create checkbox
                checkbox = ctk.CTkCheckBox(
                    product_frame,
                    text=f"{row[1]}",  # Show only the name
                    variable=self.checkbox_vars[row[0]],
                    font=("Helvetica", 16),
                    fg_color=self.colors['checkbox_bg'],
                    hover_color=self.colors['checkbox_hover'],
                    text_color=self.colors['text_primary']
                )
                checkbox.pack(side=ctk.LEFT, padx=10, pady=5)

        except sqlite3.Error as e:
            self.show_error(f"Error de base de datos: {str(e)}")
        finally:
            if 'connection' in locals():
                connection.close()

    def add_product(self):
        dialog = ProductInputDialog(self.root, "Añadir Producto", self.colors)
        self.root.wait_window(dialog)
        
        if dialog.result:
            name, x, y = dialog.result
            try:
                db_dir = get_source_db_path('turtlemart', 'products.db')
                connection = sqlite3.connect(db_dir)
                cursor = connection.cursor()
                cursor.execute("INSERT INTO products (name, x, y) VALUES (?, ?, ?)", (name, x, y))
                connection.commit()
                self.refresh_products()
                self.show_info("Producto añadido exitosamente")
            except sqlite3.Error as e:
                self.show_error(f"Error al añadir producto: {str(e)}")
            finally:
                if 'connection' in locals():
                    connection.close()

    def update_product(self):
        selected = [k for k, v in self.checkbox_vars.items() if v.get()]
        if not selected:
            self.show_error("Seleccione un producto para actualizar")
            return
        elif len(selected) > 1:
            self.show_error("Seleccione solo un producto para actualizar")
            return
            
        try:
            db_dir = get_source_db_path('turtlemart', 'products.db')
            connection = sqlite3.connect(db_dir)
            cursor = connection.cursor()
            cursor.execute("SELECT * FROM products WHERE id=?", (selected[0],))
            product_data = cursor.fetchone()
            
            dialog = ProductInputDialog(self.root, "Actualizar Producto", self.colors, product_data)
            self.root.wait_window(dialog)
            
            if dialog.result:
                name, x, y = dialog.result
                cursor.execute("UPDATE products SET name=?, x=?, y=? WHERE id=?", 
                             (name, x, y, selected[0]))
                connection.commit()
                self.refresh_products()
                self.show_info("Producto actualizado exitosamente")
                
        except sqlite3.Error as e:
            self.show_error(f"Error al actualizar producto: {str(e)}")
        finally:
            if 'connection' in locals():
                connection.close()

    def delete_product(self):
        selected = [k for k, v in self.checkbox_vars.items() if v.get()]
        if not selected:
            self.show_error("Seleccione al menos un producto para eliminar")
            return
            
        dialog = ModernDialog(
            self.root,
            "Confirmar eliminación",
            "¿Está seguro de que desea eliminar los productos seleccionados?",
            type_="confirm",
            colors=self.colors
        )
        self.root.wait_window(dialog)
        
        if not dialog.result:
            return
            
        try:
            db_dir = get_source_db_path('turtlemart', 'products.db')
            connection = sqlite3.connect(db_dir)
            cursor = connection.cursor()
            
            for product_id in selected:
                cursor.execute("DELETE FROM products WHERE id=?", (product_id,))
                
            connection.commit()
            self.refresh_products()
            self.show_info("Productos eliminados exitosamente")
        except sqlite3.Error as e:
            self.show_error(f"Error al eliminar productos: {str(e)}")
        finally:
            if 'connection' in locals():
                connection.close()

    def open_second_window(self):
        if self.second_window is None or not self.second_window.winfo_exists():
            mode = self.navigation_mode.get() if self.show_mode_selector else "Real"
            self.second_window = SecondWindow(self.root, mode, self.colors)
            self.second_window.focus()
            # Desactivar el botón y cambiar su apariencia
            self.second_window_button.configure(
                state="disabled",
                fg_color=self.colors['secondary_bg'],
                hover_color=self.colors['secondary_bg'],
                text_color=self.colors['text_secondary']
            )

    def show_error(self, message):
        """Muestra un mensaje de error con el estilo moderno"""
        dialog = ModernDialog(self.root, "Error", message, type_="error", colors=self.colors)
        self.root.wait_window(dialog)

    def show_info(self, message):
        """Muestra un mensaje informativo con el estilo moderno"""
        dialog = ModernDialog(self.root, "Información", message, type_="info", colors=self.colors)
        self.root.wait_window(dialog)
        
    def on_closing(self):
        """Maneja el cierre desde la interfaz gráfica"""
        if not self.is_closing:
            dialog = ModernDialog(
                self.root, 
                "Salir", 
                "¿Desea salir de la aplicación?", 
                type_="confirm", 
                colors=self.colors
            )
            self.root.wait_window(dialog)
            if dialog.result:
                self.cleanup_and_close()

if __name__ == '__main__':
    try:
        root = ctk.CTk()
        app = ModernProductManager(root)
        root.mainloop()
    except KeyboardInterrupt:
        print("\nRecibido Ctrl+C. Cerrando aplicación...")
        if 'app' in locals():
            app.cleanup_and_close()
        sys.exit(0)
    except Exception as e:
        print(f"Error inesperado: {e}")
        sys.exit(1)