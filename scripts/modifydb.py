#!/usr/bin/env python3

import sqlite3
import customtkinter as ctk
from tkinter import simpledialog
import os
import datetime

def show_info(message, title="Info"):
    info_window = ctk.CTkToplevel()
    info_window.title(title)
    info_window.geometry("300x150")
    label = ctk.CTkLabel(info_window, text=message, padx=20, pady=20)
    label.pack(expand=True)
    ok_button = ctk.CTkButton(info_window, text="OK", command=info_window.destroy)
    ok_button.pack(pady=10)

def refresh_treeview(search_term=""):
    # Limpiar el frame de productos
    for widget in treeview_frame.winfo_children():
        widget.destroy()
    
    db_dir = os.path.join('src/turtlemart/database/products.db')
    connection = sqlite3.connect(db_dir)
    cursor = connection.cursor()

    # Modificar la consulta para filtrar por el término de búsqueda
    cursor.execute("SELECT * FROM products WHERE name LIKE ?", ('%' + search_term + '%',))

    for row in cursor.fetchall():
        if row[0] not in checkbox_vars:
            checkbox_vars[row[0]] = ctk.IntVar()
        checkbox = ctk.CTkCheckBox(treeview_frame, text=f"{row[1]} ({row[2]}, {row[3]})", variable=checkbox_vars[row[0]])
        checkbox.pack(anchor="w", padx=10, pady=5)
        # Restaurar el estado del checkbox
        checkbox.select() if checkbox_vars[row[0]].get() else checkbox.deselect()

    connection.close()

def add_product():
    name = simpledialog.askstring("Input", "Enter product name:")
    x = simpledialog.askfloat("Input", "Enter x coordinate:")
    y = simpledialog.askfloat("Input", "Enter y coordinate:")
    if name and x is not None and y is not None:
        db_dir = os.path.join('src/turtlemart/database/products.db')
        connection = sqlite3.connect(db_dir)
        cursor = connection.cursor()
        cursor.execute("INSERT INTO products (name, x, y) VALUES (?, ?, ?)", (name, x, y))
        connection.commit()
        connection.close()
        refresh_treeview()
    else:
        show_info("All fields must be filled out.", title="Error")

def delete_product():
    db_dir = os.path.join('src/turtlemart/database/products.db')
    connection = sqlite3.connect(db_dir)
    cursor = connection.cursor()
    
    selected_items = [key for key, var in checkbox_vars.items() if var.get()]
    if selected_items:
        for product_id in selected_items:
            cursor.execute("DELETE FROM products WHERE id = ?", (product_id,))
        connection.commit()
        refresh_treeview()
    else:
        show_info("No item selected.", title="Error")
    connection.close()

def update_product():
    selected_items = [key for key, var in checkbox_vars.items() if var.get()]
    if len(selected_items) == 1:
        product_id = selected_items[0]
        name = simpledialog.askstring("Input", "Enter new product name:")
        x = simpledialog.askfloat("Input", "Enter new x coordinate:")
        y = simpledialog.askfloat("Input", "Enter new y coordinate:")
        if name and x is not None and y is not None:
            db_dir = os.path.join('src/turtlemart/database/products.db')
            connection = sqlite3.connect(db_dir)
            cursor = connection.cursor()
            cursor.execute("UPDATE products SET name = ?, x = ?, y = ? WHERE id = ?", (name, x, y, product_id))
            connection.commit()
            connection.close()
            refresh_treeview()
        else:
            show_info("All fields must be filled out.", title="Error")
    elif len(selected_items) > 1:
        show_info("Select only one item to update.", title="Error")
    else:
        show_info("No item selected.", title="Error")

def select_products():
    selected_items = [key for key, var in checkbox_vars.items() if var.get()]
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
        show_info("Products selected successfully.")
        view_selected_products()
    else:
        show_info("No items selected.", title="Error")

def view_selected_products():
    for widget in selected_frame.winfo_children():
        widget.destroy()

    db_dir = os.path.join('src/turtlemart/database/products.db')
    connection = sqlite3.connect(db_dir)
    cursor = connection.cursor()
    cursor.execute("SELECT * FROM selected_products")
    for row in cursor.fetchall():
        ctk.CTkLabel(selected_frame, text=f"{row[0]} ({row[1]}, {row[2]})").pack(anchor="w", padx=10, pady=5)
    connection.close()

def perform_search():
    search_term = search_entry.get()
    refresh_treeview(search_term)

def actualizar_reloj_y_fecha():
    now = datetime.datetime.now()
    label_reloj.configure(text=now.strftime("%H:%M:%S"))
    label_fecha.configure(text=now.strftime("%Y-%m-%d"))
    root.after(1000, actualizar_reloj_y_fecha)  # Actualiza cada segundo

def navigation_window():
    print('y que esperabas un dulce')



if __name__ == '__main__':
    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("blue")

    root = ctk.CTk()
    root.title("Shop Vision")
   # root.geometry("800x600")
    w, h = root.winfo_screenwidth(), root.winfo_screenheight()                                    
    root.geometry("%dx%d+0+0" % (w, h))
    root.resizable(width=1, height=1)
    root.configure()

    # Frame en la parte superior
    top_frame = ctk.CTkFrame(root, height=200)
    top_frame.pack(side=ctk.TOP, fill=ctk.X)
    title_label = ctk.CTkLabel(top_frame, text="Product Management Interface", font=("Helvetica", 28, "bold"))
    title_label.pack(padx=20, pady=10)

    # Frame a la izquierda
    left_frame = ctk.CTkFrame(root, width=200)  # Ajustado para el tamaño del frame de búsqueda
    left_frame.pack(side=ctk.LEFT, fill=ctk.Y)

    # Reloj
    label_reloj = ctk.CTkLabel(left_frame, font=('ARIAL', 18, 'bold'))
    label_reloj.pack(side=ctk.TOP, padx=10, pady=10)

    # Fecha
    label_fecha = ctk.CTkLabel(left_frame, font=('ARIAL', 18, 'bold'))
    label_fecha.pack(side=ctk.TOP, padx=10, pady=70)

    # Actualizar reloj y fecha
    actualizar_reloj_y_fecha()

    # Texto Shop Vision
    shop_vision_label = ctk.CTkLabel(left_frame, text="Shop Vision", font=('Helvetica', 20, 'bold'))
    shop_vision_label.pack(side=ctk.BOTTOM, padx=10, pady=10)

    # Frame de búsqueda dentro del frame izquierdo
    search_frame = ctk.CTkFrame(left_frame)
    search_frame.pack(side=ctk.TOP, fill=ctk.X, padx=10, pady=10)

    # Campo de entrada para búsqueda
    search_entry = ctk.CTkEntry(search_frame, font=("Arial", 12))
    search_entry.pack(side=ctk.TOP, padx=10, pady=5)

    # Botón de búsqueda
    search_button = ctk.CTkButton(search_frame, text="Buscar", command=perform_search)
    search_button.pack(side=ctk.TOP, padx=10, pady=5)

    # Frame para lista de productos
    frame1 = ctk.CTkFrame(root, width=350)
    frame1.pack(side=ctk.LEFT, fill=ctk.BOTH, padx=20, pady=20, expand=True)

    # Frame para productos seleccionados
    frame2 = ctk.CTkFrame(root, width=350)
    frame2.pack(side=ctk.RIGHT, fill=ctk.BOTH, padx=20, pady=20, expand=True)

    # Título para la lista de productos
    products_label = ctk.CTkLabel(frame1, text="Productos", font=("Helvetica", 24, "bold"))
    products_label.pack(pady=10)

    # Scrollable Frame para la lista de productos
    treeview_frame = ctk.CTkScrollableFrame(frame1, width=300, height=400)
    treeview_frame.pack(fill=ctk.BOTH, expand=True, pady=10)

    # Variables para los checkboxes
    checkbox_vars = {}

    select_button = ctk.CTkButton(frame1, text="Select Products", command=select_products, width=200, height=50, font=("Helvetica", 16))
    select_button.pack(pady=10)
   

    # Título para productos seleccionados
    selected_label = ctk.CTkLabel(frame2, text="Productos Seleccionados", font=("Helvetica", 24, "bold"))
    selected_label.pack(pady=10)

    selected_frame = ctk.CTkScrollableFrame(frame2, width=300, height=400)
    selected_frame.pack(fill=ctk.BOTH, expand=True, pady=10)
    go_to_products_button = ctk.CTkButton(frame2, text="Dirigirse a productos", command=navigation_window, width=200, height=50, font=("Helvetica", 16))
    go_to_products_button.pack(pady=10)


    refresh_treeview()

    root.mainloop()

