#!/usr/bin/env python3

import sqlite3
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
from ament_index_python.packages import get_package_share_directory
import os


def refresh_treeview():
    treeview.delete(*treeview.get_children())
    db_dir = os.path.join( 'src/turtlemart/database/products.db')
    
    connection = sqlite3.connect(db_dir)
    cursor = connection.cursor()
    cursor.execute("SELECT * FROM products")
    for row in cursor.fetchall():
        # Default to unchecked checkbox symbol
        treeview.insert('', tk.END, values=('☐', row[0], row[1], row[2], row[3]))
    connection.close()

def add_product():
    name = simpledialog.askstring("Input", "Enter product name:")
    x = simpledialog.askfloat("Input", "Enter x coordinate:")
    y = simpledialog.askfloat("Input", "Enter y coordinate:")
    if name and x is not None and y is not None:
   
        db_dir = os.path.join( 'src/turtlemart/database/products.db')
   
        connection = sqlite3.connect(db_dir)
        cursor = connection.cursor()
        cursor.execute("INSERT INTO products (name, x, y) VALUES (?, ?, ?)", (name, x, y))
        connection.commit()
        connection.close()
        refresh_treeview()
    else:
        messagebox.showerror("Error", "All fields must be filled out.")

def delete_product():
    selected_items = [item for item in treeview.get_children() if treeview.item(item, 'values')[0] == '✔️']
    if selected_items:
        for selected_item in selected_items:
            product_id = treeview.item(selected_item)['values'][1]

            db_dir = os.path.join( 'src/turtlemart/database/products.db')
    
            connection = sqlite3.connect(db_dir)
            cursor = connection.cursor()
            cursor.execute("DELETE FROM products WHERE id = ?", (product_id,))
            connection.commit()
            connection.close()
        refresh_treeview()
    else:
        messagebox.showerror("Error", "No item selected.")

def update_product():
    selected_items = [item for item in treeview.get_children() if treeview.item(item, 'values')[0] == '✔️']
    if selected_items:
        if len(selected_items) > 1:
            messagebox.showerror("Error", "Select only one item to update.")
        else:
            product_id = treeview.item(selected_items[0])['values'][1]
            name = simpledialog.askstring("Input", "Enter new product name:")
            x = simpledialog.askfloat("Input", "Enter new x coordinate:")
            y = simpledialog.askfloat("Input", "Enter new y coordinate:")
            if name and x is not None and y is not None:
                bringup_dir = get_package_share_directory('turtlemart')
                db_dir = os.path.join( 'src/turtlemart/database/products.db')
    
                connection = sqlite3.connect(db_dir)
                cursor = connection.cursor()
                cursor.execute("UPDATE products SET name = ?, x = ?, y = ? WHERE id = ?", (name, x, y, product_id))
                connection.commit()
                connection.close()
                refresh_treeview()
            else:
                messagebox.showerror("Error", "All fields must be filled out.")
    else:
        messagebox.showerror("Error", "No item selected.")

def select_products():
    selected_items = [item for item in treeview.get_children() if treeview.item(item, 'values')[0] == '✔️']
    if selected_items:
        db_dir = os.path.join( 'src/turtlemart/database/products.db')
    
        connection = sqlite3.connect(db_dir)
        cursor = connection.cursor()
        
        # Limpiar la tabla temporal
        cursor.execute("DELETE FROM selected_products")
        
        for item in selected_items:
            product_id = treeview.item(item)['values'][1]
            cursor.execute("SELECT name, x, y FROM products WHERE id = ?", (product_id,))
            product_info = cursor.fetchone()
            cursor.execute("INSERT INTO selected_products (name, x, y) VALUES (?, ?, ?)",
                           (product_info[0], product_info[1], product_info[2]))
        
        connection.commit()
        connection.close()
        messagebox.showinfo("Info", "Products selected successfully.")
    else:
        messagebox.showerror("Error", "No items selected.")

def view_selected_products():
    selected_treeview.delete(*selected_treeview.get_children())
    db_dir = os.path.join( 'src/turtlemart/database/products.db')
    
    connection = sqlite3.connect(db_dir)
    cursor = connection.cursor()
    cursor.execute("SELECT * FROM selected_products")
    for row in cursor.fetchall():
        selected_treeview.insert('', tk.END, values=row)
    connection.close()

def toggle_selection(event):
    row_id = treeview.identify_row(event.y)
    if row_id:
        current_value = treeview.item(row_id, 'values')[0]
        new_value = '✔️' if current_value == '☐' else '☐'
        treeview.item(row_id, values=(new_value, *treeview.item(row_id, 'values')[1:]))

if __name__ == '__main__':
  
    db_dir = os.path.join( 'src/turtlemart/database/products.db')
    print(db_dir)
    
    connection = sqlite3.connect(db_dir)
    cursor = connection.cursor()
    
    # Eliminar la tabla temporal si ya existe
    cursor.execute("DROP TABLE IF EXISTS selected_products")
    
    # Crear la tabla temporal sin columna de id
    cursor.execute('''CREATE TABLE IF NOT EXISTS selected_products (
                        name TEXT,
                        x REAL,
                        y REAL,
                        PRIMARY KEY (name, x, y)
                    )''')
    connection.commit()
    connection.close()

    root = tk.Tk()
    root.title("Product Manager")
    root.geometry("800x600")

    # Configurar estilo moderno
    style = ttk.Style()
    style.theme_use('clam')

    style.configure("TButton", font=("Helvetica", 12), padding=10, relief="flat", background="#4285F4", foreground="white", borderwidth=0)
    style.map("TButton", background=[("active", "#3367D6")])
    
    style.configure("Treeview", font=("Helvetica", 12), rowheight=30, background="#F9F9F9", foreground="#000000", fieldbackground="#FFFFFF")
    style.configure("Treeview.Heading", font=("Helvetica", 14, "bold"), background="#4285F4", foreground="white")

    frame1 = ttk.Frame(root, padding=20)
    frame1.pack(side=tk.LEFT, fill=tk.BOTH, expand=1)

    frame2 = ttk.Frame(root, padding=20)
    frame2.pack(side=tk.RIGHT, fill=tk.BOTH, expand=1)

    # Título para la lista de productos
    products_label = ttk.Label(frame1, text="Productos", font=("Helvetica", 16, "bold"))
    products_label.pack(pady=10)

    columns = ('Select', 'ID', 'Name', 'X', 'Y')
    treeview = ttk.Treeview(frame1, columns=columns, show='headings')
    for col in columns:
        treeview.heading(col, text=col)
        treeview.column(col, width=100)
    treeview.bind("<ButtonRelease-1>", toggle_selection)  # Bind the click event to toggle selection
    treeview.pack(fill=tk.BOTH, expand=1)

    refresh_button = ttk.Button(frame1, text="Refresh", command=refresh_treeview)
    refresh_button.pack(pady=5)

    add_button = ttk.Button(frame1, text="Add Product", command=add_product)
    add_button.pack(pady=5)

    update_button = ttk.Button(frame1, text="Update Product", command=update_product)
    update_button.pack(pady=5)

    delete_button = ttk.Button(frame1, text="Delete Product", command=delete_product)
    delete_button.pack(pady=5)

    select_button = ttk.Button(frame1, text="Select Products", command=select_products)
    select_button.pack(pady=5)

    # Título para la lista de productos seleccionados
    selected_label = ttk.Label(frame2, text="Lista de Compras", font=("Helvetica", 16, "bold"))
    selected_label.pack(pady=10)

    selected_columns = ('Name', 'X', 'Y')
    selected_treeview = ttk.Treeview(frame2, columns=selected_columns, show='headings')
    for col in selected_columns:
        selected_treeview.heading(col, text=col)
        selected_treeview.column(col, width=100)
    selected_treeview.pack(fill=tk.BOTH, expand=1)

    view_selected_button = ttk.Button(frame2, text="View Selected Products", command=view_selected_products)
    view_selected_button.pack(pady=5)

    refresh_treeview()
    root.mainloop()

