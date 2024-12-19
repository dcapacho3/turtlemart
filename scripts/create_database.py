#!/usr/bin/env python3

import sqlite3
from ament_index_python.packages import get_package_share_directory
import os

def create_database():
    # Obtener la ruta del directorio actual del script
    
    # Construir la ruta relativa a la carpeta database

    bringup_dir = get_package_share_directory('turtlemart')
    db_dir = os.path.join(bringup_dir, 'database/products.db')
    
    connection = sqlite3.connect(db_dir)
    cursor = connection.cursor()
    
    # Crear tabla si no existe
 # Crear tabla de usuarios
    cursor.execute('''CREATE TABLE IF NOT EXISTS users (
                        user_id INTEGER PRIMARY KEY,
                        username TEXT NOT NULL,
                        password TEXT NOT NULL
                    )''')
    
    # Crear tabla de productos
    cursor.execute('''CREATE TABLE IF NOT EXISTS products (
                        id INTEGER PRIMARY KEY,
                        name TEXT NOT NULL,
                        x REAL NOT NULL,
                        y REAL NOT NULL
                    )''')

    # Crear tabla de selecciones de usuarios
    cursor.execute('''CREATE TABLE IF NOT EXISTS user_selections (
                        selection_id INTEGER PRIMARY KEY,
                        user_id INTEGER,
                        product_id INTEGER,
                        FOREIGN KEY(user_id) REFERENCES users(user_id),
                        FOREIGN KEY(product_id) REFERENCES products(id)
                    )''')
    
    connection.commit()
    connection.close()

if __name__ == '__main__':
    create_database()
    print("Database and table created successfully.")

