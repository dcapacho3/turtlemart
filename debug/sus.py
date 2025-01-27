import tkinter as tk
from tkinter import messagebox
import datetime

# Función para calcular el puntaje SUS
def calcular_sus():
    try:
        # Recoger las respuestas de los usuarios
        respuestas_impares = [
            int(r1.get()) - 1, int(r3.get()) - 1, int(r5.get()) - 1, 
            int(r7.get()) - 1, int(r9.get()) - 1
        ]
        respuestas_pares = [
            5 - int(r2.get()), 5 - int(r4.get()), 5 - int(r6.get()), 
            5 - int(r8.get()), 5 - int(r10.get())
        ]
        
        # Sumar las respuestas
        suma_impares = sum(respuestas_impares)
        suma_pares = sum(respuestas_pares)
        
        # Calcular el puntaje SUS
        sus = (suma_impares + suma_pares) * 2.5
        
        # Mostrar el resultado
        messagebox.showinfo("Resultado SUS", f"El puntaje SUS es: {sus:.2f} sobre 100.")
        
        # Guardar las respuestas en un archivo de texto
        guardar_respuestas(sus)
        
        # Limpiar las opciones (desmarcar las respuestas)
        resetear_respuestas()
        
    except ValueError:
        messagebox.showerror("Error", "Por favor, asegúrese de haber seleccionado una respuesta para cada enunciado.")

# Función para guardar las respuestas en un archivo de texto con la fecha
def guardar_respuestas(sus):
    # Obtener la fecha y hora actual para el nombre del archivo
    fecha_hora = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    nombre_archivo = f"respuestas_sus_{fecha_hora}.txt"
    
    with open(nombre_archivo, "w") as file:
        file.write("Respuestas SUS:\n")
        file.write(f"Enunciados y respuestas:\n")
        
        for i, enunciado in enumerate(enunciados):
            file.write(f"{enunciado}: {eval(f'r{i+1}').get()}\n")
        
        file.write(f"\nPuntaje SUS: {sus:.2f}\n")
        file.write("====================================\n")
    
    # Mostrar mensaje de guardado
    messagebox.showinfo("Guardado", f"Las respuestas se han guardado en '{nombre_archivo}'.")

# Función para resetear las respuestas (desmarcar las opciones)
def resetear_respuestas():
    for var in [r1, r2, r3, r4, r5, r6, r7, r8, r9, r10]:
        var.set(None)

# Crear la ventana principal
root = tk.Tk()
root.title("Encuesta SUS - Sistema de Usabilidad")

# Título
titulo = tk.Label(root, text="Encuesta SUS - Sistema de Usabilidad", font=("Arial", 14))
titulo.grid(row=0, column=0, columnspan=3, pady=10)

# Enunciados y opciones de respuesta (Escala Likert)
enunciados = [
    "1. Creo que me gustaría utilizar este sistema con frecuencia",
    "2. Encontré el sistema innecesariamente complejo",
    "3. Pensé que el sistema era fácil de usar",
    "4. Creo que necesitaría el apoyo de un técnico para poder utilizar este sistema",
    "5. Encontré que las diversas funciones de este sistema estaban bien integradas",
    "6. Pensé que había demasiada inconsistencia en este sistema",
    "7. Me imagino que la mayoría de la gente aprendería a utilizar este sistema muy rápidamente",
    "8. Encontré el sistema muy complicado de usar",
    "9. Me sentí muy seguro usando el sistema",
    "10. Necesitaba aprender muchas cosas antes de empezar con este sistema"
]

# Crear las variables para las respuestas
r1, r2, r3, r4, r5, r6, r7, r8, r9, r10 = [tk.StringVar() for _ in range(10)]

# Crear etiquetas y botones de opción (radio buttons) para cada enunciado
for i, enunciado in enumerate(enunciados):
    enunciado_label = tk.Label(root, text=enunciado, wraplength=300)
    enunciado_label.grid(row=i + 1, column=0, sticky="w", padx=10, pady=5)
    
    # Opciones de respuesta (Escala Likert: 1-5)
    tk.Radiobutton(root, text="1", variable=eval(f"r{i+1}"), value="1").grid(row=i + 1, column=1)
    tk.Radiobutton(root, text="2", variable=eval(f"r{i+1}"), value="2").grid(row=i + 1, column=2)
    tk.Radiobutton(root, text="3", variable=eval(f"r{i+1}"), value="3").grid(row=i + 1, column=3)
    tk.Radiobutton(root, text="4", variable=eval(f"r{i+1}"), value="4").grid(row=i + 1, column=4)
    tk.Radiobutton(root, text="5", variable=eval(f"r{i+1}"), value="5").grid(row=i + 1, column=5)

    # Etiquetas de la escala Likert debajo de las opciones
    if i == 0:  # Solo en el primer enunciado
        etiquetas = ["Totalmente en desacuerdo", "En desacuerdo", "Neutro", "De acuerdo", "Totalmente de acuerdo"]
        for col, etiqueta in enumerate(etiquetas, start=1):
            tk.Label(root, text=etiqueta).grid(row=i + 11, column=col, padx=5, pady=5)

# Botón para calcular el SUS
calcular_btn = tk.Button(root, text="Calcular SUS", command=calcular_sus, bg="green", fg="white")
calcular_btn.grid(row=len(enunciados) + 1, column=0, columnspan=1, pady=12)

# Botón para cerrar la ventana
salir_btn = tk.Button(root, text="Salir", command=root.quit, bg="red", fg="white")
salir_btn.grid(row=len(enunciados) + 1, column=0, columnspan=2, pady=12)

# Ejecutar la aplicación
root.mainloop()