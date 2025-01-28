#!/usr/bin/env python3

import customtkinter as ctk
import datetime
import signal


class ThanksWindow(ctk.CTk):
    def __init__(self, master=None):
        super().__init__()
        self.master = master
        self.after_id = None

        # Configuración de la interfaz
        self.setup_ui()
        signal.signal(signal.SIGINT, self.signal_handler)
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_ui(self):
        self.title("Smart Autonomous Retail Assistant")
        self.geometry("%dx%d+0+0" % (self.winfo_screenwidth(), self.winfo_screenheight()))
        self.resizable(width=1, height=1)

        # Frame en la parte superior
        top_frame = ctk.CTkFrame(self, height=100, fg_color="bisque2")
        top_frame.pack(side=ctk.TOP, fill=ctk.X)
        
        # Configurar grid para la disposición
        top_frame.grid_columnconfigure(0, weight=1)  # Columna central para el título

        # Título centrado
        title_label = ctk.CTkLabel(
            top_frame, 
            text="¡Hemos llegado a la caja!", 
            font=("Helvetica", 35, "bold"),
            justify="center"
        )
        title_label.grid(row=0, column=0, padx=20, pady=10, sticky="nsew")

        # Contenedor para fecha y hora en esquina superior derecha
        clock_container = ctk.CTkFrame(top_frame, fg_color="bisque2")
        clock_container.grid(row=0, column=1, padx=20, pady=5, sticky="e")

        # Reloj
        self.label_reloj = ctk.CTkLabel(
            clock_container, 
            font=('ARIAL', 25, 'bold')
        )
        self.label_reloj.pack(side=ctk.TOP, pady=2)

        # Fecha
        self.label_fecha = ctk.CTkLabel(
            clock_container, 
            font=('ARIAL', 25, 'bold')
        )
        self.label_fecha.pack(side=ctk.TOP, pady=2)

        self.actualizar_reloj_y_fecha()

        # Frame principal central
        main_frame = ctk.CTkFrame(self)
        main_frame.pack(side=ctk.LEFT, fill=ctk.BOTH, padx=20, pady=20, expand=True)
        
        thank_you_label = ctk.CTkLabel(
            main_frame,
            text="\n\n"
                "Gracias por confiar en SARA.\n\n"
                "¿Qué te pareció tu experiencia?\n\n"
                "[😊] [😐] [😞]\n\n"
                "No olvides retirar todos tus productos del carrito.\n\n"
                "¡Vuelve pronto!",
            font=('Helvetica', 35, 'bold'),
            wraplength=500,
        )
        thank_you_label.pack(expand=True, padx=20, pady=20, anchor='center')

    def actualizar_reloj_y_fecha(self):
        now = datetime.datetime.now()
        self.label_reloj.configure(text=now.strftime("%H:%M:%S"))
        self.label_fecha.configure(text=now.strftime("%Y-%m-%d"))
        self.after_id = self.after(1000, self.actualizar_reloj_y_fecha)

    def signal_handler(self, sig, frame):
        print("Ctrl+C detectado, cerrando la aplicación...")
        self.on_closing()

    def on_closing(self):
        print("Cerrando la ventana correctamente...")
        if self.after_id is not None:
            self.after_cancel(self.after_id)
        self.quit()
        self.destroy()

if __name__ == '__main__':
    app = ThanksWindow()
    app.mainloop()