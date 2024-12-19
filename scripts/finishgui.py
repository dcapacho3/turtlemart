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

        # Configurar el cierre de ventana con la "X"
        self.protocol("WM_DELETE_WINDOW", self.on_closing)


    def setup_ui(self):
        self.title("Smart Autonomous Retail Assistant")
        self.geometry("%dx%d+0+0" % (self.winfo_screenwidth(), self.winfo_screenheight()))
        self.resizable(width=1, height=1)

        # Frame en la parte superior
        top_frame = ctk.CTkFrame(self, height=200, fg_color="bisque2")
        top_frame.pack(side=ctk.TOP, fill=ctk.X)
        title_label = ctk.CTkLabel(top_frame, text="Ha finalizado su proceso de compra", font=("Helvetica", 35, "bold"))
        title_label.pack(padx=20, pady=10)

        # Frame a la izquierda
        left_frame = ctk.CTkFrame(self, width=200, fg_color="bisque2")
        left_frame.pack(side=ctk.LEFT, fill=ctk.Y)

        # Reloj
        self.label_reloj = ctk.CTkLabel(left_frame, font=('ARIAL', 18, 'bold'))
        self.label_reloj.pack(side=ctk.TOP, padx=10, pady=10)

        # Fecha
        self.label_fecha = ctk.CTkLabel(left_frame, font=('ARIAL', 18, 'bold'))
        self.label_fecha.pack(side=ctk.TOP, padx=10, pady=70)

        # Actualizar reloj y fecha
        self.actualizar_reloj_y_fecha()

        # Frame grande con el texto de agradecimiento
        main_frame = ctk.CTkFrame(self)
        main_frame.pack(side=ctk.LEFT, fill=ctk.BOTH, padx=20, pady=20, expand=True)
        
        thank_you_label = ctk.CTkLabel(main_frame, text="Gracias por comprar", font=('Helvetica', 35, 'bold'))
        thank_you_label.pack(expand=True, padx=20, pady=20, anchor='center')

        # Texto Shop Vision
        shop_vision_label = ctk.CTkLabel(left_frame, text="SARA", font=('Helvetica', 30, 'bold'))
        shop_vision_label.pack(side=ctk.BOTTOM, padx=10, pady=10)
        


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

