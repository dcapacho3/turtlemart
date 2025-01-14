import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.gridspec import GridSpec
import re

def load_navigation_data(csv_file):
    """Carga los datos del CSV y los separa por navegación"""
    try:
        df = pd.read_csv(csv_file)
        navigations = {}
        
        for nav_id in df['navigation_id'].unique():
            nav_data = df[df['navigation_id'] == nav_id].copy()
            nav_data['elapsed_time'] = nav_data['elapsed_time'] - nav_data['elapsed_time'].min()
            navigations[nav_id] = nav_data
            
        return navigations
    except Exception as e:
        print(f"Error cargando datos CSV: {e}")
        return None

def plot_paths(navigations):
    """Genera gráfico de rutas planeadas vs ejecutadas"""
    try:
        plt.figure(figsize=(15, 5))
        
        for i, (nav_id, data) in enumerate(navigations.items(), 1):
            plt.subplot(1, 3, i)
            plt.plot(data['planned_x'], data['planned_y'], 'b-', label='Planeado', linewidth=2)
            plt.plot(data['executed_x'], data['executed_y'], 'r--', label='Ejecutado', linewidth=2)
            plt.grid(True)
            plt.title(f'Navegación {nav_id}')
            plt.xlabel('X (metros)')
            plt.ylabel('Y (metros)')
            plt.legend()
            plt.axis('equal')
        
        plt.tight_layout()
        plt.savefig('navigation_paths.png', dpi=300, bbox_inches='tight')
        plt.close()
    except Exception as e:
        print(f"Error generando gráficas de rutas: {e}")

def plot_metrics_over_time(navigations):
    """Genera gráficos de métricas a lo largo del tiempo"""
    try:
        fig = plt.figure(figsize=(15, 10))
        gs = GridSpec(2, 2, figure=fig)
        
        # Error de seguimiento
        ax1 = fig.add_subplot(gs[0, 0])
        for nav_id, data in navigations.items():
            ax1.plot(data['elapsed_time'], data['path_error'], label=f'Nav {nav_id}', linewidth=2)
        ax1.set_title('Error de Seguimiento vs Tiempo')
        ax1.set_xlabel('Tiempo (s)')
        ax1.set_ylabel('Error (m)')
        ax1.grid(True)
        ax1.legend()

        # Velocidad instantánea
        ax2 = fig.add_subplot(gs[0, 1])
        for nav_id, data in navigations.items():
            ax2.plot(data['elapsed_time'], data['instantaneous_velocity'], 
                    label=f'Nav {nav_id}', linewidth=2)
        ax2.set_title('Velocidad Instantánea vs Tiempo')
        ax2.set_xlabel('Tiempo (s)')
        ax2.set_ylabel('Velocidad (m/s)')
        ax2.grid(True)
        ax2.legend()

        # Distancia acumulada
        ax3 = fig.add_subplot(gs[1, 0])
        for nav_id, data in navigations.items():
            ax3.plot(data['elapsed_time'], data['accumulated_distance'], 
                    label=f'Nav {nav_id}', linewidth=2)
        ax3.set_title('Distancia Acumulada vs Tiempo')
        ax3.set_xlabel('Tiempo (s)')
        ax3.set_ylabel('Distancia (m)')
        ax3.grid(True)
        ax3.legend()

        # Error vs Velocidad
        ax4 = fig.add_subplot(gs[1, 1])
        for nav_id, data in navigations.items():
            ax4.scatter(data['instantaneous_velocity'], data['path_error'], 
                       alpha=0.5, label=f'Nav {nav_id}')
        ax4.set_title('Error vs Velocidad')
        ax4.set_xlabel('Velocidad (m/s)')
        ax4.set_ylabel('Error (m)')
        ax4.grid(True)
        ax4.legend()

        plt.tight_layout()
        plt.savefig('navigation_metrics.png', dpi=300, bbox_inches='tight')
        plt.close()
    except Exception as e:
        print(f"Error generando gráficas de métricas: {e}")

def plot_error_distribution(navigations):
    """Genera gráficos de distribución de errores"""
    try:
        plt.figure(figsize=(15, 5))
        
        # Boxplot de errores
        plt.subplot(121)
        error_data = [data['path_error'] for data in navigations.values()]
        plt.boxplot(error_data, labels=[f'Nav {i}' for i in navigations.keys()])
        plt.title('Distribución de Errores por Navegación')
        plt.ylabel('Error (m)')
        plt.grid(True)

        # Histograma de errores
        plt.subplot(122)
        for nav_id, data in navigations.items():
            plt.hist(data['path_error'], bins=30, alpha=0.5, 
                    label=f'Nav {nav_id}', density=True)
        plt.title('Histograma de Errores')
        plt.xlabel('Error (m)')
        plt.ylabel('Densidad')
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.savefig('error_distribution.png', dpi=300, bbox_inches='tight')
        plt.close()
    except Exception as e:
        print(f"Error generando gráficas de distribución: {e}")

def parse_txt_file(txt_file):
    """Extrae métricas del archivo TXT"""
    try:
        with open(txt_file, 'r') as f:
            content = f.read()
        
        # Diccionario para almacenar métricas por navegación
        navigations = {}
        
        # Buscar todas las secciones de navegación
        sections = content.split("=== Navegación")
        
        for section in sections[1:]:  # Ignorar la primera sección (encabezado)
            try:
                # Extraer número de navegación
                nav_match = re.search(r'(\d+) Completada', section)
                if not nav_match:
                    continue
                
                nav_id = int(nav_match.group(1))
                metrics = {}
                
                # Extraer métricas básicas
                error_match = re.search(r'Error radial: ([\d.]+)', section)
                time_match = re.search(r'Tiempo de navegación: ([\d.]+)', section)
                vel_match = re.search(r'Velocidad media: ([\d.]+)', section)
                
                if error_match and time_match and vel_match:
                    metrics['error_radial'] = float(error_match.group(1))
                    metrics['tiempo_nav'] = float(time_match.group(1))
                    metrics['velocidad_media'] = float(vel_match.group(1))
                
                # Extraer métricas de trayectoria
                path_section = section.split("=== Métricas de Trayectoria")[1].split("===")[0]
                error_medio_match = re.search(r'Error medio de trayectoria: ([\d.]+)', path_section)
                std_match = re.search(r'Desviación estándar: ([\d.]+)', path_section)
                error_max_match = re.search(r'Error máximo: ([\d.]+)', path_section)
                
                if error_medio_match and std_match and error_max_match:
                    metrics['error_medio'] = float(error_medio_match.group(1))
                    metrics['std_error'] = float(std_match.group(1))
                    metrics['error_max'] = float(error_max_match.group(1))
                
                # Extraer métricas de eficiencia
                eff_section = re.search(r'Distancia planeada inicial: ([\d.]+).*?' +
                                      r'Distancia ejecutada: ([\d.]+).*?' +
                                      r'Eficiencia: ([\d.]+)', section, re.DOTALL)
                
                if eff_section:
                    metrics['dist_planeada'] = float(eff_section.group(1))
                    metrics['dist_ejecutada'] = float(eff_section.group(2))
                    metrics['eficiencia'] = float(eff_section.group(3))
                
                navigations[nav_id] = metrics
                
            except Exception as e:
                print(f"Error procesando navegación {nav_id}: {e}")
                continue
        
        # Extraer estadísticas globales
        global_stats = {}
        if '=== RESULTADOS FINALES ===' in content:
            final_section = content.split('=== RESULTADOS FINALES ===')[1]
            mse_match = re.search(r'MSE Global: ([\d.]+)', final_section)
            rmse_match = re.search(r'RMSE Global: ([\d.]+)', final_section)
            if mse_match and rmse_match:
                global_stats['mse_global'] = float(mse_match.group(1))
                global_stats['rmse_global'] = float(rmse_match.group(1))
        
        return navigations, global_stats
    
    except Exception as e:
        print(f"Error procesando archivo TXT: {e}")
        return {}, {}

def plot_replanning_analysis(txt_file):
    """Analiza y visualiza el proceso de replaneación"""
    try:
        with open(txt_file, 'r') as f:
            content = f.read()
        
        # Extraer datos de replaneación por navegación
        nav_plans = {}
        current_nav = None
        
        for line in content.split('\n'):
            if 'Nav ' in line and 'puntos' in line and 'metros' in line:
                nav_match = re.search(r'Nav (\d+)', line)
                points_match = re.search(r'(\d+) puntos', line)
                distance_match = re.search(r'([\d.]+) metros', line)
                
                if nav_match and points_match and distance_match:
                    nav_id = int(nav_match.group(1))
                    points = int(points_match.group(1))
                    distance = float(distance_match.group(1))
                    
                    if nav_id not in nav_plans:
                        nav_plans[nav_id] = {'points': [], 'distances': []}
                    
                    nav_plans[nav_id]['points'].append(points)
                    nav_plans[nav_id]['distances'].append(distance)
        
        plt.figure(figsize=(15, 6))
        
        # Subplot para puntos del plan
        plt.subplot(121)
        for nav_id, data in nav_plans.items():
            plt.plot(data['points'], marker='o', label=f'Nav {nav_id}', linewidth=2)
        plt.title('Evolución de Puntos en el Plan')
        plt.xlabel('Número de Replaneación')
        plt.ylabel('Cantidad de Puntos')
        plt.grid(True)
        plt.legend()
        
        # Subplot para distancia planeada
        plt.subplot(122)
        for nav_id, data in nav_plans.items():
            plt.plot(data['distances'], marker='o', label=f'Nav {nav_id}', linewidth=2)
        plt.title('Evolución de Distancia Planeada')
        plt.xlabel('Número de Replaneación')
        plt.ylabel('Distancia (m)')
        plt.grid(True)
        plt.legend()
        
        plt.tight_layout()
        plt.savefig('replanning_analysis.png', dpi=300, bbox_inches='tight')
        plt.close()
        
    except Exception as e:
        print(f"Error en análisis de replaneación: {e}")

def plot_efficiency_comparison(txt_metrics):
    """Visualiza comparación de eficiencia entre navegaciones"""
    try:
        nav_ids = []
        efficiencies = []
        distances_plan = []
        distances_exec = []
        
        for nav_id, metrics in txt_metrics.items():
            nav_ids.append(nav_id)
            efficiencies.append(metrics['eficiencia'])
            distances_plan.append(metrics['dist_planeada'])
            distances_exec.append(metrics['dist_ejecutada'])
        
        plt.figure(figsize=(15, 5))
        
        # Eficiencia
        plt.subplot(131)
        plt.bar(nav_ids, efficiencies)
        plt.axhline(y=100, color='r', linestyle='--', label='Óptimo')
        plt.title('Eficiencia por Navegación')
        plt.xlabel('Navegación')
        plt.ylabel('Eficiencia (%)')
        plt.grid(True)
        plt.legend()
        
        # Comparación de distancias
        plt.subplot(132)
        width = 0.35
        plt.bar(np.array(nav_ids) - width/2, distances_plan, width, label='Planeada')
        plt.bar(np.array(nav_ids) + width/2, distances_exec, width, label='Ejecutada')
        plt.title('Distancia Planeada vs Ejecutada')
        plt.xlabel('Navegación')
        plt.ylabel('Distancia (m)')
        plt.legend()
        plt.grid(True)
        
        # Error radial vs eficiencia
        plt.subplot(133)
        errors = [metrics['error_radial'] for metrics in txt_metrics.values()]
        plt.scatter(errors, efficiencies)
        plt.title('Error Radial vs Eficiencia')
        plt.xlabel('Error Radial (m)')
        plt.ylabel('Eficiencia (%)')
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig('efficiency_analysis.png', dpi=300, bbox_inches='tight')
        plt.close()
        
    except Exception as e:
        print(f"Error en análisis de eficiencia: {e}")

def main():
    try:
        print("Iniciando análisis de métricas de navegación...")
        
        # Cargar datos
        print("\nCargando datos del CSV...")
        csv_data = load_navigation_data('nav2_path_metrics_20241219_115357.csv')
        if csv_data is None:
            raise Exception("Error cargando datos CSV")
            
        print("Cargando datos del TXT...")
        txt_metrics, global_stats = parse_txt_file('nav2_metrics_20241219_115357.txt')
        if not txt_metrics:
            raise Exception("Error cargando datos TXT")
        
        # Generar gráficas
        print("\nGenerando gráficas...")
        print("- Trayectorias")
        plot_paths(csv_data)
        
        print("- Métricas temporales")
        plot_metrics_over_time(csv_data)
        
        print("- Distribución de errores")
        plot_error_distribution(csv_data)
        
        print("- Análisis de replaneación")
        plot_replanning_analysis('nav2_metrics_20241219_115357.txt')
        
        print("- Análisis de eficiencia")
        plot_efficiency_comparison(txt_metrics)
        
        # Mostrar estadísticas
        print("\nEstadísticas Globales:")
        if global_stats:
            print(f"MSE Global: {global_stats.get('mse_global', 'N/A'):.6f}")
            print(f"RMSE Global: {global_stats.get('rmse_global', 'N/A'):.6f}")
        
        print("\nEstadísticas por Navegación:")
        stats_df = pd.DataFrame(txt_metrics).T
        pd.set_option('display.float_format', lambda x: '%.3f' % x)
        print(stats_df)
        
        # Guardar estadísticas en CSV
        stats_df.to_csv('navigation_stats.csv')
        print("\nEstadísticas guardadas en 'navigation_stats.csv'")
        
        print("\nAnálisis completado. Se han generado los siguientes archivos:")
        print("- navigation_paths.png")
        print("- navigation_metrics.png")
        print("- error_distribution.png")
        print("- replanning_analysis.png")
        print("- efficiency_analysis.png")
        print("- navigation_stats.csv")
        
    except Exception as e:
        print(f"\nError en la ejecución del análisis: {e}")
        raise

if __name__ == "__main__":
    main()
