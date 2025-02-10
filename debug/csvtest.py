import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.gridspec import GridSpec

# Crear los datos
sus_data = np.array([
    # ALTO (2 usuarios)
    72.50, 72.50,
    # MEDIO (14 usuarios)
    100.00, 100.00, 97.50, 82.50, 87.50, 80.00, 87.50, 87.50, 90.00, 75.00, 
    100.00, 100.00, 82.50, 75.00,
    # BAJO (5 usuarios)
    57.50, 97.50, 92.50, 67.50, 85.00
])

# Separar datos por nivel de experiencia
datos_alto = sus_data[0:2]
datos_medio = sus_data[2:16]
datos_bajo = sus_data[16:]

per_question_scores = {
    'P1': 8.81, 'P2': 8.93, 'P3': 7.50, 'P4': 8.33,
    'P5': 8.93, 'P6': 8.45, 'P7': 8.33, 'P8': 9.52,
    'P9': 7.62, 'P10': 8.81
}

# Calcular estadísticas descriptivas
mean_score = np.mean(sus_data)
median_score = np.median(sus_data)
std_dev = np.std(sus_data)
min_score = np.min(sus_data)
max_score = np.max(sus_data)
quartiles = np.percentile(sus_data, [25, 50, 75])

# Configuración de estilo y fuente
plt.rcParams['figure.figsize'] = [15, 10]
plt.rcParams['font.family'] = 'sans-serif'

# Crear figura con subplots
fig = plt.figure(figsize=(15, 10))
gs = GridSpec(2, 2, figure=fig)

# 1. Boxplots separados por nivel de experiencia
ax1 = fig.add_subplot(gs[0, 0])
data_to_plot = [datos_alto, datos_medio, datos_bajo]
bp = ax1.boxplot(data_to_plot, patch_artist=True, labels=['Alto', 'Medio', 'Bajo'])

# Colores para cada boxplot
colors = ['lightblue', 'lightgreen', 'lightcoral']
for patch, color in zip(bp['boxes'], colors):
    patch.set_facecolor(color)

# Agregar puntos individuales
for i, datos in enumerate([datos_alto, datos_medio, datos_bajo], 1):
    for punto in datos:
        ax1.plot(i, punto, 'ko', alpha=0.6)

# Agregar líneas de referencia y etiquetas
ax1.axhline(y=68, color='green', linestyle='--', alpha=0.5)
ax1.text(3.2, 71, 'Aceptable (>68)', ha='left', va='center')

ax1.axhline(y=50, color='red', linestyle='--', alpha=0.5)
ax1.text(3.2, 53, 'Inaceptable (<50)', ha='left', va='center')

ax1.set_title('Distribución de Puntuaciones SUS por Nivel de Experiencia', pad=20)
ax1.set_ylim(0, 100)
ax1.set_ylabel('Puntuación')
ax1.grid(True, linestyle='--', alpha=0.7)

# Agregar estadísticas por grupo
stats_text = (f'Alto: {np.mean(datos_alto):.1f} (n=2)\n'
             f'Medio: {np.mean(datos_medio):.1f} (n=14)\n'
             f'Bajo: {np.mean(datos_bajo):.1f} (n=5)')
ax1.text(0.02, 0.98, stats_text, transform=ax1.transAxes, 
         verticalalignment='top', bbox=dict(facecolor='white', alpha=0.8))


# 2. Escala de evaluación
ax2 = fig.add_subplot(gs[0, 1])
evaluation_ranges = [
    ('Inaceptable', 0, 50, 'red'),
    ('Marginal', 50, 68, 'yellow'),
    ('Aceptable', 68, 100, 'green')
]

for desc, min_val, max_val, color in evaluation_ranges:
    rect = Rectangle((0, min_val), 1, max_val-min_val, facecolor=color, alpha=0.3)
    ax2.add_patch(rect)
    ax2.text(0.5, (min_val + max_val)/2, desc, ha='center', va='center', fontsize=12)

# Agregar marcador para el puntaje promedio
ax2.axhline(y=mean_score, color='blue', linestyle='--', alpha=0.5)
ax2.text(1.1, 90, f'Promedio: {mean_score:.1f}', ha='left', va='center')

ax2.set_xlim(-0.5, 1.5)
ax2.set_ylim(0, 100)
ax2.set_title('Escala de Evaluación', pad=20)
ax2.set_xticks([])
ax2.grid(True, linestyle='--', alpha=0.7)

# 3. Gráfico de promedios por nivel de experiencia
ax3 = fig.add_subplot(gs[1, 0])
niveles = ['Alto', 'Medio', 'Bajo']
promedios = [np.mean(datos_alto), np.mean(datos_medio), np.mean(datos_bajo)]

# Crear barras
bars = ax3.bar(niveles, promedios, color=['lightblue', 'lightgreen', 'lightcoral'])
ax3.set_xlabel('Nivel de Experiencia')
ax3.set_ylabel('Puntuación Promedio')
ax3.set_title('Promedio de Puntuaciones por Nivel de Experiencia', pad=20)
ax3.grid(True, linestyle='--', alpha=0.7)
ax3.set_ylim(0, 100)

# Agregar valores sobre las barras
for bar in bars:
    height = bar.get_height()
    ax3.text(bar.get_x() + bar.get_width()/2., height + 1,
             f'{height:.1f}', ha='center', va='bottom')

# 4. Radar chart para puntuaciones por pregunta
ax4 = fig.add_subplot(gs[1, 1], projection='polar')
questions = list(per_question_scores.keys())
values = list(per_question_scores.values())
angles = np.linspace(0, 2*np.pi, len(questions), endpoint=False)
values = np.concatenate((values, [values[0]]))
angles = np.concatenate((angles, [angles[0]]))

ax4.plot(angles, values, 'o-', linewidth=2)
ax4.fill(angles, values, alpha=0.25)
ax4.set_xticks(angles[:-1])
ax4.set_xticklabels(questions)
ax4.set_ylim(0, 10)
ax4.set_title('Puntuación por Pregunta', pad=20)

# Agregar círculos de referencia
circles = np.arange(2, 11, 2)
ax4.set_rticks(circles)
for circle in circles:
    ax4.text(0, circle, str(circle), ha='center', va='bottom')

# Ajustar layout
plt.tight_layout()

# Agregar título general
fig.suptitle('Análisis de Usabilidad (SUS)', fontsize=16, y=1.05)

# Guardar la figura
plt.savefig('analisis_sus.png', dpi=300, bbox_inches='tight')
plt.close()

# Imprimir estadísticas
print("\nEstadísticas descriptivas del SUS:")
print(f"Media: {mean_score:.2f}")
print(f"Mediana: {median_score:.2f}")
print(f"Desviación estándar: {std_dev:.2f}")
print(f"Mínimo: {min_score:.2f}")
print(f"Máximo: {max_score:.2f}")

print("\nPuntuaciones por pregunta:")
for q, score in per_question_scores.items():
    print(f"{q}: {score:.2f}")

# Evaluación según los rangos especificados
if mean_score > 68:
    evaluacion = "Aceptable"
elif mean_score > 50:
    evaluacion = "Marginal - Aspectos por mejorar"
else:
    evaluacion = "Inaceptable"

print(f"\nEvaluación general:")
print(f"Puntuación SUS: {mean_score:.2f}")
print(f"Evaluación: {evaluacion}")

print(f"\nAnálisis por cuartiles:")
print(f"Q1 (25%): {quartiles[0]:.2f}")
print(f"Q2 (50%): {quartiles[1]:.2f}")
print(f"Q3 (75%): {quartiles[2]:.2f}")

print("\nAnálisis por nivel de experiencia:")
print(f"Alto (n=2): Media = {np.mean(datos_alto):.2f}, Desv. Est. = {np.std(datos_alto):.2f}")
print(f"Medio (n=14): Media = {np.mean(datos_medio):.2f}, Desv. Est. = {np.std(datos_medio):.2f}")
print(f"Bajo (n=5): Media = {np.mean(datos_bajo):.2f}, Desv. Est. = {np.std(datos_bajo):.2f}")

# Conclusiones del análisis
print("\nConclusiones principales:")
print("1. Distribución por nivel de experiencia:")
print("   - Alto: Puntuaciones consistentes alrededor de 72.5")
print("   - Medio: Mayor rango de puntuaciones (75-100), mayoría sobre nivel aceptable")
print("   - Bajo: Distribución amplia (57.5-97.5), algunos valores bajo nivel aceptable")

print("\n2. Usabilidad general:")
print("   - La mayoría de puntuaciones superan el nivel aceptable (68)")
print("   - Usuarios de nivel medio dan las puntuaciones más altas")
print("   - Pocos usuarios, principalmente de nivel bajo, puntuaron bajo el nivel aceptable")

print("\n3. Implicaciones:")
print("   - Sistema más efectivo para usuarios de experiencia media")
print("   - Oportunidades de mejora para usuarios de baja experiencia")
print("   - Buena aceptación general del sistema")
