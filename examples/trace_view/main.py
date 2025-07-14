import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Supponiamo che il tuo file CSV si chiami 'punti_3d.csv'
# e che abbia tre colonne chiamate 'X', 'Y' e 'Z'.
# Se i nomi delle colonne sono diversi, modificali di conseguenza.

try:
    df = pd.read_csv('punti_3d.csv')
except FileNotFoundError:
    print("Errore: il file 'punti_3d.csv' non è stato trovato.")
    print("Assicurati che il file si trovi nella stessa directory dello script o specifica il percorso completo.")
    exit()

# Crea una nuova figura e un asse 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Estrai le coordinate X, Y e Z dal DataFrame
whois = df["Node"]
x = df['X']
y = df['Y']
z = df['Z']

# Ottieni i nodi unici per assegnare colori diversi
nodi_unici = whois.unique()
colori = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']

# Crea lo scatter plot 3D con colori diversi per ogni nodo
for i, nodo in enumerate(nodi_unici):
    mask = whois == nodo
    colore = colori[i % len(colori)]  # Cicla i colori se ci sono più nodi che colori
    ax.scatter(x[mask], y[mask], z[mask], c=colore, marker='o', label=f'Nodo {nodo}')

# Aggiungi una leggenda per identificare i nodi
ax.legend()

# Aggiungi la sfera che rappresenta la Terra
radius_earth = 6.371e6-1e5  # Raggio della Terra in metri -100km per evitare sovrapposizioni

# Crea i dati per la sfera
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)
x_sphere = radius_earth * np.outer(np.cos(u), np.sin(v))
y_sphere = radius_earth * np.outer(np.sin(u), np.sin(v))
z_sphere = radius_earth * np.outer(np.ones(np.size(u)), np.cos(v))

# Plot della sfera
ax.plot_surface(x_sphere, y_sphere, z_sphere, color='green', alpha=0.7, label='Terra')

# Imposta le etichette degli assi
ax.set_xlabel('Asse X')
ax.set_ylabel('Asse Y')
ax.set_zlabel('Asse Z')

# Imposta un titolo per il grafico
ax.set_title('Visualizzazione 3D dei Punti')

# Mostra il grafico
plt.show()

# Puoi anche salvare il grafico come immagine
# plt.savefig('grafico_punti_3d.png'):
