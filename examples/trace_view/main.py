import pandas as pd
import matplotlib.pyplot as plt

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
