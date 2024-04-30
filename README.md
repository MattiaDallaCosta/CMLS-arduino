# CMLS-arduino

# Descrizione del Codice Arduino

## Librerie Utilizzate
- `Arduino.h`: Libreria standard di Arduino.
- `BLEMidi.h`: Libreria per la comunicazione MIDI via Bluetooth.
- `Wire.h`: Libreria per la comunicazione I2C.

## Struttura dei Dati
- `state_t`: Struttura che contiene la posizione, la velocità e la matrice di orientamento (rotation matrix) del corpo rispetto all'ambiente.
- `acel`, `gyro`, `init_g`: Variabili per i dati dei sensori di accelerazione e giroscopio.
- `curr`: Variabile booleana per determinare lo stato attuale.
- `t`: Array per memorizzare il tempo.
- `touch`: Array per memorizzare lo stato del tocco.
- `test_low`, `c_t`: Variabili booleane per la gestione del tocco.
- `threshold`: Soglia per il tocco.

## Funzioni per le Operazioni con le Matrici
- `eul2Rotm`: Converte gli angoli di Eulero in una matrice di rotazione.
- `rotm2Eul`: Converte una matrice di rotazione in angoli di Eulero.
- `matMul`: Moltiplica due matrici.
- `matVecMul`: Moltiplica una matrice per un vettore.
- `matTrans`: Trasposta una matrice di rotazione.

## Funzioni per le Operazioni Fisiche
- `getAcelData`: Ottiene i dati dell'accelerometro.
- `getInertialData`: Ottiene i dati inerziali dai sensori.
- `updateState`: Aggiorna lo stato del sistema in base ai dati inerziali.

## Gestione del Touch
- `gotTouchEvent`: Gestisce l'interrupt del touch.

## Setup Iniziale
- Configurazione iniziale delle connessioni Bluetooth e del touch.
- Calibrazione dell'accelerometro per determinare la gravità iniziale.

## Loop Principale
- Se il touch cambia stato, invia un messaggio seriale.
- Se è connesso un dispositivo Bluetooth, invia una nota MIDI (A4) a piena velocità per un secondo, quindi la spegne.
- Se non è connesso un dispositivo Bluetooth, attende un secondo.
