# CMLS-arduino

# Descrizione del Codice Arduino

Questo codice Arduino è progettato per implementare un sistema di tracciamento del movimento utilizzando un accelerometro e un giroscopio. Il sistema è in grado di rilevare variazioni nella posizione, nella velocità e nell'orientamento del dispositivo a cui è collegato.

### Funzionalità Principali:
1. **Rappresentazione dello Stato**: Il codice definisce una struttura di dati per memorizzare lo stato corrente del dispositivo, includendo la posizione, la velocità e l'orientamento.
2. **Sensori e Lettura dei Dati**: Utilizzando un accelerometro e un giroscopio, il codice legge costantemente i dati sensoriali per determinare le variazioni nella posizione e nell'orientamento del dispositivo nel tempo.
3. **Calcolo della Gravità**: Durante l'inizializzazione, il codice stima la forza di gravità per compensare gli effetti gravitazionali sui dati dell'accelerometro.
4. **Interruzione del Tocco**: Viene utilizzato un sensore di tocco per rilevare gli eventi di tocco sul dispositivo e attivare azioni di conseguenza.
5. **Comunicazione MIDI Bluetooth**: Il codice stabilisce una connessione Bluetooth e comunica tramite il protocollo MIDI per inviare segnali musicali o comandi a dispositivi esterni compatibili con MIDI.

### Workflow Tipico:
1. **Inizializzazione**: Durante l'avvio, il codice inizializza i sensori, calcola la gravità iniziale e stabilisce la connessione Bluetooth.
2. **Aggiornamento Continuo dello Stato**: Il codice legge costantemente i dati dai sensori e aggiorna lo stato del dispositivo in base alle variazioni rilevate.
3. **Rilevamento del Tocco**: Quando viene rilevato un tocco sul sensore, il codice attiva azioni di conseguenza, come la trasmissione di segnali MIDI o l'avvio di altre funzionalità.
4. **Comunicazione Bluetooth MIDI**: Quando la connessione Bluetooth è attiva, il codice invia periodicamente segnali MIDI a dispositivi esterni compatibili.


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
