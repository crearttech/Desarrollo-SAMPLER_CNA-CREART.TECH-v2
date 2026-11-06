# DESARROLLO DE SAMPLER V2

**Looper / Sampler de audio en tiempo real construido sobre Daisy Seed.**

SAMPLER es un dispositivo de audio portátil que permite grabar, reproducir, sobreponer capas (overdub) y manipular loops en tiempo real con efectos y control visual mediante pantalla TFT.

Proyecto desarrollado por **Crearttech** y entregado al **CNA Centro Nacional de las Artes Delia Zapata Olivella** en Bogotá, Colombia.

---

## Características

- **Grabación y Reproducción** — Loop de hasta 10 segundos a 48 kHz (auto-stop al llenarse)
- **Overdub** — Sobregrabar capas sobre el loop (la salida se silencia mientras se graba)
- **Efectos en tiempo real** — Delay de 10 ms a 1 s, Reverb estéreo de 8 líneas con SIZE, DECAY (0,4–8,4 s) y DAMP independientes, Pitch ±6 semitonos, Filtros HP/LP — todos con parámetros suavizados
- **Reproducción reversa** — Inversión de la dirección de playback
- **Control de región** — Start/End point y movimiento del loop
- **Sin clicks** — Crossfade de 2,7 ms al cerrar el loop y fade-in de 30 ms al empezar a grabar
- **Pantalla TFT** — Waveform en tiempo real con la región del loop y el cabezal, y knobs circulares con su valor en unidades reales
- **Limitador suave** en la salida

## Hardware

| Componente | Descripción |
|---|---|
| Microcontrolador | Electro-Smith Daisy Seed (ARM Cortex-M7) |
| Pantalla | TFT ST7735 de 1,8" (160×128) |
| Micrófono | MAX9814 (amplificador con control de ganancia) |
| Entrada de audio | ADC integrado del Daisy Seed (jack de entrada 3,5 mm y micrófono) |
| Salida de audio | DAC integrado del Daisy Seed (jack de salida 3,5 mm) |
| Controles | 3 teclas (grabar, play/pausa y reversa), 4 encoders rotativos con botón y 1 botón de borrado de efectos |

La PCB de esta versión no lleva alimentación propia, amplificador ni LEDs. El esquemático está en `Schematic_SAMPLER_CREARTTECH_CNAV2.pdf`.

## Controles

| Control | Función |
|---|---|
| Tecla **REC** | Mantener para grabar; con el loop sonando, mantener para overdub |
| Tecla **PLAY / PAUSA** | Reproducir y pausar; doble pulsación borra el loop |
| Tecla **REVERSA** | Activa y desactiva la reproducción en reversa |
| **ENC1** | Pitch, filtro pasa altos y pasa bajos |
| **ENC2** | Reverb: REVERB, SIZE, DECAY y DAMP |
| **ENC3** | Delay: tiempo, feedback y mezcla |
| **ENC4** | Ganancia, punto de inicio, punto final y mover la región del loop |
| Botón de **borrado de efectos** | Doble pulsación: borra los efectos. Mantenido 1 s: borra también la ganancia y la región del loop |

El pulsador de cada encoder cambia el parámetro que controla.

## Estructura del Código

```
Desarrollo-SAMPLER_CNA-CREART.TECH-v2/
├── SAMPLER.ino                # Programa principal (pines, pantalla, controles, audio callback)
├── sampler_engine.h           # Motor de audio (grabación, playback, overdub)
├── sampler_reverb.h           # Reverb estéreo de 8 líneas
└── Schematic_SAMPLER_CREARTTECH_CNAV2.pdf   # Esquemático V2: placa principal y placa de teclas
```

## Esquemático

`Schematic_SAMPLER_CREARTTECH_CNAV2.pdf` — esquemático de la placa principal visto desde atrás (lado de soldadura), con la forma de la PCB y una tabla de conexiones. Debajo incluye el esquemático de la placa de teclas.

## Instalación

1. Instalar [Arduino IDE](https://www.arduino.cc/en/software)
2. Instalar el board package **Electrosmith Daisy** desde el Board Manager
3. Instalar las bibliotecas:
   - `DaisyDuino`
   - `Adafruit GFX Library`
   - `Adafruit ST7735`
4. Copiar `SAMPLER.ino`, `sampler_engine.h` y `sampler_reverb.h` a una carpeta llamada `SAMPLER` (Arduino exige que la carpeta se llame igual que el `.ino`) y abrir `SAMPLER/SAMPLER.ino`
5. Seleccionar board **Daisy Seed** y el puerto correspondiente
6. Compilar y subir

## Desarrolladores

**Crearttech**
- Andrés Melo
- Kelly Melo
- Rodrigo Aguilar

**En colaboración con DELIA**
- Javier Beltrán
- Santiago Gasca

**Crearttech** — Bogotá, Colombia, 2026

Proyecto entregado al **CNA Centro Nacional de las Artes Delia Zapata Olivella**, Bogotá, Colombia.

---

## Bibliografía y Referencias

### Hardware y Librerías Base
- **Electro-Smith**. (2020). *Daisy Seed Datasheet & Wiki*. [Daisy Seed Documentation](https://www.electro-smith.com/daisy/daisy).
- **DaisyDuino**. *Librería de soporte para el ecosistema Daisy en Arduino*. [GitHub Repository](https://github.com/electro-smith/DaisyDuino).
- **Adafruit Industries**. *Adafruit GFX Library*. [Documentation](https://learn.adafruit.com/adafruit-gfx-graphics-library).

### Procesamiento Digital de Señales (DSP)
- **Smith, S. W.** (1997). *The Scientist and Engineer's Guide to Digital Signal Processing*. California Technical Publishing.
- **Pirkle, W. C.** (2012). *Designing Audio Effect Plug-ins in C++*. Focal Press.
- **Zölzer, U.** (2011). *DAFX: Digital Audio Effects*. Wiley.

### Optimización y Algoritmos
- **ARM**. *CMSIS DSP Software Library Documentation*. [ARM Developer](https://arm-software.github.io/CMSIS_5/DSP/html/index.html).
- **D’Angelo, S., & Välimäki, V.** (2013). *Generalized Moog Ladder Filter*.
