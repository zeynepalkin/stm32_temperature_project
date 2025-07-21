STM32 Temperature Monitoring with LCD, Fan, UART and LEDs

 Overview
This is a simple STM32F103 project that reads temperature from a **TMP36 analog sensor**, displays it on an **LCD screen**, and sends it to the **UART terminal**. Based on the temperature, it also controls:
- A **fan** (using a transistor switch)
- A **green LED** (safe temperature)
- A **red LED** (high temperature)

---

Components

- STM32F103C8T6 (Blue Pill)
- TMP36 temperature sensor
- 2 LEDs (Green, Red)
- 1 Fan + NPN transistor (2N2222)
- LM016L LCD (16x2)
- UART Virtual Terminal
- Proteus for simulation

---

##  How It Works

- Reads analog temperature from TMP36 using ADC.
- Converts voltage to Celsius.
- Displays value on LCD.
- Sends temperature info to UART (Virtual Terminal).
- Activates:
  - **Green LED** if temp < 20°C  
  - **Red LED + Fan** if temp ≥ 60°C
 
    
  - <img width="1274" height="865" alt="Ekran_Goruntusu_349" src="https://github.com/user-attachments/assets/13980d80-c682-4d33-a325-87512a3863c5" />
