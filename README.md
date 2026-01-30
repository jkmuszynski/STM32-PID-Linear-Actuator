# Projekt Regulatora PID - Pozycjonowanie Liniowe (STM32)

![Status Projektu](https://img.shields.io/badge/Status-Ukończony-success)
![Platforma](https://img.shields.io/badge/Platforma-STM32-blue)
![Język](https://img.shields.io/badge/Język-C%20%7C%20Python-yellow)

Projekt zrealizowany w ramach laboratorium "Systemy Mikroprocesorowe". Jest to kompletny system sterowania pozycją wózka liniowego w pętli zamkniętej (Closed-Loop Control) z wykorzystaniem regulatora PID.

System składa się z dwóch części:
1.  **Firmware (STM32):** Odpowiedzialny za sterowanie silnikiem w czasie rzeczywistym.
2.  **Software (PC):** Aplikacja Dashboard do wizualizacji danych i strojenia regulatora.


## 🚀 Funkcjonalności

* **Regulator PID:** Implementacja z członem proporcjonalnym, całkującym (z Anti-Windup) i różniczkującym.
* **Enkoder Absolutny:** Obsługa enkodera magnetycznego AS5600 po magistrali I2C.
* **Auto-Homing:** Automatyczne bazowanie wózka po starcie systemu (dojazd do krańcówki).
* **Bezpieczeństwo:**
    * **Hardware Limits:** Obsługa krańcówek fizycznych (blokada ruchu w stronę przeszkody).
    * **Soft Limits:** Programowe ograniczenie zakresu ruchu (0-255mm).
    * **Stall Detection:** Wykrywanie zablokowania silnika i awaryjne wyłączenie.
* **Tryb Hybrydowy:** Możliwość sterowania z poziomu aplikacji PC lub manualnie za pomocą enkodera obrotowego (gałki).
* **Telemetria Real-Time:** Wysyłanie danych (pozycja, uchyb, PWM) do PC z częstotliwością 10Hz.

## 🛠️ Sprzęt (Hardware)

* **Mikrokontroler:** STM32 Nucleo (F411RE).
* **Czujnik Pozycji:** AS5600 (Magnes neodymowy diametralny).
* **Napęd:** Silnik DC +Śruba.
* **Sterownik Silnika:** Mostek H (BTS7960).
* **Interfejs:** Enkoder obrotowy (KY-040) z przyciskiem.

## 💻 Oprogramowanie

### 1. Firmware (STM32)
Kod napisany w **STM32CubeIDE** przy użyciu bibliotek HAL.
* **TIM10:** Główna pętla sterowania (Przerwanie co 2ms).
* **TIM4:** Generacja PWM dla silnika.
* **I2C1:** Komunikacja z czujnikiem AS5600.
* **UART2 (DMA):** Komunikacja z komputerem.

### 2. Dashboard (Python)
Aplikacja napisana w Pythonie przy użyciu biblioteki `tkinter` oraz `matplotlib`.
* Rysowanie wykresów w czasie rzeczywistym (Pozycja, Uchyb, PWM).
* Eliminacja opóźnień (Anti-Lag buffering).
* Dynamiczne skalowanie wykresów.

## ⚙️ Instalacja i Uruchomienie

### Wymagania
* STM32CubeIDE (do edycji kodu C).
* Python 3.x.

### Uruchomienie Aplikacji PC
1.  Zainstaluj wymagane biblioteki:
    ```bash
    pip install pyserial matplotlib
    ```
2.  Podłącz STM32 do USB.
3.  Uruchom skrypt:
    ```bash
    python pid_tuner.py
    ```

## 📡 Protokół Komunikacyjny (UART)

Baudrate: `115200`

**Komendy PC -> STM32:**
* `SET:200` - Ustaw pozycję zadaną na 200mm (Tryb Auto).
* `MAN` - Przełącz w tryb manualny (Sterowanie gałką).
* `PID:1.5:0.01:0.5` - Zmień nastawy (Kp, Ki, Kd).

**Dane STM32 -> PC (Format CSV):**
`DAT, czas, cel, pozycja, uchyb, pwm, kp, kd, tryb`

## 👥 Autorzy

* **Jakub Muszyński**
* **Kacper Nele**

