# Modernizacja układu napędowego i sterowania pojazdu typu Dicykl
**Praca inżynierska zrealizowana na Politechnice Poznańskiej.**

*Read this in English below.*

## Opis projektu
Projekt obejmuje modernizację sprzętową oraz programową pojazdu typu dicykl (dwukołowego). Pojazd napędzany jest dwoma silnikami szczotkowymi wyposażonymi w enkodery. Do pomiaru pozycji gondoli wykorzystano czujnik MPU6050. Silniki sterowane są za pomocą niezależnych sterowników VESC, które komunikują się z systemem za pośrednictwem magistrali CAN.

## Architektura i komponenty
* **STM32 (Serce projektu):** Główny mikrokontroler realizujący logikę. Zaimplementowano w nim 3 typy sterowania:
  * PWM napięciowe - bez regulatora
  * Prądowe - z regulatorem prędkości
  * Prądowe - z regulatorem kąta/pozycji
* **ESP32:** Mikrokontroler odpowiedzialny za bezprzewodową komunikację Bluetooth pomiędzy kontrolerem a główną płytką STM32.
* **VESC:** Sterowniki silników.

## Struktura plików i folderów
* `STM32` - Główny projekt sterowania.
* `ESP32` - Projekt komunikacji bezprzewodowej.
* `3D Model` - Modele 3D użyte w projekcie. Zostały zaprojektowane z myślą o druku 3D. Ponieważ są to elementy silnie narażone na uszkodzenia mechaniczne, zaleca się stosowanie znacznie większego wypełnienia (infill) podczas druku.
* `VESC_BACKUP` - Zapis konfiguracji dla silników. **UWAGA:** Należy używać starszej wersji programu VESC Tool. Nowsze wersje posiadają ograniczone wsparcie dla silników szczotkowych.
* `measurements` - Pomiary i odczyty z czujników pobrane za pomocą narzędzia SerialPlot, przeanalizowane w programie MATLAB (struktura danych: Rzeczywisty prąd prawego silnika, Rzeczywisty prąd lewego silnika, Zadany prąd prawego silnika, Zadany prąd lewego silnika, Kąt pochylenia gondoli, Zadany kąt pochylenia gondoli, Prędkość prawego silnika, Prędkość lewego silnika, Zadana Prędkość prawego silnika, Zadana prędkość lewego silnika).
* `code_copy` oraz `code_copy_v2` - Kopie zapasowe starszych wersji kodu oraz specjalnych programów pomiarowych do identyfikacji obiektu.
* `Use_Instruction` - Plik zawierający szczegółowe instrukcje dotyczące sposobu sterowania pojazdem.

*Uwaga: Część dokumentacji, tekstów oraz komentarzy w tym repozytorium jest napisana w języku polskim.*

---
---

# Modernization of the Drive System and Control of a Dicycle Vehicle
**Engineering thesis completed at Poznań University of Technology (Politechnika Poznańska).**

## Project Description
This project covers the hardware and software modernization of a two-wheeled dicycle vehicle. The vehicle is powered by two brushed motors equipped with encoders. An MPU6050 sensor is used to track the gondola's position. The motors are driven by VESC controllers, communicating with the main system via the CAN bus.

## Architecture and Components
* **STM32 (Core of the project):** The main microcontroller handling the logic. It implements 3 types of control:
  * Voltage PWM - without regulator
  * Current - with speed regulator
  * Current - with angle/position regulator
* **ESP32:** Microcontroller handling Bluetooth communication between the user controller and the STM32 board.
* **VESC:** Motor controllers.

## Repository Structure
* `STM32` - Main control project files.
* `ESP32` - Wireless communication project files.
* `3D Model` - 3D models used in the project. Designed for 3D printing. Since these parts are highly susceptible to mechanical damage, printing with a high infill percentage is strongly recommended.
* `VESC_BACKUP` - Configuration backups for the motors. **WARNING:** An older version of the VESC Tool must be used. Newer versions have deprecated or reduced support for brushed motors.
* `measurements` - Sensor readings and data collected using SerialPlot, analyzed in MATLAB (data structure: ctual right motor current, Actual left motor current, Target right motor current, Target left motor current, Nacelle tilt angle, Target nacelle tilt angle, Right motor speed, Left motor speed, Target right motor speed, Target left motor speed).
* `code_copy` and `code_copy_v2` - Backups of older code versions and specific measurement programs used for system identification.
* `Use_Instruction` - A file containing detailed instructions on how to control and operate the vehicle.

*Note: Some of the documentation, texts, and code comments in this repository are written in Polish.*
