# 💡 Smart Lamp – Control prin Gesturi & Sunet

Lampa inteligentă controlată fără butoane, doar prin gesturi și sunete, construită pe platforma **Arduino Nano**.

## 🔧 Funcționalități

- 🤚 Control prin gesturi (HC-SR04):
  - mișcare laterală → pornește/oprește lumina
  - mișcare verticală → reglează intensitatea (PWM fade)
- 👏 Control prin sunet (KY-038):
  - 3 clapuri → activează modul „color mode”
  - 1 clap → schimbă culoarea
  - 2 clapuri → schimbă animația
- 🌈 Bandă LED WS2812B cu animații
- 🕒 Afișaj LCD (I²C) cu oră + temperatură (LM35D + DS1302 RTC)
- 💤 Timer de inactivitate → stinge lampa automat

## 📷 Demo video

▶️ [YouTube – Demo Smart Lamp](https://www.youtube.com/watch?v=rxGVqdD0l4w)


## ⚙️ Tehnologii folosite

- PlatformIO + Visual Studio Code
- Arduino Nano (ATmega328P)
- FastLED, LiquidCrystal_I2C, DS1302RTC, VirtualButton

## 📌 Componente hardware

- HC-SR04 – senzor ultrasonic
- KY-038 – microfon (sunet)
- LM35D – senzor temperatură analogic
- DS1302 – ceas RTC + baterie
- WS2812B – bandă LED RGB adresabilă
- LCD 1602 + modul I²C
- Sursă 5 V / 2 A
- Breadboard + cabluri mama-tata

## 📈 Calibrare & filtrare

- **Ultrasonic**: 3 pași – mediană, skip spikes, exponential smoothing
- **KY-038**: prag analogic + debounce + timeout hardware
- **LM35D**: offset -5 °C (calibrat manual)

## 🧠 Optimizări

- Timer hardware în loc de `millis()` pentru inactivitate
- Tranziție PWM fără flicker (smoothSetBrightness)
- Filtrare multi-nivel pe citiri de distanță

## 🧪 Rezultate

- timp răspuns gesturi ≈ 120 ms
- detecție clapuri robustă (~1 m)
- consum total ~1.7 A la luminozitate maximă
- interfață fără butoane, complet hands-free

---

📬 Proiect realizat ca parte a cursului **Proiect PM** – Universitatea Politehnica București, 2025.
