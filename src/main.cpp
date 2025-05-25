#include <VirtualButton.h>
#include <util/delay.h>

#define TRIG_PIN 3
#define ECHO_PIN 2

float duration_us, distance_cm;

#include <FastLED.h>
#include <LiquidCrystal_I2C.h>

#define LED_PIN 6
#define NUM_LEDS 3
#define BRIGHTNESS 255
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

#define SOUND_PIN 5
#define TEMP_SENSOR_PIN A1

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define CLK_PIN 7
#define DATA_PIN 8
#define RST_PIN 9

#include <DS1302.h>

DS1302 rtc(RST_PIN, DATA_PIN, CLK_PIN);

struct MYRGB {
  uint8_t r, g, b;
};

const MYRGB COLORS[] = {
    {255, 0, 0},     // RED
    {0, 255, 0},     // GREEN
    {0, 0, 255},     // BLUE
    {255, 255, 255}, // WHITE
    {128, 0, 128},   // PURPLE
    {255, 255, 0},   // YELLOW
    {0, 255, 255},   // CYAN
};
// ─── Timere pentru clap ───────────────────────────────────────
const unsigned long DEBOUNCE_MS = 50;
const unsigned long DOUBLE_CLAP_MS = 400;
const unsigned long COLOR_MODE_TIMEOUT = 2000;

volatile unsigned long lastEdgeTime = 0; // pt. debounce
volatile bool clapFlag = false;          // setat în ISR

// TIMER
#define CLK_HZ 16000000UL                         // freq. cristal
#define PRESC 1024UL                              // prescaler ales
#define TIMER_TICKS_2S ((CLK_HZ / PRESC) * 2 - 1) // 31249

const uint16_t OCR1A_VAL = (uint16_t)TIMER_TICKS_2S; // 0…65535

CRGB leds[NUM_LEDS];

class Lamp {

public:
  struct Settings {
    byte mode = 0;
    byte brightness[3] = {BRIGHTNESS, BRIGHTNESS, BRIGHTNESS};
    byte color[3] = {COLORS[0].r, COLORS[0].g, COLORS[0].b};
  };
  Settings settings;

  VButton gesture;
  uint32_t timer;
  uint32_t tout;
  int filterBuffer[3] = {0, 0, 0};
  byte filterCount = 0;
  volatile byte modeClicks = 0;
  volatile bool animFlag = false;

  int prevValue = 0;
  int skipCount = 0;

  long filt = 0;

  int offset_d = 0;
  int offset_v = 0;

  float bootTime = 0.5;

  // For sound sensor
  unsigned long lastClapTime = 0;
  bool lastPinState = false;

  unsigned int currentColor = 0;

  volatile unsigned int clapCount = 0;
  volatile bool changingColorMode = false;
  volatile bool flashPulse = false;
  volatile bool ledOn = false;

  Lamp() {
    timer = 0;
    tout = 0;
  }
};

Lamp lamp;

void ADC_init() {
  ADMUX = (1 << REFS0) | (0 << REFS1) | (0 << ADLAR) | (0 << MUX3) |
          (0 << MUX2) | (0 << MUX1) | (1 << MUX0);
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

void setup() {
  Serial.begin(9600);
  Serial.print("OCR1A_VAL = ");
  Serial.println(OCR1A_VAL);

  DDRD |= (1 << TRIG_PIN);  // TRIG = output
  DDRD &= ~(1 << ECHO_PIN); // ECHO = input

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  // Set lef off
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
  lamp.ledOn = false;

  // Sound sensor
  DDRD &= ~(1 << SOUND_PIN);
  PORTD |= (1 << SOUND_PIN);
  // ENABLE pin change interrupt
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT21);
  sei();

  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = OCR1A_VAL;
  TIMSK1 = 0;

  // Display
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Salut, lume!");

  lcd.setCursor(0, 1);
  lcd.print("LCD I2C test");

  // RTC
  rtc.writeProtect(false);
  rtc.halt(false);

  ADC_init();
}

// TIMER
void startInactivityTimer() {
  TCCR1B = 0;              // Stop timer
  TCNT1 = 0;               // Reset counter
  TIFR1 |= (1 << OCF1A);   // Clear *pending compare match flag*
  TIMSK1 |= (1 << OCIE1A); // Enable interrupt
  TCCR1B =
      (1 << WGM12) | (1 << CS12) | (1 << CS10); // Start timer: CTC + presc=1024
}

void stopInactivityTimer() {
  TCCR1B = 0; // stop timer
  TIMSK1 &= ~(1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect) {
  lamp.changingColorMode = false;
  stopInactivityTimer();
}

int getDistance() {
  PORTD |= (1 << TRIG_PIN); // Send pulse
  _delay_us(10);
  PORTD &= ~(1 << TRIG_PIN);

  unsigned long timeout = 30000;
  unsigned long startWait = micros();

  // Wait for echo HIGH
  while (!(PIND & (1 << ECHO_PIN))) {
    if (micros() - startWait > timeout)
      return 0;
  }

  unsigned long start = micros();

  // Wait for echo LOW
  while (PIND & (1 << ECHO_PIN)) {
    if (micros() - start > timeout)
      return 0;
  }

  unsigned long end = micros();

  float duration_us = end - start;
  return (int)(0.017 * duration_us);
}

int filterMedian(int newValue) {
  int *buf = lamp.filterBuffer;
  buf[lamp.filterCount] = newValue;
  lamp.filterCount++;

  if (lamp.filterCount >= 3) {
    lamp.filterCount = 0;
  }

  return (max(buf[0], buf[1]) == max(buf[1], buf[2]))
             ? max(buf[0], buf[2])
             : max(buf[1], min(buf[0], buf[2]));
}

int expFilter(int newValue) {
  if (newValue > 0) {
    lamp.filt += (newValue * 16L - lamp.filt) / 4L;
  } else {
    lamp.filt = 0;
  }

  return lamp.filt / 16L;
}

int filterSkip(int newValue) {

  if (lamp.prevValue == 0 && newValue > 0) {
    lamp.prevValue = newValue;
  }

  if (abs(lamp.prevValue - newValue) > 40 || newValue == 0) {
    lamp.skipCount++;

    if (lamp.skipCount > 6) {
      lamp.prevValue = newValue;
      lamp.skipCount = 0;
    } else {
      newValue = lamp.prevValue;
    }
  } else {
    lamp.skipCount = 0;
  }

  lamp.prevValue = newValue;

  return newValue;
}

void pulse() {
  CRGB original[NUM_LEDS];
  for (int i = 0; i < NUM_LEDS; i++) {
    original[i] = leds[i];
  }

  // Fade out
  for (int b = 255; b >= 0; b -= 10) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = original[i];
      leds[i].nscale8_video(b);
    }
    FastLED.show();
    delay(10);
  }

  // Fade in
  for (int b = 0; b <= 255; b += 10) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = original[i];
      leds[i].nscale8_video(b);
    }
    FastLED.show();
    delay(10);
  }
}

ISR(PCINT2_vect) {
  if (!lamp.ledOn)
    return;
  bool pinLevel = PIND & (1 << PIND5);
  if (!pinLevel)
    return;
  unsigned long now = millis();

  if (now - lastEdgeTime < DEBOUNCE_MS) {
    return;
  }
  lastEdgeTime = now;

  // ————— Color‑Mode ACTIV ————————————
  if (lamp.changingColorMode) {
    // reset hardware-timer
    TCNT1 = 0;

    if (millis() - lamp.lastClapTime < DOUBLE_CLAP_MS)
      lamp.modeClicks++;
    else
      lamp.modeClicks = 1;

    lamp.lastClapTime = millis();

    if (lamp.modeClicks == 1) {
      clapFlag = true;
    } else if (lamp.modeClicks == 2) {
      lamp.animFlag = true;
      lamp.modeClicks = 0;
    }
    return;
  }

  // ————— Color‑Mode INACTIV: numărăm claps ——
  if (now - lamp.lastClapTime < DOUBLE_CLAP_MS)
    lamp.clapCount++;
  else
    lamp.clapCount = 1;

  lamp.lastClapTime = now;

  if (lamp.clapCount >= 3) {
    lamp.changingColorMode = true;
    lamp.clapCount = 0;
    lamp.flashPulse = true;
    startInactivityTimer();
  }
}

void setupPulse() {
  CRGB original[NUM_LEDS];
  for (int i = 0; i < NUM_LEDS; i++) {
    original[i] = leds[i]; // Save original colors
  }

  for (int b = 0; b <= 255; b += 10) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = original[i];
      leds[i].nscale8_video(b);
    }
    FastLED.show();
    delay(10);
  }
}

void shutdownPulse() {
  CRGB original[NUM_LEDS];
  for (int i = 0; i < NUM_LEDS; i++) {
    original[i] = leds[i];
  }

  for (int b = 255; b >= 0; b -= 10) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = original[i];
      leds[i].nscale8_video(b);
    }
    FastLED.show();
    delay(10);
  }

  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}

// // Fade-away simplu (se stinge lin în ~300 ms)
// void fadeAway(uint8_t step = 8, uint8_t delay_ms = 12) {
//   for (int b = FastLED.getBrightness(); b >= 0; b -= step) {
//     FastLED.setBrightness(b);
//     FastLED.show();
//     delay(delay_ms);
//   }
//   fill_solid(leds, NUM_LEDS, CRGB::Black);
//   FastLED.show();
// }

void smoothSetBrightness(uint8_t targetBrightness, uint8_t steps = 10,
                         uint8_t delay_ms = 10) {
  uint8_t currentBrightness = FastLED.getBrightness();
  int diff = targetBrightness - currentBrightness;

  if (abs(diff) <= 1) {
    FastLED.setBrightness(targetBrightness);
    FastLED.show();
    return;
  }

  for (uint8_t i = 1; i <= steps; i++) {
    int newBrightness = currentBrightness + diff * i / steps;
    FastLED.setBrightness(newBrightness);
    FastLED.show();
    delay(delay_ms);
  }
}

uint16_t ADC_read() {
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
    ;
  return ADC;
}

void float_to_string(float value, char *buffer, int precision = 2) {
  dtostrf(value, 6, precision, buffer);
}

String dayAsString(const Time::Day day) {
  switch (day) {
  case Time::kSunday:
    return "Sunday";
  case Time::kMonday:
    return "Monday";
  case Time::kTuesday:
    return "Tuesday";
  case Time::kWednesday:
    return "Wednesday";
  case Time::kThursday:
    return "Thursday";
  case Time::kFriday:
    return "Friday";
  case Time::kSaturday:
    return "Saturday";
  }
  return "(unknown day)";
}

void displayTemperatureAndTime(float tempC, Time t) {
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print((int)tempC);
  lcd.print((char)223);
  lcd.print("C   ");

  lcd.setCursor(0, 1);
  lcd.print("Ora: ");
  if (t.hr < 10)
    lcd.print("0");
  lcd.print(t.hr);
  lcd.print(":");
  if (t.min < 10)
    lcd.print("0");
  lcd.print(t.min);

  lcd.print(" ");

  String day = dayAsString(t.day);
  lcd.print(day.substring(0, 3));
}

char buf[16];

unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 1000;

void loop() {

  if (millis() - lastDisplayUpdate >= displayInterval) {
    lastDisplayUpdate = millis();

    int analogValue = ADC_read();
    float voltage = analogValue * (5.0 / 1024.0);
    float tempC = voltage * 100.0;
    tempC -= 5;

    Serial.print("ADC: ");
    char buf[16];
    float_to_string(tempC, buf);
    Serial.print(buf);
    Serial.print(" °C, ");

    Time t = rtc.time();
    displayTemperatureAndTime(tempC, t);
  }

  if (lamp.flashPulse) {
    lamp.flashPulse = false;
    pulse();
  }
  if (clapFlag) {
    clapFlag = false;
    if (lamp.changingColorMode) {
      lamp.currentColor++;
      if (lamp.currentColor >= sizeof(COLORS) / sizeof(COLORS[0])) {
        lamp.currentColor = 0;
      }
      MYRGB color = COLORS[lamp.currentColor];
      for (int i = 0; i < NUM_LEDS; i++) {
        lamp.settings.color[0] = color.r;
        lamp.settings.color[1] = color.g;
        lamp.settings.color[2] = color.b;
        leds[i] = CRGB(color.r, color.g, color.b);
      }
      FastLED.show();
    }
  }

  // if (lamp.animFlag) {
  //   lamp.animFlag = false;

  //   static bool rainbow = false;
  //   rainbow = !rainbow;

  //   if (rainbow) {
  //     fill_rainbow(leds, NUM_LEDS, 0, 7);
  //   } else {
  //     MYRGB c = COLORS[lamp.currentColor];
  //     fill_solid(leds, NUM_LEDS, CRGB(c.r, c.g, c.b));
  //   }
  //   FastLED.show();
  // }

  if (millis() - lamp.timer > 50) {
    lamp.timer = millis();

    int dist = getDistance();
    if (dist > 55)
      dist = 0;

    dist = filterMedian(dist);
    dist = filterSkip(dist);
    dist = expFilter(dist);

    lamp.gesture.poll(dist);

    if (lamp.ledOn) {
      if (lamp.gesture.hasClicks() && millis() - lamp.tout > 1500) {
        lamp.ledOn = false;
        for (int i = 0; i < NUM_LEDS; i++) {
          leds[i] = CRGB(0, 0, 0);
        }
        Serial.println("LED OFF");
        Serial.print("Distance: ");
        Serial.print(dist);
        Serial.println(" cm");
        FastLED.show();
      }
    }
    // Serial.println(lamp.ledOn);
    if (!lamp.ledOn && lamp.gesture.click()) {
      // Turn on LED
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(lamp.settings.color[0], lamp.settings.color[1],
                       lamp.settings.color[2]);
      }

      FastLED.show();
      lamp.ledOn = true;
      lamp.gesture.reset();
      setupPulse();

    } else if (lamp.gesture.click()) {
      // Toggle LED state
      shutdownPulse();
    }

    if (lamp.gesture.held() && lamp.ledOn) {
      lamp.offset_d = dist;
      lamp.offset_v = lamp.settings.brightness[lamp.settings.mode];
      pulse();
    }

    if (lamp.gesture.hold() && lamp.ledOn) {

      lamp.tout = millis();

      int diff = lamp.offset_d - dist;
      int brightness = lamp.offset_v - diff * 12;

      brightness = constrain(brightness, 10, BRIGHTNESS);

      lamp.settings.brightness[lamp.settings.mode] = brightness;

      smoothSetBrightness(brightness, 10, 1);

      FastLED.show();
    }
  }
}
