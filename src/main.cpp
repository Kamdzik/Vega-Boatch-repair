/*
Program do gowowego sterownika łódki zanetowej opartej na mikrokontrolerze atmega 48 kóry steruje 2 silnikami , 2 burty, 2 pstrykacze i wlacznik swiateł ,
program odczytuje impulcy ppwm na wejsciu in0  podobnie jak w pwm tyle ze odbiornik wysyła jednym przewodem 6 kanałowy sygnał i powtarza cyklicznie jedyne co to 
musi to przetwarza go na wyjscia sterujace . program był pisany na zlecenie , ostatnia data kompilaci i wgrania 30.06.2025
finałowy kod
*/
 

#include <Arduino.h>
#define NUM_CHANNELS 6
#define SYNC_GAP 4000 // [us] - powyżej tej wartości uznajemy początek nowej ramki

/// piny do odbioru sygnału PWM z pilota RC
#define input_ppm PIN_PD2 // Pin odbioru sygnału PWM (ICP1)

#define pin_przekaznika1 PIN_PD7 // Pin przekaźnika
#define pin_przekaznika2 PIN_PB0 // Drugi pin przekaźnika

#define silnik1_pwm PIN_PB1 // Pin PWM silnika 1
#define silnik2_pwm PIN_PB2 // Pin PWM silnika 2

#define pin_led1 PIN_PC0 // Pin diody LED
#define pin_led2 PIN_PC1 // Drugi pin diody LED
#define pin_led3 PIN_PC2 // Trzeci pin diody LED
#define pin_led4 PIN_PC3 // Czwarty pin diody LED
#define pin_led5 PIN_PD1 // Piąty pin diody LED
#define pin_led6 PIN_PB5 // Szósty pin diody LED
#define pin_led7 PIN_PD0 // Siódmy pin diody LED
#define pin_led8 PIN_PB3 // Ósmy pin diody LED

#define mosfet1 PIN_PB6 // Pin mosfet
#define mosfet2 PIN_PB7 // Drugi pin mosfet
#define mosfet3 PIN_PD3 // Trzeci pin mosfet
#define mosfet4 PIN_PD4 // Czwarty pin mosfet
#define mosfet5 PIN_PB6 // Piąty pin mosfet

#define adc_voltage A7

#define DEADZONE 30      // strefa martwa drążka
#define DIR_HYSTERESIS 5 // histereza kierunku (dla przekaźników)

uint16_t minVal[NUM_CHANNELS] = {1110, 1098, 1058, 1098, 899, 1016};
uint16_t maxVal[NUM_CHANNELS] = {1805, 1858, 1897, 1858, 1970, 1917};
uint8_t scaledValues[NUM_CHANNELS];

volatile uint16_t pulseTimes[NUM_CHANNELS];
volatile uint8_t currentChannel = 0;
volatile uint32_t lastMicros = 0;

const uint8_t margines = 30;
const uint8_t margines_motorPLUS = 159; // 127 margines do porownania wartosci PWM
const uint8_t margines_motorMINUS = 95; // margines do porownania wartosci PWM

uint8_t pilot_podlaczony = 0; // Flaga informująca, czy pilot jest podłączony

unsigned long czas = 0, czas_przekazniki;
unsigned long czas_burta1, czas_burta2 = 0;
unsigned long czasburtaR_led = 0, czasburtaL_led = 0, czas_brak_aku;
unsigned long led_burtaRR, led_burtaLL = 0;
byte led_burta_stanR = 0, led_burta_stanL = 0;

int16_t left;
int16_t right;
int16_t x, y;

uint8_t burtaL = 0;
uint8_t burtaR = 0;

uint8_t kulkaL, kulkaR = 0, stan_swiatlokulka;
unsigned long czas_kulkaR, czas_kulkaL, czas_kulka_swiatlo1, czas_kulka_swiatlo2;

uint8_t currentDirLeft = 1; // zapamiętany stan kierunku
uint8_t currentDirRight = 1;

uint16_t pwmL;
uint16_t pwmR;
int adc_vcc_var = 0;

byte stan_led_brak_aku = 0;

byte wyj_kulka1 = 1, wyj_kulka2 = 1, wyj_kulka11 = 1, wyj_kulka22 = 1;
unsigned long czas_pik_kulka = 0, czas_pik_kulka1;

// Przerwanie od INT0 (D2) – zbocze opadające
ISR(INT0_vect)
{
  uint32_t now = micros();
  uint16_t duration = now - lastMicros;
  lastMicros = now;

  if (duration > SYNC_GAP)
  {
    currentChannel = 0;
  }
  else if (currentChannel < NUM_CHANNELS)
  {
    pulseTimes[currentChannel] = duration;
    currentChannel++;
  }
}

int16_t scaledAxis(uint8_t val)
{
  if (val > 127 + DEADZONE)
    return map(val, 127 + DEADZONE, 255, 0, 255);
  else if (val < 127 - DEADZONE)
    return map(val, 0, 127 - DEADZONE, -255, 0);
  else
    return 0;
}

//

void setup()
{
  // Serial.begin(115200);
  pinMode(input_ppm, INPUT);   // ICP1 pin
  pinMode(adc_voltage, INPUT); // adc napiecia aku

  pinMode(pin_przekaznika1, OUTPUT); // Pin przekaźnika 1
  pinMode(pin_przekaznika2, OUTPUT); // Pin przekaźnika 2

  pinMode(silnik1_pwm, OUTPUT); // Pin PWM silnika 1
  pinMode(silnik2_pwm, OUTPUT); // Pin PWM silnika 2

  pinMode(pin_led1, OUTPUT); // Pin diody LED 1
  pinMode(pin_led2, OUTPUT); // Pin diody LED 2
  pinMode(pin_led3, OUTPUT); // Pin diody LED 3
  pinMode(pin_led4, OUTPUT); // Pin diody LED 4
  pinMode(pin_led5, OUTPUT); // Pin diody LED 5
  pinMode(pin_led6, OUTPUT); // Pin diody LED 6
  pinMode(pin_led7, OUTPUT); // Pin diody LED 7
  pinMode(pin_led8, OUTPUT); // Pin diody LED 7

  digitalWrite(pin_led1, HIGH); // Wyłącz diodę LED 1
  digitalWrite(pin_led2, HIGH); // Wyłącz diodę LED 2
  digitalWrite(pin_led3, HIGH); // Wyłącz diodę LED 3
  digitalWrite(pin_led4, HIGH); // Wyłącz diodę LED 4
  digitalWrite(pin_led5, HIGH); // Wyłącz diodę LED 5
  digitalWrite(pin_led6, HIGH); // Wyłącz diodę LED 6
  digitalWrite(pin_led7, HIGH); // Wyłącz diodę LED 7
  digitalWrite(pin_led8, HIGH); // Wyłącz diodę LED 8

  pinMode(mosfet1, OUTPUT);    // Pin mosfet 1
  digitalWrite(mosfet1, HIGH); // modfet1
  pinMode(mosfet2, OUTPUT);    // Pin mosfet 2
  digitalWrite(mosfet2, HIGH); // modfet2
  pinMode(mosfet3, OUTPUT);    // Pin mosfet 3
  digitalWrite(mosfet3, HIGH); // modfet3
  pinMode(mosfet4, OUTPUT);    // Pin mosfet 4
  digitalWrite(mosfet4, HIGH); // modfet4
  pinMode(mosfet5, OUTPUT);    // Pin mosfet 5
  digitalWrite(mosfet5, HIGH); // modfet5

  analogReference(INTERNAL1V1);

  for (uint8_t i = 0; i < NUM_CHANNELS; i++)
  {
    scaledValues[i] = (uint8_t)126;
  }

  // Timer1 konfiguracja
  // Tryb: Fast PWM z TOP = ICR1
  // Częstotliwość: 8 MHz / (1 * (1 + 399)) = 20 kHz

  // DDRB |= (1 << PB1) | (1 << PB2);  // Ustaw wyjścia
  // // Timer1: Fast PWM, TOP = ICR1
  // TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);

  // Timer1: Fast PWM, TOP = ICR1 (mode 14)
  DDRB |= (1 << PB1) | (1 << PB2); // ustaw PWM jako wyjścia

  // Timer1 – Fast PWM, TOP = ICR1 (mode 14)
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10); // prescaler = 1
  ICR1 = 500;

  OCR1A = 0; // około 22% wypełnienia
  OCR1B = 0; // około 68% wypełnienia

  // Start przerwań INT0 na zbocze opadające (dla CPPM zwykle takie)
  EICRA |= (1 << ISC01); // ISC01 = 1, ISC00 = 0 → zbocze OPADAJĄCE
  EICRA &= ~(1 << ISC00);
  EIMSK |= (1 << INT0); // Włącz przerwanie INT0
}

/*
jesli pilot rozłaczony to 2 ostatnie kanału dwuch przełączników
sa na domyslnie na okolice srodkowych wartosci  136 i 129
*/
void loop()
{
  czas = millis();
  adc_vcc_var = analogRead(adc_voltage);

  noInterrupts();
  uint16_t pulses[NUM_CHANNELS];
  memcpy(pulses, (const void *)pulseTimes, sizeof(pulseTimes));
  interrupts();

  for (uint8_t i = 0; i < NUM_CHANNELS; i++)
  {
    int raw = pulses[i];
    raw = constrain(raw, minVal[i], maxVal[i]);

    long scaled = (long)(raw - minVal[i]) * 255 / (maxVal[i] - minVal[i]);
    scaledValues[i] = (uint8_t)scaled;
  }

  if (((scaledValues[5] > 180) || (scaledValues[5] < 80)) && ((scaledValues[4] > 180) || (scaledValues[4] < 80)))
  {                       // Przełącznik 1 i 2
    pilot_podlaczony = 1; // Pilot rozłączony
  }
  else
  {
    pilot_podlaczony = 0; // Pilot podłączony
  }

  //  scaledValues[0] kierunek lewo prawo drugi drazek2
  //  scaledValues[1] drazek1
  //  scaledValues[2] kierunek jazdy 0-127 - do przodu, 128-255 - do tyłu drugi drazek2
  //  scaledValues[3] drazek1
  //  scaledValues[4] przycisk1
  //  scaledValues[5] przycisk2

  if (pilot_podlaczony == 1)
  {
    x = scaledAxis(scaledValues[0]); // skręt
    y = scaledAxis(scaledValues[2]); // gaz

    if (y >= 0)
    {
      left = y - x;
      right = y + x;
    }
    else
    {
      left = y + x;
      right = y - x;
    }

    // ogranicz do -255...255
    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);

    // histereza kierunku
    if (left > DIR_HYSTERESIS)
      currentDirLeft = 1;
    else if (left < -DIR_HYSTERESIS)
      currentDirLeft = 0;

    if (right > DIR_HYSTERESIS)
      currentDirRight = 1;
    else if (right < -DIR_HYSTERESIS)
      currentDirRight = 0;

    if (czas - czas_przekazniki < 10000)
    {
      // ustaw kierunki przekaźników
      digitalWrite(pin_przekaznika1, !currentDirLeft);
      digitalWrite(pin_przekaznika2, !currentDirRight);
    }
    else
    {
      digitalWrite(pin_przekaznika1, 0);
      digitalWrite(pin_przekaznika2, 0);
    }
    //  = abs(left);
    //  = abs(right);

    // wylicz PWM jako wartość absolutną
    // OCR1A = abs(left);
    // OCR1B = abs(right);

    pwmL = map(abs(left), 0, 255, 270, 500);
    pwmR = map(abs(right), 0, 255, 270, 500);

    if (pwmL >= 280)
    {
      OCR1A = pwmL;
      czas_przekazniki = czas;
    }
    else
    {
      OCR1A = 0;
    }

    if (pwmR >= 280)
    {
      czas_przekazniki = czas;
      OCR1B = pwmR;
    }
    else
    {
      OCR1B = 0;
    }

    // analogWrite(silnik1_pwm, pwmL); // PB1 (D9)
    // analogWrite(silnik2_pwm, pwmR); // PB2 (D10)

    // pin_led1 tylne swiatło niebieskie
    // pin_led2 lewe swiatło zielone
    // pin_led3 przod swiatło białe
    // pin_led4 prawe swiatło czerwone

    // od lewej
    // pin_led5 1 tylne bateria czerwony
    // pin_led6 2 tylne bateria czerwony
    // pin_led7 3 tylne bateria pomarańczowy
    // pin_led8 4 tylne bateria zielony NIEDZIAŁA nie łaczy na wtyczce

    // mosfet1 lewe kulka
    // mosfet2 lewa burta zanetowa
    // mosfet3 prawa burta zanetowa
    // mosfet4 prawa kulka
    // mosfet5 niepodpiety(kable są podwujne)

    //////////////////////////////////////////////////////////////////////////////////////
    if (scaledValues[3] < 80)
    {
      if (burtaL == 0)
      {
        digitalWrite(mosfet2, 0); // lewa burta zanetowa
        czas_burta1 = czas;
        czasburtaL_led = czas;
        burtaL = 1; // lewa burta zanetowa
      }
    }
    else if (100 < scaledValues[3] && scaledValues[3] < 160)
    {
      if (burtaL == 2)
      {
        burtaL = 0;
      }
    }

    if (burtaL == 1 && czas - czas_burta1 > 400)
    {
      digitalWrite(mosfet2, 1); // lewa burta zanetowa
      burtaL = 2;               // resetujemy lewą burtę zanetową
    }
    //
    /////////////////////////////////////////////////////////////////////////////////////////
    /// lewa burta zanetowa
    if (scaledValues[3] > 200)
    {
      if (burtaR == 0)
      {
        digitalWrite(mosfet3, 0); // prawa burta zanetowa
        czas_burta2 = czas;
        czasburtaR_led = czas;
        burtaR = 1; // prawa burta zanetowa
      }
    }
    else if (100 < scaledValues[3] && scaledValues[3] < 160)
    {
      if (burtaR == 2)
      {
        burtaR = 0;
      }
    }

    if (burtaR == 1 && czas - czas_burta2 > 400)
    {
      digitalWrite(mosfet3, 1); // prawa burta zanetowa
      burtaR = 2;               // resetujemy prawa burtę zanetową
    }
    ///
    //////////////////////////////////////////////////////////////////////////////////////////////
    //// światła
    if (czas - czasburtaR_led < 3000)
    {
      if (czas - led_burtaRR > 100)
      {
        led_burtaRR = czas;
        led_burta_stanR = !led_burta_stanR;
        digitalWrite(pin_led2, led_burta_stanR);
      }
    }
    else
    {
      digitalWrite(pin_led2, 1);
    }

    /////// lewe światło
    if (czas - czasburtaL_led < 3000)
    {
      if (czas - led_burtaLL > 100)
      {
        led_burtaLL = czas;
        led_burta_stanL = !led_burta_stanL;
        digitalWrite(pin_led4, led_burta_stanL);
      }
    }
    else
    {
      digitalWrite(pin_led4, 1);
    }

    /////////////////////////////////////////////

    if (scaledValues[4] < 80)
    {
      /*
      10V - 0,476V;
      11V - 0,523V
      12V - 0,571429
      13v - 0,619
 */

      if (adc_vcc_var >= 560)
      {
        digitalWrite(pin_led8, 0);
        digitalWrite(pin_led7, 0);
        digitalWrite(pin_led6, 0);
        digitalWrite(pin_led5, 0);
      }
      else if (adc_vcc_var >= 530)
      {
        digitalWrite(pin_led8, 1);
        digitalWrite(pin_led7, 0);
        digitalWrite(pin_led6, 0);
        digitalWrite(pin_led5, 0);
      }
      else if (adc_vcc_var >= 500)
      {
        digitalWrite(pin_led8, 1);
        digitalWrite(pin_led7, 1);
        digitalWrite(pin_led6, 0);
        digitalWrite(pin_led5, 0);
      }
      else if (adc_vcc_var >= 470)
      {
        digitalWrite(pin_led8, 1);
        digitalWrite(pin_led7, 1);
        digitalWrite(pin_led6, 1);
        digitalWrite(pin_led5, 0);
      }
      else
      {
        digitalWrite(pin_led8, 1);
        digitalWrite(pin_led7, 1);
        digitalWrite(pin_led6, 1);

        if (czas - czas_brak_aku > 100)
        {
          czas_brak_aku = czas;
          stan_led_brak_aku = !stan_led_brak_aku;
        }

        digitalWrite(pin_led5, stan_led_brak_aku);
      }

      digitalWrite(pin_led3, LOW);

      if (czas - czas_kulka_swiatlo1 > 3000)
      {
        digitalWrite(pin_led1, LOW);
      }
    }
    else
    {
      digitalWrite(pin_led3, HIGH);

      if (czas - czas_kulka_swiatlo1 > 3000)
      {
        digitalWrite(pin_led1, HIGH);
      }

      digitalWrite(pin_led8, 1);
      digitalWrite(pin_led7, 1);
      digitalWrite(pin_led6, 1);
      digitalWrite(pin_led5, 1);
    }

    ///////////////////////////////

    if (scaledValues[1] < 80)
    {
      if (kulkaL == 0)
      {
        wyj_kulka1 = 0; // lewa kulka
        czas_kulkaL = czas;
        czas_kulka_swiatlo1 = czas;
        kulkaL = 1; // lewa kulka
      }
    }
    else if (100 < scaledValues[1] && scaledValues[1] < 160)
    {
      if (kulkaL == 2)
      {
        kulkaL = 0;
      }
    }

    if (kulkaL == 1 && czas - czas_kulkaL > 50)
    {
      wyj_kulka1 = 1; // lewa kulka
      kulkaL = 2;     // resetujemy // lewa kulka
    }
    //
    /////////////////////////////////////////////////////////////////////////////////////////
    /// prawa kuluka
    if (scaledValues[1] > 200)
    {
      if (kulkaR == 0)
      {
        wyj_kulka2 = 0; // prawa  kulka
        czas_kulkaR = czas;
        czas_kulka_swiatlo1 = czas;
        kulkaR = 1; // prawa  kulka
      }
    }
    else if (100 < scaledValues[1] && scaledValues[1] < 160)
    {
      if (kulkaR == 2)
      {
        kulkaR = 0;
      }
    }

    if (kulkaR == 1 && czas - czas_kulkaR > 50)
    {
      wyj_kulka2 = 1; // prawa  kulka
      kulkaR = 2;     // resetujemy prawa kulka
    }

    //// swiatło do kulek

    if (czas - czas_kulka_swiatlo1 < 3000)
    {
      if (czas - czas_kulka_swiatlo2 > 100)
      {
        czas_kulka_swiatlo2 = czas;
        stan_swiatlokulka = !stan_swiatlokulka;
        digitalWrite(pin_led1, stan_swiatlokulka);
      }
    }

    if (scaledValues[5] > 200)
    {
      if (czas - czas_pik_kulka > 500)
      {
        czas_pik_kulka = czas;

        wyj_kulka22 = 0;
        wyj_kulka11 = 1;
        czas_pik_kulka1 = czas;
      }

      if (czas - czas_pik_kulka1 > 100)
      {
        wyj_kulka22 = 1;
        wyj_kulka11 = 0;
      }
    }
    else
    {
      wyj_kulka22 = 1;
      wyj_kulka11 = 1;
    }

    if (wyj_kulka2 == 0 || wyj_kulka22 == 0)
    {
      digitalWrite(mosfet4, 0);
    }
    else
    {
      digitalWrite(mosfet4, 1);
    }

    if (wyj_kulka1 == 0 || wyj_kulka11 == 0)
    {
      digitalWrite(mosfet1, 0);
    }
    else
    {
      digitalWrite(mosfet1, 1);
    }
  }
  else
  {
    // Pilot rozłączony - zatrzymaj silniki

    digitalWrite(pin_przekaznika1, LOW);
    digitalWrite(pin_przekaznika2, LOW);

    // analogWrite(silnik1_pwm, 0);
    // analogWrite(silnik2_pwm, 0);

    digitalWrite(mosfet1, HIGH); // modfet1
    digitalWrite(mosfet2, HIGH); // modfet2
    digitalWrite(mosfet3, HIGH); // modfet3
    digitalWrite(mosfet4, HIGH); // modfet4
    digitalWrite(mosfet5, HIGH); // modfet5

    OCR1A = 0;
    OCR1B = 0;
  }
}
