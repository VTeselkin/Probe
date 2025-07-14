/*
ATtiny84A QFN20 pin mapping (AttinyCore - Clockwise, QFN-20):

Arduino | Port | Phys.Pin | Comment
----------------------------------------
   0    |  PB0 |  11      | Digital pin 0
   1    |  PB1 |  12      | Digital pin 1
   2    |  PB2 |  14      | Digital pin 2
   3    |  PB3 |  13      | Digital pin 3 (RESET, если не отключён)
   4    |  PA0 |   5      | Digital pin 4 (AREF / Analog In)
   5    |  PA1 |   4      | Digital pin 5
   6    |  PA2 |   3      | Digital pin 6
   7    |  PA3 |   2      | Digital pin 7
   8    |  PA4 |   1      | Digital pin 8
   9    |  PA5 |  20      | Digital pin 9 (PWM)
  10    |  PA6 |  16      | Digital pin 10 (PWM)
  11    |  PA7 |  15      | Digital pin 11 (PWM)

Additional pins (QFN-20 only, **manual access via PORTB**):

   -    |  PB4 |  15      | Manual use only (no Arduino pin)
   -    |  PB5 |  16      | Manual use only (no Arduino pin)
   -    |  PB6 |  17      | Manual use only (no Arduino pin)
   -    |  PB7 |  18      | Manual use only (no Arduino pin)

   QFN-20 Top View (Clockwise Mapping)

    +----+----+
  1 | PA4 | PA3 | 2
 20 | PA5 | PA2 | 3
 19 | NC  | PA1 | 4
 18 | NC  | PA0 | 5
 17 | NC  | NC  | 6
 16 | PA6 | GND | 7
 15 | PA7 | VCC | 8
 14 | PB2 | PB0 | 9
 13 | PB3 | PB1 | 12
    +----+----+
*/

#include <IRremote.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/power.h>

#define IR_FREQ_KHZ 38

// Пины для ATtiny84 QFN-20 (Clockwise Mapping, AttinyCore)

#define TOUCH_PIN 8  // PB2 - Кнопка (физический пин 14)

#define LOW_POW_LED_PIN 9  // PB1 - Индикатор low power (физический пин 12)
#define LOW_POW_PIN 7      // PA7 - Вход низкого напряжения (физический пин 15)

#define IR_RECEIVER_PIN 3  // PA2 - IR-приёмник (физический пин 3)
#define IR_SEND_PIN 10     // PB0 - IR светодиоды (физический пин 4)
#define IRREC_POW 2        // PA3 - IR-приёмник питание (физический пин 2)

// Команды
#define CMD_REQUEST_STATUS 0xA0
#define CMD_RESPONSE_STATUS 0xA1

IRrecv irrecv(IR_RECEIVER_PIN);
IRsend irsend;

decode_results results;

const int debounceDelay = 1;
const int minXmtLoops = 2000;
const int minLoopTime = 1;

volatile bool isTouchReleased = false;
volatile bool lastTouchState = false;

void setup() {
  pinMode(TOUCH_PIN, INPUT_PULLUP);
  pinMode(IR_SEND_PIN, OUTPUT);
  pinMode(LOW_POW_LED_PIN, OUTPUT);
  pinMode(LOW_POW_PIN, INPUT);
  pinMode(IR_RECEIVER_PIN, INPUT_PULLUP);

  irrecv.enableIRIn();
  irsend.enableIROut(IR_FREQ_KHZ);

  digitalWrite(IR_SEND_PIN, LOW);
  digitalWrite(LOW_POW_LED_PIN, LOW);

  // Начальное состояние кнопки
  lastTouchState = digitalRead(TOUCH_PIN);
  if (lastTouchState == HIGH) {
    isTouchReleased = true;  // Кнопка уже отжата на старте — сразу включаем светодиод
  }
  byte count = 3;
  if (lastTouchState) count = 1;
  for (int i = 0; i < count; i++) {
    digitalWrite(LOW_POW_LED_PIN, HIGH);
    delay(50);
    digitalWrite(LOW_POW_LED_PIN, LOW);
    delay(50);
  }

  // Настройка Pin Change Interrupt на PB (где находится TOUCH_PIN)
  GIMSK |= _BV(PCIE1);
  PCMSK1 |= _BV(PCINT10);  // PB2 - PCINT10

  sei();
}

void sleep() {
  power_all_disable();
  PRR |= _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRUSI);

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sei();
  sleep_cpu();

  // После пробуждения
  sleep_disable();
  power_all_enable();
  PRR &= ~(_BV(PRTIM0) | _BV(PRTIM1) | _BV(PRUSI));
}

// Прерывание по изменению на PB (включая PB2 - TOUCH_PIN)
ISR(PCINT1_vect) {
  bool currentState = digitalRead(TOUCH_PIN);
  if (currentState == HIGH && lastTouchState == LOW) {
    isTouchReleased = true;  // Только отжатие
  }
  lastTouchState = currentState;
}

bool isLowPower() {
  return digitalRead(LOW_POW_PIN) == LOW;
}

void handleTouch() {
  if (isTouchReleased) {
    isTouchReleased = false;
    delay(debounceDelay);

    while (digitalRead(TOUCH_PIN) == HIGH) {
      digitalWrite(IR_SEND_PIN, HIGH);
      delayMicroseconds(12);
      digitalWrite(IR_SEND_PIN, LOW);
      delayMicroseconds(12);

      if (isLowPower() && digitalRead(LOW_POW_LED_PIN) == LOW) {
        digitalWrite(LOW_POW_LED_PIN, HIGH);
      }
    }
  }
}

void loop() {
  handleTouch();
  digitalWrite(LOW_POW_LED_PIN, LOW);
  digitalWrite(IR_SEND_PIN, LOW);
  sleep();
}
