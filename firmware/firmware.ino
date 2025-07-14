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

#define TOUCH_PIN 8        // PB2 - Кнопка (физический пин 14)

#define LOW_POW_LED_PIN 9  // PB1 - Индикатор low power (физический пин 12)
#define LOW_POW_PIN 7      // PA7 - Вход низкого напряжения (физический пин 15)

#define IR_RECEIVER_PIN 3  // PA2 - IR-приёмник (физический пин 3)
#define IR_SEND_PIN 10     // PB0 - IR светодиоды (физический пин 4)
#define IRREC_POW 2         // PA3 - IR-приёмник питание (физический пин 2)

// Команды
#define CMD_REQUEST_STATUS 0xA0   // Команда для запроса статуса
#define CMD_RESPONSE_STATUS 0xA1  // Команда для запроса статуса

IRrecv irrecv(IR_RECEIVER_PIN);
IRsend irsend;

decode_results results;  // Для хранения данных от приёмника

const int debounceDelay = 30;  // Задержка для обработки дребезга (мс)
const int minXmtLoops = 2000;  //2000 x 26usec = 52msec
const int minLoopTime = 1;
const unsigned long wakeTime = 1000;  // Время ожидания перед сном (мс)

volatile bool isTouch = false;
volatile bool isIRsend = false;


void setup() {
  // Настройка используемых пинов
  pinMode(TOUCH_PIN, INPUT_PULLUP);        // Кнопка с подтяжкой

  pinMode(IR_SEND_PIN, OUTPUT);            // Светодиод как выход
  pinMode(LOW_POW_LED_PIN, OUTPUT);        // Светодиод как выход
  pinMode(LOW_POW_PIN, INPUT);             // Вход низкого уровня заряда 1 - высокий уровень (>2.4V) 0 - низкий уровень (<2.4V)
  pinMode(IR_RECEIVER_PIN, INPUT_PULLUP);  // Вход и IR Led reciver

  irrecv.enableIRIn();
  irsend.enableIROut(IR_FREQ_KHZ);


  digitalWrite(IR_SEND_PIN, LOW);      // Выключить светодиод
  digitalWrite(LOW_POW_LED_PIN, LOW);  // Выключить светодиод

  isTouch = digitalRead(TOUCH_PIN) == HIGH;
  byte count = 3;
  if (isTouch) {
    count = 1;
  }
  for (int i = 0; i < count; i++) {

    digitalWrite(LOW_POW_LED_PIN, HIGH);
    delay(50);
    digitalWrite(LOW_POW_LED_PIN, LOW);
    delay(50);
  }

}  // setup

void sleep() {
  // Разрешаем прерывания на изменение состояния пинов:
  // - PA2 (PCINT2): TOUCH_PIN
  // - PA3 (PCINT3): IR_RECEIVER_PIN
  // - PB2 (PCINT10): возможно, тоже используется как TOUCH_PIN в зависимости от макросов

  // Разрешаем Pin Change Interrupts на портах A и B
  GIMSK |= _BV(PCIE0) | _BV(PCIE1);

  // Включаем прерывания на PA2 и PA3 (PCINT2, PCINT3)
  PCMSK0 |= _BV(PCINT2) | _BV(PCINT3);

  // Включаем прерывание на PB2 (PCINT10) — если он действительно используется (TOUCH_PIN = 2)
  PCMSK1 |= _BV(PCINT10);

  // Выключаем все ненужные модули питания
  power_all_disable();

  // Отключаем таймеры и USI (SPI/I2C)
  PRR |= _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRUSI);

  // Устанавливаем режим глубокого сна (Power-down)
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  sleep_enable();  // Разрешаем сон
  sei();           // Разрешаем прерывания
  sleep_cpu();     // Переходим в спящий режим

  // Здесь программа просыпается после прерывания

  cli();  // Отключаем прерывания перед очисткой

  // Отключаем разрешения на Pin Change Interrupts
  PCMSK0 &= ~(_BV(PCINT2) | _BV(PCINT3));
  PCMSK1 &= ~_BV(PCINT10);
  sleep_disable();  // Запрещаем дальнейший сон

  power_all_enable();  // Включаем питание обратно

  // Включаем обратно отключенные модули
  PRR &= ~(_BV(PRTIM0) | _BV(PRTIM1) | _BV(PRUSI));
}  // sleep

ISR(PCINT0_vect) {
  // Обработка PA (PCIE0)
  if (digitalRead(TOUCH_PIN) == HIGH) {
    isTouch = true;
  }
}

ISR(PCINT1_vect) {
  // Обработка PB (PCIE1)
  if (irrecv.decode(&results) && results.value != 0xFFFFFFFF) {
    isIRsend = true;
  }
}

bool isLowPower() {
  return digitalRead(LOW_POW_PIN) == LOW;
}

void handleTouch() {
  if (isTouch) {
    isTouch = false;
    delay(debounceDelay);
    while (digitalRead(TOUCH_PIN) == HIGH) {
      digitalWrite(IR_SEND_PIN, HIGH);  // this takes about 1 microsecond to happen
      delayMicroseconds(12);            // hang out for 12 microseconds
      digitalWrite(IR_SEND_PIN, LOW);   // this also takes about 1 microsecond
      delayMicroseconds(12);            // hang out for 12 microseconds
      if (isLowPower() && digitalRead(LOW_POW_LED_PIN) == LOW) {
        digitalWrite(LOW_POW_LED_PIN, HIGH);
      }
    }
  }
}

void handleIRCommand() {
  if (isIRsend) {
    if (results.value == CMD_REQUEST_STATUS) {
      int counter = minLoopTime;
      while (counter-- >= 0 && isIRsend) {
        irsend.sendNEC(isLowPower(), 16);
      }
    }
    irrecv.resume();
    isIRsend = false;
  }
}

void loop() {
  isLowPower();
  handleIRCommand();
  handleTouch();
  digitalWrite(LOW_POW_LED_PIN, LOW);
  digitalWrite(IR_SEND_PIN, LOW);


  unsigned long sleepStart = millis();
  while (millis() - sleepStart < wakeTime) {
    if (digitalRead(TOUCH_PIN) == HIGH) {
      sleepStart = millis();
      handleTouch();
    }
  }
  sleep();
}
