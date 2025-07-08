#include <IRremote.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/power.h>


// #define IR_LED_PIN PB0       //11      // Пин для ИК-светодиода
// #define LOW_POW_LED_PIN PB1  //12      // Пин для led low power
// #define TOUCH_PIN PB2        //14      // Пин для touch
// #define LOW_POW_PIN PB3      //2       // Пин для low power
// #define IR_RECEIVER_PIN PB4  //5       // Пин для ИК-приёмника
#define IR_FREQ_KHZ 38

// Пины, соответствующие ATtiny84
#define IR_LED_PIN A1       // PA1 - IR светодиоды (через транзистор)
#define LOW_POW_LED_PIN A2  // PA2 - индикатор low power (через транзистор)
#define TOUCH_PIN 2         // PB2 - touch
#define LOW_POW_PIN A0      // PA0 - вход низкого напряжения
#define IR_RECEIVER_PIN A3  // PA3 - IR-приемник (data output)


// Команды
#define CMD_REQUEST_STATUS 0xA0   // Команда для запроса статуса
#define CMD_RESPONSE_STATUS 0xA1  // Команда для запроса статуса

IRrecv irrecv(IR_RECEIVER_PIN);
IRsend irsend(IR_LED_PIN);

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
  pinMode(IR_LED_PIN, OUTPUT);             // Светодиод как выход
  pinMode(LOW_POW_LED_PIN, OUTPUT);        // Светодиод как выход
  pinMode(LOW_POW_PIN, INPUT);             // Вход низкого уровня заряда 1 - высокий уровень (>2.4V) 0 - низкий уровень (<2.4V)
  pinMode(IR_RECEIVER_PIN, INPUT_PULLUP);  // Вход и IR Led reciver

  irrecv.enableIRIn();
  irsend.enableIROut(IR_FREQ_KHZ);

  digitalWrite(IR_LED_PIN, LOW);       // Выключить светодиод
  digitalWrite(LOW_POW_LED_PIN, LOW);  // Выключить светодиод

  isTouch = digitalRead(TOUCH_PIN) == HIGH;
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
      digitalWrite(IR_LED_PIN, HIGH);  // this takes about 1 microsecond to happen
      delayMicroseconds(12);           // hang out for 12 microseconds
      digitalWrite(IR_LED_PIN, LOW);   // this also takes about 1 microsecond
      delayMicroseconds(12);           // hang out for 12 microseconds
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
  digitalWrite(IR_LED_PIN, LOW);

  unsigned long sleepStart = millis();
  while (millis() - sleepStart < wakeTime) {
    if (digitalRead(TOUCH_PIN) == HIGH) {
      sleepStart = millis();
      handleTouch();
    }
  }
  sleep();
}
