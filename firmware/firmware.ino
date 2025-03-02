#include "tiny_IRremote.h"
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/power.h>

#define adc_disable() (ADCSRA &= ~_BV(ADEN))
#define adc_enable() (ADCSRA |= _BV(ADEN))
#define comparator_disable() (ACSR |= _BV(ACD))
#define comparator_enable() (ACSR &= ~_BV(ACD))

#define IR_LED_PIN PB0       // Пин для ИК-светодиода
#define LOW_POW_LED_PIN PB1  // Пин для led low power
#define TOUCH_PIN PB2        // Пин для touch
#define LOW_POW_PIN PB3      // Пин для low power
#define IR_RECEIVER_PIN PB4  // Пин для ИК-приёмника
#define IR_FREQ_KHZ 38

// Команды
#define CMD_REQUEST_STATUS 0xA0   // Команда для запроса статуса
#define CMD_RESPONSE_STATUS 0xA1  // Команда для запроса статуса

IRrecv irrecv(IR_RECEIVER_PIN);
IRsend irsend(IR_LED_PIN);
decode_results results;  // Для хранения данных от приёмника

const int debounceDelay = 30;  // Задержка для обработки дребезга (мс)
const int minXmtLoops = 2000;  //2000 x 26usec = 52msec
const int minLoopTime = 1;

volatile bool isTouch = false;
volatile bool isIRsend = false;

void setup() {
  // Настройка используемых пинов
  pinMode(TOUCH_PIN, INPUT_PULLUP);  // Кнопка с подтяжкой
  pinMode(IR_LED_PIN, OUTPUT);       // Светодиод как выход
  pinMode(LOW_POW_LED_PIN, OUTPUT);  // Светодиод как выход
  pinMode(LOW_POW_PIN, INPUT);       // Вход низкого уровня заряда 1 - высокий уровень (>2.4V) 0 - низкий уровень (<2.4V)
  pinMode(IR_RECEIVER_PIN, INPUT);   // Вход и IR Led reciver

  irrecv.enableIRIn();
  irsend.enableIROut(IR_FREQ_KHZ);

  digitalWrite(IR_LED_PIN, LOW);       // Выключить светодиод
  digitalWrite(LOW_POW_LED_PIN, LOW);  // Выключить светодиод

}  // setup

void sleep() {

  // разрешает генерацию прерываний на изменение состояния пинов, заданных в регистре PCMSK.
  // Макрос _BV (Bit Value) вычисляет значение, при котором только указанный бит равен 1, а остальные равны 0.
  // Например, _BV(PCIE) для ATtiny85 означает 0b00000010.
  GIMSK |= _BV(PCIE);

  PCMSK |= _BV(PCINT2) | _BV(PCINT4);  // Когда на пине PCINT2 и PCINT4 происходит изменение уровня (LOW → HIGH или HIGH → LOW), генерируется прерывание PCINT0_vect, и микроконтроллер просыпается.
  power_all_disable();                 // Выключаем всё ненужное
  PRR |= _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRUSI);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  sleep_enable();  // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
  sei();           // Enable interrupts
  sleep_cpu();     // sleep

  //This is the point in the program flow where the CPU waits in low power mode until interrupt occurs.
  //Execution continues from this point when interrupt occurs (when probe touches and switch opens).

  cli();                                  // Disable interrupts
  PCMSK &= ~(_BV(PCINT2) | _BV(PCINT4));  // Turn off PB2 and PB4 as interrupt pins
  sleep_disable();                        // Clear SE bit
  power_all_enable();
  PRR &= ~(_BV(PRTIM0) | _BV(PRTIM1) | _BV(PRUSI));



}  // sleep
volatile unsigned long lastInterruptTime = 0;
ISR(PCINT0_vect) {
  unsigned long interruptTime = micros();        // Используем микросекунды вместо 
  if (interruptTime - lastInterruptTime > debounceDelay) {  // Дебаунс 
    // This is called when the interrupt occurs, No code is here so it immediately returns to point in program flow
    //following the statement in the sleep() function that initially put the CPU to sleep.
    if (digitalRead(TOUCH_PIN) == HIGH) {
      isTouch = true;
    }
    if (irrecv.decode(&results) && results.value != 0xFFFFFFFF) {  // Проверяем, действительно ли есть сигнал
      isIRsend = true;
    }
  }
  lastInterruptTime = interruptTime;
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
    }
  }
}

void handleIRCommand() {
  if (isIRsend) {
    if (results.value == CMD_REQUEST_STATUS) {
      int counter = minLoopTime;
      bool isPower = isLowPower();
      while (counter-- >= 0 && isIRsend) {
        irsend.sendNEC(isPower, 16);
      }
    }
    irrecv.resume();
    isIRsend = false;
  }
}

void loop() {
  handleIRCommand();
  handleTouch();
  digitalWrite(IR_LED_PIN, LOW);
  sleep();
}
