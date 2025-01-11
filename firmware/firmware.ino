
#include <avr/sleep.h>
#include <avr/interrupt.h>

#define adc_disable() (ADCSRA &= ~_BV(ADEN))
#define adc_enable() (ADCSRA |= _BV(ADEN))
#define comparator_disable() (ACSR |= _BV(ACD))
#define comparator_enable() (ACSR &= ~_BV(ACD))

const int switchPin = 2;
const int IRled = 1;
const int minXmtLoops = 2000;  //2000 x 26usec = 52msec
volatile bool isTousch = false;

void setup() {
  // Настройка неиспользуемых пинов для минимизации энергопотребления

  pinMode(0, OUTPUT);
  digitalWrite(0, LOW);
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  // Настройка используемых пинов
  pinMode(switchPin, INPUT);  // Кнопка с подтяжкой
  pinMode(IRled, OUTPUT);            // Светодиод как выход
  digitalWrite(IRled, LOW);          // Выключить светодиод
}  // setup

void sleep() {

  // разрешает генерацию прерываний на изменение состояния пинов, заданных в регистре PCMSK.
  // Макрос _BV (Bit Value) вычисляет значение, при котором только указанный бит равен 1, а остальные равны 0.
  // Например, _BV(PCIE) для ATtiny85 означает 0b00000010.
  GIMSK |= _BV(PCIE);

  // Когда на пине PCINT2 происходит изменение уровня (LOW → HIGH или HIGH → LOW), генерируется прерывание PCINT0_vect, и микроконтроллер просыпается.
  PCMSK |= _BV(PCINT2);
  comparator_disable();
  adc_disable();  // ADC off
  PRR |= _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRUSI);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // replaces above statement(?DW-what does this comment mean?)

  sleep_enable();  // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
  sei();           // Enable interrupts
  sleep_cpu();     // sleep

  //This is the point in the program flow where the CPU waits in low power mode until interrupt occurs.
  //Execution continues from this point when interrupt occurs (when probe touches and switch opens).

  cli();                  // Disable interrupts
  PCMSK &= ~_BV(PCINT2);  // Turn off PB1 as interrupt pin
  sleep_disable();        // Clear SE bit

  comparator_enable();
  adc_enable();  // ADC on
  PRR &= ~(_BV(PRTIM0) | _BV(PRTIM1) | _BV(PRUSI));



}  // sleep

ISR(PCINT0_vect) {
  // This is called when the interrupt occurs, No code is here so it immediately returns to point in program flow
  //following the statement in the sleep() function that initially put the CPU to sleep.
  // pinMode(switchPin, INPUT_PULLUP);  // Восстановить кнопку
  // pinMode(IRled, OUTPUT);  // Восстановить светодиод
  // digitalWrite(IRled, LOW);
  isTousch = true;
}

void loop() {
  int i = minXmtLoops;  //now awake, we will send at least this many cycles of 38KHz for switch debounce


  if (isTousch) {
    delay(50);  // Optional: Adjust debounce time if needed
  }

  while ((digitalRead(switchPin) == HIGH) || isTousch)  //Expecting a normally closed switch
  {
    isTousch = true;
    // 38 kHz is about 13 microseconds high and 13 microseconds low
    digitalWrite(IRled, HIGH);  // this takes about 1 microsecond to happen
    delayMicroseconds(12);      // hang out for 12 microseconds
    digitalWrite(IRled, LOW);   // this also takes about 1 microsecond
    delayMicroseconds(12);      // hang out for 12 microseconds

    if (i > 0) {
      i--;  //decrement debounce counter
    } else {
      isTousch = false;
    }
  }

  delay(50);  //now that switch has closed (probe no longer touching), 100msec delay for debounce before sleeping
  isTousch = false;

  if (digitalRead(switchPin) == LOW) {
    // Перед сном: отключить пины
    // pinMode(switchPin, INPUT);  // Перевод кнопки в режим INPUT без подтяжки
    // pinMode(IRled, INPUT);  // Перевод светодиода в режим INPUT
    sleep();
  }
}  // loop