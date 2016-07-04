#include <Arduino.h>
#define FAN_INDEX 0

Blink_OS Blink_OS_v12;
void setup()
{
  Serial.begin(9600);
  Serial.println("Starting...");
  Blink_OS_v12.begin();
  Blink_OS_v12.pheripherals.Lights.flashLights();
  Blink_OS_v12.pheripherals.ShiftRegister.setHigh(FAN_INDEX);
  Blink_OS_v12.pheripherals.Steering.set2center();

  Blink_OS_v12.pheripherals.Horn.on();
  delay(70);
  Blink_OS_v12.pheripherals.Horn.off();
  delay(210);
  Blink_OS_v12.pheripherals.Horn.on();
  delay(70);
  Blink_OS_v12.pheripherals.Horn.off();

  delay(750);

  Blink_OS_v12.pheripherals.Horn.on();
  delay(70);
  Blink_OS_v12.pheripherals.Horn.off();
  delay(210);
  Blink_OS_v12.pheripherals.Horn.on();
  delay(70);
  Blink_OS_v12.pheripherals.Horn.off();
  
}

void loop()
{
  Serial.println("In loop");
  delay(1000);
  //Blink_OS_v12.pheripherals.Steering.setheading(40);
  delay(1000);
  //Blink_OS_v12.pheripherals.Steering.setheading(-40);
}

void RIGHT_WHEEL_ISR()
{
  Blink_OS_v12._RIGHT_WHEEL_ISR();
}

void LEFT_WHEEL_ISR()
{
  Blink_OS_v12._LEFT_WHEEL_ISR();
}

void RECEIVE_EVENT(int howmany)
{
  Blink_OS_v12._RECEIVE_EVENT();
}

void REQUEST_EVENT()
{
  //Blink_OS_v12._REQUEST_EVENT();
}

ISR(TIMER2_COMPA_vect)
{
  Blink_OS_v12._SCHEDULER();
}
