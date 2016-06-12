#include <Arduino.h>

Blink_OS Blink_OS_v12;
void setup()
{
  Serial.begin(9600);
  Blink_OS_v12.begin();
  //Blink_OS_v12.fd_debug(120, 100);
  Blink_OS_v12.pheripherals.CompassSensor.setTargetHeading( Blink_OS_v12pheripherals.CompassSensor.getHeading() );
}

void loop()
{
  Serial.print("Target Error = ");
  Serial.println( Blink_OS_v12.pheripherals.CompassSensor.getTargetDeviation() );
  Serial.print("Heading = ");
  Serial.println( Blink_OS_v12.pheripherals.CompassSensor.getHeading() );
  delay(250);
}

void shutdown()
{
  while(1)
  {
    delay(1000);
  }
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
  Blink_OS_v12._REQUEST_EVENT();
}

ISR(TIMER2_COMPA_vect)
{
  Blink_OS_v12._SCHEDULER();
}
