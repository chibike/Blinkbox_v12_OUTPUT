#include <Arduino.h>

Blink_OS Blink_OS_v12;
void setup()
{
  Blink_OS_v12.begin();
}

void loop()
{
}

void shutdown()
{
  while(1)
  {
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
