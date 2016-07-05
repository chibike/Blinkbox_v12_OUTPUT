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
  Blink_OS_v12.runStartSequence();

  //Blink_OS_v12.forwardDistance(150, 70);
}

void loop()
{
  Serial.println("In loop");
  switch( commandVar )
  {
    case FWD_DIST:
      commandVar = NONE_CMD;
      Blink_OS_v12.forwardDistance(150, distanceVar);//powerVar, distanceVar);
      break;
    case BWD_DIST:
      commandVar = NONE_CMD;
      Blink_OS_v12.backwardDistance(150, distanceVar);//powerVar, distanceVar);
      break;
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
  //Blink_OS_v12._REQUEST_EVENT();
}

ISR(TIMER2_COMPA_vect)
{
  Blink_OS_v12._SCHEDULER();
}

bool onTime(unsigned long lastUpdateTime, uint16_t waitTime)
{
  if (millis() - lastUpdateTime >= waitTime)
  {
    return true;
  }
  return false;
}
