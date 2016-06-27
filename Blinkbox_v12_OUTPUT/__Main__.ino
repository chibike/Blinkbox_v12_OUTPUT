#include <Arduino.h>

Blink_OS Blink_OS_v12;
void setup()
{
  Serial.begin(9600);
  Serial.println("Starting...");
  Blink_OS_v12.begin();
  //Blink_OS_v12.fd_debug(120, 40);
  //Blink_OS_v12.forwardDistance(180, 200);
  //Blink_OS_v12.pheripherals.CompassSensor.setTargetHeading( Blink_OS_v12.pheripherals.CompassSensor.getHeading() );
  Blink_OS_v12.pheripherals.Lights.flashLights();
  Blink_OS_v12.pheripherals.ShiftRegister.setHigh(0);
}

void loop()
{
  //Serial.print("Target Error = ");
  //Serial.println( Blink_OS_v12.pheripherals.CompassSensor.getTargetDeviation() );

  Serial.print("Temp = ");
  Serial.println( Blink_OS_v12.pheripherals.CompassSensor.getTemp() );
  Serial.print("Pitch = ");
  Serial.println( Blink_OS_v12.pheripherals.CompassSensor.getPitch() );
  Serial.print("Roll = ");
  Serial.println( Blink_OS_v12.pheripherals.CompassSensor.getRoll() );
  Serial.print("Acc X = ");
  Serial.println( Blink_OS_v12.pheripherals.CompassSensor.getMagX() );
  Serial.print("Acc Y = ");
  Serial.println( Blink_OS_v12.pheripherals.CompassSensor.getMagY() );
  Serial.print("Acc Z = ");
  Serial.println( Blink_OS_v12.pheripherals.CompassSensor.getMagZ() );
  Serial.print("Mag X = ");
  Serial.println( Blink_OS_v12.pheripherals.CompassSensor.getAccX() );
  Serial.print("Mag Y = ");
  Serial.println( Blink_OS_v12.pheripherals.CompassSensor.getAccY() );
  Serial.print("Mag Z = ");
  Serial.println( Blink_OS_v12.pheripherals.CompassSensor.getAccZ() );
  Serial.print("Gyr X = ");
  Serial.println( Blink_OS_v12.pheripherals.CompassSensor.getGyrX() );
  Serial.print("Gyr Y = ");
  Serial.println( Blink_OS_v12.pheripherals.CompassSensor.getGyrY() );
  Serial.print("Gyr Z = ");
  Serial.println( Blink_OS_v12.pheripherals.CompassSensor.getGyrZ() );
  Serial.print("\n");
  delay(1000);
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
