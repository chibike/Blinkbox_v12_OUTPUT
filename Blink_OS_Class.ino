Blink_OS::Blink_OS()
{
  _destroyed = true;
}
  
void Blink_OS::begin()
{
  //Sensors.begin();
  //Sensors.DistanceSensor.begin();
  //Sensors.DistanceSensor.frontSensor.begin( 2, 3 );//uint8_t trigPin, uint8_t echoPin
  //Sensors.RfidSensor.begin( 5 ); //cardDetectPin
  //#ifdef ALLOW_ACCESS_2_COMPASS
  //Sensors.CompassSensor.begin();
  //#endif
  
  IntraSystemCalls.begin();
  FileExplorer.begin( 4 );
  
  #ifdef PRINT_STATUS
  Serial.println(F("Blinkbox OS v12 [Version 0.1]"));
  Serial.println(F("(c) 2015 Blink Corporation. All rights reserved."));
  #endif
  
  _destroyed = false;
}

Blink_OS::~Blink_OS()
{
  end();
}

void Blink_OS::end()
{
  //Sensors.end();
  IntraSystemCalls.end();
  Ui.end();
  FileExplorer.end();
  _destroyed = true;
  shutdown();
}

