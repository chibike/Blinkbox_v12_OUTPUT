SensorObjects::SensorObjects()
{
  #ifdef PRINT_STATUS
  Serial.println(F("Launching Sensor Objects....."));
  #endif
  
  _destroyed = true;
}

void SensorObjects::begin()
{
  _destroyed = false;
  
  #ifdef PRINT_STATUS
  Serial.println(F("Launched"));
  #endif
}

SensorObjects::~SensorObjects()
{
  end();
}

void SensorObjects::end()
{
  DistanceSensor.end();
  RfidSensor.end();
  
#ifdef ALLOW_ACCESS_2_COMPASS
  CompassSensor.end();
#endif
  
  _destroyed = true;
}

