DistanceSensorObject::DistanceSensorObject()
{
  #ifdef PRINT_STATUS
  Serial.println(F("Launching Distance Sensor..."));
  #endif
  
  _destroyed = true;
}

void DistanceSensorObject::begin()
{
  #ifdef PRINT_STATUS
  Serial.println(F("Launched"));
  #endif
  
  _destroyed = false;
}

DistanceSensorObject::~DistanceSensorObject()
{
  end();
}

void DistanceSensorObject::end()
{
  frontSensor.end();
  _destroyed = true;
}

