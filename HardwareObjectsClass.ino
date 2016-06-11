HardwareObjects::HardwareObjects()
{
  _destroyed = true;
}

void HardwareObjects::begin()
{
  _destroyed = false;
}

HardwareObjects::~HardwareObjects()
{
  end();
}

void HardwareObjects::end()
{
  _destroyed = true;
  
  Wheels.end();
  Lights.end();
  Steering.end();
  ShiftRegister.end();
  LineSensor.end();
  Horn.end();
  CompassSensor.end();
}
