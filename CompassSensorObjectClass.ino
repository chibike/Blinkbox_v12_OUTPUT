void CompassSensorObject::_i2c_wait_timeout( int timeout)
{
  long startTime = millis();
  while(Wire.available() <= 0 && (millis()-startTime) < timeout)
  {
    //wait
  }
}

float CompassSensorObject::getHeading()
{
  if ( _destroyed == true ){return 0;}
  Wire.beginTransmission(_address);
  Wire.write(2);
  Wire.endTransmission();

  Wire.requestFrom(_address, 2);
  _i2c_wait_timeout(100);

  if (Wire.available() > 0)
  {
    float angle = ( (int)Wire.read() << 8 ) | ( (int)Wire.read() & 0x0f );
    return 360 - angle/10.0;
  }
  else
  {
    return 0;
  }
}

float CompassSensorObject::getTargetDeviation()
{
  if ( _destroyed == true ){return 0;}
  float trueHeading = getHeading();
  switch( round(trueHeading) )
  {
    case 0:
    case 360:
      trueHeading = 0;
      break;
    default:
      if (trueHeading > 180)
      {
        trueHeading = -1*(360 - trueHeading);
      }
  }

  //Centralize Heading about target Angle
  return trueHeading - _targetHeading;  
}

void CompassSensorObject::setTargetHeading(int heading)
{
  if ( _destroyed == true ){return;}
  switch(heading)
  {
    case 0:
    case 360:
    case -360:
      heading = 0;
      break;
    default:
      if (heading > 180)
      {
        heading = -1*(360 - heading);
      }
  }
  _targetHeading = heading;
}

CompassSensorObject::CompassSensorObject()
{
  _destroyed = true;
}

void CompassSensorObject::begin()
{
  _address = 0x60;
  _targetHeading = 0;
  
  _destroyed = false;
}

CompassSensorObject::~CompassSensorObject()
{
  end();
}

void CompassSensorObject::end()
{
  _destroyed = true;
}

