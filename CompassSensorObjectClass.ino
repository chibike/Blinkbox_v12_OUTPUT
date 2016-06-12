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

void CompassSensorObject::end()
{
  _destroyed = true;
}

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
  _i2c_wait_timeout(200);

  if (Wire.available() > 0)
  {
    uint16_t reading = (int)(( ((int)Wire.read()) << 8 ) | ( ((int)Wire.read()) & 0x0f ));
    float angle = (float)reading/10.0;
    angle = 360 - angle;
    _errorCounter = 0;
    return angle;
  }
  else
  {
    if( _errorCounter >= 5 )
    {
      _errorCounter = 0;
      return 0;
    }
    else
    {
      _errorCounter++;
      return getHeading();
    }
  }
}

float CompassSensorObject::getTargetDeviation()
{
  if ( _destroyed == true ){return 0;}
  float trueHeading = getHeading();
  switch( (int)round(trueHeading) )
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
