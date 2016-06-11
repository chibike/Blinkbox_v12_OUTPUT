CompassSensorObject::CompassSensorObject()
{
  _destroyed = true;
}

void CompassSensorObject::begin()
{
  _address = 0x60;
  _targetHeading = 0;
  _lastHeading = 0;
  _diffThreshold = 20;
  _confirmHeading = true;
  
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
  _i2c_wait_timeout(100);

  if (Wire.available() > 0)
  {
    float angle = ( (int)Wire.read() << 8 ) | ( (int)Wire.read() & 0x0f );
    angle = 360.0 - ((float)angle/10.0);

    if( _confirmHeading == true )
    {
      _confirmHeading = false;
      return angle;
    }
    else if(angle-_diffThreshold >= _lastHeading && angle+_diffThreshold <= _lastHeading)
    {
      _lastHeading = angle;
      return angle;
    }
    else
    {
      _confirmHeading = true;
      return getHeading();
    }
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
