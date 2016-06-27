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
    int reading = 0;
    byte data1 = Wire.read();
    byte data2 = Wire.read();
    reading = (reading | data1) << 8;
    reading = reading | data2;
    
    float angle = (float)reading/10.0;
    if (angle >= 360 || angle < 0)
    {
      angle = 0.0;
    }
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

bool CompassSensorObject::onStall()
{
  if ( _destroyed == true ){return false;}
  if( abs( getAccY() ) < 3 && abs( getAccX() < 3) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

#ifdef ENABLE_COMPASS_FULL_DEF

int8_t CompassSensorObject::getPitch()
{
  if ( _destroyed == true ){return 0;}
  Wire.beginTransmission(_address);
  Wire.write(4);
  Wire.endTransmission();

  Wire.requestFrom(_address, 1);
  _i2c_wait_timeout(200);

  if (Wire.available() > 0)
  {
    int8_t pitch = Wire.read();
    _errorCounter = 0;
    return pitch;
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
      return getPitch();
    }
  }
}

int8_t CompassSensorObject::getRoll()
{
  if ( _destroyed == true ){return 0;}
  Wire.beginTransmission(_address);
  Wire.write(5);
  Wire.endTransmission();

  Wire.requestFrom(_address, 1);
  _i2c_wait_timeout(200);

  if (Wire.available() > 0)
  {
    int8_t roll = Wire.read();
    _errorCounter = 0;
    return roll;
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
      return getRoll();
    }
  }
}

int16_t CompassSensorObject::getMagX()
{
  if ( _destroyed == true ){return 0;}
  Wire.beginTransmission(_address);
  Wire.write(6);
  Wire.endTransmission();

  Wire.requestFrom(_address, 2);
  _i2c_wait_timeout(200);

  if (Wire.available() > 0)
  {
    int16_t magX = ((int)Wire.read() << 8) | ((int)Wire.read() & 0x00ff);
    _errorCounter = 0;
    return magX;
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
      return getMagX();
    }
  }
}

int16_t CompassSensorObject::getMagY()
{
  if ( _destroyed == true ){return 0;}
  Wire.beginTransmission(_address);
  Wire.write(8);
  Wire.endTransmission();

  Wire.requestFrom(_address, 2);
  _i2c_wait_timeout(200);

  if (Wire.available() > 0)
  {
    int16_t magY = ((int)Wire.read() << 8) | ((int)Wire.read() & 0x00ff);
    _errorCounter = 0;
    return magY;
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
      return getMagY();
    }
  }
}

int16_t CompassSensorObject::getMagZ()
{
  if ( _destroyed == true ){return 0;}
  Wire.beginTransmission(_address);
  Wire.write(10);
  Wire.endTransmission();

  Wire.requestFrom(_address, 2);
  _i2c_wait_timeout(200);

  if (Wire.available() > 0)
  {
    int16_t magZ = ((int)Wire.read() << 8) | ((int)Wire.read() & 0x00ff);
    _errorCounter = 0;
    return magZ;
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
      return getMagZ();
    }
  }
}

int16_t CompassSensorObject::getAccX()
{
  if ( _destroyed == true ){return 0;}
  Wire.beginTransmission(_address);
  Wire.write(12);
  Wire.endTransmission();

  Wire.requestFrom(_address, 2);
  _i2c_wait_timeout(200);

  if (Wire.available() > 0)
  {
    int16_t accX = ((int)Wire.read() << 8) | ((int)Wire.read() & 0x00ff);
    _errorCounter = 0;
    return accX;
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
      return getAccX();
    }
  }
}

int16_t CompassSensorObject::getAccY()
{
  if ( _destroyed == true ){return 0;}
  Wire.beginTransmission(_address);
  Wire.write(14);
  Wire.endTransmission();

  Wire.requestFrom(_address, 2);
  _i2c_wait_timeout(200);

  if (Wire.available() > 0)
  {
    int16_t accY = ((int)Wire.read() << 8) | ((int)Wire.read() & 0x00ff);
    _errorCounter = 0;
    return accY;
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
      return getAccY();
    }
  }
}

int16_t CompassSensorObject::getAccZ()
{
  if ( _destroyed == true ){return 0;}
  Wire.beginTransmission(_address);
  Wire.write(16);
  Wire.endTransmission();

  Wire.requestFrom(_address, 2);
  _i2c_wait_timeout(200);

  if (Wire.available() > 0)
  {
    int16_t accZ = ((int)Wire.read() << 8) | ((int)Wire.read() & 0x00ff);
    _errorCounter = 0;
    return accZ;
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
      return getAccZ();
    }
  }
}

int16_t CompassSensorObject::getGyrX()
{
  if ( _destroyed == true ){return 0;}
  Wire.beginTransmission(_address);
  Wire.write(18);
  Wire.endTransmission();

  Wire.requestFrom(_address, 2);
  _i2c_wait_timeout(200);

  if (Wire.available() > 0)
  {
    int16_t gyrX = ((int)Wire.read() << 8) | ((int)Wire.read() & 0x00ff);
    _errorCounter = 0;
    return gyrX;
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
      return getGyrX();
    }
  }
}

int16_t CompassSensorObject::getGyrY()
{
  if ( _destroyed == true ){return 0;}
  Wire.beginTransmission(_address);
  Wire.write(20);
  Wire.endTransmission();

  Wire.requestFrom(_address, 2);
  _i2c_wait_timeout(200);

  if (Wire.available() > 0)
  {
    int16_t gyrY = ((int)Wire.read() << 8) | ((int)Wire.read() & 0x00ff);
    _errorCounter = 0;
    return gyrY;
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
      return getGyrY();
    }
  }
}

int16_t CompassSensorObject::getGyrZ()
{
  if ( _destroyed == true ){return 0;}
  Wire.beginTransmission(_address);
  Wire.write(22);
  Wire.endTransmission();

  Wire.requestFrom(_address, 2);
  _i2c_wait_timeout(200);

  if (Wire.available() > 0)
  {
    int16_t gyrZ = ((int)Wire.read() << 8) | ((int)Wire.read() & 0x00ff);
    _errorCounter = 0;
    return gyrZ;
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
      return getGyrZ();
    }
  }
}

int16_t CompassSensorObject::getTemp()
{
  if ( _destroyed == true ){return 0;}
  Wire.beginTransmission(_address);
  Wire.write(24);
  Wire.endTransmission();

  Wire.requestFrom(_address, 2);
  _i2c_wait_timeout(200);

  if (Wire.available() > 0)
  {
    int16_t temp = ((int)Wire.read() << 8) | ((int)Wire.read() & 0x00ff);
    _errorCounter = 0;
    return temp;
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
      return getTemp();
    }
  }
}

#endif
