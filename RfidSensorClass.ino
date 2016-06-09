RfidSensorObject::RfidSensorObject()
{
  #ifdef PRINT_STATUS
  Serial.println(F("Launching RFID Sensor"));
  #endif
  
  _destroyed = true;
}

void RfidSensorObject::begin( uint8_t cardDetectPin )
{
  _address = 0xA0;
  _cardDetectPin = cardDetectPin;
  pinMode(_cardDetectPin, INPUT_PULLUP);

  #ifdef PRINT_STATUS
  Serial.println(F("Launched"));
  #endif
  
  _destroyed = false;
}

RfidSensorObject::~RfidSensorObject()
{

  end();
}

void RfidSensorObject::end()
{
  _destroyed = true;
}

void RfidSensorObject::_i2c_wait_timeout( int timeout )
{
  long startTime = millis();
  while(Wire.available() <= 0 && (millis()-startTime) < timeout)
  {
    //wait
  }
}

bool RfidSensorObject::compareTag( char* buffer, char* tag )
{
  if( !getTagId(buffer) == true );
  {
    return false;
  }
  return strcmp(buffer, tag);
}

bool RfidSensorObject::getTagId( char* buffer )
{
  if(_destroyed = true){return false;}

  while ( !_loadTag(buffer) )
  {
    switch(_errorCode)
    {
      case NO_CARD_DETECTED:
        //Serial.println("No Card Detected");
        return false;
        break;
      case WIRE_NOT_AVAILABLE:
        //Serial.println("Wire Not Available");
        break;
      case COLLISION_DETECTED:
        //Serial.println("Collision Detected");
        break;
      case NO_TAG:
        //Serial.println("No Tag");
        return false;
        break;
      default:
        //Serial.println("Unexpected Result");
        return false;
    }
  }
}

bool RfidSensorObject::cardAvailable()
{
  return !digitalRead(_cardDetectPin);
}

bool RfidSensorObject::_loadTag( char* buffer )
{
  if ( !cardAvailable() )
  {
    _errorCode = NO_CARD_DETECTED;
    return false;
  }
  
  Wire.beginTransmission(_address);
  Wire.write(1);
  Wire.write(1);
  Wire.endTransmission();
  
  int index = 0;
  delay(5);
  Wire.requestFrom(_address, 11);
  _i2c_wait_timeout(5);
  if( Wire.available() )
  {
    byte len = Wire.read();
    while( Wire.available() < len )
    {//wait for all data to arrive
      if( !cardAvailable() )
      {
        _errorCode = NO_CARD_DETECTED;
        return false;
      }
    }

    byte data = Wire.read();//read command
    if(data != 1)
    {
      _errorCode = UNEXPECTED_RESULT;
      return false;
    }

    data = Wire.read();//read status
    switch(data)
    {
      case 0://Operation successfull
        len = len - 2;
        while(--len)
        {
          data = Wire.read();//read tag character by character
          if(data < 0x10)
          {
            buffer[index++] = '0';
          }
          buffer[index++] = (data >> 4) > 0x09 ? 
                             char( (data >> 4) + 55 ):char( (data >> 4) + 48 );
          buffer[index++] = (data & 0x0f) > 0x09 ? 
                             char( (data & 0x0f) + 55):char( (data & 0x0f) + 48 );
        }
        return true;
        break;
      case 0x0A:
        _errorCode = COLLISION_DETECTED;
        return false;
        break;
      case 1:
        _errorCode = NO_TAG;
        return false;
        break;
      default:
        _errorCode = UNEXPECTED_RESULT;
        return false;
        break;
    }
  }
  _errorCode = WIRE_NOT_AVAILABLE;
  return false;
}

