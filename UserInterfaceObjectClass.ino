UserInterfaceObject::UserInterfaceObject()
{
  begin(9600);
}

void UserInterfaceObject::begin( uint16_t baudrate )
{
  Serial.begin(baudrate);
  #ifdef PRINT_STATUS
  Serial.println(F("Launching User Interface...."));
  #endif
  
  #ifdef PRINT_STATUS
  Serial.println(F("Launched"));
  #endif
  _destroyed = false;
}

UserInterfaceObject::~UserInterfaceObject()
{
  end();
}

void UserInterfaceObject::end()
{
  Serial.end();
  _destroyed = true;
}

