IntraSystemCallsObject::IntraSystemCallsObject()
{
  #ifdef PRINT_STATUS
  Serial.println(F("Launching Intra Sys Calls...."));
  #endif
  _destroyed = true;
}

void IntraSystemCallsObject::begin()
{
  _destroyed = false;

  #ifdef PRINT_STATUS
  Serial.println(F("Launched"));
  #endif
}

IntraSystemCallsObject::~IntraSystemCallsObject()
{
  end();
}

void IntraSystemCallsObject::end()
{
  _destroyed = true;
}

