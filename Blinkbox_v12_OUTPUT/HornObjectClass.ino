HornObject::HornObject()
{
  _destroyed = true;
}

void HornObject::begin( uint8_t pin )
{
  _pin = pin;
  pinMode(_pin, OUTPUT);
  off();
  _destroyed = false;
}

void HornObject::end()
{
  undoFlash();
  _state = 0x00;
  _destroyed = true;
}

void HornObject::on()
{
  if (_destroyed == true){return;}
  _state = HIGH;
  digitalWrite(_pin, _state);
}

void HornObject::off()
{
  if (_destroyed == true){return;}
  _state = LOW;
  digitalWrite(_pin, _state);
}

void HornObject::flash()
{
  if (_destroyed == true){return;}
  _flash = true;
}

void HornObject::undoFlash()
{
  if (_destroyed == true){return;}
  _flash = false;
}

void HornObject::flashUpdate()
{
  if(_flash != true | _destroyed == true) return;
  switch(_state)
  {
    case false:
      on();
      break;
    default:
      off();
  }
}

bool HornObject::state()
{
  if (_destroyed == true){return false;}
  return _state;
}
