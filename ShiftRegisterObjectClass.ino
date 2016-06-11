ShiftRegisterObject::ShiftRegisterObject()
{
  _destroyed = true;
}

void ShiftRegisterObject::begin( uint8_t latch )
{
  _latch = latch;
  pinMode(_latch, OUTPUT);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.begin();

  _destroyed = false;
}

void ShiftRegisterObject::end()
{
  _state = 0x00;
  _flashState = 0x00;
  undoFlash(0xff);
  _destroyed = true;
  SPI.end();
}

void ShiftRegisterObject::_update()
{
  if (_destroyed == true){return;}
  SPI.transfer(_state);
  digitalWrite(_latch, HIGH);digitalWrite( _latch, LOW );
}

void ShiftRegisterObject::setHigh( uint8_t index )
{
  if (_destroyed == true){return;}
  bitWrite(_state, index, HIGH);
  _update();
}

void ShiftRegisterObject::setLow( uint8_t index )
{
  if (_destroyed == true){return;}
  bitWrite(_state, index, LOW);
  _update();
}

byte ShiftRegisterObject::state()
{
  if (_destroyed == true){return 0x00;}
  return _state;
}

bool ShiftRegisterObject::getState( uint8_t index )
{
  if (_destroyed == true){return false;}
  return bitRead(_state, index);
}

void ShiftRegisterObject::setState( byte state )
{
  if (_destroyed == true){return;}
  _state = state;
  _update();
}

void ShiftRegisterObject::flashIndex(uint8_t index)
{
  if (_destroyed == true){return;}
  _flash = true;
  _flashState = _flashState | (0x01 << index);
}

void ShiftRegisterObject::undoFlashIndex( uint8_t index )
{
  if (_destroyed == true){return;}
  _flashState = _flashState & ~(0x00 << index);
}

void ShiftRegisterObject::flash( byte state )
{
  if (_destroyed == true){return;}
  _flash = true;
  _flashState = _flashState | state;
}

void ShiftRegisterObject::undoFlash( byte state )
{
  if (_destroyed == true){return;}
  _flashState = _flashState & ~state;
}

void ShiftRegisterObject::flashUpdate()
{
  if(_flash != true | _destroyed == true) return;
  static bool flashCount = false;
  switch(flashCount)
  {
    case true:
      _state = _state & (~_flashState);
      break;
    default:
      _state = _state | _flashState;
      break;
  }
  flashCount = !flashCount;
  setState( _state );
}

