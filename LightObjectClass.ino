LightObject::LightObject()
{
  _destroyed = true;
}

void LightObject::begin( uint8_t latch, uint8_t leftIndex, uint8_t rightIndex )
{
  _lightRegister.begin(latch);
  _leftIndex = leftIndex;
  _rightIndex = rightIndex;
  turnOffLights();
  
  _destroyed = false;
}

void LightObject::end()
{
  undoFlashLights();
  _destroyed = true;
  _lightRegister.end();
}

void LightObject::flashLights()
{
  if (_destroyed == true){return;}
  _lightRegister.flashIndex(_leftIndex);
  _lightRegister.flashIndex(_rightIndex);
}

void LightObject::undoFlashLights()
{
  _lightRegister.undoFlash(_leftIndex);
  _lightRegister.undoFlash(_rightIndex);
}

void LightObject::flashLeftLights()
{
  if (_destroyed == true){return;}
  _lightRegister.flashIndex(_leftIndex);
}

void LightObject::undoFlashLeftLights()
{
  if (_destroyed == true){return;}
  _lightRegister.undoFlash(_leftIndex);
}

void LightObject::flashRightLights()
{
  if (_destroyed == true){return;}
  _lightRegister.flashIndex(_rightIndex);
}

void LightObject::undoFlashRightLights()
{
  if (_destroyed == true){return;}
  _lightRegister.undoFlash(_rightIndex);
}

void LightObject::flashUpdate()
{
  _lightRegister.flashUpdate();
}

void LightObject::turnOnLights()
{
  if (_destroyed == true){return;}
  _lightRegister.setHigh(_leftIndex);
  _lightRegister.setHigh(_rightIndex);
  _leftLightState = HIGH;
  _rightLightState = HIGH;
}

void LightObject::turnOffLights()
{
  if (_destroyed == true){return;}
  _lightRegister.setLow(_leftIndex);
  _lightRegister.setLow(_rightIndex);
  _leftLightState = LOW;
  _rightLightState = LOW;
}

void LightObject::turnOnLeftLights()
{
  if (_destroyed == true){return;}
  _lightRegister.setHigh(_leftIndex);
  _leftLightState = HIGH;
}

void LightObject::turnOffLeftLights()
{
  if (_destroyed == true){return;}
  _lightRegister.setLow(_leftIndex);
  _leftLightState = LOW;
}

void LightObject::turnOnRightLights()
{
  if (_destroyed == true){return;}
  _lightRegister.setHigh(_rightIndex);
  _rightLightState = HIGH;
}

void LightObject::turnOffRightLights()
{
  if (_destroyed == true){return;}
  _lightRegister.setLow(_rightIndex);
  _rightLightState = LOW;
}

bool LightObject::getLeftLightState()
{
  if (_destroyed == true){return false;}
  return _leftLightState;
}

bool LightObject::getRightLightState()
{
  if (_destroyed == true){return false;}
  return _rightLightState;
}
