WheelObject::WheelObject()
{
  _destroyed = true;
}

void WheelObject::begin( uint8_t in1, uint8_t in2, uint8_t pwm, uint8_t stb )
{
  _in1,_in2,_pwm,_stb = in1, in2, pwm, stb;
  pinMode(_in1, OUTPUT);
  pinMode(_in2, OUTPUT);
  pinMode(_pwm, OUTPUT);
  pinMode(_stb, OUTPUT);
  
  stop();
  standby();

  _destroyed = false;
}

WheelObject::~WheelObject()
{
  end();
}

void WheelObject::end()
{
  standby();
  _destroyed = true;
}

void WheelObject::forward( uint8_t power )
{
  if (_destroyed == true){return;}
  _power = power;
  digitalWrite(_in1, LOW);
  digitalWrite(_in2, HIGH);
  analogWrite(_pwm, _power);
  digitalWrite(_stb, HIGH);
}

void WheelObject::forward()
{
  if (_destroyed == true){return;}
  digitalWrite(_in1, LOW);
  digitalWrite(_in2, HIGH);
  analogWrite(_pwm, _power);
  digitalWrite(_stb, HIGH);
}

void WheelObject::backward( uint8_t power )
{
  if (_destroyed == true){return;}
  _power = power;
  digitalWrite(_in1, HIGH);
  digitalWrite(_in2, LOW);
  analogWrite(_pwm, _power);
  digitalWrite(_stb, HIGH);
}

void WheelObject::backward()
{
  if (_destroyed == true){return;}
  digitalWrite(_in1, HIGH);
  digitalWrite(_in2, LOW);
  analogWrite(_pwm, _power);
  digitalWrite(_stb, HIGH);
}

uint8_t WheelObject::power()
{
  if (_destroyed == true){return 0;}
  return _power;
}

void WheelObject::setPower( uint8_t power )
{
  if (_destroyed == true){return;}
  _power = power;
}

void WheelObject::stop()
{
  if (_destroyed == true){return;}
  _power = 0;
  digitalWrite(_in1, HIGH);
  digitalWrite(_in2, HIGH);
  analogWrite(_pwm, _power);
  digitalWrite(_stb, HIGH);
}

void WheelObject::standby()
{
  if (_destroyed == true){return;}
  digitalWrite(_stb, LOW);
}
