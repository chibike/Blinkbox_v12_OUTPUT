SteeringObject::SteeringObject()
{
  _destroyed = true;
}

void SteeringObject::begin( uint8_t pin )
{
  _myServo.attach(pin);
  _center = 103;
  _angle = 0;
  _maxAngle = _center + 45;
  _minAngle = _center - 45;
  set2center();

  _destroyed = false;
}

void SteeringObject::end()
{
  _destroyed = true;
  _myServo.detach();
}

void SteeringObject::setheading( int8_t angle )
{
  if (_destroyed == true){return;}
  angle = constrain(angle, -45, 45);
  _angle = constrain((int)(-1*angle) + _center, _minAngle, _maxAngle);
  _myServo.write(_angle);
}

void SteeringObject::set2center()
{
  if (_destroyed == true){return;}
  setheading(0);
}

int8_t SteeringObject::heading()
{
  if (_destroyed == true){return 0;}
  return _angle;
}

int8_t SteeringObject::center()
{
  if (_destroyed == true){return 0;}
  return _center;
}
