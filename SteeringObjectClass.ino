SteeringObject::SteeringObject()
{
  _destroyed = true;
}

void SteeringObject::begin( uint8_t pin )
{
  _myServo.attach(pin);
  _center = 103;
  _angle = 0;
  _maxAngle = 150;
  _minAngle = 55;
  setheading(_center);

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
  _angle = constrain(angle + _center, _minAngle, _maxAngle);
  _myServo.write(_angle);
}

void SteeringObject::set2center()
{
  if (_destroyed == true){return;}
  _angle = 0;
  setheading(_angle);
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
