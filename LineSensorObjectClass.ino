LineSensorObject::LineSensorObject()
{
  _destroyed = true;
}

void LineSensorObject::begin()
{
  _destroyed = false;
}

void LineSensorObject::end()
{
  _destroyed = true;
}
