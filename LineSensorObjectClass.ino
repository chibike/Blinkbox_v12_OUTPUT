LineSensorObject::LineSensorObject()
{
  _destroyed = true;
}

void LineSensorObject::begin()
{
  _destroyed = false;
}

LineSensorObject::~LineSensorObject()
{
  end();
}

void LineSensorObject::end()
{
  _destroyed = true;
}
