UltrasonicSensor::UltrasonicSensor()
{
  #ifdef PRINT_STATUS
  Serial.println(F("Launching Ultrasonic Sensor...."));
  #endif
  _destroyed = true;
}

void UltrasonicSensor::begin( uint8_t trigPin, uint8_t echoPin )
{
  _trigPin, _echoPin = trigPin, echoPin;
  pinMode(_trigPin, OUTPUT);
  pinMode(_echoPin, OUTPUT);

  #ifdef PRINT_STATUS
  Serial.println(F("Launched"));
  #endif
  
  _destroyed = false;
}

UltrasonicSensor::~UltrasonicSensor()
{
  end();
}

void UltrasonicSensor::end()
{
  _destroyed = true;
}

uint16_t UltrasonicSensor::getDistance()
{
  if (_destroyed == true){return 0;}
  digitalWrite(_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(_trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(_trigPin, LOW);

  float duration = pulseIn( _echoPin, HIGH );
  duration /= 2;

  return (int)_microseconds2mm(duration);
}

float UltrasonicSensor::_microseconds2mm( float duration )
{// sound_speed = 2.94117647 um/mm
  return duration/2.94117647;
}

