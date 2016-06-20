Blink_OS::Blink_OS()
{
  _destroyed = true;
}

void Blink_OS::begin()
{
  noInterrupts();
  //Begin Pheripherals
  pheripherals.begin();
  pheripherals.Wheels.begin(12, 10, 6, 4);//in1,in2,pwm,stb
  pheripherals.Lights.begin(8, 7, 8);//latch,leftIndex,rightIndex
  pheripherals.Steering.begin(5);//pin
  pheripherals.ShiftRegister.begin(8);//latch
  pheripherals.Horn.begin(9);//pin
  pheripherals.CompassSensor.begin();

  //Begin Timer
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  OCR2A = 254; //980.45Hz
  TCCR2A |= (1<<WGM21);
  TCCR2B |= (1<<CS22);
  TIMSK2 |= (1<<OCIE2A);

  //Begin Wheel ISR
  pinMode(RIGHT_CHA_PIN, INPUT_PULLUP);
  pinMode(RIGHT_CHB_PIN, INPUT_PULLUP);
  pinMode(LEFT_CHA_PIN, INPUT_PULLUP);
  pinMode(LEFT_CHB_PIN, INPUT_PULLUP);
  attachInterrupt(0, RIGHT_WHEEL_ISR, CHANGE);
  attachInterrupt(1, LEFT_WHEEL_ISR, CHANGE);
  _targetSpeed = 0;

  //Begin Communication
  Wire.begin(I2C_ADDRESS);
  Wire.setTimeout(1000);
  Wire.onReceive(RECEIVE_EVENT);
  Wire.onRequest(REQUEST_EVENT);

  #ifdef DEBUG_MODE
  Serial.begin(9600);
  #endif
  
  _destroyed = false;
  interrupts();
}

void Blink_OS::end()
{
  _destroyed = true;
  pheripherals.end();
  detachInterrupt(0);
  detachInterrupt(1);
  noInterrupts();
  shutdown();
}

void Blink_OS::forwardDistance(uint8_t power, unsigned int distance)
{
  if ( _destroyed == true ){return;}
  pheripherals.Wheels.stop();
  
  /* -- There are some many reasons why I made this decision -- */
  /* 1. This robot drives in cm and before long the distance    */
  /*    counting buffer will overflow.                          */
  /* 2. We don't have to store the end distance and calculate   */
  /*     because it should be equal to the distance.            */
  /* 3. Reseting the counters will reduce accumulated errors    */
  /*                                                            */
  /* --              Quirks About this decision              -- */
  /* 1. This will trigger a false speed reading that will       */
  /*    affect any speed control algorithm used unless          */
  /*    accounted for. But this can be ignored given that the   */
  /*    speed should be corrected after the following two ticks */
  /*    of the encoders.                                        */
  _RESET_WHEEL_COUNTER();
  #define AVERAGE_DISTANCE (float)(_leftWheelDisplacement+_rightWheelDisplacement)/2

  pheripherals.Steering.set2center();
  delay(200);
  pheripherals.CompassSensor.setTargetHeading( pheripherals.CompassSensor.getHeading() );
  delay(10);
  pheripherals.Wheels.forward(power);
  
  const float Kp = 1.2;
  const uint16_t updateTime = 250;
  long lastUpdateTime = 0;
  while(AVERAGE_DISTANCE < distance)
  {
    if( millis() - lastUpdateTime > updateTime )
    {
      lastUpdateTime = millis();
      float error = pheripherals.CompassSensor.getTargetDeviation();
      float value = Kp * error;
      value = constrain(value, -50, 50);
      Serial.println(AVERAGE_DISTANCE);
      pheripherals.Steering.setheading( value );
    }
  }
  
  pheripherals.Wheels.stop();
}

#ifdef DEBUG_TOOLS

void Blink_OS::fd_debug(uint8_t power, uint8_t samples)
{
  Serial.println("Begining Debug");
  if ( _destroyed == true ){return;}
  pheripherals.Wheels.stop();
  _RESET_WHEEL_COUNTER();
  #define AVERAGE_DISTANCE (float)(_leftWheelDisplacement+_rightWheelDisplacement)/2

  pheripherals.Steering.set2center();
  delay(200);
  pheripherals.CompassSensor.setTargetHeading( pheripherals.CompassSensor.getHeading() );
  delay(10);
  
  const float Kp = 2;
  const uint16_t updateTime = 250;
  long lastUpdateTime = 0;
  ErrorMonitor bar;
  bar.begin(samples, 200, -200, 0);

  pheripherals.Wheels.forward(power);
  while(1)
  {
    if( millis() - lastUpdateTime > updateTime )
    {
      lastUpdateTime = millis();
      float error = pheripherals.CompassSensor.getTargetDeviation();
      Serial.println(error);
      
      float value = Kp * error;
      value = constrain(value, -50, 50);
      Serial.print("value = ");
      Serial.println(value);
      pheripherals.Steering.setheading( value );

      if( bar.appendError( error ) == false )
      {
        Serial.println("Samples Ended");
        break;
      }
    }
  }
  
  pheripherals.Wheels.stop();
  bar.plotTable();
}

#endif

void Blink_OS::_RESET_WHEEL_COUNTER()
{
  if ( _destroyed == true ){return;}
  _rightWheelDisplacement = 0;
  _leftWheelDisplacement = 0;
}

void Blink_OS::_RIGHT_WHEEL_ISR()
{
  if ( _destroyed == true ){return;}
  if(digitalRead(RIGHT_CHA_PIN) == digitalRead(RIGHT_CHB_PIN))
  {//forward rotation
    _rightWheelDisplacement += 0.29;
  }
  else
  {
    _rightWheelDisplacement -= 0.29;
  }
}

void Blink_OS::_LEFT_WHEEL_ISR()
{
  if ( _destroyed == true ){return;}
  if(digitalRead(LEFT_CHA_PIN) == digitalRead(LEFT_CHB_PIN))
  {//forward rotation
    _leftWheelDisplacement -= 0.29;
  }
  else
  {
    _leftWheelDisplacement += 0.29;
  }
}

void Blink_OS::_RECEIVE_EVENT()
{
  #define STOP_CMD    0x00
  #define FWD         0x01
  #define FWD_DIST    0x02
  #define BWD         0x03
  #define BWD_DIST    0x04
  #define STB         0x05
  #define ST_HD       0x06
  #define ST_PWR      0x08
  #define LT_IND      0x09
  #define RT_IND      0x10
  #define FL_AL       0x11
  #define HDL         0x12
  #define FL_HDL      0x13
  #define FL_HRN      0x14
  #define PAUSE       0x15
  #define PLAY        0x16
  #define FOLLOW_LINE 0x17
  #define TRN_FWD     0x20
  #define TRN_BWD     0x21
  #define AVD_OBS     0x23
  #define NONE_CMD    0x50
  if ( _destroyed == true ){return;}
  int buffer = Wire.read();
  char burntData;

  switch(buffer)
  {
  case STOP_CMD:
      pheripherals.Wheels.stop();
      break;
    case FWD:
      pheripherals.Wheels.forward();
      break;
    case FWD_DIST:
      //forwardDistance( (uint16_t)( ((((uint8_t)Wire.read()) << 8) & 0xff00) | (uint8_t)Wire.read() ) );
      break;
    case BWD:
      pheripherals.Wheels.backward();
      break;
    case BWD_DIST:
      //backwardDistance( (uint16_t)( ((((uint8_t)Wire.read()) << 8) & 0xff00) | (uint8_t)Wire.read() ) );
      break;
    case STB:
      pheripherals.Wheels.standby();
      break;
    case ST_HD:
      pheripherals.Steering.setheading( ((int8_t)Wire.read()) - 50 );
      break;
    case ST_PWR:
      pheripherals.Wheels.setPower( (uint8_t)Wire.read() );
      break;
    case FL_HRN:
      pheripherals.Horn.flash();
      break;
    case FL_HDL:
      //pheripherals.Lights.
      break;
    case HDL:
      //pheripherals.Lights.
      break;
    case TRN_FWD:
      //arcForwardDistance( (int)( ((((uint8_t)Wire.read()) << 8) & 0xff00) | (uint8_t)Wire.read() ),
      //                    (uint16_t)( ((((uint8_t)Wire.read()) << 8) & 0xff00) | (uint8_t)Wire.read()) );
      break;
    case TRN_BWD:
      //arcBackwardDistance( (int)( ((((uint8_t)Wire.read()) << 8) & 0xff00) | (uint8_t)Wire.read() ),
      //                     (uint16_t)( ((((uint8_t)Wire.read()) << 8) & 0xff00) | (uint8_t)Wire.read()) );
      break;
    case NONE_CMD:
      break;
    case PAUSE:
      //onPause = HIGH;
      //currentFunctionIndex = PAUSE;
      //variable_1 = 0;
      //variable_2 = 0;
      break;
    case PLAY:
      //onPause = LOW;
      //currentFunctionIndex = PAUSE;
      //variable_1 = 0;
      //variable_2 = 0;
      break;
    case AVD_OBS:
      //avoidObstacle = HIGH;
      break;
    default:
      pheripherals.Wheels.stop();
  }
}

void Blink_OS::_SCHEDULER()
{
  static int _counter = 0;
  static float lastLeftDisplacment = _leftWheelDisplacement;
  static float lastRightDisplacment = _rightWheelDisplacement;
  
  if(_counter++ >= 980)
  {
    _counter = 0;
    _wheelSpeed = ((float)abs(_leftWheelDisplacement - lastLeftDisplacment) + abs(_rightWheelDisplacement - lastRightDisplacment))/2;
    lastLeftDisplacment = _leftWheelDisplacement;
    lastRightDisplacment = _rightWheelDisplacement;

    #ifdef SPEED_CONTROL
    float Error = _targetSpeed - _wheelSpeed;
    int power = 127 + 10*(Error);
    pheripherals.Wheels.setPower(abs(power));
    
    #endif

    //#define STALL_CONTROL
    #ifdef STALL_CONTROL
    if( pheripherals.CompassSensor.onStall() == true )
    {
      pheripherals.Wheels.setPower( pheripherals.Wheels.power()/2 );
    }
    #endif

    pheripherals.Lights.flashUpdate();
    pheripherals.ShiftRegister.flashUpdate();
    pheripherals.Horn.flashUpdate();
  }
}

#ifdef SPEED_CONTROL
void Blink_OS::setSpeed( float speed )
{
  _targetSpeed = speed;
}
float Blink_OS::speed()
{
  return _targetSpeed;
}
void Blink_OS::brake()
{
  _targetSpeed = 0;
  pheripherals.Wheels.stop();
}

void Blink_OS::slowdown()
{
  _targetSpeed = _targetSpeed/2;
  pheripherals.Wheels.setPower( pheripherals.Wheels.power()/2 );
}
#endif
void Blink_OS::restart()
{
  digitalWrite(RESTART_PIN, LOW);
}

void Blink_OS::shutdown()
{
  if( !_destroyed )
  {
    end();
  }
  else
  {
     while(1);
  }
}
