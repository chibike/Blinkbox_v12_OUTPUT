Blink_OS::Blink_OS()
{
  _destroyed = true;
}

void Blink_OS::begin()
{
  //Begin Pheripherals
  pheripherals.begin();
  pheripherals.Wheels.begin(12, 10, 6, 4);//in1,in2,pwm,stb
  pheripherals.Lights.begin(8, 7, 8);//latch,leftIndex,rightIndex
  pheripherals.Steering.begin(5);//pin
  pheripherals.ShiftRegister.begin(8);//latch
  pheripherals.Horn.begin(9);//pin
  pheripherals.CompassSensor.begin();

  //Begin Timer
  cli();
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  OCR2A = 254; //980.45Hz
  TCCR2A |= (1<<WGM21);
  TCCR2B |= (1<<CS22);
  TIMSK2 |= (1<<OCIE2A);
  sei();

  //Begin Wheel ISR
  pinMode(RIGHT_CHA_PIN, INPUT_PULLUP);
  pinMode(RIGHT_CHB_PIN, INPUT_PULLUP);
  pinMode(LEFT_CHA_PIN, INPUT_PULLUP);
  pinMode(LEFT_CHB_PIN, INPUT_PULLUP);
  attachInterrupt(0, RIGHT_WHEEL_ISR, CHANGE);
  attachInterrupt(1, LEFT_WHEEL_ISR, CHANGE);

  //Begin Communication
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(RECEIVE_EVENT);
  Wire.onRequest(REQUEST_EVENT);

  #ifdef DEBUG_MODE
  Serial.begin(9600);
  #endif
  
  _destroyed = false;
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
    _leftWheelDisplacement += 0.29;
  }
  else
  {
    _leftWheelDisplacement -= 0.29;
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
  #define ST_THD      0x07
  #define ST_PWR      0x08
  #define LT_IND      0x09
  #define RT_IND      0x10
  #define FL_AL       0x11
  #define FL_HDL      0x12
  #define FL_HRN      0x13
  #define PAUSE       0x14
  #define PLAY        0x15
  #define FOLLOW_LINE 0x16
  #define DWN_MAP     0x17
  #define END_MAP     0x18
  #define HDL         0x19
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
      forwardDistance( (uint16_t)( ((((uint8_t)Wire.read()) << 8) & 0xff00) | (uint8_t)Wire.read() ) );
      break;
    case BWD:
      pheripherals.Wheels.backward();
      break;
    case BWD_DIST:
      backwardDistance( (uint16_t)( ((((uint8_t)Wire.read()) << 8) & 0xff00) | (uint8_t)Wire.read() ) );
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
      arcForwardDistance( (int)( ((((uint8_t)Wire.read()) << 8) & 0xff00) | (uint8_t)Wire.read() ),
                          (uint16_t)( ((((uint8_t)Wire.read()) << 8) & 0xff00) | (uint8_t)Wire.read()) );
      break;
    case TRN_BWD:
      arcBackwardDistance( (int)( ((((uint8_t)Wire.read()) << 8) & 0xff00) | (uint8_t)Wire.read() ),
                           (uint16_t)( ((((uint8_t)Wire.read()) << 8) & 0xff00) | (uint8_t)Wire.read()) );
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
    //pass
    #endif

    pheripherals.Lights.flashUpdate();
    pheripherals.ShiftRegister.flashUpdate();
    pheripherals.Horn.flashUpdate();
  }
}
