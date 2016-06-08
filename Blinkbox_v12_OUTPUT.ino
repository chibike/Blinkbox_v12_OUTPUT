#include <Wire.h>
#include <SPI.h>
#include <Servo.h>

void shutdown();
void RIGHT_WHEEL_ISR();
void LEFT_WHEEL_ISR();
void RECEIVE_EVENT(int howmany);
void REQUEST_EVENT();

class HornObject
{
  public:
    void on();
    void off();
    void flash();
    void undoFlash();
    bool state();
    void flashUpdate();
    
    void begin( uint8_t pin );
    void end();
    HornObject();
    ~HornObject();
  private:
    bool _destroyed;
    uint8_t _pin;
    bool _state;
    bool _flash;
};

HornObject::HornObject()
{
  _destroyed = true;
}

void HornObject::begin( uint8_t pin )
{
  _pin = pin;
  pinMode(_pin, OUTPUT);
  off();
  _destroyed = false;
}

HornObject::~HornObject()
{
  end();
}

void HornObject::end()
{
  undoFlash();
  _state = 0x00;
  _destroyed = true;
}

void HornObject::on()
{
  if (_destroyed == true){return;}
  _state = HIGH;
  digitalWrite(_pin, _state);
}

void HornObject::off()
{
  if (_destroyed == true){return;}
  _state = LOW;
  digitalWrite(_pin, _state);
}

void HornObject::flash()
{
  if (_destroyed == true){return;}
  _flash = true;
}

void HornObject::undoFlash()
{
  if (_destroyed == true){return;}
  _flash = false;
}

void HornObject::flashUpdate()
{
  if(_flash != true | _destroyed == true) return;
  switch(_state)
  {
    case false:
      on();
      break;
    default:
      off();
  }
}

bool HornObject::state()
{
  if (_destroyed == true){return false;}
  return _state;
}

class ShiftRegisterObject
{
  public:
    void setHigh( uint8_t index );
    void setLow( uint8_t index );
    void setState( byte state );
    byte state();
    bool getState( uint8_t index );
    void flash( byte state );
    void flashIndex( uint8_t index );
    void undoFlash( byte state );
    void undoFlashIndex( uint8_t index );
    void flashUpdate();
    
    void begin( uint8_t latch );
    void end();
    ShiftRegisterObject();
    ~ShiftRegisterObject();
  private:
    bool _destroyed;
    void _update();
    uint8_t _latch;
    byte _state;
    byte _flashState;
    bool _flash;
};

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

ShiftRegisterObject::~ShiftRegisterObject()
{
  end();
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

class WheelObject
{
  public:
    void forward( uint8_t power );
    void backward( uint8_t power );
    void forward();
    void backward();
    void stop();
    void standby();
    void setPower( uint8_t power );
    uint8_t power();
    
    void begin( uint8_t in1, uint8_t in2, uint8_t pwm, uint8_t stb );
    void end();
    WheelObject();
    ~WheelObject();
  private:
    bool _destroyed;
    uint8_t _in1;
    uint8_t _in2;
    uint8_t _pwm;
    uint8_t _stb;
    uint8_t _power;
};

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

class SteeringObject
{
  public:
    void setheading( int8_t angle );
    void set2center();
    int8_t heading();
    int8_t center();
    
    void begin(uint8_t pin);
    void end();
    SteeringObject();
    ~SteeringObject();
  private:
    Servo _myServo;
    bool _destroyed;
    byte _pin;
    int8_t _angle;
    int8_t _center;
    int8_t _maxAngle;
    int8_t _minAngle;
};

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

SteeringObject::~SteeringObject()
{
  end();
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

class LightObject
{
  public:
    void flashLights();
    void undoFlashLights();
    void flashLeftLights();
    void undoFlashLeftLights();
    void flashRightLights();
    void undoFlashRightLights();
    void flashUpdate();
    void turnOnLights();
    void turnOffLights();
    void turnOnLeftLights();
    void turnOffLeftLights();
    void turnOnRightLights();
    void turnOffRightLights();
    bool getLeftLightState();
    bool getRightLightState();
    
    void begin(uint8_t latch, uint8_t leftIndex, uint8_t rightIndex );
    void end();
    LightObject();
    ~LightObject();
  private:
    ShiftRegisterObject _lightRegister;
    bool _destroyed;
    bool _leftLightState;
    bool _rightLightState;
    uint8_t _leftIndex;
    uint8_t _rightIndex;
};

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

LightObject::~LightObject()
{
  end();
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

class LineSensorObject
{
  public:
    LineSensorObject();
    ~LineSensorObject();
    void begin();
    void end();
  private:
    bool _destroyed;
};

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

class CompassSensorObject
{
  public:
    float getHeading();
    float getTargetDeviation();
    void setTargetHeading( int heading );
    
    CompassSensorObject();
    void begin();
    ~CompassSensorObject();
    void end();
  private:
    bool _destroyed;
    byte _address;
    int _targetHeading;
    void _i2c_wait_timeout( int timeout );   
};

void CompassSensorObject::_i2c_wait_timeout( int timeout)
{
  long startTime = millis();
  while(Wire.available() <= 0 && (millis()-startTime) < timeout)
  {
    //wait
  }
}

float CompassSensorObject::getHeading()
{
  if ( _destroyed == true ){return 0;}
  Wire.beginTransmission(_address);
  Wire.write(2);
  Wire.endTransmission();

  Wire.requestFrom(_address, 2);
  _i2c_wait_timeout(100);

  if (Wire.available() > 0)
  {
    float angle = ( (int)Wire.read() << 8 ) | ( (int)Wire.read() & 0x0f );
    return 360 - angle/10.0;
  }
  else
  {
    return 0;
  }
}

float CompassSensorObject::getTargetDeviation()
{
  if ( _destroyed == true ){return 0;}
  float trueHeading = getHeading();
  switch( round(trueHeading) )
  {
    case 0:
    case 360:
      trueHeading = 0;
      break;
    default:
      if (trueHeading > 180)
      {
        trueHeading = -1*(360 - trueHeading);
      }
  }

  //Centralize Heading about target Angle
  return trueHeading - _targetHeading;  
}

void CompassSensorObject::setTargetHeading(int heading)
{
  if ( _destroyed == true ){return;}
  switch(heading)
  {
    case 0:
    case 360:
    case -360:
      heading = 0;
      break;
    default:
      if (heading > 180)
      {
        heading = -1*(360 - heading);
      }
  }
  _targetHeading = heading;
}

CompassSensorObject::CompassSensorObject()
{
  _destroyed = true;
}

void CompassSensorObject::begin()
{
  _address = 0x60;
  _targetHeading = 0;
  
  _destroyed = false;
}

CompassSensorObject::~CompassSensorObject()
{
  end();
}

void CompassSensorObject::end()
{
  _destroyed = true;
}

class HardwareObjects
{
  public:
    WheelObject Wheels;
    LightObject Lights;
    SteeringObject Steering;
    ShiftRegisterObject ShiftRegister;
    LineSensorObject LineSensor;
    CompassSensorObject CompassSensor;
    HornObject Horn;
    
    void begin();
    void end();
    HardwareObjects();
    ~HardwareObjects();
  private:
    bool _destroyed;
};

HardwareObjects::HardwareObjects()
{
  _destroyed = true;
}

void HardwareObjects::begin()
{
  _destroyed = false;
}

HardwareObjects::~HardwareObjects()
{
  end();
}

void HardwareObjects::end()
{
  _destroyed = true;
  
  Wheels.end();
  Lights.end();
  Steering.end();
  ShiftRegister.end();
  LineSensor.end();
  Horn.end();
  CompassSensor.end();
}

class Blink_OS
{
  #define I2C_ADDRESS 0x00
  #define MCU2_ADDRESS 0x00
  #define COMP_ADDRESS 0x00
  public:
    HardwareObjects pheripherals;
    void forwardDistance(uint16_t distance){}
    void arcForwardDistance(int radius, uint16_t distance){}
    void backwardDistance(uint16_t distance){}
    void arcBackwardDistance(int radius, uint16_t distance){}

    void _RIGHT_WHEEL_ISR();
    void _LEFT_WHEEL_ISR();
    void _RECEIVE_EVENT();
    void _REQUEST_EVENT(){}
    void _SCHEDULER();
    
    Blink_OS();
    void begin();
    ~Blink_OS();
    void end();
  private:
    #define RIGHT_CHA_PIN 2
    #define RIGHT_CHB_PIN A1
    #define LEFT_CHA_PIN  3
    #define LEFT_CHB_PIN  7
    boolean _destroyed;
    volatile float _leftWheelDisplacement;
    volatile float _rightWheelDisplacement;
    volatile float _wheelSpeed;
};

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

Blink_OS::~Blink_OS()
{
  end();
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

Blink_OS Blink_OS_v12;
void setup()
{
  Blink_OS_v12.begin();
}

void loop()
{
}

void shutdown()
{
  while(1)
  {
  }
}

void RIGHT_WHEEL_ISR()
{
  Blink_OS_v12._RIGHT_WHEEL_ISR();
}

void LEFT_WHEEL_ISR()
{
  Blink_OS_v12._LEFT_WHEEL_ISR();
}

void RECEIVE_EVENT(int howmany)
{
  Blink_OS_v12._RECEIVE_EVENT();
}

void REQUEST_EVENT()
{
  Blink_OS_v12._REQUEST_EVENT();
}

ISR(TIMER2_COMPA_vect)
{
  Blink_OS_v12._SCHEDULER();
}

