#include <SPI.h>
#include <Servo.h>

class HornObject
{
  public:
    void on();
    void off();
    void flash();
    bool state();
    void begin( uint8_t pin );
    void end();
    HornObject();
    ~HornObject();
  private:
    bool _destroyed;
    uint8_t _pin;
    bool _state;
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
  _destroyed = true;
}

void HornObject::end()
{
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
    void begin( uint8_t latch );
    void end();
    ShiftRegisterObject();
    ~ShiftRegisterObject();
  private:
    bool _destroyed;
    void _update();
    uint8_t _latch;
    byte _state;
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
  _destroyed = true;
  SPI.end();
}

void ShiftRegisterObject::end()
{
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

class WheelObject
{
  public:
    void forward( uint8_t power );
    void backward( uint8_t power );
    void stop();
    void standby();
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
  _destroyed = true;
}

void WheelObject::end()
{
  _destroyed = true;
}

void WheelObject::forward( uint8_t power )
{
  if (_destroyed == true){return;}
  _power = power;
  digitalWrite(_in1, LOW);
  digitalWrite(_in2, HIGH);
  analogWrite(_pwm, power);
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
    int8_t heading();
    void begin(uint8_t pin);
    void end();
    SteeringObject();
    ~SteeringObject();
  private:
    Servo myServo;
    bool _destroyed;
    byte _pin;
    int8_t _angle;
};

SteeringObject::SteeringObject()
{
  _destroyed = true;
}

void SteeringObject::begin( uint8_t pin )
{
  myServo.attach(pin);
  setheading(90);

  _destroyed = false;
}

SteeringObject::~SteeringObject()
{
  _destroyed = true;
  myServo.detach();
}

void SteeringObject::end()
{
  _destroyed = true;
  myServo.detach();
}

void SteeringObject::setheading( int8_t angle )
{
  if (_destroyed == true){return;}
  _angle = angle;
  myServo.write(_angle);
}

int8_t SteeringObject::heading()
{
  if (_destroyed == true){return 0;}
  return _angle;
}

class LightObject
{
  public:
    void flashLights();
    void flashLeftLights();
    void flashRightLights();
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
  _destroyed = true;
  _lightRegister.end();
}

void LightObject::end()
{
  _destroyed = true;
  _lightRegister.end();
}

void LightObject::flashLights()
{
  if (_destroyed == true){return;}
  //pass
}

void LightObject::flashLeftLights()
{
  if (_destroyed == true){return;}
  //pass
}

void LightObject::flashRightLights()
{
  if (_destroyed == true){return;}
  //pass
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
  _destroyed = true;
}

void LineSensorObject::end()
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
  _destroyed = true;
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
}

void setup()
{
  //pass
}

void loop()
{
  //pass
}
