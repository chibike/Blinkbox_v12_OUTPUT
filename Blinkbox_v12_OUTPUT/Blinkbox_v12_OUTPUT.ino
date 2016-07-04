#include <Wire.h>
#include <SPI.h>
#include <Servo.h>

#define DEBUG_TOOLS
//#define SPEED_CONTROL
#define I2C_ADDRESS 8
#define MCU2_ADDRESS 7
#define RESTART_PIN 0
#define RIGHT_CHA_PIN 2
#define RIGHT_CHB_PIN A1
#define LEFT_CHA_PIN  3
#define LEFT_CHB_PIN  7

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
  private:
    bool _destroyed;
    uint8_t _pin;
    bool _state;
    bool _flash;
};

byte STATE = 0;
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
  private:
    bool _destroyed;
    void _update();
    uint8_t _latch;
    byte _flashState;
    bool _flash;
};

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
  private:
    bool _destroyed;
    uint8_t _in1;
    uint8_t _in2;
    uint8_t _pwm;
    uint8_t _stb;
    uint8_t _power;
};

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
  private:
    Servo _myServo;
    bool _destroyed;
    byte _pin;
    uint8_t _angle;
    int8_t _center;
    uint8_t _maxAngle;
    int _minAngle;
};

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
  private:
    ShiftRegisterObject _lightRegister;
    bool _destroyed;
    bool _leftLightState;
    bool _rightLightState;
    uint8_t _leftIndex;
    uint8_t _rightIndex;
};

class LineSensorObject
{
  public:
    LineSensorObject();
    void begin();
    void end();
  private:
    bool _destroyed;
};

#define ENABLE_COMPASS_FULL_DEF
class CompassSensorObject
{
  public:
    float getHeading();
    float getTargetDeviation();
    void setTargetHeading( int heading );
    bool onStall();
    
#ifdef ENABLE_COMPASS_FULL_DEF
    int8_t getPitch();
    int8_t getRoll();
    int16_t getMagX();
    int16_t getMagY();
    int16_t getMagZ();
    int16_t getAccX();
    int16_t getAccY();
    int16_t getAccZ();
    int16_t getGyrX();
    int16_t getGyrY();
    int16_t getGyrZ();
    int16_t getTemp();
#endif
    CompassSensorObject();
    void begin();
    void end();
  private:
    bool _destroyed;
    byte _address;
    int _targetHeading;
    uint8_t _errorCounter;
    void _i2c_wait_timeout( int timeout );   
};

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
  private:
    bool _destroyed;
};

class Blink_OS
{
  public:
    HardwareObjects pheripherals;
    void forwardDistance(uint8_t power, unsigned int distance);
    void arcForwardDistance(uint8_t power, int radius, unsigned int distance);
    void backwardDistance(uint8_t power, unsigned int distance);
    void arcBackwardDistance(uint8_t power, int radius, unsigned int distance);

    void restart();
    void shutdown();
#ifdef DEBUG_TOOLS
    void fd_debug(uint8_t power, uint8_t samples);
#endif
#ifdef SPEED_CONTROL
    void setSpeed( float speed );
    float speed();
    void brake();
    void slowdown();
#endif
    void _RIGHT_WHEEL_ISR();
    void _LEFT_WHEEL_ISR();
    void _RESET_WHEEL_COUNTER();
    void _RECEIVE_EVENT();
    void _REQUEST_EVENT();
    void _SCHEDULER();
    
    Blink_OS();
    void begin();
    void end();
  private:
    boolean _destroyed;
    volatile float _leftWheelDisplacement;
    volatile float _rightWheelDisplacement;
    volatile float _wheelSpeed;
    float _targetSpeed;
};

#ifdef DEBUG_TOOLS

#define DEBUG
class ErrorMonitor
{
  public:
    bool appendError( float error );
    void plotTable();
    
    ErrorMonitor( void );
    void begin(uint8_t graphWidth, float maxPossibleError, float minPossibleError, float centerVal);
    void end();
  private:
    boolean _destroyed;
    uint8_t _graphWidth;
    uint8_t _graphHeight;
    uint8_t _index;
    int _maxPossibleError;
    int _minPossibleError;
    int _centerOn;
    int _errorBuffer[];
};

#endif
