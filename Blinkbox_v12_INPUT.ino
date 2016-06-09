#include <SPI.h>
#include <SD.h>
#include <Wire.h>

#define PRINT_STATUS

class UltrasonicSensor
{
  public:
    uint16_t getDistance();
    
    UltrasonicSensor();
    void begin( uint8_t trigPin, uint8_t echoPin );
    ~UltrasonicSensor();
    void end();
  private:
    bool _destroyed;
    uint8_t _trigPin;
    uint8_t _echoPin;
    float _microseconds2mm( float duration );
};

class DistanceSensorObject
{
  public:
    UltrasonicSensor frontSensor;
    
    DistanceSensorObject();
    void begin();
    ~DistanceSensorObject();
    void end();
  private:
    bool _destroyed;
};

class RfidSensorObject
{
  #define NO_CARD_DETECTED   0x00
  #define WIRE_NOT_AVAILABLE 0x01
  #define COLLISION_DETECTED 0x02
  #define NO_TAG             0x03
  #define UNEXPECTED_RESULT  0x04
  
  public:
    bool getTagId( char* buffer );
    bool compareTag( char* buffer, char* tag );
    bool cardAvailable();
    
    RfidSensorObject();
    void begin( uint8_t cardDetectPin );
    ~RfidSensorObject();
    void end();
  private:
    bool _destroyed;
    uint8_t _cardDetectPin;
    byte _address;
    byte _errorCode;
    void _i2c_wait_timeout( int timeout );
    bool _loadTag(char* buffer);
};


#ifdef ALLOW_ACCESS_2_COMPASS

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

#endif

class SensorObjects
{
  public:
    DistanceSensorObject DistanceSensor;
    RfidSensorObject RfidSensor;

    SensorObjects();
    void begin();
    ~SensorObjects();
    void end();
#ifdef ALLOW_ACCESS_2_COMPASS
    CompassSensorObject CompassSensor;
#endif
  private:
    bool _destroyed;
};

class IntraSystemCallsObject
{
  public:
  
    IntraSystemCallsObject();
    void begin();
    ~IntraSystemCallsObject();
    void end();
  private:
    bool _destroyed;
};


class FileExplorerObject
{
  public:
    SDClass drive;
    File openRoot()
    {
      return drive.open("/");
    }

    void getDir(char* name, char* buffer)
    {
      File dir = openRoot();
      while(1)
      {
        File entry = dir.openNextFile();
        strcat(buffer, "/");
        if(!entry)
        {
          strcat(buffer, entry.name());
          return;
        }
        else if( strcmp(entry.name(), name) )
        {
          strcat(buffer, entry.name());
          return;
        }
        else if( entry.isDirectory() )
        {
          dir = search(entry, name);
          if( strcmp( dir.name(), name ) )
          {
            strcat(buffer, dir.name());
            return;
          }
        }
      }
    }

    File search(File dir, char* name)
    {
      while(1)
      {
        File entry = dir.openNextFile();
        if(!entry)
        {
          return entry;
        }
        else if( strcmp(entry.name(), name) )
        {
          return entry;
        }
        else if( entry.isDirectory() )
        {
          dir = search(entry, name);
          if( strcmp( dir.name(), name ) )
          {
            return dir;
          }
        }
      }
    }
    FileExplorerObject();
    void begin( uint8_t chipSelectPin );
    ~FileExplorerObject();
    void end();
  private:
    bool _destroyed;
    SdVolume volume;
    SdFile root;
};

class UserInterfaceObject
{
  public:
    char* input( char* prompt, long timeout )
    {
      Serial.print(prompt);
      long startTime = millis();
      while(!Serial.available() && millis()-startTime < timeout){}//wait for response
      char buffer[Serial.available()];
      
      for (uint8_t i=0; i<Serial.available(); i++)
      {
        buffer[i] = Serial.read();
      }
      return buffer;
    }
    
    float inputFloat( char* prompt, long timeout )
    {
      Serial.print(prompt);
      long startTime = millis();
      while(!Serial.available() && millis()-startTime < timeout)//wait for response
      return Serial.parseFloat();
    }
    
    int inputInt( char* prompt, long timeout )
    {
      Serial.print(prompt);
      long startTime = millis();
      while(!Serial.available() && millis()-startTime < timeout)//wait for response
      return Serial.parseInt();
    }
    UserInterfaceObject();
    void begin( uint16_t baudrate );
    ~UserInterfaceObject();
    void end();
  private:
    bool _destroyed;
};

class Blink_OS
{
  public:
    UserInterfaceObject Ui;
    IntraSystemCallsObject IntraSystemCalls;
    FileExplorerObject FileExplorer;
    SensorObjects Sensors;
    
    Blink_OS();
    void begin();
    ~Blink_OS();
    void end();
  private:
    boolean _destroyed;
};

void REQUEST_EVENT()
{
  //pass
}

void RECEIVE_EVENT()
{
  //pass
}

//ISR(TIMER2_COMPA_vect)
//{
//  //pass
//}

void shutdown()
{
  while(1)
  {
    //pass
  }
}




