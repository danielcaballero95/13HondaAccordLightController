#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
/#include <SoftwareSerial.h>
#endif



#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"


//Off
#define OFF 0

//input pins
#define VOLTAGESENSOR 15


#define PARKINGBRAKESENSOR 5
#define HEADLIGHTSENSOR 6
#define DRLSSENSOR 14
#define LEFTTURNSIGNAL 18
#define RIGHTTURNSIGNAL 19

#define RELAYLEFT 16
#define RELAYRIGHT 17

#define LEDSTATUSPIN 13

#define VOLTAGEDIVIDER 62.9


//Neopixel Settings
#define NUMPARKINGNEOPIXELSLEFT 144
#define PARKINGNEOPIXELSLEFTPIN 11

#define NUMPARKINGNEOPIXELSRIGHT 144
#define PARKINGNEOPIXELSRIGHTPIN 12

//function protocols
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

Adafruit_NeoPixel leftParkingNeopixels = Adafruit_NeoPixel(NUMPARKINGNEOPIXELSLEFT, PARKINGNEOPIXELSLEFTPIN, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel rightParkingNeopixels = Adafruit_NeoPixel(NUMPARKINGNEOPIXELSRIGHT, PARKINGNEOPIXELSRIGHTPIN, NEO_GRBW + NEO_KHZ800);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];







bool carIgnitionSensor = false;
bool dRLSSensor = false;
bool headlightsSensor = false;
bool parkingBrakeSensor = false; //use this for for the car movement sensor instead of another one
bool leftTurnSensor = false;
bool rightTurnSensor = false;
bool bluefruitConnection = false;
bool setupMode = false;
bool bluefruitTimeout = false;
bool overrideMode = false;


byte mainMode = 0;
byte carOffMode = 0;
byte carOnMode = 0;

double batteryVoltage=0;


//DRLS color
byte daytimeR = 0;
byte daytimeG = 0;
byte daytimeB = 0;
byte daytimeW = 2;

//Turn Signal color
byte turnSignalR = 255;
byte turnSignalG = 30;
byte turnSignalB = 0;
byte turnSignalW = 0;

//timers
unsigned long currentBLEMillis = 0;
unsigned long previousBLEMillis = 0;
const  long intervalBLE = 60 * 1000; //seconds

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const long interval = 1 * 1000;//seconds

unsigned long currentRelayMillis = 0;
unsigned long previousRelayMillis = 0;
const long intervalRelay = 60 * 1000;//seconds


//loopControl
byte bleLoopControl = 0;

bool relayState=false;//false means off



void setup() {
  // put your setup code here, to run once:

  leftParkingNeopixels.begin();
  rightParkingNeopixels.begin();

  pinMode(VOLTAGESENSOR, INPUT);
  pinMode(DRLSSENSOR, INPUT_PULLUP);
  pinMode(HEADLIGHTSENSOR, INPUT_PULLUP);
  pinMode(PARKINGBRAKESENSOR, INPUT_PULLUP);
  pinMode(LEFTTURNSIGNAL, INPUT_PULLUP);
  pinMode(RIGHTTURNSIGNAL, INPUT_PULLUP);


  pinMode(RELAYLEFT, OUTPUT);
  digitalWrite(RELAYLEFT, HIGH);
  pinMode(RELAYRIGHT, OUTPUT);
  digitalWrite(RELAYRIGHT, HIGH);



  pinMode(LEDSTATUSPIN, OUTPUT);
  Serial.begin(115200);

  // turn off neopixel

  leftParkingNeopixels.begin();
  rightParkingNeopixels.begin(); // This initializes the NeoPixel library.
  for (uint8_t i = 0; i < NUMPARKINGNEOPIXELSLEFT; i++) {
    leftParkingNeopixels.setPixelColor(i, leftParkingNeopixels.Color(0, 0, 0, 0 )); // off
    rightParkingNeopixels.setPixelColor(i, rightParkingNeopixels.Color(0, 0, 0, 0 )); // off
  }
  leftParkingNeopixels.show();
  rightParkingNeopixels.show();

  Serial.println(F("Adafruit Bluefruit Neopixel Color Picker Example"));
  Serial.println(F("------------------------------------------------"));

  //Initialise the module
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    //Perform a factory reset to make sure everything is in a known state
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  //Disable command echo from Bluefruit
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  //  Print Bluefruit information
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!



}




void loop() {

  byte daytimeDivVal = 5;
  
  connectionManager();
  
  batteryVoltage=analogRead(VOLTAGESENSOR);
  batteryVoltage=batteryVoltage/VOLTAGEDIVIDER;

  if (carIgnitionSensor == false)   //car is off
    mainMode = 0;
  if (carIgnitionSensor == true) //car is on
    mainMode = 1;


        determineParkingBakeState();


  turnSignalsState();

  
  switch (mainMode)
  {
    case 0://car is off
      {
        
        dRLSState();
        determineHeadlightsState();
        determineCarOffMode();
        if (((dRLSSensor == true) || (leftTurnSensor==true) || (rightTurnSensor==true) || (bluefruitConnection ==true) && (relayState ==false)))
          {
          digitalWrite(RELAYLEFT, LOW);//low is on
          digitalWrite(RELAYRIGHT, LOW);//low is on
          relayState=true;//turns on tracker
          previousRelayMillis = millis();
          }
          else if ((dRLSSensor == true) || (leftTurnSensor==true) || (rightTurnSensor==true) && (bluefruitConnection ==true)) 
          previousRelayMillis = millis();
          
          
        currentRelayMillis = millis();
        if (((currentRelayMillis - previousRelayMillis) >= intervalRelay) && ((dRLSSensor == false) && (leftTurnSensor==false) && (rightTurnSensor==false) && (bluefruitConnection == false)))
          {
          digitalWrite(RELAYLEFT, HIGH);//high is off
          digitalWrite(RELAYRIGHT, HIGH);//high is off
          relayState=false;
          }

          
        
        switch (carOffMode)
        {
          case 1://bluefruit connection
            {

              bluefruitNeopixelColorPicker();
              break;
            }

          case 6:
            {
              neopixelDisconnectedTimer();//turns off timer
              break;
            }
          case 2://all off
            {
              parkingNeopixels(OFF, OFF, OFF, OFF);
              turnLedIndicatorOn();
              bluefruitTimeout = false;
              break;
            }
          case 3://parking lights turn on
            {
              parkingNeopixels(daytimeR, daytimeG, daytimeB, daytimeW);
              turnLedIndicatorOn();
              bluefruitTimeout = false;
              break;
            }
          case 4://headlights turn on
            {

              parkingNeopixels((daytimeR / daytimeDivVal), (daytimeG / daytimeDivVal), (daytimeB / daytimeDivVal), (daytimeW / daytimeDivVal));
              bluefruitTimeout = false;
              break;
            }
          case 5:
            {
              parkingNeopixels(OFF, OFF, OFF, OFF);
              bluefruitTimeout = false;
              break;
            }

          case 7://// turn signals on
          {
            bool left=false;
            bool right=true;//left is 0 right is 1
            
            if ((leftTurnSensor==true) && (rightTurnSensor==true))
            parkingNeopixels((turnSignalR), (turnSignalG), (turnSignalB), (turnSignalW));
            else if (leftTurnSensor==true)
            signalParkingNeopixels((turnSignalR), (turnSignalG), (turnSignalB), (turnSignalW), (left));
            else if (rightTurnSensor==true)
            signalParkingNeopixels((turnSignalR), (turnSignalG), (turnSignalB), (turnSignalW), (right));

            break;
          }
        }//end Switch for CarOFFMODE

        break;
      }//end Case 1 for mainMode
    case 1://car is on
      {
        if (relayState==false)// tells relay to turn on
        {
          digitalWrite(RELAYLEFT, LOW);//low is on
          digitalWrite(RELAYRIGHT, LOW);//low is on
          relayState=true;
        }
        
        dRLSState();
        determineHeadlightsState();
        determineParkingBakeState();

        
        determineCarOnMode();
        
        switch (carOnMode)
        {
          case 1:
            {
              if (parkingBrakeSensor == true)
                bluefruitNeopixelColorPicker();
              else if ((parkingBrakeSensor == true) && (overrideMode == false))
                bluefruitNeopixelColorPicker();
              else
              { ble.disconnect();
                bluefruitTimeout = false;
                parkingNeopixels(OFF, OFF, OFF, OFF);
              }
              break;
            }
          case 2://all off contingent on the parking brake
            {
              parkingNeopixels(OFF, OFF, OFF, OFF);
              turnLedIndicatorOn();
              
              break;
            }
          case 3://ParkingLights turn ON via the parking Brake
            {
              parkingNeopixels(daytimeR, daytimeG, daytimeB, daytimeW);
              turnLedIndicatorOn();
             
              break;
            }
          case 4://parking lights turn on via the cars command
            {
              parkingNeopixels(daytimeR, daytimeG, daytimeB, daytimeW);
              turnLedIndicatorOn();
             
              break;
            }
          case 5://parking lights off but headlights on
            { 
              parkingNeopixels(OFF, OFF, OFF, OFF);
              
              break;
            }
          case 6://// parkingLihts on and the headlights on
           {
            parkingNeopixels((daytimeR / daytimeDivVal), (daytimeG / daytimeDivVal), (daytimeB / daytimeDivVal), (daytimeW / daytimeDivVal));
            turnLedIndicatorOn();
            
            break;
           }
          case 7://// turn signals on
          {
            bool left=false;
            bool right=true;//left is 0 right is 1
            
            if ((leftTurnSensor==true) && (rightTurnSensor==true))
            parkingNeopixels((turnSignalR), (turnSignalG), (turnSignalB), (turnSignalW));
            else if (leftTurnSensor==true)
            signalParkingNeopixels((turnSignalR), (turnSignalG), (turnSignalB), (turnSignalW), (left));
            else if (rightTurnSensor==true)
            signalParkingNeopixels((turnSignalR), (turnSignalG), (turnSignalB), (turnSignalW),(right));

            break;
          }



        }
        break;//break for mainMode case 2
      }


    default:
      {
        parkingNeopixels(OFF, OFF, OFF, OFF);
        break;
      }

  }//end switch statement
  serialDisplay();
}//end main loop


void determineCarOffMode()
{
  if (bluefruitConnection == true)
    carOffMode = 1;
  else if ((dRLSSensor == false) && (headlightsSensor == false) && (bluefruitTimeout == false))
    carOffMode = 2; //all off
  else if ((dRLSSensor == true) && (headlightsSensor == false))
    carOffMode = 3; //parking lights turn on
  else if ((dRLSSensor == true) && (headlightsSensor == true))
    carOffMode = 4; //parkinglights On with headlights
  else if ((dRLSSensor == false) && (headlightsSensor == true))
    carOffMode = 5;//parking lights off but headlights on
  else if ((bluefruitConnection == false) && (bluefruitTimeout == true))
    carOffMode = 6;

  if ((leftTurnSensor || rightTurnSensor == true))
     carOffMode = 7;
     
  //Serial.println(carOffMode);
}

void determineCarOnMode()
{
  if (bluefruitConnection == true)
    carOnMode = 1;
  else if ((dRLSSensor == false) && (headlightsSensor == false) && (parkingBrakeSensor == false))
    carOnMode = 2;//all off contingent on the parking brake
  else if ((dRLSSensor == false) && (headlightsSensor == false) && (parkingBrakeSensor == true))
    carOnMode = 3; //ParkingLights turn ON via the parking Brake not being engaged
  else if ((dRLSSensor == true) && (headlightsSensor == false) )
    carOnMode = 4; //parking lights turn on via the cars command
  else if ((dRLSSensor == false) && (headlightsSensor == true) )
    carOnMode = 5;// parkingLihts off but headlights on via car command
  else if ((dRLSSensor == true) && (headlightsSensor == true) )
    carOnMode = 6;// parkingLihts on and the headlights on

  if((leftTurnSensor || rightTurnSensor == true))
    carOnMode = 7;

  //Serial.println(carOffMode);




}


/*
void carTurnsOn()
{
  //bool serialStatus = false;
  if (digitalRead(IGNITIONPIN) == LOW)
     carIgnitionSensor = true;
  else
    carIgnitionSensor = false;


  /*if (setDisp != dispUp)
    {
     Serial.print("\nCar Ignition Status ");
     Serial.print(carIgnitionStatus);
    }*/

//end carTurnsOn */

void dRLSState()
{

  if (digitalRead(DRLSSENSOR) == LOW)
    dRLSSensor = true;
  else
    dRLSSensor = false;
  //bool serialStatus = false;
  /*if (parkingLightsSensor != serialStatus)
    {
    Serial.print("\nParking Lights Status ");
    Serial.println(parkingLightsSensor);

    }*/
}

void determineHeadlightsState()
{

  if (digitalRead(HEADLIGHTSENSOR) == LOW)
    headlightsSensor = true;
  else
    headlightsSensor = false;
 // bool serialStatus = false;
  /*if (parkingLightsSensor != serialStatus)
    {
    Serial.print("\nHeadlights State ");
    Serial.println(parkingLightsSensor);
    }*/
}//end determine Headlights State

/*void determineCarMovement()
{

  if (digitalRead(CARMOVEMENTSENSORPIN) == true)
    carMovementSensor = true;
  else
    carMovementSensor = false;

}//end determine Car Movement*/

void determineParkingBakeState()
{

  if (digitalRead(PARKINGBRAKESENSOR) == LOW)
    parkingBrakeSensor = true;
  else
    parkingBrakeSensor = false;
}//end determine Parking Brake State

void turnSignalsState()
{

  if (digitalRead(LEFTTURNSIGNAL) == LOW)
    leftTurnSensor = true;
  else
    leftTurnSensor = false;

  if (digitalRead(RIGHTTURNSIGNAL) == LOW)
    rightTurnSensor = true;
  else
    rightTurnSensor = false;
}

void turnLedIndicatorOn()
{
  if (dRLSSensor == true)
    digitalWrite(LEDSTATUSPIN, HIGH);
  else
    digitalWrite(LEDSTATUSPIN, LOW);
  // Serial.println(parkingLightsSensor);
}//end TurnLedIndicatorON

void parkingNeopixels(byte r, byte g, byte b, byte w)
{
  for (int i = 0; i < NUMPARKINGNEOPIXELSLEFT ; i++)
  {
    leftParkingNeopixels.setPixelColor(i, leftParkingNeopixels.Color(r, g, b, w));
    rightParkingNeopixels.setPixelColor(i, rightParkingNeopixels.Color(r, g, b, w));

  }
  leftParkingNeopixels.show();
  rightParkingNeopixels.show();

}//end daytime Running Lights

void signalParkingNeopixels(byte r, byte g, byte b, byte w, byte leftOrRight)
{
  
 for (int i = 0; i < NUMPARKINGNEOPIXELSLEFT ; i++)
  {
    if (leftOrRight==false)
      leftParkingNeopixels.setPixelColor(i, leftParkingNeopixels.Color(r, g, b, w));
    if (leftOrRight==true)
      rightParkingNeopixels.setPixelColor(i, rightParkingNeopixels.Color(r, g, b, w));

  }
  leftParkingNeopixels.show();
  rightParkingNeopixels.show();





  
}



void serialDisplay()
{


  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.print("\nCar Mode ");
    Serial.print(mainMode);

    Serial.print("\nCar Off Mode ");
    Serial.print(carOffMode);

    Serial.print("\nCar On Mode ");
    Serial.print(carOnMode);

    Serial.print("\nCar Ignition Sensor ");
    Serial.print(carIgnitionSensor);

    Serial.print("\nParking Lights Sensor ");
    Serial.print(dRLSSensor);

    Serial.print("\nHeadlights Sensor ");
    Serial.print(headlightsSensor);

    //Serial.print("\nCar Movement Sensor ");
    //Serial.print(carMovementSensor);

    Serial.print("\nParking Brake Sensor ");
    Serial.print(parkingBrakeSensor);

    Serial.print("\nLeft Turn Sensor ");
    Serial.print(leftTurnSensor);

    Serial.print("\nRight Turn Sensor ");
    Serial.print(rightTurnSensor);

    Serial.print("\nRelay State ");
    Serial.print(relayState);
    
    Serial.print("\nBluefruit Status ");
    Serial.print(bluefruitConnection);

    Serial.print("\nVoltage Sensor ");
    Serial.print(batteryVoltage);


  }

}// end serial display








