/*********************************************************************
CONTROL APP for team B4 in EGEN 310R, Fall 2018
Programmed by Payton Harrison, code based on DC Motor sketch example 
from Arduino and BLE LED Light code from gerrikoio (referenced in Final Design Doc)

This sketch connects to an Adafruit Bluefruit 32u4 board and sends different
motor commands based on the control pad buttons pushed in the app interface.

Coded from September 2018-November 2018

*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

/*=========================================================================
    APPLICATION SETTINGS

  FACTORYRESET_ENABLE     Perform a factory reset when running this sketch
 
                            Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                            running this at least once is a good idea.
 
                            When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0. If you are making changes to your
                            Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why. Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
   
                            Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Pin Configuration and Firmware Declarations

//variables used for transferring information from app to board...names kept same as LED example

#define LED_PIN       13

const unsigned long
  BLINKTIME =         1000;
  
unsigned long 
  t_blink =           0L;

int
  blinkState =        LOW;

//motor initialization
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

//speed 
int mySpeed = 75;


Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
//  Serial.println(err);
  while (1);
}


void setup(void)
{
  pinMode(LED_PIN, OUTPUT);
 
  delay(500);

  //set up Bluefruit and connect to board
  
  if ( !ble.begin() )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }


  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
  //  Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  /* Print Bluefruit information */
  ble.info();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

    AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed of motor 1 to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(0);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);

    // Set the speed of motor 2 to start, from 0 (off) to 255 (max speed)
  myMotor2->setSpeed(0);
  myMotor2->run(FORWARD);
  // turn on motor
  myMotor2->run(RELEASE);

    // Set the speed of motor 3 to start, from 0 (off) to 255 (max speed)
  myMotor3->setSpeed(0);
  myMotor3->run(FORWARD);
  // turn on motor
  myMotor3->run(RELEASE);

    // Set the speed of motor 4 to start, from 0 (off) to 255 (max speed)
  myMotor4->setSpeed(0);
  myMotor4->run(FORWARD);
  // turn on motor
  myMotor4->run(RELEASE);

}

void loop(void)
{
  // Now Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  ble.readline();
  ble.waitForOK();

  String BLEbuffer = ble.buffer;

  if (BLEbuffer.length() && BLEbuffer.indexOf("OK") == -1) 
 //   Serial.print(F("[Recv] ")); Serial.println(BLEbuffer);
  
  
  if (BLEbuffer.indexOf("Status") >= 0) {
  //  Serial.println(F("Status Request Received"));
    ble.print("AT+BLEUARTTX=");
    if (t_blink) {
      ble.println("BLNK");
    }
    else {
      if (blinkState)
        ble.println("ON");
      else
        ble.println("OFF");
    }

    // check response stastus
    if (! ble.waitForOK() ) {
  //    Serial.println(F("Failed to get response"));
    }

    ble.println("AT+BLEUARTRX");
  }

  //First 3 blocks here are for the speed control buttons on the app
  
  //BASE speed
  else if (BLEbuffer.indexOf("BASE") >= 0) {
    if (!t_blink) t_blink = millis();
    ble.print("AT+BLEUARTTX=");
    ble.println("BLNK");
    ble.println("AT+BLEUARTRX");

    //setting base speed here:
    mySpeed = 75;
  }

    //SLOWER
  else if (BLEbuffer.indexOf("SLOW") >= 0) {
    if (!t_blink) t_blink = millis();
    ble.print("AT+BLEUARTTX=");
    ble.println("BLNK");
    ble.println("AT+BLEUARTRX");

    //increment speed downward by 20
    if(mySpeed > 15){
       mySpeed = (mySpeed - 20);
    }
    else{
      mySpeed = mySpeed;
    }

  }

    //FASTER
  else if (BLEbuffer.indexOf("FAST") >= 0) {
    if (!t_blink) t_blink = millis();
    ble.print("AT+BLEUARTTX=");
    ble.println("BLNK");
    ble.println("AT+BLEUARTRX");

   //increment speed upward by 20
    if(mySpeed < 250){
      mySpeed = (mySpeed + 20);
    }
    else{
      mySpeed = mySpeed;
    }
    
  }

  //These next blocks are for the control pad. Forward, Reverse, Left, Right, 
  //and release (stops motors if no buttons are being pressed down
  
  //Forward
  else if (BLEbuffer.indexOf("FORWARD") >= 0) {
    if (!t_blink) t_blink = millis();
    ble.print("AT+BLEUARTTX=");
    ble.println("BLNK");
    ble.println("AT+BLEUARTRX");

    //set motors to so that car moves forward
    myMotor->run(FORWARD);
    myMotor2->run(FORWARD);
    myMotor3->run(BACKWARD);
    myMotor4->run(BACKWARD);

    //set speed to whatever speed is passed though speed buttons
    myMotor->setSpeed(mySpeed);
    myMotor2->setSpeed(mySpeed);
    myMotor3->setSpeed(mySpeed);
    myMotor4->setSpeed(mySpeed);
  }
  
  //Left
  else if (BLEbuffer.indexOf("LEFT") >= 0){
    blinkState = HIGH;
    digitalWrite(LED_PIN, blinkState);
    t_blink = 0;
    ble.print("AT+BLEUARTTX=");
      ble.println("BLNK");
    ble.println("AT+BLEUARTRX");
    
    //power motors on right side of car on forward (1 and 3)
    //and power motors on left side of car backward (2 and 4)
    myMotor->run(FORWARD);
    myMotor4->run(BACKWARD);

    myMotor2->run(BACKWARD);
    myMotor3->run(FORWARD);

    //set speed to whatever speed is passed though speed buttons
    myMotor->setSpeed(mySpeed);
    myMotor2->setSpeed(mySpeed);
    myMotor3->setSpeed(mySpeed);
    myMotor4->setSpeed(mySpeed);
  }

  //Right
  else if (BLEbuffer.indexOf("RIGHT") >= 0) {
    blinkState = LOW;
    digitalWrite(LED_PIN, blinkState);
    t_blink = 0;
    ble.print("AT+BLEUARTTX=");
      ble.println("BLNK");
    ble.println("AT+BLEUARTRX");
      
     //power motors on left side of car on forward (2 and 4)
     //and power motors on right side of car backward (1 and 3)
     myMotor2->run(FORWARD);
     myMotor3->run(BACKWARD);
  
     myMotor->run(BACKWARD);
     myMotor4->run(FORWARD);

    //set speed to whatever speed is passed though speed buttons
    myMotor->setSpeed(mySpeed);
    myMotor2->setSpeed(mySpeed);
    myMotor3->setSpeed(mySpeed);
    myMotor4->setSpeed(mySpeed);
  }

  
  //Reverse
  else if (BLEbuffer.indexOf("REVERSE") >= 0) {
    if (!t_blink) t_blink = millis();
    ble.print("AT+BLEUARTTX=");
    ble.println("BLNK");
    ble.println("AT+BLEUARTRX");

    //set motors so that car moves backward
    myMotor->run(BACKWARD);
    myMotor2->run(BACKWARD);
    myMotor3->run(FORWARD);
    myMotor4->run(FORWARD);

    //set speed to whatever speed is passed though speed buttons
    myMotor->setSpeed(mySpeed);
    myMotor2->setSpeed(mySpeed);
    myMotor3->setSpeed(mySpeed);
    myMotor4->setSpeed(mySpeed);

  }

    //Stop moving
  else if (BLEbuffer.indexOf("RELEASE") >= 0) {
    if (!t_blink) t_blink = millis();
    ble.print("AT+BLEUARTTX=");
    ble.println("BLNK");
    ble.println("AT+BLEUARTRX");

    //set all motors to STOP running
    myMotor->run(RELEASE);
    myMotor2->run(RELEASE);
    myMotor3->run(RELEASE);
    myMotor4->run(RELEASE);

    //set all speeds to 0...the "release" command should do this,
    //but this is just a backup method
    myMotor->setSpeed(0);
    myMotor2->setSpeed(0);
    myMotor3->setSpeed(0);
    myMotor4->setSpeed(0);
  
  }

  //this section deals with the timing of sending states back and forth from app
  //to this sketch
  BLEbuffer = "";

  if (t_blink) {
    if (t_blink > millis()) t_blink = millis();
    if ((millis() - t_blink) > BLINKTIME) {
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
      t_blink = millis();
    }
  }
}
