// Remote Sensor Project slave device prototype. Receives request from master. Reads data from MMA7455 accelerometer
// and BMP280 barometric pressure sensor and transmits to master.
// Code based on:
// http://gammon.com.au/forum/?id=11428
// https://code.google.com/p/mma-7455-arduino-library/
// https://github.com/sparkfun/SparkFun_BME280_Arduino_Library

#include <SoftwareSerial.h> // Include the Software Serial library to enable serial communication on digital pins
#include <Wire.h> // Include the Wire library to allow communication with I2C / TWI devices
#include "RS485_protocol.h" // Include the RS485 error checking protocol library
#include <MMA_7455.h> // Include the MMA_7455 accelerometer library
#include <stdint.h> // Include header to declare integer types with specified widths
#include "MD_BMP280.h" // Include custom BMP280 barometric pressure sensor library

const byte myAddress = 2; // Declare the address of this slave
const byte ENABLE_PIN = 4; // Declare the digital pin for driver output
const byte LED_PIN = 13; // Declare the diginal pin number for built in LED

SoftwareSerial rs485 (2, 3);  // Assign the receive and transmit pin for the RS485 bus

MMA_7455 accSensor = MMA_7455(); // Make an instance of MMA_7455
BMP280 bmpSensor; // Create the BMP280 object

// RS485 callback routines
void fWrite (const byte what) { // Write mode
  rs485.write (what);
}
int fAvailable () { // Available mode
  return rs485.available ();
}
int fRead () { // Read mode
  return rs485.read ();
}
// end RS485 callback routines

// Function to convert float to byte for fast and precise RS485 transmission
// source: http://mbed.org/forum/helloworld/topic/2053/
typedef union float2bytes_t
{
  float f;
  byte b[sizeof(float)];
};

void setup()
{
  rs485.begin (9600); // Set data rate for RS485 communication
  Serial.begin (9600); // Set data rate for serial communication
  pinMode (ENABLE_PIN, OUTPUT);  // Define driver output
  pinMode (LED_PIN, OUTPUT);  // Define built-in LED

  accSensor.initSensitivity(2); // Set the sensitivity for the accelerometer (2 = 2g, 4 = 4g, 8 = 8g)
  // accSensor.calibrateOffset(5, 20, -68); // Calibrate the Offset, that values corespond in flat position to: xVal = -30, yVal = -20, zVal = +20
  // Remove the first two forward slashes on the previous line to activate this after having the first values read out

  // Define BMP280 run settings
  bmpSensor.settings.I2CAddress = 0x76; // I2C address
  bmpSensor.settings.runMode = 3; // Normal run mode
  bmpSensor.settings.tStandby = 3; // Standby 250ms
  bmpSensor.settings.filter = 4; // FIR Filter 16 coefficients
  bmpSensor.settings.tempOverSample = 2; // Temperature oversample *2
  bmpSensor.settings.pressOverSample = 5; // Pressure oversample * 16

  bmpSensor.begin(); //  Initialize the barometric pressure sensor with above settings

  delay(10);  // Make sure sensor had enough time to turn on. BMP280 requires 2ms to start up.

} // end of setup

void loop()
{
  // Receive request from master
  byte buf [10]; // Declare buffer array to receive into. Spot 0 will always be the address for the receiving device, spot 1 will be the request/command (relevant for future versions).
  byte received = recvMsg (fAvailable, fRead, buf, sizeof buf); // Receive message, if nothing/error return 0, otherwise, returns length of received data

  if (received > 0) // If vaild request is received, check address, compile data and create message to master
  {
    if (buf [0] != myAddress) // Read the address in the message. If this is not this slave's address, ignore.
      return;
    digitalWrite (LED_PIN, HIGH);  // turn on LED while processing

    switch (buf [1]) { // Determine type of message...
      case 1: // If the message requests altitude data, read from the BMP280
        {
          float bmpTemp, bmpPres, bmpAlt; // Create variables for the values from the barometric pressure sensor

          bmpTemp = bmpSensor.readTempC(); // Read the temp
          bmpPres = bmpSensor.readFloatPressure(); // Read the pressure
          bmpAlt = bmpSensor.readFloatAltitudeMeters(); // Read the pressure
          // NEED TO ADD LOOP TO GRAB MULTIPLE PRESSURE READINGS, DISCARD HIGHEST AND LOWEST, AND AVERAGE ??

          // Convert readings to 4-byte arrays
          float2bytes_t f2bTemp;
          f2bTemp.f = bmpTemp;
          float2bytes_t f2bPres;
          f2bPres.f = bmpPres;
          float2bytes_t f2bAlt;
          f2bAlt.f = bmpAlt;

          // Assemble sensor data using 15-byte message format
          byte msg [] = {
            0,  // Address for receiving device (0 = MASTER)
            myAddress, // Address for this device
            buf [1], // Reiteration of message type (for error checking on master)
            // temp reading must be converted to 4 bytes
            f2bTemp.b[0],
            f2bTemp.b[1],
            f2bTemp.b[2],
            f2bTemp.b[3],
            // pressure reading must be converted to 4 bytes
            f2bPres.b[0],
            f2bPres.b[1],
            f2bPres.b[2],
            f2bPres.b[3],
            // altitude reading must be converted to 4 bytes
            f2bAlt.b[0],
            f2bAlt.b[1],
            f2bAlt.b[2],
            f2bAlt.b[3]
          };

          delay (1);  // give the master a moment to prepare to receive

          // Send sensor data to master
          digitalWrite (ENABLE_PIN, HIGH);  // enable sending
          sendMsg (fWrite, msg, sizeof msg); // send a message of "length" bytes (max 255) to other end
          digitalWrite (ENABLE_PIN, LOW);  // disable sending

          // Write data to the serial console
          Serial.print ("Temp: ");
          Serial.print (bmpTemp, 2);
          Serial.print (" degrees C");
          Serial.print ("\t");
          Serial.print ("Pressure: ");
          Serial.print (bmpPres, 3);
          Serial.print (" Pa");
          Serial.print ("\t");
          Serial.print ("Altitude: ");
          Serial.print (bmpAlt, 3);
          Serial.println(" m");

          break;
        }
      case 2: // If the message requests accelerometer data, read from the MMA7455
        {
          byte xVal, yVal, zVal; // Create variables for the values from the accelerometer
          xVal = accSensor.readAxis('x'); // Read the 'x' Axis
          yVal = accSensor.readAxis('y'); // Read the 'y' Axis
          zVal = accSensor.readAxis('z'); // Read the 'z' Axis

          // Assemble sensor data using 6-byte message format
          byte msg [] = {
            0,  // Address for receiving device (0 = MASTER)
            myAddress, // Address for this device
            buf [1], // Reiteration of message type (for error checking on master)
            xVal,  // Value for X
            yVal,  // Value for Y
            zVal  // Value for Z
          };

          delay (1);  // give the master a moment to prepare to receive

          // Send sensor data to master
          digitalWrite (ENABLE_PIN, HIGH);  // enable sending
          sendMsg (fWrite, msg, sizeof msg); // send a message of "length" bytes (max 255) to other end
          digitalWrite (ENABLE_PIN, LOW);  // disable sending

          // Write data to the serial console
          Serial.print ("X Value: ");
          Serial.print (xVal, DEC);
          Serial.print ("\t");
          Serial.print ("Y Value: ");
          Serial.print (yVal, DEC);
          Serial.print ("\t");
          Serial.print ("Z Value: ");
          Serial.println (zVal, DEC);

          break;
        }
      case 3: // FUTURE PHASE: If the message commands turn on laser...
        {
          // Code to turn on laser goes here
          break;
        }
      case 4: // FUTURE PHASE: If the message commands turn off laser...
        {
          // Code to turn off laser goes here
          break;
        }
      case 5: // FUTURE PHASE: If the message commands turn off buzzer...
        {
          // Code to turn off buzzer goes here
          break;
        }
      default: // If message request does not match accepted types, return an error
        {
          // Code for error goes here
          break;
        }
    } // end switch case to determine type of message

    digitalWrite (LED_PIN, LOW);  // Done processing. Turn off LED

  }  // end if something received

}  // end of loop
