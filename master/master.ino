// Remote Sensor Project master device prototype. Sends requests to slaves for baromatric pressure readins and accelerometer readings.
// Receives data back, compares pressure and temp readings to its own BMP280 readings, calculates difference in altitude, and displays
// differential height and XYZ coordinates on serial console.
// Code based on:
// http://gammon.com.au/forum/?id=11428
// https://code.google.com/p/mma-7455-arduino-library/
// https://github.com/sparkfun/SparkFun_BME280_Arduino_Library

#include <SoftwareSerial.h> // Include the Software Serial library to enable serial communication on digital pins
#include <Wire.h> // Include the Wire library to allow communication with I2C / TWI devices
#include "RS485_protocol.h" // Include the RS485 error checking protocol library
#include <stdint.h> // Include header to declare integer types with specified widths
#include "MD_BMP280.h" // Include custom BMP280 barometric pressure sensor library

const byte myAddress = 0; // Declare the address of the master
const byte numSlaves = 4; // Declare the max number of slave devices
const byte ENABLE_PIN = 4; // Declare the digital pin for driver output
const byte LED_PIN = 13; // Declare the diginal pin number for built in LED

SoftwareSerial rs485 (2, 3);  // Assign the receive and transmit pin for the RS485 bus

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

// Function to combine bytes in message back to float (helps to facilitate fast and accurate RS485 transmission)
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

} // end of setup

void loop()
{
  // HEIGHT: Create loop to step through each slave, request barometer data, compare to master barometer data, and display on the serial console
  for (byte i = 1; i <= numSlaves; i++)
  { // The variable i represents the slave address, and increments every time it moves through the loop until it reaches 4, then the loop repeats, starting from 0

    // Assemble request for sensor data using 2-byte message format
    byte msgType = 1; // Declare message type. Accepted types are: 1 = request altitude data, 2 = request accelerometer data (future versions to include 3 = turn on laser, 4 = turn off laser, 5 = turn off buzzer)
    byte msg [] = {
      i,    // Add address for receiving device
      msgType    // Add message type
    };

    // Send message to slaves
    digitalWrite (ENABLE_PIN, HIGH);  // enable sending
    sendMsg (fWrite, msg, sizeof msg); // send a message of "length" bytes (max 255) to other end
    digitalWrite (ENABLE_PIN, LOW);  // disable sending

    // Receive response from slave
    byte buf [16]; // Declare buffer array to receive into. Spot 0 will always be the address for the receiving device, spot 1 will be the address for the transmitting
    // device, spot 2 will be the message type (for error checking), spots 3, 4, 5, & 6 will be the temperature float value, split into bytes for transmission,
    // spots 7, 8, 9, & 10 will be the pressure float value, split into bytes for transmission, spots 11, 12, 13, & 14 will be the pressure float value, split into
    // bytes for transmission.
    byte received = recvMsg (fAvailable, fRead, buf, sizeof buf); // Receive message, if nothing/error return 0, otherwise, returns length of received data

    if (received > 0) // If response is received, compile data and write to output
    {
      if (buf [0] != myAddress) // Read the address in the message. If this is not the master address, ignore.
        return;
      if (buf [1] != i) // Read the sender's address. If this is not the expected address, ignore.
        return;
      if (buf [2] != msgType) // Read the message type of response. If this is not the barometer message type (1), display error.
      {
        // Write error message to the serial console
        Serial.print ("ERROR: WRONG DATA TYPE FROM DEVICE ");
        Serial.println (i);

      } else { // If the message type is barometer, proceed with gathering and displaying data

        digitalWrite (LED_PIN, HIGH);  // turn on LED while processing

        float bmpSlvTemp, bmpSlvPres, bmpSlvAlt; // Create variables for the values from the slave's barometric pressure sensor
        // Convert 4 single bytes in message array back to float
        float2bytes_t b2fTemp;
        float2bytes_t b2fPres;
        float2bytes_t b2fAlt;

        // Retrieve data from buffer array for floating point temp
        b2fTemp.b[0] = buf [3];
        b2fTemp.b[1] = buf [4];
        b2fTemp.b[2] = buf [5];
        b2fTemp.b[3] = buf [6];
        bmpSlvTemp = b2fTemp.f;
        // Retrieve data from buffer array for floating point pressure
        b2fPres.b[0] = buf [7];
        b2fPres.b[1] = buf [8];
        b2fPres.b[2] = buf [9];
        b2fPres.b[3] = buf [10];
        bmpSlvPres = b2fPres.f;
        // Retrieve data from buffer array for floating point altitude
        b2fAlt.b[0] = buf [11];
        b2fAlt.b[1] = buf [12];
        b2fAlt.b[2] = buf [13];
        b2fAlt.b[3] = buf [14];
        bmpSlvAlt = b2fAlt.f;

        // Write data to the serial console
        Serial.print ("Slave ");
        Serial.print (i, DEC);
        Serial.print (":");
        Serial.print ("\t");
        Serial.print ("Temp: ");
        Serial.print (bmpSlvTemp, 2);
        Serial.print (" degrees C");
        Serial.print ("\t");
        Serial.print ("Pressure: ");
        Serial.print (bmpSlvPres, 3);
        Serial.print (" Pa");
        Serial.print ("\t");
        Serial.print ("Altitude: ");
        Serial.print (bmpSlvAlt, 3);
        Serial.println(" m");

        digitalWrite (LED_PIN, LOW);  // Done processing. Turn off LED
      }
    }
    else // If no response received from slave, display error
    {
      // Write error message to the serial console
      Serial.print ("ERROR: NO HEIGHT DATA RECEIVED FROM DEVICE ");
      Serial.println (i);
    }
    delay (1000);
  } // end of barometer for loop

  // 3D POSITIONING: Create loop to step through each slave, request accelerometer data, and display on the serial console and LCD
  for (byte i = 1; i <= numSlaves; i++)
  { // The variable i represents the slave address, and increments every time it moves through the loop until it reaches 4, then the loop repeats, starting from 0

    // Assemble request for sensor data using 2-byte message format
    byte msgType = 2; // Declare message type. Accepted types are: 1 = request altitude data, 2 = request accelerometer data (future versions to include 3 = turn on laser, 4 = turn off laser, 5 = turn off buzzer)
    byte msg [] = {
      i,    // Add address for receiving device
      msgType    // Add message type
    };

    // Send message to slaves
    digitalWrite (ENABLE_PIN, HIGH);  // enable sending
    sendMsg (fWrite, msg, sizeof msg); // send a message of "length" bytes (max 255) to other end
    digitalWrite (ENABLE_PIN, LOW);  // disable sending

    // Receive response from slave
    byte buf [10]; // Declare buffer array to receive into. Spot 0 will always be the address for the receiving device, spot 1 will be the address for the transmitting
    // device, spot 2 will be the message type (for error checking), spots 3, 4, & 5 will be the X, Y, & Z values.
    byte received = recvMsg (fAvailable, fRead, buf, sizeof buf); // Receive message, if nothing/error return 0, otherwise, returns length of received data

    if (received > 0) // If response is received, compile data and write to output
    {
      if (buf [0] != myAddress) // Read the address in the message. If this is not the master address, ignore.
        return;
      if (buf [1] != i) // Read the sender's address. If this is not the expected address, ignore.
        return;
      if (buf [2] != msgType) // Read the message type of response. If this is not the accelerometer message type (2), display error.
      {
        // Write error message to the serial console
        Serial.print ("ERROR: WRONG DATA TYPE FROM DEVICE ");
        Serial.println (i);

      } else { // If the message type is accelerometer, proceed with gathering and displaying data

        digitalWrite (LED_PIN, HIGH);  // turn on LED while processing

        byte xVal, yVal, zVal; // Variables for the values from the slave
        // Retrieve data from buffer array for each value
        xVal = buf [3];
        yVal = buf [4];
        zVal = buf [5];

        // Write data to the serial console
        Serial.print ("Slave ");
        Serial.print (i, DEC);
        Serial.print (":");
        Serial.print ("\t");
        Serial.print ("X Value: ");
        Serial.print (xVal, DEC);
        Serial.print ("\t");
        Serial.print ("Y Value: ");
        Serial.print (yVal, DEC);
        Serial.print ("\t");
        Serial.print ("Z Value: ");
        Serial.println (zVal, DEC);

        digitalWrite (LED_PIN, LOW);  // Done processing. Turn off LED
      }
    }
    else // If no response received from slave, display error
    {
      // Write error message to the serial console
      Serial.print ("ERROR: NO POSITION DATA RECEIVED FROM DEVICE ");
      Serial.println (i);
    }
    delay (1000);
  } // end of accelerometer for loop
}  // end of loop
