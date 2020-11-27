// Wire Slave Sender
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Sends data as an I2C/TWI slave device
// Refer to the "Wire Master Reader" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event when master request to send 
  Wire.onReceive(receiveEvent); // register event when arduino receives
  Serial.begin(115200);         // start serial for output
  Serial.println("I2C Slave Sender + Receiver");
}

void loop() {
  delay(100);
}

// Copied from "Wire Slave Sender"
// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  Wire.write("hello "); // respond with message of 6 bytes
  Serial.println("send to master");
  // as expected by master
}

// Copied from "Wire Slave Reader"
// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  while (Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
}
