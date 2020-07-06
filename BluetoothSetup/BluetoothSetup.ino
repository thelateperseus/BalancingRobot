// Basic Bluetooth sketch HC-06_01
// Connect the Hc-06 module and communicate using the serial monitor
//
// The HC-06 defaults to AT mode when first powered on.
// The default baud rate is 9600
// The Hc-06 requires all AT commands to be in uppercase. NL+CR MUST be added to the command string
//
// AT+NAME:BarbieSegway
// AT+PSWD="0852"
// AT+UART:115200,0,0
// AT+UART=57600,0,0 for remote programming (apparently)
// HC-06 address is 98D3:11:F86879
// HC-05 address is 0019:10:094345

#include <SoftwareSerial.h>
SoftwareSerial BTserial(2, 3); // RX | TX
// Connect the HC-06 TX to the Arduino RX on pin 2.
// Connect the HC-06 RX to the Arduino TX on pin 3 through a voltage divider.
//


void setup()
{
  Serial.begin(9600);
  Serial.println("Enter AT commands:");

  // HC-06 default serial speed is 9600
  BTserial.begin(38400);
}

void loop()
{

  // Keep reading from HC-06 and send to Arduino Serial Monitor
  if (BTserial.available()) {
    Serial.write(BTserial.read());
    //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // Keep reading from Arduino Serial Monitor and send to HC-06
  if (Serial.available()) {
    BTserial.write(Serial.read());
  }

}
