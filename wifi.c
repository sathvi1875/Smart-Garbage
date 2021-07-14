//
//  wifi.c
//  
//
//  Created by Sathvik Mula on 7/14/21.
//

#include <stdio.h>
char val = 101;    //    HEX/Char/DEC    0x65 / 'e' / 101 . (ASCII table) String header = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n";

String html_1 = R"=====(
<!DOCTYPE html>
<html>
<head>
<meta name='viewport' content='width=device-width, initial-scale=1.0'/>
<meta charset='utf-8'>
<style>
body {font-size:140%;}
#main {display: table; margin: auto; padding: 0 10px 0 10px; } h2 {text-align:center; }
.button { padding:10px 10px 10px 10px; width:100%; background-color: #50FF50; font-size: 120%;}
</style>

<title>Embedded Course</title>
</head>
<body>
<div id='main'>
<h2>Embedded Course</h2>
)=====";

String html_2 = "";

String html_4 = R"=====(
</div>
</body>
</html>
)=====";
 
#include <ESP8266WiFi.h>

char ssid[] = "MySpectrumWiFi98-2G";        // your network SSID (name) char pass[] = "watchglobal436";    // your network password

WiFiServer server(80);    //Port 80 is one of the most commonly used port numbers in the Transmission Control Protocol (TCP) suite. Any Web/HTTP client, such as a Web browser, uses port 80 to send and receive requested Web pages from a HTTP server.

String request = "";    LEDOFF
int LED_Pin = 16;    // Builtin nodemcu LED uint8_t GPIO1 = 9;

void setup()
{
pinMode(LED_Pin, OUTPUT); pinMode(GPIO1, OUTPUT);
Serial.begin(9600);    // baudrate of PIC and nodemcu should be same Serial.println();
delay(5000);
Serial.println("Serial started at 9600"); Serial.println("Nodemcu"); Serial.println();

// Connecting to a WiFi network Serial.print(F("Connecting to ")); Serial.println(ssid); WiFi.begin(ssid, pass);

while (WiFi.status() != WL_CONNECTED)
{
Serial.print("."); delay(500);
}

Serial.println(""); Serial.println(F("[CONNECTED]")); Serial.print("[IP "); Serial.print(WiFi.localIP()); Serial.println("]");

// start a server server.begin();
Serial.println("Server started");
 
}

void loop()
{
 If (Serial.available()>0)
{
  Char ch = Serial.read();
// Get the LED pin status and create the LED status message if (ch== “1”)
{
html_2 = "Garbage FULL";
}
If ( ch ==”0”)
{
html_2 = " EMPTY ";
}
}
// Check if a client has connected WiFiClient client = server.available(); if (!client) { return; }

// Read the first line of the request request = client.readStringUntil('\r');

Serial.print("request: "); Serial.println(request);

if    ( request.indexOf("LEDON") > 0 ) { digitalWrite(LED_Pin, HIGH);

Serial.println("on");
}
else if ( request.indexOf("LEDOFF") > 0 ) { digitalWrite(LED_Pin, LOW); Serial.println(val);
}



client.flush();

client.print( header ); client.print( html_1 ); client.print( html_2 ); client.print( html_4); delay(5);

}
