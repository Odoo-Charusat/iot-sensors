#include "secrets.h"

#define RXp2 16
#define TXp2 17


//connect to wifi - wificlient, wifi


//msg handler


//connect to aws 
////need certificates, endpoint, callback msg handler func

void connectToAWS() {
    Serial
}


//mpu data from arduino - read n process


//send that mpu data to aws



//two serials, 9600 pe arduino, 115200 pe aws/monitor
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2);

  Serial.println("SENDING DATA TO AWS IOT CORE !!")

  //CONNECT TO WIFI FIRST

  connectToWifi();

  //CONNECT TO AWS THEM

  connectToAWS();
}
void loop() {
    Serial.println("Message Received: ");
    Serial.println(Serial2.readString());
}
