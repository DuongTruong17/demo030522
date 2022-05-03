/***************************************************
//  Adafruit MQTT Library ESP8266 Example

  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino

//  Works great with Adafruit's Huzzah ESP board & Feather
  ----> https://www.adafruit.com/product/2471
  ----> https://www.adafruit.com/products/2821

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "ESP8266TimerInterrupt.h"

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "Tang2 MEO"
#define WLAN_PASS       "11111111"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "ngoinhaiot.com"//"io.adafruit.com"
#define AIO_SERVERPORT  1111//1883                   // use 8883 for SSL
#define AIO_USERNAME    "duong17"//"DuongTruong"
#define AIO_KEY         "BCD359D840744444"//"aio_OTpr4932hwLSXQ5rRBFfbVPF6Sz0"


volatile bool statusLed = false;
volatile uint32_t lastMillis = 0;

#define TIMER_INTERVAL_MS       20

// Init ESP8266 timer 1
ESP8266Timer ITimer;
/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiClientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish photocell = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/photocell");
Adafruit_MQTT_Publish ngoaiVi = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/conCong/ngoaiVi");
Adafruit_MQTT_Publish ngoaiVic = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/conCong/test");

// Setup a feed called 'onoff' for subscribing to changes.
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/onoff");
Adafruit_MQTT_Subscribe conQuayE = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/conCong/conQuay");
//Adafruit_MQTT_Subscribe conQuayP = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/conQuayP");
//Adafruit_MQTT_Subscribe conQuayT = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/conQuayT");
Adafruit_MQTT_Subscribe ngoaiViE = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/ngoaiVi");

/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();
int ah, pos, kt, SBUF, countSend;
String text, dataRec;
char kiemTraNhan, modeUart;
char message[10], messOld[10];
char temp_b, temp_p, temp_q, temp_c, temp_a;
void nhanUart()
{
  if(Serial.available()>0)
  {
    SBUF = Serial.read();
    kiemTraNhan++;
    if(kiemTraNhan > 205) kiemTraNhan = 0;
    if(SBUF == 'b')
    {
      kiemTraNhan = 1;
    }
    else if(SBUF == 'a')
    {
      kiemTraNhan = 11;
    }
    else if(SBUF == 'c')
    {
      kiemTraNhan = 21;
    }
    else if(SBUF == 'o')     //trai
    {
      kiemTraNhan = 31;
    }
    else if(SBUF == 'k')   //phai
    {
      kiemTraNhan = 41;
    }
    else if(SBUF == 'm')
    {
      kiemTraNhan = 51;
    }
    else if(SBUF == '$')
    {
      kiemTraNhan = 61;
    }
    else if(SBUF == 'z')
    {
      kiemTraNhan = 71;
    }
    else if(SBUF == 'W')
    {
      kiemTraNhan = 81;
    }
    else if(SBUF == 'p')
    {
      kiemTraNhan = 91;
    }
    else if(SBUF == 'q')
    {
      kiemTraNhan = 101; 
    }
    else if(SBUF == 't')
    {
      kiemTraNhan = 201;
    }
    else if(SBUF == '*')
    {
      if(kiemTraNhan == 2) modeUart = 1;//phai
      else if(kiemTraNhan == 12) modeUart = 2;//trai
      else if(kiemTraNhan == 22) modeUart = 3;// giua
//      else if(kiemTraNhan == 32) pos = 1;
//      else if(kiemTraNhan == 42) pos = 2;
//      else if(kiemTraNhan == 52) pos = 3;
      else if(kiemTraNhan == 62) modeUart = 54;
      else if(kiemTraNhan == 72) modeUart = 11;
      else if(kiemTraNhan == 82) modeUart = 22;
      else if(kiemTraNhan == 92) modeUart = 92;
      else if(kiemTraNhan == 102) modeUart = 102;
      else if(kiemTraNhan == 202) modeUart = 202;
    }
    if(modeUart == 1)
    {
      countSend++;
      if(countSend>=3)
      {
        ngoaiVi.publish("b\n");
//        Serial.println("11111111111");
        countSend = 0;
      }
    }
    else if(modeUart == 2)
    {
      countSend++;
      if(countSend>=3)
      {
        ngoaiVi.publish("a\n");
        countSend = 0;
      }
    }
    else if(modeUart == 3)
    {
      ngoaiVi.publish("c\n");
    }
    else if(modeUart == 92)
    {
      countSend++;
      if(countSend>=5)
      {
        ngoaiVi.publish("p\n");
        countSend = 0;
      }
    }
    else if(modeUart == 102)
    {
      countSend++;
      if(countSend>=5)
      {
        ngoaiVi.publish("q\n");
        countSend = 0;
//        Serial.println("aaaA");
      }
    }
    else if(modeUart == 202)
    {
      countSend++;
      if(countSend>=5)
      {
        ngoaiVi.publish("t\n");
        countSend = 0;
      }
    }
    modeUart = 0;
    SBUF = 0;
  }
}

void IRAM_ATTR TimerHandler() // uart
{
    nhanUart();
#if (TIMER_INTERRUPT_DEBUG > 0)
//  Serial.println("Delta ms = " + String(millis() - lastMillis));
  lastMillis = millis();
#endif
}

int counting;

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);
  Serial.begin(9600);//115200
  delay(10);

  Serial.println(F("Adafruit MQTT demo"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&onoffbutton);
  mqtt.subscribe(&conQuayE);
  mqtt.subscribe(&ngoaiViE);

  if (ITimer.attachInterruptInterval(TIMER_INTERVAL_MS * 1000, TimerHandler))
  {
    lastMillis = millis();
    Serial.print(F("Starting ITimer OK, millis() = ")); Serial.println(lastMillis);
  }
  else
    Serial.println(F("Can't set ITimer correctly. Select another freq. or interval"));
  
}

uint32_t x=0;

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();
//nhanUart();
  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here
  
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &onoffbutton) {
      Serial.print(F("Got: "));
      Serial.println((char *)onoffbutton.lastread);
      if(strcmp((char *)onoffbutton.lastread,"a*") == 0)
      {
        digitalWrite(BUILTIN_LED,1);
        Serial.println("1");
      }
      else if(strcmp((char *)onoffbutton.lastread,"b*") == 0)
      {
        digitalWrite(BUILTIN_LED,0);
        Serial.println("0");
      }
    }
    else if(subscription == &conQuayE)
    {
      if(strcmp((char *)conQuayE.lastread,"a*") == 0)
      {
        Serial.write("a*\n");
        temp_a = 0;
      }
      else if(strcmp((char *)conQuayE.lastread,"b*") == 0)
      {
        Serial.write("b*\n");
        temp_b = 0;
      }
      else if(strcmp((char *)conQuayE.lastread,"c*") == 0)
      {
        Serial.write("c*\n");
        temp_c = 0;
      }
      else if(strcmp((char *)conQuayE.lastread,"k*") == 0)
      {
        Serial.write("k*\n");
      }
      else if(strcmp((char *)conQuayE.lastread,"m*") == 0)
      {
        Serial.write("m*\n");
      }
      else if(strcmp((char *)conQuayE.lastread,"o*") == 0)
      {
        Serial.write("o*\n");
      }
    }
  }

  // Now we can publish stuff!
//  Serial.print(F("\nSending photocell val "));
//  Serial.print(x);
//  Serial.print("...");
//  ngoaiVi.publish(x++);
//  if (! ngoaiVi.publish(x++)) {
//    Serial.println(F("Failed"));
//  } else {
//    Serial.println(F("OK!"));
//  }

  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  /*
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
  */
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
  ngoaiVic.publish("hello f");
}
