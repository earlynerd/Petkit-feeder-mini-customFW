#include <Arduino.h>
#include <SPI.h>
#include <time.h>
#include <coredecls.h> // optional settimeofday_cb() callback to check on server
#include "RTClib.h"
// #include "NTPClient.h"
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESPAsyncTCP.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "petkitbus.h"
#include "Wire.h"
#include <ESPAsyncWebServer.h>
#include <WebSerialLite.h>
#include "timeconfig.hpp"
#include <PubSubClient.h>

#ifndef STASSID
#define STASSID "Sly.fi"
#define STAPSK "chiroptera"
#endif

#define MY_TZ "PST8PDT,M3.2.0,M11.1.0"
#define MY_NTP_SERVER "pool.ntp.org"

void commandCallback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}

RTC_PCF8563 rtc;

AsyncWebServer server(80);

WiFiClient wificlient;
IPAddress mqttserver(192, 168, 4, 212);
PubSubClient client(mqttserver, 1883, commandCallback, wificlient);

/*
   opional: set SNTP interval
*/

uint32_t sntp_update_delay_MS_rfc_not_less_than_15000()
{
  return 60 * 60 * 1000UL; // 1 hour
}

/*
   optional: by default, the NTP will be started after 60 secs
   lets start at a random time in 5 seconds
*/
uint32_t sntp_startup_delay_MS_rfc_not_less_than_60000()
{
  randomSeed(A0);
  return random(5000 + millis());
}

const int wifiResetPin = 0;
const int buttonPin = 13;
const int uart_rx_pin = 3;
const int uart_tx_pin = 1;
const int isd_reset_pin = 15;
const int i2c_sda_pin = 5;
const int i2c_scl_pin = 14;

const char *ssid = STASSID;
const char *password = STAPSK;
const int esp_pins[] = {16, 12, 4, 2}; // unknown function

PetkitBus petkitbus(&Serial);
void printFrame(PetkitBus::Frame *frame);
void dispense();
void printPinstates();
void scan();



void setup()
{
  /*
    for (int i = 0; i < sizeof(esp_pins) / sizeof(esp_pins[0]); i++)
    {
      pinMode(esp_pins[i], INPUT_PULLUP);
      digitalWrite(esp_pins[i], HIGH);
    }
    */
  // settimeofday_cb(time_is_set); // optional: callback if time was sent
  pinMode(wifiResetPin, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(isd_reset_pin, OUTPUT);
  digitalWrite(15, LOW);
  Serial.setRxBufferSize(1024);
  Serial.begin(115200);

  Wire.begin(i2c_sda_pin, i2c_scl_pin);
  // Serial1.begin(115200);
  rtc.begin();
  configTime(MY_TZ, MY_NTP_SERVER);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  uint32_t now = millis();
  while ((WiFi.waitForConnectResult() != WL_CONNECTED) && (millis() - now < 30000))
  {
    // Serial.println("Connection Failed! Rebooting...");
    delay(10);
    // ESP.restart();
  }
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
    ESP.restart();

  settimeofday_cb(time_is_set); // register callback if time was sent

  petkitbus.init();
  delay(10);
  petkitbus.blink(1, 100, 100, 2);
  // while (Serial.available())
  // if (petkitbus.parseFrame(&frame, 25))
  // printFrame(&frame);

  petkitbus.blink(2, 100, 100, 2);
  // while (Serial.available())
  // if (petkitbus.parseFrame(&frame, 25))
  // printFrame(&frame);
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]()
                     {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type); });
  ArduinoOTA.onEnd([]()
                   { Serial.println("\nEnd"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
  ArduinoOTA.onError([](ota_error_t error)
                     {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    } });
  ArduinoOTA.begin();
  petkitbus.blink(1, 100, 100, 3);
  petkitbus.blink(2, 100, 100, 3);

  WebSerial.begin(&server);
  /* Attach Message Callback */
  WebSerial.onMessage(recvMsg);
  server.begin();

  PetkitBus::Frame frame = {0};

  while (Serial.available())
    if (petkitbus.parseFrame(&frame, 25))
      printFrame(&frame);
  delay(50);
  for (int i = 0; i < 16; i++)
  {
    WebSerial.printf("...");
    delay(200);
  }
  WebSerial.printf("PetkitMini Hack begin!\n");
  if (time(nullptr) < 1600000000)
    getRTC();
  // time_t dtnow;
  // tm tm;

  // timeClient.begin();
  // timeClient.setTimeOffset(0);
  // time_t epochTime = timeClient.getEpochTime();
  // struct tm *ptm = gmtime((time_t *)&epochTime);

  if (rtc.lostPower())
  {
    WebSerial.printf("RTC Lost Power!\n");
  }
  String m = WiFi.macAddress();
  char buf[128];
  m.toCharArray(buf, 128);
  
  if (client.connect(buf, "node1", "notraspberry")) {
    m = m + "/outTopic";
    m.toCharArray(buf, 128);
    client.publish(buf ,"hello world");
    client.subscribe("inTopic");
  }
  // time(&dtnow); // read the current time
  // localtime_r(&dtnow, &tm);
  // DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour = 0, uint8_t min = 0, uint8_t sec = 0);
  // DateTime newdt(tm.tm_year, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
  // rtc.adjust(newdt);
}

bool lastButton = true;

void loop()
{

  ArduinoOTA.handle();
  PetkitBus::Frame frame;
  while (Serial.available())
    if (petkitbus.parseFrame(&frame, 5))
      printFrame(&frame);
  bool buttonPressed = digitalRead(buttonPin);
  if (!buttonPressed && lastButton)
    dispense();
  lastButton = buttonPressed;
  // showTime();
  delay(25);
  // WebSerial.printf("Free heap=[%u]\n", ESP.getFreeHeap());
}

void printPinstates()
{
  uint32_t count = 0;
  const uint32_t bufsize = 256;
  char printbuf[bufsize];
  char *p = printbuf;
  for (unsigned int i = 0; i < sizeof(esp_pins) / sizeof(esp_pins[0]); i++)
  {
    count += snprintf(p + count, bufsize - count, "%d:%d,", esp_pins[i], digitalRead(esp_pins[i]));
  }
  count += snprintf(p + count, bufsize - count, "\n");
  WebSerial.write((uint8_t *)printbuf, count);
}

void scan()
{
  byte error, address;
  int nDevices;
  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      WebSerial.printf("I2C device found at address 0x%02X\n", address);
      nDevices++;
    }
    else if (error == 4)
    {
      WebSerial.printf("Unknown error at address 0x%02X\n", address);
    }
    else
    {
      WebSerial.printf("No device found at address 0x%02X\n", address);
    }
    delay(100);
  }
  if (nDevices == 0)
    WebSerial.printf("No I2C devices found\n");
  else
    WebSerial.printf("found %d devices!\n", nDevices);
  // delay(5000); // wait 5 seconds for next scan
}

void dispense()
{
  PetkitBus::Frame frame;
  petkitbus.open();
  while (Serial.available())
    if (petkitbus.parseFrame(&frame, 25))
      printFrame(&frame);
  petkitbus.beep(80, 80, 3);
  while (!digitalRead(buttonPin))
  {
    petkitbus.blink(1, 2000, 0, 1);
    petkitbus.dispense(1, 1, 1, 0x50);
    while (Serial.available())
      if (petkitbus.parseFrame(&frame, 25))
        printFrame(&frame);
    delay(1000);
  }
  // delay(2000);
  petkitbus.close();
  while (Serial.available())
    if (petkitbus.parseFrame(&frame, 25))
      printFrame(&frame);
}

void printFrame(PetkitBus::Frame *frame)
{

  const uint32_t bufsize = 512;
  uint32_t count = 0;
  char printbuf[bufsize];
  char *p = printbuf;
  count += snprintfTimestamp(p + count, bufsize - count);
  count += snprintf(p + count, bufsize - count, "time:%u,packet type:%hhu,length:%hhu,seq:%hhu,crc:%04X,valid:%hhu,", frame->timestamp, frame->type, frame->length, frame->sequence, frame->crc, frame->isValid);

  if (frame->length >= 8)
  {
    count += snprintf((char *)(p + count), bufsize - count, "Data:0x");
    for (int i = 5; i < frame->length - 2; i++)
    {
      // if (frame->raw[i] < 0xF)
      //  count += snprintf(p+count, bufsize - count, "0");
      // WebSerial.print(, HEX);
      count += snprintf((char *)(p + count), bufsize - count, "%02X", frame->raw[i]);
    }
    // count += snprintf(p+count,  bufsize - count, "\n");
    count += snprintf(p + count, bufsize - count, ",");
  }

  else
    count += snprintf(p + count, bufsize - count, ",");

  switch (frame->type)
  {
  case 0:
  {
    count += snprintf(p + count, bufsize - count, "boot\n");
  }
  break;
  case 1:
  {
    count += snprintf(p + count, bufsize - count, "ack get status\n");
  }
  break;
  case 2:
  {
    count += snprintf(p + count, bufsize - count, "status reply,");
    uint16_t mv0, mv1;
    // uint16_t adc0, adc1;
    // adc0 = (uint16_t)frame->raw[8] << 8 | (uint16_t)frame->raw[9];
    mv0 = (uint16_t)frame->raw[10] << 8 | (uint16_t)frame->raw[11];
    // adc1 = (uint16_t)frame->raw[12] << 8 | (uint16_t)frame->raw[13];
    mv1 = (uint16_t)frame->raw[14] << 8 | (uint16_t)frame->raw[15];
    count += snprintf(p + count, bufsize - count, "Door:%d,Food:%d,unk:%d,Battery_mV:%d,Adapter_mV:%d\n", frame->raw[5], frame->raw[6], frame->raw[7], mv0, mv1);
  }
  break;
  case 3:
  {
    count += snprintf(p + count, bufsize - count, "ack set config\n");
  }
  break;
  case 4:
  {
    count += snprintf(p + count, bufsize - count, "ack set config\n");
  }
  break;
  case 5:
  {
    count += snprintf(p + count, bufsize - count, "ack set config\n");
  }
  break;
  case 6:
  {
    count += snprintf(p + count, bufsize - count, "ack set config\n");
  }
  break;
  case 7:
  {
    count += snprintf(p + count, bufsize - count, "ack open door\n");
  }
  break;
  case 8:
  {
    count += snprintf(p + count, bufsize - count, "door open now\n");
  }
  break;
  case 9:
  {
    count += snprintf(p + count, bufsize - count, "ack close door\n");
  }
  break;
  case 10:
  {
    count += snprintf(p + count, bufsize - count, "door closed now\n");
  }
  break;
  case 11:
  {
    count += snprintf(p + count, bufsize - count, "ack dispense\n");
  }
  break;
  case 12:
  {
    count += snprintf(p + count, bufsize - count, "dispense info\n");
  }
  break;
  case 13:
  {
    count += snprintf(p + count, bufsize - count, "ack set config\n");
  }
  break;
  case 14:
  {
    count += snprintf(p + count, bufsize - count, "ack blink/beep\n");
  }
  break;
  case 15:
  {
    count += snprintf(p + count, bufsize - count, "ack Sleep\n");
  }
  break;
  case 16:
  {
    count += snprintf(p + count, bufsize - count, "Unknown\n");
  }
  break;
  case 17:
  {
    count += snprintf(p + count, bufsize - count, "Unknown\n");
  }
  break;
  case 18:
  {
    count += snprintf(p + count, bufsize - count, "unknown\n");
  }
  break;
  case 19:
  {
    count += snprintf(p + count, bufsize - count, "config\n");
  }
  break;
  case 20:
  {
    count += snprintf(p + count, bufsize - count, "config reply\n");
  }
  break;
  case 21:
  {
    count += snprintf(p + count, bufsize - count, "dispense complete\n");
  }
  break;
  }

  WebSerial.write((uint8_t *)printbuf, count);
  if (count >= bufsize)
    WebSerial.print(" ...printbuf overflow\n");
  // WebSerial.println();
}