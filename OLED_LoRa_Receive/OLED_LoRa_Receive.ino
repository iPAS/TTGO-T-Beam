#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include "SSD1306.h"
#include "images.h"
#include <TinyGPS++.h>

#define SCK 5    // GPIO5  -- SX1278's SCK
#define MISO 19  // GPIO19 -- SX1278's MISO
#define MOSI 27  // GPIO27 -- SX1278's MOSI
#define SS 18    // GPIO18 -- SX1278's CS
#define RST 14   // GPIO14 -- SX1278's RESET
#define DI0 26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

//#define BAND    868E6
#define BAND 923E6

SSD1306 display(0x3C, 21, 22);
String  rssi     = "RSSI --";
String  packSize = "--";
String  packet;

TinyGPSPlus gps;
HardwareSerial GPS_Serial1(1);

void loraData() {
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 15, "Received " + packSize + " bytes");
    display.drawStringMaxWidth(0, 26, 128, packet);
    display.drawString(0, 0, rssi);
    display.display();
    Serial.println(rssi);
}

void cbk(int packetSize) {
    packet   = "";
    packSize = String(packetSize, DEC);
    for (int i = 0; i < packetSize; i++) {
        packet += (char)LoRa.read();
    }
    rssi = "RSSI " + String(LoRa.packetRssi(), DEC);
    loraData();
}

void setup() {
    pinMode(16, OUTPUT);
    digitalWrite(16, LOW);  // set GPIO16 low to reset OLED
    delay(50);
    digitalWrite(16, HIGH);  // while OLED is running, must set GPIO16 in high、

    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println();
    Serial.println("LoRa Receiver Callback");
    SPI.begin(SCK, MISO, MOSI, SS);
    LoRa.setPins(SS, RST, DI0);
    if (!LoRa.begin(BAND)) {
        Serial.println("Starting LoRa failed!");
        while (1)
            ;
    }
    // LoRa.onReceive(cbk);
    LoRa.receive();
    Serial.println("init ok");
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);

    // GPS
    GPS_Serial1.begin(9600, SERIAL_8N1, 12, 15);   //17-TX 18-RX

    delay(1500);
}

void loop() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        cbk(packetSize);
    }
    //   delay(10);


Serial.print("Latitude  : ");
  Serial.println(gps.location.lat(), 5);
  Serial.print("Longitude : ");
  Serial.println(gps.location.lng(), 4);
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.print("Altitude  : ");
  Serial.print(gps.altitude.feet() / 3.2808);
  Serial.println("M");
  Serial.print("Time      : ");
  Serial.print(gps.time.hour());
  Serial.print(":");
  Serial.print(gps.time.minute());
  Serial.print(":");
  Serial.println(gps.time.second());
  Serial.println("**********************");

    smartDelay(1000);                                      


  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

    digitalWrite(14, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(500);              // wait for a second
    digitalWrite(14, LOW);   // turn the LED off by making the voltage LOW
    delay(500);              // wait for a second
}


static void smartDelay(unsigned long ms)                
{
  unsigned long start = millis();
  do
  {
    while (GPS_Serial1.available())
      gps.encode(GPS_Serial1.read());
  } while (millis() - start < ms);
}