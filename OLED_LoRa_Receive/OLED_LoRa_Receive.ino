#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SSD1306.h>
#include "images.h"

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
String  packet   = "";

TinyGPSPlus gps;
HardwareSerial GPS_Serial1(1);
#define GPS_BAUDRATE 9600
#define GPS_TX 12
#define GPS_RX 15
String gps_time = "T --";
String gps_loc  = "SAT --, LAT --, LON --, ALT --";

void loraData() {
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 15, "Received " + packSize + " bytes");
    display.drawStringMaxWidth(0, 26, 128, packet);
    display.drawString(0, 0, rssi);
    display.display();

    Serial.println(gps_time + ", " + gps_loc + ", " + rssi + ", " + packet);
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
    GPS_Serial1.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_TX, GPS_RX);  // 17-TX 18-RX

    delay(1500);
}

void smartDelay(unsigned long ms) {
    unsigned long start = millis();
    do {
        while (GPS_Serial1.available() > 0) {
            gps.encode(GPS_Serial1.read());
        }
    } while (millis() - start < ms);
}

void loop() {
    digitalWrite(14, HIGH);  // turn the LED on (HIGH is the voltage level)
    smartDelay(500);

    if (gps.satellites.isValid() && gps.time.isUpdated() && gps.location.isValid()) {
        // "T --, SAT --, LAT --, LON --, ALT --, ";
        gps_time = "T " + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
        gps_loc  = "SAT " + gps.satellites.value();
        gps_loc  = gps_loc + ", " + "LAT " + gps.location.lat();
        gps_loc  = gps_loc + ", " + "LON " + gps.location.lng();
        gps_loc  = gps_loc + ", " + "ALT " + gps.altitude.meters();
    }

    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        cbk(packetSize);
    }

    digitalWrite(14, LOW);   // turn the LED off by making the voltage LOW
    smartDelay(500);
}
