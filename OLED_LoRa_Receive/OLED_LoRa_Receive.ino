#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <SSD1306.h>
#include "images.h"
#include <TinyGPS++.h>
#include <axp20x.h>
#include <SoftwareSerial.h>


// T-Beam version selection. Differnt version has different hardware design.
// #define TBEAM_V1


#ifndef TBEAM_V1
#define LED_IO 14  // 4 -- V1.0
#else
#define LED_IO 4
#endif

#define LORA_SCK  5   // GPIO5  -- SX1278's SCK
#define LORA_MISO 19  // GPIO19 -- SX1278's MISO
#define LORA_MOSI 27  // GPIO27 -- SX1278's MOSI
#define LORA_SS   18  // GPIO18 -- SX1278's CS
#define LORA_DI0  26  // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define LORA_RST  23  // GPIO23 -- SX1278's RESET

//#define BAND    868E6
#define BAND 923E6

SSD1306 display(0x3C, 21, 22);
String  rssi     = "RSSI --";
String  snr      = "SNR --";
String  packSize = "--";
String  packet   = "";

#define GPS_BAUDRATE 9600

#ifndef TBEAM_V1
#define GPS_TX 12  // 34 -- V1.0
#define GPS_RX 15  // 12 -- V1.0
#else
#define GPS_TX 34
#define GPS_RX 12
#endif

#ifndef TBEAM_V1
HardwareSerial GPS_Serial1(1);
#else
SoftwareSerial GPS_Serial1(GPS_RX, GPS_TX);
#endif

TinyGPSPlus gps;
String gps_time = "T --";
String gps_loc  = "SAT --, LAT --, LON --, ALT --";

AXP20X_Class axp;

void loraData() {
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);

    display.drawString(0, 0, rssi + ", " + snr);
    display.drawString(0, 15, "Recv: " + packSize + " bytes");
    display.drawStringMaxWidth(0, 26, 128, packet);

    display.display();

    Serial.println(gps_time + ", " + gps_loc + ", " + rssi + ", " + snr + ", " + packet);
}

void cbk(int packetSize) {
    packet   = "";
    packSize = String(packetSize, DEC);
    for (int i = 0; i < packetSize; i++) {
        packet += (char)LoRa.read();
    }

    rssi = "RSSI " + String(LoRa.packetRssi(), DEC);
    snr  = "SNR "  + String(LoRa.packetSnr(), 2);

    loraData();
}

void smartDelay(unsigned long ms) {
    unsigned long start = millis();
    do {
        while (GPS_Serial1.available() > 0) {
            gps.encode(GPS_Serial1.read());
        }
    } while (millis() - start < ms);
}

void setup() {
    pinMode(LED_IO, OUTPUT);
    digitalWrite(LED_IO, LOW);

    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println();
    Serial.println("LoRa Receiver Callback");
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DI0);
    if (!LoRa.begin(BAND)) {
        Serial.println("Starting LoRa failed!");
        while (1)
            ;
    }

    LoRa.setSpreadingFactor(12);  // ranges from 6-12, default 7 see API docs. Changed for ver 0.1 Glacierjay
    // LoRa.setSignalBandwidth(7.8E3);  // signalBandwidth - signal bandwidth in Hz, defaults to 125E3.
                                     // Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3.
    // LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);  // Set maximum Tx power to 20 dBm (17 is default).
                                                  // https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md#tx-power
    // LoRa.setGain(0);  // Supported values are between 0 and 6. If gain is 0, AGC will be enabled and LNA gain will not be used. 
                      // Else if gain is from 1 to 6, AGC will be disabled and LNA gain will be used.

    // LoRa.onReceive(cbk);
    LoRa.receive();
    Serial.println("init ok");
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);

    #ifdef TBEAM_1
    // -- V1.0
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);  // LORA radio
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);  // GPS main power
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    axp.setDCDC1Voltage(3300);  // for the OLED power
    #endif

    // GPS
    #ifndef TBEAM_V1
    GPS_Serial1.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_TX, GPS_RX);
    #else
    GPS_Serial1.begin(GPS_BAUDRATE);
    #endif
    smartDelay(1500);
}

void loop() {
    digitalWrite(LED_IO, HIGH);  // turn the LED on (HIGH is the voltage level)
    smartDelay(500);

    if (gps.satellites.isValid() && gps.time.isUpdated() && gps.location.isValid()) {
        // "T --, SAT --, LAT --, LON --, ALT --, ";
        gps_time = "T " + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
        gps_loc  = "SAT " + String(gps.satellites.value());
        gps_loc  = gps_loc + ", " + "LAT " + String(gps.location.lat());
        gps_loc  = gps_loc + ", " + "LON " + String(gps.location.lng());
        gps_loc  = gps_loc + ", " + "ALT " + String(gps.altitude.meters());
    }

    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        cbk(packetSize);
    }

    digitalWrite(LED_IO, LOW);   // turn the LED off by making the voltage LOW
    smartDelay(500);
}
