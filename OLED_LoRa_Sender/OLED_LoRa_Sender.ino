#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include "SSD1306.h"
#include "images.h"

#define SCK 5    // GPIO5  -- SX1278's SCK
#define MISO 19  // GPIO19 -- SX1278's MISnO
#define MOSI 27  // GPIO27 -- SX1278's MOSI
#define SS 18    // GPIO18 -- SX1278's CS
#define RST 14   // GPIO14 -- SX1278's RESET
#define DI0 26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

//#define BAND  868E6
#define BAND 923E6

unsigned int counter = 0;

SSD1306 display(0x3C, 21, 22);
String  rssi     = "RSSI --";
String  packSize = "--";
String  packet;

void setup() {
    pinMode(16, OUTPUT);
    pinMode(2, OUTPUT);

    digitalWrite(16, LOW);  // set GPIO16 low to reset OLED
    delay(50);
    digitalWrite(16, HIGH);  // while OLED is running, must set GPIO16 in high

    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println();
    Serial.println("LoRa Sender Test");

    SPI.begin(SCK, MISO, MOSI, SS);
    LoRa.setPins(SS, RST, DI0);
    if (!LoRa.begin(BAND)) {
        Serial.println("Starting LoRa failed!");
        while (1)
            ;
    }

    LoRa.setTxPower(20);    // Set maximum Tx power to 20 dBm (17 is default).
                            // https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md#tx-power

    // LoRa.onReceive(cbk);
    //  LoRa.receive();
    Serial.println("init ok");
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);

    delay(1500);
}

void loop() {
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);

    display.drawString(0, 0, "Sending packet: ");
    display.drawString(90, 0, String(counter));
    Serial.println(String(counter));
    display.display();

    // send packet
    LoRa.beginPacket();
    LoRa.printf("Hello! I'm %04X", ESP.getEfuseMac());
    LoRa.print(counter);
    LoRa.endPacket();
    counter++;

    digitalWrite(14, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(500);              // wait for a second
    digitalWrite(14, LOW);   // turn the LED off by making the voltage LOW
    delay(500);              // wait for a second
}
