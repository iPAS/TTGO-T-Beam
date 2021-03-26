#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include "SSD1306.h"
#include "images.h"
#include <axp20x.h>


//#define BAND    868E6
#define BAND 923E6

#define LORA_SCK  5   // GPIO5  -- SX1278's SCK
#define LORA_MISO 19  // GPIO19 -- SX1278's MISO
#define LORA_MOSI 27  // GPIO27 -- SX1278's MOSI
#define LORA_SS   18  // GPIO18 -- SX1278's CS
#define LORA_DI0  26  // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define LORA_RST  23  // GPIO23 -- SX1278's RESET


#define LED_IO_V07 14
#define LED_IO_V10 4
static uint8_t led_io;


unsigned int counter = 0;

SSD1306 display(0x3C, 21, 22);
String  rssi     = "RSSI --";
String  packSize = "--";
String  packet;


AXP20X_Class axp;
#define AXP_SDA 21
#define AXP_SCL 22
#define AXP_IRQ 35

// ---------- AXP192 ----------
void axp_setup() {
    axp.setLDO2Voltage(3300);   // LoRa VDD
    axp.setLDO3Voltage(3300);   // GPS  VDD
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);  // LORA radio
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);  // GPS main power

    axp.setDCDC1Voltage(3300);  // for the OLED power
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);

    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
}

// ---------- LoRa ----------
void on_tx_done() {
    Serial.println(counter);
}

// ---------- Setup ----------
void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println();

    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);

    delay(500);


    // Version validation
    Wire.begin(AXP_SDA, AXP_SCL);
    if (axp.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_FAIL) {
        Serial.println("Starting AXP192 failed! -- guessing, this is the V0.7");

        led_io = LED_IO_V07;
        // gps_tx = GPS_TX_V07;
        // gps_rx = GPS_RX_V07;

    } else {
        Serial.println("Starting AXP192 succeeded! -- guessing, its version >= V1.0");

        axp_setup();
        led_io = LED_IO_V10;
        // gps_tx = GPS_TX_V10;
        // gps_rx = GPS_RX_V10;
    }


    // LED
    pinMode(led_io, OUTPUT);
    digitalWrite(led_io, LOW);


    // LoRa
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
    LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);  // Set maximum Tx power to 20 dBm (17 is default).
                                                  // https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md#tx-power
    // LoRa.setGain(0);  // Supported values are between 0 and 6. If gain is 0, AGC will be enabled and LNA gain will not be used. 
                      // Else if gain is from 1 to 6, AGC will be disabled and LNA gain will be used.
    LoRa.onTxDone(on_tx_done);
    // LoRa.onReceive(cbk);
    // LoRa.receive();


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
    LoRa.printf("<%04X> #", ESP.getEfuseMac());
    LoRa.print(counter);
    LoRa.endPacket();
    counter++;

    uint8_t i;
    for (i=0; i<2; i++) {
        digitalWrite(led_io, HIGH);  // turn the LED on (HIGH is the voltage level)
        delay(500);              // wait for a second
        digitalWrite(led_io, LOW);   // turn the LED off by making the voltage LOW
        delay(500);              // wait for a second
    }
}
