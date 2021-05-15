#include <Arduino.h>
#include <axp20x.h>


#define BLINK_ON_PERIOD 500
#define BLINK_OFF_PERIOD 1000

#define LED_IO_V07 14
#define LED_IO_V10 4
static uint8_t led_io;

void led_toggle_process() {
    static uint8_t state = 0;
    static uint32_t next = 0;

    if (millis() > next) {
        switch (state) {
        case 0:
            digitalWrite(led_io, HIGH);
            next = millis() + BLINK_ON_PERIOD;
            state = 1;
            break;

        case 1:
            digitalWrite(led_io, LOW);
            next = millis() + BLINK_OFF_PERIOD;
            state = 0;
            break;
        }
    }
}

void led_setup() {
    pinMode(led_io, OUTPUT);
    digitalWrite(led_io, LOW);
}


AXP20X_Class axp;
#define AXP_SDA 21
#define AXP_SCL 22
#define AXP_IRQ 35

void axp_setup() {
    axp.setLDO2Voltage(3300);   // LoRa VDD
    axp.setLDO3Voltage(3300);   // GPS  VDD
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);  // LORA radio
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);  // GPS main power

    axp.setDCDC1Voltage(3300);  // for the OLED power
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);

    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);

    // axp.setChgLEDMode(AXP20X_LED_BLINK_1HZ);
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Version validation
    Wire.begin(AXP_SDA, AXP_SCL);
    if (axp.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_FAIL) {
        Serial.println("[DEBUG] Starting AXP192 failed! -- guessing, this is the V0.7");

        led_io = LED_IO_V07;
        // gps_tx = GPS_TX_V07;
        // gps_rx = GPS_RX_V07;

    } else {
        Serial.println("[DEBUG] Starting AXP192 succeeded! -- guessing, its version >= V1.0");

        axp_setup();
        led_io = LED_IO_V10;
        // gps_tx = GPS_TX_V10;
        // gps_rx = GPS_RX_V10;
    }

    led_setup();    // LED
}

// the loop function runs over and over again forever
void loop() {
    led_toggle_process();
}
