#include "inkaLogicModbus.h"

#define MB_PORT_NUM  UART_NUM_2
#define MB_TX_PIN    13
#define MB_RX_PIN    4
#define MB_DIR_PIN   12
#define SLAVE_ID     7

InkaLogicModbus modbus;

void setup() {
    Serial.begin(115200);
    delay(1000);

    modbus.config(MB_PORT_NUM, MB_TX_PIN, MB_RX_PIN, MB_DIR_PIN);
    esp_err_t result = modbus.begin(SLAVE_ID, 9600);
    if (result != ESP_OK) {
        Serial.printf("ERROR: %s (0x%04X)\n", esp_err_to_name(result), result);
        while (1) delay(1000);
    }
}

void loop() {
    modbus.clearResponseBuffer();
    esp_err_t status = modbus.readHoldingRegisters(0x0000, 7);

    if (status == ESP_OK) {
        const char* nombres[] = {"Hum", "Temp", "EC", "pH", "N", "P", "K"};
        uint8_t i = 0;
        while (modbus.available()) {
            Serial.printf("%s = %d\n", nombres[i++], modbus.receive());
        }
    } else {
        Serial.printf("ERROR: %s (0x%04X)\n", esp_err_to_name(status), status);
    }

    delay(2000);
}
