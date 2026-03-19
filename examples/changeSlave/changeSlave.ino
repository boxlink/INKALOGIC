#include "inkaLogicModbus.h"

#define MB_PORT_NUM  UART_NUM_2
#define MB_TX_PIN    13
#define MB_RX_PIN    4
#define MB_DIR_PIN   12

#define SLAVE_1      1
#define SLAVE_2      7

InkaLogicModbus modbus;

void setup() {
    Serial.begin(115200);
    delay(1000);

    modbus.config(MB_PORT_NUM, MB_TX_PIN, MB_RX_PIN, MB_DIR_PIN);
    esp_err_t result = modbus.begin(SLAVE_1, 9600);
    if (result != ESP_OK) {
        Serial.printf("ERROR: %s (0x%04X)\n", esp_err_to_name(result), result);
        while (1) delay(1000);
    }
}

void loop() {
    uint8_t sensores[] = {SLAVE_1, SLAVE_2};
    esp_err_t status;

    for (uint8_t s = 0; s < 2; s++) {
        modbus.changeSlave(sensores[s]);
        modbus.clearResponseBuffer();
        status = modbus.readHoldingRegisters(0x0000, 3);

        if (status == ESP_OK)
            Serial.printf("Slave %d -> [0]=%d [1]=%d [2]=%d\n",
                sensores[s],
                modbus.getResponseBuffer(0),
                modbus.getResponseBuffer(1),
                modbus.getResponseBuffer(2));
        else
            Serial.printf("Slave %d -> ERROR: %s (0x%04X)\n",
                sensores[s], esp_err_to_name(status), status);

        delay(200);
    }

    delay(5000);
}
