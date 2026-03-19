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
    modbus.beginTransmission(0x0050);
    modbus.send((uint16_t)3000);
    modbus.send((uint16_t)4000);
    esp_err_t status = modbus.readWriteMultipleRegisters(0x0000, 3);

    if (status == ESP_OK) {
        Serial.println("Escrito: 3000, 4000 en 0x0050-0x0051");
        uint8_t idx = 0;
        while (modbus.available())
            Serial.printf("Leido[%d] = %d\n", idx++, modbus.receive());
    } else {
        Serial.printf("ERROR: %s (0x%04X)\n", esp_err_to_name(status), status);
    }

    delay(5000);
}
