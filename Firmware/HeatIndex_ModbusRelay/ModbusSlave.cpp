#include "ModbusSlave.h"
#include "Globals.h"

ModbusSlave* ModbusSlave::instance = nullptr;
bool ModbusSlave::serialNumberUpdatePending = false;  // Initialize the static flag

ModbusSlave::ModbusSlave()
  : mb() {}

void ModbusSlave::begin() {
  instance = this;

  // Initialize Serial2 for Modbus communication
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  mb.begin(&Serial2, TXRX_CONTROL_PIN);

  // Initialize or load registers
  initializeRegisters();
}

void ModbusSlave::handleModbus() {
  mb.task();  // Run Modbus communication
}

uint16_t ModbusSlave::onSetCoil(TRegister* reg, uint16_t value) {
    uint16_t address = reg->address.address;

    //digitalWrite(MODBUS_LED, HIGH);  // Indicate Modbus activity

    if (!instance) return value;  // Safety check for the instance pointer

    relaysUpdated = true;

    // Serial.print("Register type: COIL, Address: ");
    // Serial.print(address);
    // Serial.print(", Updated value: ");
    // Serial.println(value);

    return value;
}



void ModbusSlave::initializeRegisters() {

  Serial.println("Fresh KVS detected. Initializing registers to defaults...");

  // Initialize the registers
  mb.addCoil(0, 0, 4);
  mb.onSetCoil(0, onSetCoil);
  //mb.onSetHreg(reg.address, onSetHreg);
  mb.slave(DEFAULT_SLAVE_ID);

  Serial.printf("Slave ID initialized to default value: %d\n", DEFAULT_SLAVE_ID);

  preferences.putBool("initialized", true);
}
