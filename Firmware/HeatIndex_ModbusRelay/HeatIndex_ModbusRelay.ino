#include "ModbusSlave.h"
#include "Globals.h"
#include "PinConfig.h"      
#include "ShiftRegister.h" 

ModbusSlave modbusSlave;
bool relaysUpdated = false;

ShiftRegister RELAY_CONTROL_PORT(OUT_CTRL_DIN, OUT_CTRL_CS, OUT_CTRL_CLK, 1);



void modbusTask(void *pvParameters) {
  while (true) {
    modbusSlave.handleModbus();           // Run Modbus task
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Delay to prevent task hogging CPU
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Modular Modbus Slave with FreeRTOS Task starting...");

  // Initialize Modbus Slave
  modbusSlave.begin();

  // Create a FreeRTOS task for Modbus
  xTaskCreate(
    modbusTask,    // Task function
    "ModbusTask",  // Name of the task (for debugging)
    4096,          // Stack size (in words)
    NULL,          // Task parameters
    1,             // Priority (1 = low, higher = higher priority)
    NULL           // Task handle
  );

  RELAY_CONTROL_PORT.setBitOrder(LSBFIRST);
  RELAY_CONTROL_PORT.setAll(BIT_OFF);
  RELAY_CONTROL_PORT.updateRegisters();

  Serial.println("Slave Relay Started");


}

void loop() {

  if(relaysUpdated){
    relaysUpdated = false;


    
    // modbusSlave
    Serial.println("Relay 1:" + String(ModbusSlave::instance->mb.Coil(0)));
    Serial.println("Relay 2:" + String(ModbusSlave::instance->mb.Coil(1)));
    Serial.println("Relay 3:" + String(ModbusSlave::instance->mb.Coil(2)));
    Serial.println("Relay 4:" + String(ModbusSlave::instance->mb.Coil(3)));

    RELAY_CONTROL_PORT.setBit(1, !ModbusSlave::instance->mb.Coil(0));
    RELAY_CONTROL_PORT.setBit(2, !ModbusSlave::instance->mb.Coil(1));
    RELAY_CONTROL_PORT.setBit(3, !ModbusSlave::instance->mb.Coil(2));
    RELAY_CONTROL_PORT.setBit(4, !ModbusSlave::instance->mb.Coil(3));
    RELAY_CONTROL_PORT.updateRegisters();

  }
 
}

