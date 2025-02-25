#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "sbus.h"

// SBUS Transmitter on Serial1 (TX = GPIO 17, RX = GPIO 16)
bfs::SbusTx sbus_tx(&Serial1, 16, 17, true, false);
bfs::SbusData data;


int value = 172;  // Start at minimum throttle
bool increasing = true;  // Control direction

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando SBUS...");

  sbus_tx.Begin();
  data.failsafe = false;
  data.ch17 = true;
  data.ch18 = true;
  data.lost_frame = false;

  for (int i = 500; i > 172; i--) {
    for (int j = 0; j < 16; j++) {
      data.ch[j] = i;
    }
    Serial.println(i);
    sbus_tx.data(data);
    sbus_tx.Write();
  }
  

  Serial.println("SBUS Inicializado");
}

void loop() {
  // Smoothly increase and decrease throttle
  if (increasing) {
    value += 10;  // Increase throttle
    if (value >= 1811) increasing = false;  // Reverse at max
  } else {
    value -= 10;  // Decrease throttle
    if (value <= 172) increasing = true;  // Reverse at min
  }

  // Assign values to SBUS channels
  data.ch[0] = 992;   // Roll neutral
  data.ch[1] = 992;   // Pitch neutral
  data.ch[2] = 172;  // Throttle 
  data.ch[3] = 992;   // Yaw neutral
  data.ch[6] = 172; // AUX3 (HORIZON/ANGLE MODE)
  data.ch[8] = value; //AUX5 (ARM/DISARM)

  // Send SBUS data
  sbus_tx.data(data);
  sbus_tx.Write();

  // Print current throttle value
  Serial.print("Value AUX 3: ");
  Serial.println(value);

  // SBUS runs at ~66Hz (~14ms per packet)
  delay(14);
}
