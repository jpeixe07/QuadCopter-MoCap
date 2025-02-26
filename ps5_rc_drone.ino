#include <ps5Controller.h>
#include "sbus.h"

#define ARMED 1800
#define DISARMED 181 
#define FLIGHTMODE 181 //AUX5 LOW(172) = AIRMODE // HIGH(1803) = ANGLE
#define LOW_SBUS 181
#define MID_SBUS 992
#define HIGH_SBUS 1803
#define LOW_PS5 -127
#define HIGH_PS5 128

bfs::SbusTx sbus_tx(&Serial1, 16, 17, true, false);
bfs::SbusData data;

bool armed = false;
float sbusFrequency = 50.0;
int throttlePs5 = 0;
int throttleValue = LOW_SBUS;
int pitchValue = MID_SBUS;
int yawValue = MID_SBUS;
int rollValue = MID_SBUS;
bool lastR1State = false;
unsigned long lastR1PressTime = 0;
const unsigned long debounceDelay = 200; // Mininum time between change (ms)

unsigned long lastSbusSend = micros();

void setup() {
  Serial.begin(921600);
  ps5.begin("88:03:4c:33:b2:f7"); //replace with MAC address of your controller
  Serial.println("PS5 Ready.");

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

  lastSbusSend = micros();

}

void loop() {

  if (ps5.isConnected() == true) {

    // Press L1 for Failsafe trigger
    data.failsafe = ps5.L1();

    bool currentR1State = ps5.R1();
    if (currentR1State && !lastR1State) {
      if (millis() - lastR1PressTime > debounceDelay) {
        armed = !armed;
        data.ch[8] = armed ? ARMED : DISARMED;
        Serial.println(armed ? "Drone Armed" : "Drone Disarmed");
        lastR1PressTime = millis();
      }
    }
    lastR1State = currentR1State;

      // Map the value to Yaw PWM SBUS
      yawValue = map(ps5.LStickX(), LOW_PS5, HIGH_PS5, LOW_SBUS, HIGH_SBUS);
      // Map the value to Throttle PWM SBUS (Only the upper half of the button)
      int throttlePs5 = ps5.LStickY();
      if (throttlePs5 <= 0) {
        throttleValue = LOW_SBUS; // Garante que a metade inferior do stick seja ignorada
      } else {
        throttleValue = map(throttlePs5, 0, HIGH_PS5, LOW_SBUS, HIGH_SBUS);
      }
      pitchValue = map(ps5.RStickY(), LOW_PS5, HIGH_PS5, LOW_SBUS, HIGH_SBUS);
      rollValue = map(ps5.RStickX(), LOW_PS5, HIGH_PS5, LOW_SBUS, HIGH_SBUS);

    data.ch[0] = rollValue;
    data.ch[1] = pitchValue;
    data.ch[2] = throttleValue;
    data.ch[3] = yawValue;
    data.ch[6] = FLIGHTMODE;
    data.failsafe = false;
  }
  else{
    //If ps5 is not connected : Disarm && Failsafe && send low/neutral values
    data.ch[0] = 992;   // Roll neutral
    data.ch[1] = 992;   // Pitch neutral
    data.ch[2] = 172;  // Low Throttle 
    data.ch[3] = 992;   // Yaw neutral
    data.ch[6] = FLIGHTMODE; // AUX3 (HORIZON/ANGLE MODE)
    data.ch[8] = DISARMED; //AUX5 (ARM/DISARM)
    data.failsafe = true; //Failsafe True
  }
  if (micros() - lastSbusSend > 1e6 / sbusFrequency) {
      lastSbusSend = micros();
      sbus_tx.data(data);
      sbus_tx.Write();
      Serial.println("Sending SBUS msg");
  }
}
