#include <IRremote.hpp>

int RECV_PIN = 11;
bool isOn = false;

void setup() {
  Serial.begin(9600);
  IrReceiver.begin(RECV_PIN, DISABLE_LED_FEEDBACK);
}

void loop() {
  if (IrReceiver.decode()) {
    uint32_t receivedCode = IrReceiver.decodedIRData.decodedRawData;
    if (receivedCode == 0xBA44FF00 || receivedCode == 0xE817BF40) {
      isOn = !isOn;
    }
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
    IrReceiver.printIRResultShort(&Serial);
    IrReceiver.resume();
  }
}