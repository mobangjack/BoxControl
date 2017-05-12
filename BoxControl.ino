#include <IRremote.h>
#include <SoftwareSerial.h>
#include <Servo.h>

const uint8_t SERVO_CMD_LOOSE = 0x68; // Key : 0
const uint8_t SERVO_CMD_TIGHT = 0x30; // Key : 1

const uint32_t SERVO_ANGLE_LOOSE = 0;
const uint32_t SERVO_ANGLE_TIGHT = 100;

#define LED_PIN LED_BUILTIN
#define IR_RX_PIN 3
#define BT_RX_PIN 10
#define BT_TX_PIN 11
#define SERVO_PIN 9

IRrecv ir(IR_RX_PIN, LED_PIN);
SoftwareSerial bt(BT_RX_PIN, BT_TX_PIN); // RX, TX
Servo servo;

#define SERVO_ATTACH() servo.attach(SERVO_PIN)
#define SERVO_TIGHT() servo.write(SERVO_ANGLE_LOOSE)
#define SERVO_LOOSE() servo.write(SERVO_ANGLE_TIGHT)
#define LED_ON() digitalWrite(LED_PIN, HIGH)
#define LED_OFF() digitalWrite(LED_PIN, LOW)
#define LED_TOG() digitalWrite(LED_PIN, !digitalRead(LED_PIN))

decode_results results;
uint8_t irKeyCode = 0;
uint8_t btKeyCode = 0;
uint8_t spKeyCode = 0;

void setup()
{
  SERVO_ATTACH();
  SERVO_LOOSE();
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
  bt.begin(9600);
  ir.enableIRIn(); // Start the IR receiver
}

void switchStateCmd(uint8_t cmd)
{
  if (cmd == SERVO_CMD_LOOSE) {
    SERVO_LOOSE();
  } else if (cmd == SERVO_CMD_TIGHT) {
    SERVO_TIGHT();
  }
}

void blinkLed() {
  static uint32_t count = 0;
  if (++count == 10000) {
    count = 0;
    LED_TOG();
  }
}

void loop() {
  if (ir.decode(&results)) {
    if (results.bits == 32) {
      irKeyCode = (results.value >> 8) & 0xFF;
      if (irKeyCode + ((uint8_t)(results.value & 0xFF)) == 0xFF) {
        switchStateCmd(irKeyCode);
      }
    }
    ir.resume(); // Receive the next value
  }
  if (bt.available()) {
    btKeyCode = bt.read();
    switchStateCmd(btKeyCode);
  }
  if (Serial.available()) {
    spKeyCode = Serial.read();
    switchStateCmd(spKeyCode);
  }
  blinkLed();
}
