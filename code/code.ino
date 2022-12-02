#define DEBUG

//Alpine remote signal raw data collected using IRreceive_dump and an IR receiver module
#define A_MUTE       0xE9167286
#define A_VOLDOWN    0xEA157286
#define A_VOLUP      0xEB147286
#define A_PHONEUP    0xA9567286
#define A_PHONEDOWN  0xA35C7286
#define A_TRACKPREV  0xED127286
#define A_TRACKNEXT  0xED137286
#define A_VOICE      0xAE517286
#define A_SOURCE     0xF50A7286

//NEC IR timing info found within the IRremote library
#define NEC_UNIT                560
#define NEC_HEADER_MARK         (16 * NEC_UNIT) // 9000
#define NEC_HEADER_SPACE        (8 * NEC_UNIT)  // 4500
#define NEC_BIT_MARK            NEC_UNIT
#define NEC_ONE_SPACE           (3 * NEC_UNIT)  // 1690
#define NEC_ZERO_SPACE          NEC_UNIT
#define NEC_REPEAT_HEADER_SPACE (4 * NEC_UNIT)  // 2250

// Pin Definitions
#define SW_IN_PIN  A0
#define HU_OUT_PIN 3

// Steering Wheel Audio Control Values
//int      resistor_ladder[6] = {2496, 880, 534, 316, 176, 86};
int      resistor_ladder[6] = {2459, 850, 510, 285, 144, 55};
uint32_t hu_cmd_map_sht[5]  = {A_TRACKNEXT, A_TRACKPREV, A_VOLUP, A_VOLDOWN, A_VOICE};
uint32_t hu_cmd_map_lng[5]  = {A_PHONEDOWN, A_PHONEUP,   A_MUTE,  A_MUTE,    A_SOURCE};
int      divider_resistance = 364;

// Button Press State Variables
int button_i = 0;
int button_p = 0;
int button_t = 0;

void setup() {
  Serial.begin(9600);
  pinMode(HU_OUT_PIN, OUTPUT);
  digitalWrite(HU_OUT_PIN, LOW);
}

void loop() {
  if(btnScan()) {
    #if defined(DEBUG)
    Serial.print("Pressed Button: ");
    Serial.print(resistor_ladder[button_p]);
    Serial.print(" for ");
    Serial.print(button_t);
    Serial.println("ms");
    #endif

    // Long/Short Button Press
    if(button_t >= 750) {
      sendRemoteCommand(hu_cmd_map_lng[button_p - 1]);
    } else {
      sendRemoteCommand(hu_cmd_map_sht[button_p - 1]);
    }
  }

  delay(10);
}

bool btnScan() {
  // Average ADC Reading
  int btn = 0;

  for(byte i = 0; i < 5; i++) {
    btn += analogRead(SW_IN_PIN);
    delay(1);
  }

  btn /= 5;

  // Calculate Resistance Value from ADC
  int r1 = ((divider_resistance / ((btn / 1024.0) * 5)) * 5) - divider_resistance;

  // Match Resistance Value to Resistance Ladder
  for(byte i = 0; i < sizeof(resistor_ladder) / sizeof(int); i++) {
    int hval = (i == 0) ? resistor_ladder[0] * 2 : resistor_ladder[i] + ((resistor_ladder[i - 1] - resistor_ladder[i]) / 2);
    int lval = (i == (sizeof(resistor_ladder) / sizeof(int))  - 1) ? 0 : resistor_ladder[i] - ((resistor_ladder[i] - resistor_ladder[i + 1]) / 2);

    if(r1 >= lval && r1 < hval) {
      if(button_i != i) {
        if(i > 0) {
          button_t = millis();
        } else {
          button_t = millis() - button_t;
        }

        button_p = button_i;
        button_i = i;

        if(i == 0) {
          // Button Pressed and Recognised
          return true;
        }
      }
    }
  }

  return false;
}

void sendRemoteCommand(uint32_t data) {
  // Send NEC style 32 bits (2x address bytes + 2x command bytes) using NEC timing to a pin.

  #if defined(DEBUG)
  uint32_t send_timer = millis();
  #endif

  uint32_t mask = 0x00000001;

  // Send NEC header burst
  digitalWrite(HU_OUT_PIN, HIGH);
  delayMicroseconds(NEC_HEADER_MARK);
  digitalWrite(HU_OUT_PIN, LOW);

  // Send NEC header space
  delayMicroseconds(NEC_HEADER_SPACE);

  // Send the data LSB first
  for(int i = 0; i < 32; i++) {
    if((mask << i) & data) { // bitwise AND operation reveals each bit
      // Send a '1'
      digitalWrite(HU_OUT_PIN, HIGH);
      delayMicroseconds(NEC_BIT_MARK);
      digitalWrite(HU_OUT_PIN, LOW);
      delayMicroseconds(NEC_ONE_SPACE);
    } else {
      // Send a '0'
      digitalWrite(HU_OUT_PIN, HIGH);
      delayMicroseconds(NEC_BIT_MARK);
      digitalWrite(HU_OUT_PIN, LOW);
      delayMicroseconds(NEC_ZERO_SPACE);
    }
  }

  // Send NEC stop bit
  digitalWrite(HU_OUT_PIN, HIGH);
  delayMicroseconds(NEC_BIT_MARK);
  digitalWrite(HU_OUT_PIN, LOW);
  delayMicroseconds(NEC_ZERO_SPACE);

  #if defined(DEBUG)
  Serial.print("Command 0x");
  Serial.print(data, HEX);
  Serial.print(" Executed in: ");
  Serial.print(millis() - send_timer);
  Serial.println("ms");
  #endif
}
