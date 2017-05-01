// additional helpers migrated from the dreamster/arduino_lib repo

char debug_string_buffer[20];
// sprintf + serial of 20 bytes takes ~200us
// sprintf + serial of 10 bytes takes ~144us
// sprintf + serial of  5 bytes takes ~108us
#define debug(formato, valor) \
  sprintf(debug_string_buffer, formato, valor); \
  Serial.print(debug_string_buffer)

class Leds {
 public:
  Leds() {
    pinMode(kPinLedBlue, OUTPUT);
    pinMode(kPinLedRed, OUTPUT);
    pinMode(kPinLedGreen, OUTPUT);
  }
  
  void blueOn() {
    digitalWrite(kPinLedBlue, HIGH);
  }
  void blueOff() {
    digitalWrite(kPinLedBlue, LOW);
  }
  void blueToggle() {
    digitalWrite(kPinLedBlue, (digitalRead(kPinLedBlue) == 0 ? HIGH : LOW));
  }
  
  void redOn() {
    digitalWrite(kPinLedRed, HIGH);
  }
  void redOff() {
    digitalWrite(kPinLedRed, LOW);
  }
  void redToggle() {
    digitalWrite(kPinLedRed, (digitalRead(kPinLedRed) == 0 ? HIGH : LOW));
  }
  
  void greenOn() {
    digitalWrite(kPinLedGreen, HIGH);
  }
  void greenOff() {
    digitalWrite(kPinLedGreen, LOW);
  }
  void greenToggle() {
    digitalWrite(kPinLedGreen, (digitalRead(kPinLedGreen) == 0 ? HIGH : LOW));
  }
  
 private:
  // pins
  static const int kPinLedBlue = 13;
  static const int kPinLedRed = 12;
  static const int kPinLedGreen = 11;

};