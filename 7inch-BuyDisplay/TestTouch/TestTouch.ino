#include <Wire.h>

#define FT5216_ADDR 0x38

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Wire.begin((uint8_t)35, (uint8_t)34, (uint32_t)400E3);
  Serial.println("Capacitive touchscreen started");
}

void loop() {
  Wire.beginTransmission(FT5216_ADDR);
  Wire.write(0x02); // Set point to TD_STATUS 
  if (Wire.endTransmission(false) != 0) {
    Serial.println("Write error !");
    delay(1000);
    return;
  }
  uint8_t count = Wire.requestFrom(FT5216_ADDR, 5, true);
  if (Wire.available() <= 0) {
    Serial.println(String("Read error !, ") + Wire.available());
    delay(1000);
    return;
  }

  // Process Data
  uint8_t TD_STATUS = Wire.read();
  uint8_t TOUCH1_XH = Wire.read();
  uint8_t TOUCH1_XL = Wire.read();
  uint8_t TOUCH1_YH = Wire.read();
  uint8_t TOUCH1_YL = Wire.read();

  if ((TD_STATUS&0x0F) > 0) {
    uint16_t cx = (((uint16_t)TOUCH1_XH&0x0F)<<8)|TOUCH1_XL;
    uint16_t cy = (((uint16_t)TOUCH1_YH&0x0F)<<8)|TOUCH1_YL;
    Serial.println(String("Touch: ") + cx + ", " + cy);
  }
  delay(100);
}
