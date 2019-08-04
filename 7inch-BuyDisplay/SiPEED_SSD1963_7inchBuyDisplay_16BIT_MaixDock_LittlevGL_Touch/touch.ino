
#define FT5216_ADDR 0x38

void touch_init() {
  Wire.begin((uint8_t)35, (uint8_t)34, (uint32_t)400E3);
}

int touch_read(uint16_t *cx, uint16_t *cy) {
  Wire.beginTransmission(FT5216_ADDR);
  Wire.write(0x02); // Set point to TD_STATUS 
  if (Wire.endTransmission(false) != 0) {
    Serial.println("Write error !");
    return 0;
  }

  uint8_t count = Wire.requestFrom(FT5216_ADDR, 5);
  if (count != 5) {
    Serial.println("Read error !");
    return 0;
  }

  // Process Data
  uint8_t TD_STATUS = Wire.read();
  uint8_t TOUCH1_XH = Wire.read();
  uint8_t TOUCH1_XL = Wire.read();
  uint8_t TOUCH1_YH = Wire.read();
  uint8_t TOUCH1_YL = Wire.read();

  *cx = (((uint16_t)TOUCH1_XH&0x0F)<<8)|TOUCH1_XL;
  *cy = (((uint16_t)TOUCH1_YH&0x0F)<<8)|TOUCH1_YL;

  return TD_STATUS&0x0F;
}
