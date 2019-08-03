// Pin Control
#define SSD1963_RS     32    // Data or Command
#define SSD1963_WR     33    // Write
#define SSD1963_RD     34    // Read
#define SSD1963_CS     35    // Chip select
#define SSD1963_RESET  36    // Reset

#define SET_LCD_RESET() { digitalWrite(SSD1963_RESET, 1); \
                          delay(50); \
                          digitalWrite(SSD1963_RESET, 0); \
                          delay(100); \
                          digitalWrite(SSD1963_RESET, 1); \
                          delay(100); }


#define SET_LCD_CS(a) digitalWrite(SSD1963_CS, a)
#define SET_LCD_RS(a) digitalWrite(SSD1963_RS, a)
#define SET_LCD_WR(a) digitalWrite(SSD1963_WR, a)

#define SET_LCD_WR_ACTIVE() *gpiohs->output_val.u32 |= (1<<17);
#define SET_LCD_WR_NON_ACTIVE() *gpiohs->output_val.u32 &= ~(1<<17);

#define SET_LCD_RS_DATA() *gpiohs->output_val.u32 |= (1<<16);
#define SET_LCD_RS_COMMAND() *gpiohs->output_val.u32 &= ~(1<<16);

#define SET_LCD_DATA(a) *gpiohs->output_val.u32=(*gpiohs->output_val.u32&0xFFFFFF00)|(uint32_t)(a)

int nsleep(uint64_t nsec) {
    uint64_t cycle = read_cycle();
    uint64_t nop_all = nsec * 4;
    while (1)
    {
        if(read_cycle() - cycle >= nop_all)
            break;
    }
    return 0;
}

#define SET_LCD_WRITE() { SET_LCD_WR_NON_ACTIVE(); \
                          SET_LCD_WR_ACTIVE(); }

// Config LCD
#define LCD_WIDTH  480
#define LCD_HEIGHT 272

void write_command(uint8_t c) {
  SET_LCD_RS_COMMAND();

  SET_LCD_DATA(c);
  SET_LCD_WRITE();
}

void Write_Data_Register(uint8_t d) {
  SET_LCD_RS_DATA();

  SET_LCD_DATA(d);
  SET_LCD_WRITE();
}

void Write_Data_Color(uint32_t d) {
  SET_LCD_RS_DATA();

  SET_LCD_DATA((d>>16)&0xFF);
  SET_LCD_WRITE();

  SET_LCD_DATA((d>>8)&0xFF);
  SET_LCD_WRITE();

  SET_LCD_DATA(d&0xFF);
  SET_LCD_WRITE();
}

void LCD_clear(uint32_t i) {
  LCD_SetPos(0, LCD_WIDTH - 1, 0, LCD_HEIGHT - 1);
  for(int h=0;h<LCD_HEIGHT;h++) {
    for(int w=0;w<LCD_WIDTH;w++) {
      Write_Data_Color(i);
    }
  }
}

/*
void LCD_clear(uint16_t i) {
  LCD_SetPos(0, LCD_WIDTH - 1, 0, LCD_HEIGHT - 1);

  SET_LCD_RS_DATA();

  // Set data output
  SET_LCD_DATA(i);

  for(int h=0;h<LCD_HEIGHT;h++) {
    for(int w=0;w<LCD_WIDTH;w++) {
      // Write to LCD
      SET_LCD_WRITE();
    }
  }
}
*/

void LCD_SetPos(uint16_t xs, uint16_t xe, uint16_t ys, uint16_t ye) {
  write_command(0x002A);  
  Write_Data_Register(xs>>8);      
  Write_Data_Register(xs&0x00ff);
  Write_Data_Register(xe>>8);      
  Write_Data_Register(xe&0x00ff);
  
  write_command(0x002b);  
  Write_Data_Register(ys>>8);      
  Write_Data_Register(ys&0x00ff);
  Write_Data_Register(ye>>8);      
  Write_Data_Register(ye&0x00ff);
  write_command(0x002c); 
}


void SSD1963_Initial() {
  SET_LCD_RESET();
  delay(1);
  
  write_command(0x00E2);   //PLL multiplier, set PLL clock to 120M
  Write_Data_Register(0x0023);  //N=0x36 for 6.5M, 0x23 for 10M crystal
  Write_Data_Register(0x0002);
  Write_Data_Register(0x0004);
  write_command(0x00E0);   // PLL enable
  Write_Data_Register(0x0001);
  delay(1);
  write_command(0x00E0);
  Write_Data_Register(0x0003);   // now, use PLL output as system clock
  delay(5);
  write_command(0x0001);  // software reset
  delay(5);
  write_command(0x00E6);  //PLL setting for PCLK, depends on resolution
  Write_Data_Register(0x0003);
  Write_Data_Register(0x0033);
  Write_Data_Register(0x0033);

  write_command(0x00B0);  //LCD SPECIFICATION
  Write_Data_Register(0x0020); //24 bit TFT panel
  Write_Data_Register(0x0000); //Hsync+Vsync +DE mode  TFT mode
  Write_Data_Register((799>>8));  //Set HDP
  Write_Data_Register(799);
  Write_Data_Register(479>>8);  //Set VDP
  Write_Data_Register(479);
  Write_Data_Register(0x0000);

  write_command(0x00B4);  //HSYNC
  Write_Data_Register(0x04);  //Set HT
  Write_Data_Register(0x1f);
  Write_Data_Register(0x00);  //Set HPS
  Write_Data_Register(0xd2);
  Write_Data_Register(0x00);        //Set HPW
  Write_Data_Register(0x00);  //Set HPS
  Write_Data_Register(0x00);
  Write_Data_Register(0x00);

  write_command(0x00B6);  //VSYNC
  Write_Data_Register(0x02);   //Set VT
  Write_Data_Register(0x0c);
  Write_Data_Register(0x00);  //Set VPS
  Write_Data_Register(0x22);
  Write_Data_Register(0x00);   //Set VPW
  Write_Data_Register(0x00);  //Set FPS
  Write_Data_Register(0x00);

  write_command(0x0036); //rotation
  Write_Data_Register(0x0008);//RGB=BGR
  
  write_command(0x00F0); //Pixel Data Interface Format
  //Write_Data_int(0x0003);//16-bit(565 format) data 
  Write_Data_Register(0x0000);//8-bit data 
  delay(5);
  write_command(0x0029); //display on
}


void setup() {

  // LCD
  pinMode(16, OUTPUT); // DATA 0
  pinMode(17, OUTPUT); // DATA 1
  pinMode(18, OUTPUT); // DATA 2
  pinMode(19, OUTPUT); // DATA 3
  pinMode(20, OUTPUT); // DATA 4
  pinMode(21, OUTPUT); // DATA 5
  pinMode(22, OUTPUT); // DATA 6
  pinMode(23, OUTPUT); // DATA 7
  pinMode(24, OUTPUT); // DATA 8
  pinMode(25, OUTPUT); // DATA 9
  pinMode(26, OUTPUT); // DATA 10
  pinMode(27, OUTPUT); // DATA 11
  pinMode(28, OUTPUT); // DATA 12
  pinMode(29, OUTPUT); // DATA 13
  pinMode(30, OUTPUT); // DATA 14
  pinMode(31, OUTPUT); // DATA 15
  pinMode(SSD1963_RS, OUTPUT); // RS
  pinMode(SSD1963_WR, OUTPUT); // WR
  pinMode(SSD1963_RD, OUTPUT); // RD
  pinMode(SSD1963_CS, OUTPUT); // CS
  pinMode(SSD1963_RESET, OUTPUT); // RESET

  digitalWrite(SSD1963_CS, 0);
  digitalWrite(SSD1963_RESET, 0);
  digitalWrite(SSD1963_RS, 0);
  digitalWrite(SSD1963_WR, 1);
  digitalWrite(SSD1963_RD, 1);
  
  SSD1963_Initial();
}

void loop() {
  /*
  LCD_clear(0xF800);
  LCD_clear(0x07E0);
  LCD_clear(0x001F);
  */

  LCD_clear(0x000000);
  LCD_clear(0xFF0000);
  LCD_clear(0x00FF00);
  LCD_clear(0x0000FF);
  LCD_clear(0xFFFFFF);
}
