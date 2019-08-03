// Pin Control
#define LCD_CS     PB14    // Chip select
#define LCD_RESET  PB13    // Reset
#define LCD_RS     PC15    // Data or Command
#define LCD_WR     PC14    // Write
#define LCD_RD     PC13    // Read

#define SET_LCD_RESET() { digitalWrite(LCD_RESET, 1); \
                          delay(50); \
                          digitalWrite(LCD_RESET, 0); \
                          delay(100); \
                          digitalWrite(LCD_RESET, 1); \
                          delay(100); }


#define SET_LCD_CS(a) digitalWrite(LCD_CS, a)
#define SET_LCD_RS(a) digitalWrite(LCD_RS, a)
#define SET_LCD_WR(a) digitalWrite(LCD_WR, a)

#define SET_LCD_WR_ACTIVE() GPIOC->regs->ODR |= (1<<14);
#define SET_LCD_WR_NON_ACTIVE() GPIOC->regs->ODR &= ~(1<<14);

#define SET_LCD_RS_DATA() GPIOC->regs->ODR |= (1<<15);
#define SET_LCD_RS_COMMAND() GPIOC->regs->ODR &= ~(1<<15);

#define SET_LCD_HDATA(a) GPIOB->regs->ODR=(GPIOB->regs->ODR&0xFFFFE01F)|((uint32_t)(a)<<5)
#define SET_LCD_LDATA(a) GPIOA->regs->ODR=(GPIOA->regs->ODR&0xFFFFFF00)|a

// Config LCD
#define LCD_WIDTH  480
#define LCD_HEIGHT 272

// I don't know !
unsigned int HDP=479;
unsigned int HT=531;
unsigned int HPS=43;
unsigned int LPS=8;
unsigned char HPW=10;

unsigned int VDP=271;
unsigned int VT=288;
unsigned int VPS=12;
unsigned int FPS=4;
unsigned char VPW=10;

void write_command(uint8_t c) {
  /*
  LCD_cs=0;
  rs=0;
  P3=0x00;
  P0=c;
  wrb=0;
  wrb=1; 
  LCD_cs=1;
  */

  // SET_LCD_CS(0);

  // Set to Command
  // SET_LCD_RS(0);
  SET_LCD_RS_COMMAND();

  // Set data output
  SET_LCD_HDATA(0);
  SET_LCD_LDATA(c);

  // Write to LCD
  /*
  SET_LCD_WR(0);
  SET_LCD_WR(1);
  */
  SET_LCD_WR_NON_ACTIVE();
  SET_LCD_WR_ACTIVE();
  
  // SET_LCD_CS(1);
}

void Write_Data_int(uint16_t d) {
  /*
  LCD_cs=0;
  rs=1;
  P3=d>>8;
  P0=d;
  wrb=0;
  wrb=1;  
  LCD_cs=1;
  */

  // SET_LCD_CS(0);

  // Set to Data
  // SET_LCD_RS(1);
  SET_LCD_RS_DATA();

  // Set data output
  SET_LCD_HDATA(d>>8);
  SET_LCD_LDATA(d);

  // Write to LCD
  /*
  SET_LCD_WR(0);
  SET_LCD_WR(1);
  */
  SET_LCD_WR_NON_ACTIVE();
  SET_LCD_WR_ACTIVE();
  
  // SET_LCD_CS(1);
}

/*
void LCD_clear(unsigned int i) {
  LCD_SetPos(0, LCD_WIDTH - 1, 0, LCD_HEIGHT - 1);
  for(int h=0;h<LCD_HEIGHT;h++) {
    for(int w=0;w<LCD_WIDTH;w++) {
      Write_Data_int(i);
    }
  }
}
*/

void LCD_clear(uint16_t i) {
  LCD_SetPos(0, LCD_WIDTH - 1, 0, LCD_HEIGHT - 1);

  SET_LCD_RS_DATA();

  // Set data output
  SET_LCD_HDATA(i>>8);
  SET_LCD_LDATA(i);

  for(int h=0;h<LCD_HEIGHT;h++) {
    for(int w=0;w<LCD_WIDTH;w++) {
      // Write to LCD
      SET_LCD_WR_NON_ACTIVE();
      SET_LCD_WR_ACTIVE();
    }
  }
}

void LCD_SetPos(uint16_t xs, uint16_t xe, uint16_t ys, uint16_t ye) {
  write_command(0x002A);  
  Write_Data_int(xs>>8);      
  Write_Data_int(xs&0x00ff);
  Write_Data_int(xe>>8);      
  Write_Data_int(xe&0x00ff);
  
  write_command(0x002b);  
  Write_Data_int(ys>>8);      
  Write_Data_int(ys&0x00ff);
  Write_Data_int(ye>>8);      
  Write_Data_int(ye&0x00ff);
  write_command(0x002c); 
}


void SSD1963_Initial() {
  SET_LCD_RESET();
  delay(1);
  
  write_command(0x00E2);   //PLL multiplier, set PLL clock to 120M
  Write_Data_int(0x0023);  //N=0x36 for 6.5M, 0x23 for 10M crystal
  Write_Data_int(0x0002);
  Write_Data_int(0x0004);
  write_command(0x00E0);   // PLL enable
  Write_Data_int(0x0001);
  delay(1);
  write_command(0x00E0);
  Write_Data_int(0x0003);   // now, use PLL output as system clock
  delay(5);
  write_command(0x0001);  // software reset
  delay(5);
  write_command(0x00E6);  //PLL setting for PCLK, depends on resolution

/*  
  Write_Data_int(0x0001);  //OTA5180
  Write_Data_int(0x00ff);  //OTA5180
  Write_Data_int(0x0000);  //OTA5180


  Write_Data_int(0x0001); // HX8257C
  Write_Data_int(0x0033); // HX8257C
  Write_Data_int(0x0033); // HX8257C
*/

  Write_Data_int(0x0000); // C & P
  Write_Data_int(0x00FF); // C & P
  Write_Data_int(0x00BE); // C & P


  write_command(0x00B0);  //LCD SPECIFICATION
  Write_Data_int(0x0020);
  Write_Data_int(0x0000);
  Write_Data_int((HDP>>8)&0X00FF);  //Set HDP
  Write_Data_int(HDP&0X00FF);
  Write_Data_int((VDP>>8)&0X00FF);  //Set VDP
  Write_Data_int(VDP&0X00FF);
  Write_Data_int(0x0000);
  delay(5);
  write_command(0x00B4);  //HSYNC
  Write_Data_int((HT>>8)&0X00FF);  //Set HT
  Write_Data_int(HT&0X00FF);
  Write_Data_int((HPS>>8)&0X00FF);  //Set HPS
  Write_Data_int(HPS&0X00FF);
  Write_Data_int(HPW);         //Set HPW
  Write_Data_int((LPS>>8)&0X00FF);  //SetLPS
  Write_Data_int(LPS&0X00FF);
  Write_Data_int(0x0000);

  write_command(0x00B6);  //VSYNC
  Write_Data_int((VT>>8)&0X00FF);   //Set VT
  Write_Data_int(VT&0X00FF);
  Write_Data_int((VPS>>8)&0X00FF);  //Set VPS
  Write_Data_int(VPS&0X00FF);
  Write_Data_int(VPW);         //Set VPW
  Write_Data_int((FPS>>8)&0X00FF);  //Set FPS
  Write_Data_int(FPS&0X00FF);

  write_command(0x0036); //rotation
  Write_Data_int(0x0008);//RGB=BGR

  /*
  write_command(0x003A); //Set the current pixel format for RGB image data
  Write_Data_int(0x0050);//16-bit/pixel
  */

  write_command(0x00F0); //Pixel Data Interface Format
  Write_Data_int(0x0003);//16-bit(565 format) data 

  /*
  write_command(0x00BC); 
  Write_Data_int(0x0040);//contrast value
  Write_Data_int(0x0080);//brightness value
  Write_Data_int(0x0040);//saturation value
  Write_Data_int(0x0001);//Post Processor Enable
  */

  delay(5);

  write_command(0x0029); //display on

  write_command(0x00BE); //set PWM for B/L
  Write_Data_int(0x0006);
  Write_Data_int(0x00F0);
  Write_Data_int(0x0001);
  Write_Data_int(0x00F0);
  Write_Data_int(0x0000);
  Write_Data_int(0x0000);

  write_command(0x00D0); 
  Write_Data_int(0x000D);

  //----------LCD RESET---GPIO0-------------------//
  write_command(0x00B8);
  Write_Data_int(0x0000);    //GPIO3=input, GPIO[2:0]=output
  Write_Data_int(0x0001);    //GPIO0 normal

  write_command(0x00BA);
  Write_Data_int(0x0000);
}


void setup() {

  // Data [0-7]
  pinMode(PA0, OUTPUT);
  pinMode(PA1, OUTPUT);
  pinMode(PA2, OUTPUT);
  pinMode(PA3, OUTPUT);
  pinMode(PA4, OUTPUT);
  pinMode(PA5, OUTPUT);
  pinMode(PA6, OUTPUT);
  pinMode(PA7, OUTPUT);

  // Data [8-15]
  pinMode(PB5, OUTPUT);
  pinMode(PB6, OUTPUT);
  pinMode(PB7, OUTPUT);
  pinMode(PB8, OUTPUT);
  pinMode(PB9, OUTPUT);
  pinMode(PB10, OUTPUT);
  pinMode(PB11, OUTPUT);
  pinMode(PB12, OUTPUT);

  // Control
  pinMode(LCD_CS, OUTPUT);
  pinMode(LCD_RESET, OUTPUT);
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_WR, OUTPUT);
  pinMode(LCD_RD, OUTPUT);

  digitalWrite(LCD_CS, 0);
  digitalWrite(LCD_RESET, 0);
  digitalWrite(LCD_RS, 0);
  digitalWrite(LCD_WR, 1);
  digitalWrite(LCD_RD, 1);
  
  SSD1963_Initial();
}

void loop() {
  LCD_clear(0xF800);
  LCD_clear(0x07E0);
  LCD_clear(0x001F);
}
