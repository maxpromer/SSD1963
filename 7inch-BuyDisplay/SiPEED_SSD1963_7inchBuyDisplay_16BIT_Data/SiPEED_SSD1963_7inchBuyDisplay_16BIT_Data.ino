/* .:: 6800 interface ::.
 * Wireing
 *   D0     <-> 16
 *   D1     <-> 17
 *   D2     <-> 18
 *   D3     <-> 19
 *   D4     <-> 20
 *   D5     <-> 21
 *   D6     <-> 22
 *   D7     <-> 23
 *   D8     <-> 24
 *   D9     <-> 25
 *   D10    <-> 26
 *   D11    <-> 27
 *   D12    <-> 28
 *   D13    <-> 29
 *   D14    <-> 30
 *   D15    <-> 31
 *   RS     <-> 32
 *   E      <-> 33
 *   R/W    <-> GND
 *   CS     <-> GND
 *   /RESET <-> RST
 *   VDD    <-> 3V3
 *   VSS    <-> GND
 */

#define SET_LCD_E_HIGH() *gpiohs->output_val.u32 |= (1<<17);
#define SET_LCD_E_LOW()  *gpiohs->output_val.u32 &= ~(1<<17);

#define SET_LCD_RS_DATA()    *gpiohs->output_val.u32 |= (1<<16);
#define SET_LCD_RS_COMMAND() *gpiohs->output_val.u32 &= ~(1<<16);

#define SET_LCD_DATA(a) *gpiohs->output_val.u32=(*gpiohs->output_val.u32&0xFFFF0000)|((a)&0xFFFF)

#define SET_LCD_WRITE() { SET_LCD_E_HIGH(); \
                          SET_LCD_E_LOW(); }
                          
#define SET_LCD_WRITE_SLOW() { usleep(100); \
                               SET_LCD_E_HIGH(); \
                               usleep(100); \
                               SET_LCD_E_LOW(); }                   

// Config LCD
#define LCD_WIDTH  800
#define LCD_HEIGHT 480

void write_command(uint8_t c) {
  SET_LCD_RS_COMMAND();

  SET_LCD_DATA(c);
  SET_LCD_WRITE_SLOW();
}

void Write_Data_Register(uint8_t d) {
  SET_LCD_RS_DATA();

  SET_LCD_DATA(d);
  SET_LCD_WRITE_SLOW();
}

void Write_Data_Color(uint16_t color) {
  SET_LCD_RS_DATA();

  SET_LCD_DATA(color);
  SET_LCD_WRITE();
}

/*
void LCD_clear(uint16_t i) {
  LCD_SetPos(0, LCD_WIDTH - 1, 0, LCD_HEIGHT - 1);
  for(uint16_t h=0;h<LCD_HEIGHT;h++) {
    for(uint16_t w=0;w<LCD_WIDTH;w++) {
      Write_Data_Color(i);
    }
  }
}
*/

void LCD_clear(uint16_t i) {
  LCD_SetPos(0, LCD_WIDTH - 1, 0, LCD_HEIGHT - 1);

  SET_LCD_RS_DATA();
  SET_LCD_DATA(i);
  for(uint16_t h=0;h<LCD_HEIGHT;h++) {
    for(uint16_t w=0;w<LCD_WIDTH;w++) {
      SET_LCD_WRITE();
    }
  }
}

void LCD_SetPos(uint16_t xs, uint16_t xe, uint16_t ys, uint16_t ye) {
  write_command(0x002A);  
  Write_Data_Register(xs>>8);      
  Write_Data_Register(xs&0x00ff);
  Write_Data_Register(xe>>8);      
  Write_Data_Register(xe&0x00ff);
  
  write_command(0x002B);  
  Write_Data_Register(ys>>8);      
  Write_Data_Register(ys&0x00ff);
  Write_Data_Register(ye>>8);      
  Write_Data_Register(ye&0x00ff);
  
  write_command(0x002C); 
}


void LCD_Initial() {
  // Data bus
  fpioa_set_function(16, FUNC_GPIOHS0);
  fpioa_set_function(17, FUNC_GPIOHS1);
  fpioa_set_function(18, FUNC_GPIOHS2);
  fpioa_set_function(19, FUNC_GPIOHS3);
  fpioa_set_function(20, FUNC_GPIOHS4);
  fpioa_set_function(21, FUNC_GPIOHS5);
  fpioa_set_function(22, FUNC_GPIOHS6);
  fpioa_set_function(23, FUNC_GPIOHS7);
  fpioa_set_function(24, FUNC_GPIOHS8);
  fpioa_set_function(25, FUNC_GPIOHS9);
  fpioa_set_function(26, FUNC_GPIOHS10);
  fpioa_set_function(27, FUNC_GPIOHS11);
  fpioa_set_function(28, FUNC_GPIOHS12);
  fpioa_set_function(29, FUNC_GPIOHS13);
  fpioa_set_function(30, FUNC_GPIOHS14);
  fpioa_set_function(31, FUNC_GPIOHS15);
  gpiohs_set_drive_mode(0, GPIO_DM_OUTPUT);
  gpiohs_set_drive_mode(1, GPIO_DM_OUTPUT);
  gpiohs_set_drive_mode(2, GPIO_DM_OUTPUT);
  gpiohs_set_drive_mode(3, GPIO_DM_OUTPUT);
  gpiohs_set_drive_mode(4, GPIO_DM_OUTPUT);
  gpiohs_set_drive_mode(5, GPIO_DM_OUTPUT);
  gpiohs_set_drive_mode(6, GPIO_DM_OUTPUT);
  gpiohs_set_drive_mode(7, GPIO_DM_OUTPUT);
  gpiohs_set_drive_mode(8, GPIO_DM_OUTPUT);
  gpiohs_set_drive_mode(9, GPIO_DM_OUTPUT);
  gpiohs_set_drive_mode(10, GPIO_DM_OUTPUT);
  gpiohs_set_drive_mode(11, GPIO_DM_OUTPUT);
  gpiohs_set_drive_mode(12, GPIO_DM_OUTPUT);
  gpiohs_set_drive_mode(13, GPIO_DM_OUTPUT);
  gpiohs_set_drive_mode(14, GPIO_DM_OUTPUT);
  gpiohs_set_drive_mode(15, GPIO_DM_OUTPUT);

  // Control bus
  fpioa_set_function(32, FUNC_GPIOHS16); // RS
  fpioa_set_function(33, FUNC_GPIOHS17); // E
  gpiohs_set_drive_mode(16, GPIO_DM_OUTPUT);
  gpiohs_set_drive_mode(17, GPIO_DM_OUTPUT);

  SET_LCD_E_LOW();
  
  // SET_LCD_RESET();
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
  Write_Data_Register(799>>8);  //Set HDP
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

  // Backlight
  write_command(0x00B8);
  Write_Data_Register(0x000f);    //GPIO is controlled by host GPIO[3:0]=output   GPIO[0]=1  LCD ON  GPIO[0]=1  LCD OFF 
  Write_Data_Register(0x0001);    //GPIO0 normal

  write_command(0x00BA);
  Write_Data_Register(0x0001);    //GPIO[0] out 1 --- LCD display on/off control PIN

  write_command(0x0036); //rotation
  Write_Data_Register(0x0008);//RGB=BGR
  
  write_command(0x00F0); //Pixel Data Interface Format
  Write_Data_Register(0x0003);//16-bit(565 format) data 
  // Write_Data_Register(0x0000);//8-bit data 
  delay(5);
  write_command(0x0029); //display on

  

  write_command(0x00BE); //set PWM for B/L
  Write_Data_Register(0x0006);
  Write_Data_Register(0x0080);
  Write_Data_Register(0x0001);
  Write_Data_Register(0x00f0);
  Write_Data_Register(0x0000);
  Write_Data_Register(0x0000);

  write_command(0x00d0); 
  Write_Data_Register(0x000d);
}


void setup() {
  LCD_Initial();
}

void loop() {
  LCD_clear(0xF800);
  delay(500);
  LCD_clear(0x07E0);
  delay(500);
  LCD_clear(0x001F);
  delay(500);
}
