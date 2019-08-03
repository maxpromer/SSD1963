#include "gpio.h"
#include "fpioa.h"
#include "./kendryte-standalone-sdk/lib/drivers/include/spi.h"

// Pin Control
#define SSD1963_RS     38    // Data or Command
#define SSD1963_WR     39    // Write
// #define SSD1963_RD     34    // Read
#define SSD1963_CS     36    // Chip select
#define SSD1963_RESET  37    // Reset

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

#define SSD1963_DMA_CH DMAC_CHANNEL3
#define SSD1963_SPI_CH SPI_DEVICE_0

#define SET_LCD_WRITE() { SET_LCD_WR_NON_ACTIVE(); \
                          SET_LCD_WR_ACTIVE(); }

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
  SET_LCD_RS_COMMAND();

  /*
  SET_LCD_DATA(c);
  SET_LCD_WRITE();
  */
  spi_init(SSD1963_SPI_CH, SPI_WORK_MODE_0, SPI_FF_OCTAL, 8, 0);
  spi_init_non_standard(SPI_DEVICE_0, 8, 0, 0, SPI_AITM_AS_FRAME_FORMAT);
  spi_send_data_normal_dma(SSD1963_DMA_CH, SSD1963_SPI_CH, SPI_CHIP_SELECT_0, &c, 1, SPI_TRANS_CHAR);

  dmac_wait_done(SSD1963_DMA_CH);
}

void Write_Data_Register(uint8_t d) {
  SET_LCD_RS_DATA();

  /*
  SET_LCD_DATA(d);
  SET_LCD_WRITE();
  */
  spi_init(SSD1963_SPI_CH, SPI_WORK_MODE_0, SPI_FF_OCTAL, 8, 0);
  spi_init_non_standard(SPI_DEVICE_0, 8, 0, 0, SPI_AITM_AS_FRAME_FORMAT);
  spi_send_data_normal_dma(SSD1963_DMA_CH, SSD1963_SPI_CH, SPI_CHIP_SELECT_0, &d, 1, SPI_TRANS_CHAR);

  dmac_wait_done(SSD1963_DMA_CH);
}

uint8_t dataBuffer[3];

void Write_Data_Color(uint32_t d) {
  SET_LCD_RS_DATA();

  dataBuffer[0] = (d>>16)&0xFF;
  dataBuffer[1] = (d>>8)&0xFF;
  dataBuffer[2] = d&0xFF;

  spi_init(SSD1963_SPI_CH, SPI_WORK_MODE_0, SPI_FF_OCTAL, 8, 0);
  spi_init_non_standard(SPI_DEVICE_0, 8, 0, 0, SPI_AITM_AS_FRAME_FORMAT);
  spi_send_data_normal_dma(SSD1963_DMA_CH, SSD1963_SPI_CH, SPI_CHIP_SELECT_0, dataBuffer, 3, SPI_TRANS_CHAR);
  
  dmac_wait_done(SSD1963_DMA_CH);
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

  fpioa_set_function(SSD1963_CS, (fpioa_function_t)FUNC_SPI0_SS0);
  fpioa_set_function(SSD1963_WR, (fpioa_function_t)FUNC_SPI0_SCLK);
  sysctl_set_spi0_dvp_data(0);

  dmac_init();
  
  // SPI init
  spi_init(SSD1963_SPI_CH, SPI_WORK_MODE_0, SPI_FF_OCTAL, 8, 0);
  spi_set_clk_rate(SSD1963_SPI_CH, 200E3);
  
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

  Write_Data_Register(0x0000); // C & P
  Write_Data_Register(0x00FF); // C & P
  Write_Data_Register(0x00BE); // C & P


  write_command(0x00B0);  //LCD SPECIFICATION
  Write_Data_Register(0x0020);
  Write_Data_Register(0x0000);
  Write_Data_Register((HDP>>8)&0X00FF);  //Set HDP
  Write_Data_Register(HDP&0X00FF);
  Write_Data_Register((VDP>>8)&0X00FF);  //Set VDP
  Write_Data_Register(VDP&0X00FF);
  Write_Data_Register(0x0000);
  delay(5);
  write_command(0x00B4);  //HSYNC
  Write_Data_Register((HT>>8)&0X00FF);  //Set HT
  Write_Data_Register(HT&0X00FF);
  Write_Data_Register((HPS>>8)&0X00FF);  //Set HPS
  Write_Data_Register(HPS&0X00FF);
  Write_Data_Register(HPW);         //Set HPW
  Write_Data_Register((LPS>>8)&0X00FF);  //SetLPS
  Write_Data_Register(LPS&0X00FF);
  Write_Data_Register(0x0000);

  write_command(0x00B6);  //VSYNC
  Write_Data_Register((VT>>8)&0X00FF);   //Set VT
  Write_Data_Register(VT&0X00FF);
  Write_Data_Register((VPS>>8)&0X00FF);  //Set VPS
  Write_Data_Register(VPS&0X00FF);
  Write_Data_Register(VPW);         //Set VPW
  Write_Data_Register((FPS>>8)&0X00FF);  //Set FPS
  Write_Data_Register(FPS&0X00FF);

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
  pinMode(SSD1963_RS, OUTPUT); // RS
  pinMode(SSD1963_WR, OUTPUT); // WR
  // pinMode(SSD1963_RD, OUTPUT); // RD
  pinMode(SSD1963_CS, OUTPUT); // CS
  pinMode(SSD1963_RESET, OUTPUT); // RESET

  digitalWrite(SSD1963_CS, 0);
  digitalWrite(SSD1963_RESET, 0);
  digitalWrite(SSD1963_RS, 0);
  digitalWrite(SSD1963_WR, 1);
  // digitalWrite(SSD1963_RD, 1);
  
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
