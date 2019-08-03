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

#include <lvgl.h>

/* Display flushing */
void disp_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t *color_p) {
  // Set window
  LCD_SetPos(x1, y1, (x2 - x1 + 1), (y2 - y1 + 1)); /* set the working window */

  SET_LCD_RS_DATA();

  for (int y = y1; y <= y2; y++) {
    for (int x = x1; x <= x2; x++) {
      
      // ====== Send to LCD ======
      
      // Set data output
      SET_LCD_HDATA(color_p->full>>8);
      SET_LCD_LDATA(color_p->full);

      // Write to LCD
      SET_LCD_WR_NON_ACTIVE();
      SET_LCD_WR_ACTIVE();

      // ====== END of LCD ======
      
      color_p++;
    }
  }
  lv_flush_ready(); /* tell lvgl that flushing is done */
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


  // LittlevGL
  lv_init();

  /*Initialize the display*/
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.disp_flush = disp_flush;
  lv_disp_drv_register(&disp_drv);

  /* Create simple label */
  lv_obj_t *label = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(label, "Hello Arduino!");
  lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0);
}

void loop() {
  lv_task_handler(); /* let the GUI do its work */
  delay(5);
}
