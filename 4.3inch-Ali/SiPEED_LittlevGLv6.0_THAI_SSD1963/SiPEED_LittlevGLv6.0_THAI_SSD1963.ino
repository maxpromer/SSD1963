#include <lvgl.h>
#include <Ticker.h>

// =========== LCD ===========
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

#define SET_LCD_DATA(a) *gpiohs->output_val.u32=(*gpiohs->output_val.u32&0xFFFF0000)|(uint32_t)(a)

#define SET_LCD_WRITE() { SET_LCD_WR_NON_ACTIVE(); \
                          SET_LCD_WR_ACTIVE(); }

// =========== END of LCD ===========

// Config LCD
#define LCD_WIDTH  480
#define LCD_HEIGHT 272

#define LVGL_TICK_PERIOD 20
Ticker tick; /* timer for interrupt handler */

static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  LCD_SetWindow(area->x1, area->y1, area->x2, area->y2); // Set window
  for(uint16_t y=area->y1;y<=area->y2;y++) {
    for(uint16_t x=area->x1;x<=area->x2;x++) {
      Write_Data_int(color_p->full);
      ++color_p;
    }
  }
  lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}

static void lv_tick_handler(void) {
  lv_tick_inc(LVGL_TICK_PERIOD);
}

extern lv_font_t th_sarabun_50;

void setup() {
  Serial.begin(9600);
  Serial.println("Start !");
  
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

  // LCD
  SSD1963_Initial();
  
  // LittlevGL
  lv_init();

  lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);

  /*Initialize the display*/
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = LCD_WIDTH;
  disp_drv.ver_res = LCD_HEIGHT;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);

  /*Initialize the graphics library's tick*/
  tick.attach_ms(LVGL_TICK_PERIOD, lv_tick_handler);

  // Style
  static lv_style_t style1;
  lv_style_copy(&style1, &lv_style_plain);
  style1.text.font = &th_sarabun_50;

  /* Create simple label */
  lv_obj_t *label = lv_label_create(lv_scr_act(), NULL);
  lv_obj_set_style(label, &style1);
  lv_label_set_text(label, "ยินดีต้อนรับสู่ LittlevGL V6.0");
  lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0);
}

void loop() {
  lv_task_handler(); /* let the GUI do its work */
  delay(5);
}

void loop1() {
  LCD_clear(0xF800);
  LCD_clear(0x07E0);
  LCD_clear(0x001F);
}
