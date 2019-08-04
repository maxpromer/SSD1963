#include <lvgl.h>
#include <Ticker.h>
#include <Wire.h>

// =========== LCD ===========
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
 *   SCL    <-> 34
 *   SDA    <-> 35
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

// =========== END of LCD ===========

#define LVGL_TICK_PERIOD 20
Ticker tick; /* timer for interrupt handler */

static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  LCD_SetWindow(area->x1, area->y1, area->x2, area->y2); // Set window

  SET_LCD_RS_DATA();
  for(uint16_t y=area->y1;y<=area->y2;y++) {
    for(uint16_t x=area->x1;x<=area->x2;x++) {
      // Set data output
      SET_LCD_DATA(color_p->full);
      SET_LCD_WRITE();
      ++color_p;
    }
  }
  lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}

static void lv_tick_handler(void) {
  lv_tick_inc(LVGL_TICK_PERIOD);
}

static bool touch_pointer(lv_indev_drv_t * drv, lv_indev_data_t*data) {
  uint16_t x, y;
  data->state = touch_read(&x, &y) > 0 ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
  data->point.x = x;
  data->point.y = y;
  return false;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Start !");

  delay(100);
  
  // LCD
  LCD_Initial();
  touch_init();

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

  /*Initialize the Input*/
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touch_pointer;
  lv_indev_drv_register(&indev_drv);

  /*Initialize the graphics library's tick*/
  tick.attach_ms(LVGL_TICK_PERIOD, lv_tick_handler);

  // Show Demo
  demo_create();
  
  Serial.println("Go to loop");
}

void loop() {
  lv_task_handler(); /* let the GUI do its work */
  delay(5);
}

void loop2() {
  LCD_clear(0xF800);
  LCD_clear(0x07E0);
  LCD_clear(0x001F);
  LCD_clear(0xFFFF);
}
