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

// =========== Touch ===========
#include <TFT_Touch.h>

#define TOUCH_DOUT 12  /* Data out pin (T_DO) of touch screen */
#define TOUCH_DIN  13  /* Data in pin (T_DIN) of touch screen */
#define TOUCH_CS   14  /* Chip select pin (T_CS) of touch screen */
#define TOUCH_CLK  15  /* Clock pin (T_CLK) of touch screen */

#define HMIN 3914
#define HMAX 340
#define VMIN 3867
#define VMAX 276
#define XYSWAP 0 // 0 or 1

#define HRES 480 /* Default screen resulution for X axis */
#define VRES 272 /* Default screen resulution for Y axis */

#include <TFT_Touch.h>

TFT_Touch touch = TFT_Touch(TOUCH_CS, TOUCH_CLK, TOUCH_DIN, TOUCH_DOUT);

// =========== END of Touch ===========

// Config LCD
#define LCD_WIDTH  480
#define LCD_HEIGHT 272

#define LVGL_TICK_PERIOD 20
Ticker tick; /* timer for interrupt handler */

void disp_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t *color_p) {
  LCD_SetWindow(x1, y1, x2, y2); // Set window
  for(uint16_t y=y1;y<=y2;y++) {
    for(uint16_t x=x1;x<=x2;x++) {
      Write_Data_int(color_p->full);
      ++color_p;
    }
  }
  lv_flush_ready(); /* tell lvgl that flushing is done */
}

bool read_touchscreen(lv_indev_data_t * data) {
  if (touch.Pressed()) {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touch.X();
    data->point.y = touch.Y();
  } else {
    data->state =  LV_INDEV_STATE_REL;
  }
  return false;
}

static void lv_tick_handler(void) {
  lv_tick_inc(LVGL_TICK_PERIOD);
}

/*
static void event_handler(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_CLICKED) {
        printf("Clicked\n");
    }
    else if(event == LV_EVENT_VALUE_CHANGED) {
        printf("Toggled\n");
    }
}
*/
static lv_style_t style1;
void lv_ex_btn_1(void)
{
    lv_obj_t * label;

    lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);
    //lv_obj_set_event_cb(btn1, event_handler);
    lv_obj_align(btn1, NULL, LV_ALIGN_CENTER, 0, -40);

    label = lv_label_create(btn1, NULL);
    lv_label_set_style(label, &style1);
    lv_label_set_text(label, "ดีย์");
}

void gpio_mode_output(int pin) {
  // IO_8 -> GPIO0
  fpioa_set_function(pin , (fpioa_function_t)(FUNC_GPIO0 + (pin - 8)));
  gpio_set_drive_mode((pin - 8), GPIO_DM_OUTPUT);
}

void gpio_mode_input(int pin) {
  // IO_8 -> GPIO0
  fpioa_set_function(pin , (fpioa_function_t)(FUNC_GPIO0 + (pin - 8)));
  gpio_set_drive_mode((pin - 8), GPIO_DM_INPUT);
}

extern lv_font_t th_sarabun_20;

void setup() {
  Serial.begin(9600);
  
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

  // Touch 
  //pinMode(TOUCH_CS, OUTPUT); 
  //pinMode(TOUCH_CLK, OUTPUT); 
  //pinMode(TOUCH_DIN, OUTPUT); 
  //pinMode(TOUCH_DOUT, INPUT); 
  
  //digitalWrite(TOUCH_CS, HIGH);  
  //digitalWrite(TOUCH_CLK, LOW);
  //digitalWrite(TOUCH_DIN, LOW);

  gpio_mode_output(TOUCH_CS);
  gpio_mode_output(TOUCH_CLK);
  gpio_mode_output(TOUCH_DIN);
  gpio_mode_input(TOUCH_DOUT);

  gpio_set_pin(TOUCH_CS - 8, GPIO_PV_HIGH);
  gpio_set_pin(TOUCH_CLK - 8, GPIO_PV_LOW);
  gpio_set_pin(TOUCH_DIN - 8, GPIO_PV_LOW);

  // LCD
  SSD1963_Initial();

  // Touch
  touch.setCal(HMIN, HMAX, VMIN, VMAX, HRES, VRES, XYSWAP); // Raw xmin, xmax, ymin, ymax, width, height
  touch.setRotation(1);
  
  // LittlevGL
  lv_init();

  /*Initialize the display*/
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.disp_flush = disp_flush;
  lv_disp_drv_register(&disp_drv);

  /*Initialize the touch pad*/
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read = read_touchscreen;
  lv_indev_drv_register(&indev_drv);

  /*Initialize the graphics library's tick*/
  tick.attach_ms(LVGL_TICK_PERIOD, lv_tick_handler);

  // Style
  

  lv_style_copy(&style1, &lv_style_plain);
  style1.body.opa = LV_OPA_TRANSP;
  style1.text.font = &th_sarabun_20;
  
  
  /* Create simple label */
  lv_obj_t *label = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_style(label, &style1);
  lv_label_set_text(label, "เช้าฟาดผัดฟัก");
  lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0);

  //lv_ex_btn_1();
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
