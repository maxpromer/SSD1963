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

#define TOUCH_IRQ  11
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
  if (gpio_get_pin(TOUCH_IRQ - 8) == GPIO_PV_LOW) {
    if (touch.Pressed()) {
      data->state = LV_INDEV_STATE_PR;
      data->point.x = touch.X();
      data->point.y = touch.Y();
    } else {
      data->state =  LV_INDEV_STATE_REL;
    }
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

void lv_ex_btn_1(void)
{
    lv_obj_t * label;

    lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);
    //lv_obj_set_event_cb(btn1, event_handler);
    lv_obj_align(btn1, NULL, LV_ALIGN_CENTER, 0, -40);

    label = lv_label_create(btn1, NULL);
    lv_label_set_text(label, "Button");
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

void gpio_mode_input_pullup(int pin) {
  // IO_8 -> GPIO0
  fpioa_set_function(pin , (fpioa_function_t)(FUNC_GPIO0 + (pin - 8)));
  gpio_set_drive_mode((pin - 8), GPIO_DM_INPUT_PULL_UP);
}

#define LV_USE_DEMO
/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void write_create(lv_obj_t * parent);
static lv_res_t keyboard_open_close(lv_obj_t * ta);
static lv_res_t keyboard_hide_action(lv_obj_t * keyboard);
static void list_create(lv_obj_t * parent);
static void chart_create(lv_obj_t * parent);
static lv_res_t slider_action(lv_obj_t * slider);
static lv_res_t list_btn_action(lv_obj_t * slider);
#if LV_DEMO_SLIDE_SHOW
static void tab_switcher(void * tv);
#endif

/**********************
 *  STATIC VARIABLES
 **********************/
static lv_obj_t * chart;
static lv_obj_t * ta;
static lv_obj_t * kb;

static lv_style_t style_kb;
static lv_style_t style_kb_rel;
static lv_style_t style_kb_pr;

#if LV_DEMO_WALLPAPER
LV_IMG_DECLARE(img_bubble_pattern);
#endif

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Create a demo application
 */
void demo_create(void)
{

#if LV_DEMO_WALLPAPER
    lv_obj_t * wp = lv_img_create(lv_scr_act(), NULL);
    lv_img_set_src(wp, &img_bubble_pattern);
    lv_obj_set_width(wp, LV_HOR_RES * 4);
    lv_obj_set_protect(wp, LV_PROTECT_POS);
#endif

    static lv_style_t style_tv_btn_bg;
    lv_style_copy(&style_tv_btn_bg, &lv_style_plain);
    style_tv_btn_bg.body.main_color = LV_COLOR_HEX(0x487fb7);
    style_tv_btn_bg.body.grad_color = LV_COLOR_HEX(0x487fb7);
    style_tv_btn_bg.body.padding.ver = 0;

    static lv_style_t style_tv_btn_rel;
    lv_style_copy(&style_tv_btn_rel, &lv_style_btn_rel);
    style_tv_btn_rel.body.empty = 1;
    style_tv_btn_rel.body.border.width = 0;

    static lv_style_t style_tv_btn_pr;
    lv_style_copy(&style_tv_btn_pr, &lv_style_btn_pr);
    style_tv_btn_pr.body.radius = 0;
    style_tv_btn_pr.body.opa = LV_OPA_50;
    style_tv_btn_pr.body.main_color = LV_COLOR_WHITE;
    style_tv_btn_pr.body.grad_color = LV_COLOR_WHITE;
    style_tv_btn_pr.body.border.width = 0;
    style_tv_btn_pr.text.color = LV_COLOR_GRAY;

    lv_obj_t * tv = lv_tabview_create(lv_scr_act(), NULL);

#if LV_DEMO_WALLPAPER
    lv_obj_set_parent(wp, ((lv_tabview_ext_t *) tv->ext_attr)->content);
    lv_obj_set_pos(wp, 0, -5);
#endif

    lv_obj_t * tab1 = lv_tabview_add_tab(tv, "Write");
    lv_obj_t * tab2 = lv_tabview_add_tab(tv, "List");
    lv_obj_t * tab3 = lv_tabview_add_tab(tv, "Chart");

#if LV_DEMO_WALLPAPER == 0
    /*Blue bg instead of wallpaper*/
    lv_tabview_set_style(tv, LV_TABVIEW_STYLE_BG, &style_tv_btn_bg);
#endif
    lv_tabview_set_style(tv, LV_TABVIEW_STYLE_BTN_BG, &style_tv_btn_bg);
    lv_tabview_set_style(tv, LV_TABVIEW_STYLE_INDIC, &lv_style_plain);
    lv_tabview_set_style(tv, LV_TABVIEW_STYLE_BTN_REL, &style_tv_btn_rel);
    lv_tabview_set_style(tv, LV_TABVIEW_STYLE_BTN_PR, &style_tv_btn_pr);
    lv_tabview_set_style(tv, LV_TABVIEW_STYLE_BTN_TGL_REL, &style_tv_btn_rel);
    lv_tabview_set_style(tv, LV_TABVIEW_STYLE_BTN_TGL_PR, &style_tv_btn_pr);

    write_create(tab1);
    list_create(tab2);
    chart_create(tab3);

#if LV_DEMO_SLIDE_SHOW
    lv_task_create(tab_switcher, 3000, LV_TASK_PRIO_MID, tv);
#endif
}


/**********************
 *   STATIC FUNCTIONS
 **********************/

static void write_create(lv_obj_t * parent)
{
    lv_page_set_style(parent, LV_PAGE_STYLE_BG, &lv_style_transp_fit);
    lv_page_set_style(parent, LV_PAGE_STYLE_SCRL, &lv_style_transp_fit);

    lv_page_set_sb_mode(parent, LV_SB_MODE_OFF);

    static lv_style_t style_ta;
    lv_style_copy(&style_ta, &lv_style_pretty);
    style_ta.body.opa = LV_OPA_30;
    style_ta.body.radius = 0;
    style_ta.text.color = LV_COLOR_HEX3(0x222);

    ta = lv_ta_create(parent, NULL);
    lv_obj_set_size(ta, lv_page_get_scrl_width(parent), lv_obj_get_height(parent) / 2);
    lv_ta_set_style(ta, LV_TA_STYLE_BG, &style_ta);
    lv_ta_set_text(ta, "");
    lv_page_set_rel_action(ta, keyboard_open_close);

    lv_style_copy(&style_kb, &lv_style_plain);
    style_kb.body.opa = LV_OPA_70;
    style_kb.body.main_color = LV_COLOR_HEX3(0x333);
    style_kb.body.grad_color = LV_COLOR_HEX3(0x333);
    style_kb.body.padding.hor = 0;
    style_kb.body.padding.ver = 0;
    style_kb.body.padding.inner = 0;

    lv_style_copy(&style_kb_rel, &lv_style_plain);
    style_kb_rel.body.empty = 1;
    style_kb_rel.body.radius = 0;
    style_kb_rel.body.border.width = 1;
    style_kb_rel.body.border.color = LV_COLOR_SILVER;
    style_kb_rel.body.border.opa = LV_OPA_50;
    style_kb_rel.body.main_color = LV_COLOR_HEX3(0x333);    /*Recommended if LV_VDB_SIZE == 0 and bpp > 1 fonts are used*/
    style_kb_rel.body.grad_color = LV_COLOR_HEX3(0x333);
    style_kb_rel.text.color = LV_COLOR_WHITE;

    lv_style_copy(&style_kb_pr, &lv_style_plain);
    style_kb_pr.body.radius = 0;
    style_kb_pr.body.opa = LV_OPA_50;
    style_kb_pr.body.main_color = LV_COLOR_WHITE;
    style_kb_pr.body.grad_color = LV_COLOR_WHITE;
    style_kb_pr.body.border.width = 1;
    style_kb_pr.body.border.color = LV_COLOR_SILVER;

    keyboard_open_close(ta);
}

static lv_res_t keyboard_open_close(lv_obj_t * text_area)
{
    (void) text_area;    /*Unused*/

    lv_obj_t * parent = lv_obj_get_parent(lv_obj_get_parent(ta));   /*Test area is on the scrollable part of the page but we need the page itself*/

    if(kb) {
        return keyboard_hide_action(kb);
    } else {

        kb = lv_kb_create(parent, NULL);
        lv_obj_set_size(kb, lv_page_get_scrl_width(parent), lv_obj_get_height(parent) / 2);
        lv_obj_align(kb, ta, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
        lv_kb_set_ta(kb, ta);
        lv_kb_set_style(kb, LV_KB_STYLE_BG, &style_kb);
        lv_kb_set_style(kb, LV_KB_STYLE_BTN_REL, &style_kb_rel);
        lv_kb_set_style(kb, LV_KB_STYLE_BTN_PR, &style_kb_pr);
        lv_kb_set_hide_action(kb, keyboard_hide_action);
        lv_kb_set_ok_action(kb, keyboard_hide_action);

#if USE_LV_ANIMATION
        lv_obj_animate(kb, LV_ANIM_FLOAT_BOTTOM | LV_ANIM_IN, 300, 0, NULL);
#endif
        return LV_RES_OK;
    }
}

/**
 * Called when the close or ok button is pressed on the keyboard
 * @param keyboard pointer to the keyboard
 * @return
 */
static lv_res_t keyboard_hide_action(lv_obj_t * keyboard)
{
    (void) keyboard;    /*Unused*/

#if USE_LV_ANIMATION
    lv_obj_animate(kb, LV_ANIM_FLOAT_BOTTOM | LV_ANIM_OUT, 300, 0, (void(*)(lv_obj_t *))lv_obj_del);
    kb = NULL;
    return LV_RES_OK;
#else
    lv_obj_del(kb);
    kb = NULL;
    return LV_RES_INV;
#endif
}

static void list_create(lv_obj_t * parent)
{
    lv_page_set_style(parent, LV_PAGE_STYLE_BG, &lv_style_transp_fit);
    lv_page_set_style(parent, LV_PAGE_STYLE_SCRL, &lv_style_transp_fit);

    lv_page_set_scrl_fit(parent, false, false);
    lv_page_set_scrl_height(parent, lv_obj_get_height(parent));
    lv_page_set_sb_mode(parent, LV_SB_MODE_OFF);

    /*Create styles for the buttons*/
    static lv_style_t style_btn_rel;
    static lv_style_t style_btn_pr;
    lv_style_copy(&style_btn_rel, &lv_style_btn_rel);
    style_btn_rel.body.main_color = LV_COLOR_HEX3(0x333);
    style_btn_rel.body.grad_color = LV_COLOR_BLACK;
    style_btn_rel.body.border.color = LV_COLOR_SILVER;
    style_btn_rel.body.border.width = 1;
    style_btn_rel.body.border.opa = LV_OPA_50;
    style_btn_rel.body.radius = 0;

    lv_style_copy(&style_btn_pr, &style_btn_rel);
    style_btn_pr.body.main_color = LV_COLOR_MAKE(0x55, 0x96, 0xd8);
    style_btn_pr.body.grad_color = LV_COLOR_MAKE(0x37, 0x62, 0x90);
    style_btn_pr.text.color = LV_COLOR_MAKE(0xbb, 0xd5, 0xf1);

    lv_obj_t * list = lv_list_create(parent, NULL);
    lv_obj_set_height(list, 2 * lv_obj_get_height(parent) / 3);
    lv_list_set_style(list, LV_LIST_STYLE_BG, &lv_style_transp_tight);
    lv_list_set_style(list, LV_LIST_STYLE_SCRL, &lv_style_transp_tight);
    lv_list_set_style(list, LV_LIST_STYLE_BTN_REL, &style_btn_rel);
    lv_list_set_style(list, LV_LIST_STYLE_BTN_PR, &style_btn_pr);
    lv_obj_align(list, NULL, LV_ALIGN_IN_TOP_MID, 0, LV_DPI / 4);

    lv_list_add(list, SYMBOL_FILE, "New", list_btn_action);
    lv_list_add(list, SYMBOL_DIRECTORY, "Open", list_btn_action);
    lv_list_add(list, SYMBOL_TRASH, "Delete", list_btn_action);
    lv_list_add(list, SYMBOL_EDIT, "Edit", list_btn_action);
    lv_list_add(list, SYMBOL_SAVE, "Save", list_btn_action);
    lv_list_add(list, SYMBOL_WIFI, "WiFi", list_btn_action);
    lv_list_add(list, SYMBOL_GPS, "GPS", list_btn_action);

    lv_obj_t * mbox = lv_mbox_create(parent, NULL);
    lv_mbox_set_text(mbox, "Click a button to copy its text to the Text area ");
    lv_obj_set_width(mbox, LV_HOR_RES - LV_DPI);
    static const char * mbox_btns[] = {"Got it", ""};
    lv_mbox_add_btns(mbox, mbox_btns, NULL);    /*The default action is close*/
    lv_obj_align(mbox, parent, LV_ALIGN_IN_TOP_MID, 0, LV_DPI / 2);
}

static void chart_create(lv_obj_t * parent)
{
    lv_page_set_style(parent, LV_PAGE_STYLE_BG, &lv_style_transp_fit);
    lv_page_set_style(parent, LV_PAGE_STYLE_SCRL, &lv_style_transp_fit);

    lv_page_set_scrl_fit(parent, false, false);
    lv_page_set_scrl_height(parent, lv_obj_get_height(parent));
    lv_page_set_sb_mode(parent, LV_SB_MODE_OFF);

    static lv_style_t style_chart;
    lv_style_copy(&style_chart, &lv_style_pretty);
    style_chart.body.opa = LV_OPA_60;
    style_chart.body.radius = 0;
    style_chart.line.color = LV_COLOR_GRAY;

    chart = lv_chart_create(parent, NULL);
    lv_obj_set_size(chart, 2 * lv_obj_get_width(parent) / 3, lv_obj_get_height(parent) / 2);
    lv_obj_align(chart, NULL,  LV_ALIGN_IN_TOP_MID, 0, LV_DPI / 4);
    lv_chart_set_type(chart, LV_CHART_TYPE_COLUMN);
    lv_chart_set_style(chart, &style_chart);
    lv_chart_set_series_opa(chart, LV_OPA_70);
    lv_chart_series_t * ser1;
    ser1 = lv_chart_add_series(chart, LV_COLOR_RED);
    lv_chart_set_next(chart, ser1, 40);
    lv_chart_set_next(chart, ser1, 30);
    lv_chart_set_next(chart, ser1, 47);
    lv_chart_set_next(chart, ser1, 59);
    lv_chart_set_next(chart, ser1, 59);
    lv_chart_set_next(chart, ser1, 31);
    lv_chart_set_next(chart, ser1, 55);
    lv_chart_set_next(chart, ser1, 70);
    lv_chart_set_next(chart, ser1, 82);

    /*Create a bar, an indicator and a knob style*/
    static lv_style_t style_bar;
    static lv_style_t style_indic;
    static lv_style_t style_knob;

    lv_style_copy(&style_bar, &lv_style_pretty);
    style_bar.body.main_color =  LV_COLOR_BLACK;
    style_bar.body.grad_color =  LV_COLOR_GRAY;
    style_bar.body.radius = LV_RADIUS_CIRCLE;
    style_bar.body.border.color = LV_COLOR_WHITE;
    style_bar.body.opa = LV_OPA_60;
    style_bar.body.padding.hor = 0;
    style_bar.body.padding.ver = LV_DPI / 10;

    lv_style_copy(&style_indic, &lv_style_pretty);
    style_indic.body.grad_color =  LV_COLOR_MAROON;
    style_indic.body.main_color =  LV_COLOR_RED;
    style_indic.body.radius = LV_RADIUS_CIRCLE;
    style_indic.body.shadow.width = LV_DPI / 10;
    style_indic.body.shadow.color = LV_COLOR_RED;
    style_indic.body.padding.hor = LV_DPI / 30;
    style_indic.body.padding.ver = LV_DPI / 30;

    lv_style_copy(&style_knob, &lv_style_pretty);
    style_knob.body.radius = LV_RADIUS_CIRCLE;
    style_knob.body.opa = LV_OPA_70;

    /*Create a second slider*/
    lv_obj_t * slider = lv_slider_create(parent, NULL);
    lv_slider_set_style(slider, LV_SLIDER_STYLE_BG, &style_bar);
    lv_slider_set_style(slider, LV_SLIDER_STYLE_INDIC, &style_indic);
    lv_slider_set_style(slider, LV_SLIDER_STYLE_KNOB, &style_knob);
    lv_obj_set_size(slider, lv_obj_get_width(chart), LV_DPI / 3);
    lv_obj_align(slider, chart, LV_ALIGN_OUT_BOTTOM_MID, 0, (LV_VER_RES - chart->coords.y2 - lv_obj_get_height(slider)) / 2); /*Align to below the chart*/
    lv_slider_set_action(slider, slider_action);
    lv_slider_set_range(slider, 10, 1000);
    lv_slider_set_value(slider, 700);
    slider_action(slider);          /*Simulate a user value set the refresh the chart*/
}

/**
 * Called when a new value on the slider on the Chart tab is set
 * @param slider pointer to the slider
 * @return LV_RES_OK because the slider is not deleted in the function
 */
static lv_res_t slider_action(lv_obj_t * slider)
{
    int16_t v = lv_slider_get_value(slider);
    v = 1000 * 100 / v; /*Convert to range modify values linearly*/
    lv_chart_set_range(chart, 0, v);

    return LV_RES_OK;
}

/**
 * Called when a a list button is clicked on the List tab
 * @param btn pointer to a list button
 * @return LV_RES_OK because the button is not deleted in the function
 */
static lv_res_t list_btn_action(lv_obj_t * btn)
{
    lv_ta_add_char(ta, '\n');
    lv_ta_add_text(ta, lv_list_get_btn_text(btn));

    return LV_RES_OK;
}

#if LV_DEMO_SLIDE_SHOW
/**
 * Called periodically (lv_task) to switch to the next tab
 */
static void tab_switcher(void * tv)
{
    static uint8_t tab = 0;

    tab++;
    if(tab >= 3) tab = 0;
    lv_tabview_set_tab_act(tv, tab, true);
}
#endif

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
  gpio_mode_input_pullup(TOUCH_IRQ);

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

  
  /* Create simple label */
  //lv_obj_t *label = lv_label_create(lv_scr_act(), NULL);
  //lv_label_set_text(label, "Hello Maixduino!");
  //lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0);

  //lv_ex_btn_1();
  demo_create();
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
