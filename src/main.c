/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include <asf.h>
#include <string.h>
#include "ili9341.h"
#include "lvgl.h"
#include "touch/touch.h"
#include "arm_math.h"
#include "math.h"

LV_FONT_DECLARE(dseg70);
LV_FONT_DECLARE(dseg35);
LV_FONT_DECLARE(dseg30);
LV_FONT_DECLARE(dseg45);
LV_FONT_DECLARE(dseg60);

#define MAG_PIO           PIOA
#define MAG_PIO_ID        ID_PIOA
#define MAG_PIO_IDX       11
#define MAG_PIO_IDX_MASK  (1u << MAG_PIO_IDX)

QueueHandle_t xQueueMAG;

static void MAG_init(void);
static void task_mag(void *pvParameters);
static void mag_callback(void);

typedef struct 
{
	float velocity;
	float previus_velocity;
	char * acceleration;
	float avg_velocity;
	float previus_avg_velocity;
	float distance;
} bike_t;

typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t second;
} calendar;

enum state_t
{
	PLAY = 0,
	PAUSE,
	RESTART
};

SemaphoreHandle_t xSemaphoreMAIN;
SemaphoreHandle_t xSemaphoreCONFIG;

/************************************************************************/
/* LCD / LVGL                                                           */
/************************************************************************/

#define LV_HOR_RES_MAX          (240)
#define LV_VER_RES_MAX          (320)

/*A static or global variable to store the buffers*/
static lv_disp_draw_buf_t disp_buf;

/*Static or global buffer(s). The second buffer is optional*/
static lv_color_t buf_1[LV_HOR_RES_MAX * LV_VER_RES_MAX];
static lv_disp_drv_t disp_drv;          /*A variable to hold the drivers. Must be static or global.*/
static lv_indev_drv_t indev_drv;

/* Global */

static  lv_obj_t * labelVelocidadeAtual;
static  lv_obj_t * labelVelocidade2;
static  lv_obj_t * labelClock;
static  lv_obj_t * labelTime;
static  lv_obj_t * labelDistancia;
static  lv_obj_t * labelPlay;
static  lv_obj_t * labelPause;
static  lv_obj_t * labelRefresh;
static  lv_obj_t * labelRaio;
static	lv_obj_t * labelAc;
static  lv_obj_t * config;
static  lv_obj_t * inicio;
static  lv_obj_t * labelInit;
static  lv_obj_t * km_h;
static  lv_obj_t * btn2;
static  lv_obj_t * btn3;
static  lv_obj_t * play ;
static  lv_obj_t * pause;
static  lv_obj_t * refresh;
static  lv_obj_t * btnRaioUp;
static  lv_obj_t * labelBtnRaioUp;
static  lv_obj_t * btnRaioDown;
static  lv_obj_t * labelBtnRaioDown;
static  lv_obj_t * raio;

volatile int flag_v = 0; // 0 -> atual ; 1 -> média
volatile int s = 50;
volatile int min = 0;
volatile int h = 0;
volatile int minuto = 0;
volatile int hora = 0;
volatile float RADIUS = 0.254;
enum state_t state;
volatile char page_config = 0;

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_LCD_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);
static void lv_config(void);

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}


void RTC_Handler(void) {
    uint32_t ul_status = rtc_get_status(RTC);
	uint32_t current_hour, current_min, current_sec;
	rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
	
    /* seccond tick */
    if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {	
	// o código para irq de segundo vem aqui
	printf("ENTROU AQUI UAIS?");
	if (!page_config)
	{
		lv_label_set_text_fmt(labelClock, "%02d:%02d:%02d", current_hour, current_min ,current_sec);
	}
	
	if (state == PLAY){
		s++;
		if (s>=60){
			s = 0;
			min++;
		}
		if (min==60){
			min = 0;
			h++;
		}
		lv_label_set_text_fmt(labelTime, "%02d:%02d", h, min );
	}
    }
	
    /* Time or date alarm */
    if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
    	// o código para irq de alame vem aqui
	  }

    rtc_clear_status(RTC, RTC_SCCR_SECCLR);
    rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
    rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
    rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
    rtc_clear_status(RTC, RTC_SCCR_CALCLR);
    rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

/************************************************************************/
/* lvgl                                                                 */
/************************************************************************/

static void handler_main(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		page_config = 0;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphoreMAIN, &xHigherPriorityTaskWoken);
	}
}

static void handler_v_med(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		flag_v = 1;
		lv_color_t color = lv_color_make(0, 255, 0);
		lv_obj_set_style_text_color(btn2, color, LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(btn3, lv_color_white(), LV_STATE_DEFAULT);
		static uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(xQueueMAG, &ul_previous_time, xHigherPriorityTaskWoken);

	}
	else if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
}

static void handler_v_now(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	if(code == LV_EVENT_CLICKED) {
		flag_v = 0;
		lv_color_t color = lv_color_make(0, 255, 0);
		lv_obj_set_style_text_color(btn3, color, LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(btn2, lv_color_white(), LV_STATE_DEFAULT);
		static uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(xQueueMAG, &ul_previous_time, xHigherPriorityTaskWoken);
	}
	else if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
}

static void handler_play(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		printf("Aqui entrou no play\n");
		state = PLAY;
		lv_color_t color = lv_color_make(0, 255, 0);
		lv_obj_set_style_text_color(play, color, LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(pause, lv_color_white(), LV_STATE_DEFAULT);
	
	}
}

static void handler_pause(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		printf("Entrou no Pause\n");
		state = PAUSE;
		lv_color_t color = lv_color_make(0, 255, 0);
		lv_obj_set_style_text_color(pause, color, LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(play, lv_color_white(), LV_STATE_DEFAULT);
	}
	
}

static void handler_refresh(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		state = RESTART;
		lv_obj_set_style_text_color(pause, lv_color_white(), LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(play, lv_color_white(), LV_STATE_DEFAULT);

	}
}

static void handler_config(lv_event_t * e){
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		page_config = 1;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphoreCONFIG, &xHigherPriorityTaskWoken);
	}
}

static void handler_up_radius(lv_event_t * e){
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		RADIUS = RADIUS + 0.001;
		int r = RADIUS * 1000;
		lv_label_set_text_fmt(labelRaio, "%d", r);
	}
}

static void handler_down_radius(lv_event_t * e){
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		RADIUS = RADIUS - 0.001;
		int r = RADIUS * 1000;
		lv_label_set_text_fmt(labelRaio, "%d", r);
	}
}

static void mag_callback(void)
{
	static uint32_t ul_previous_time;
	ul_previous_time = rtt_read_timer_value(RTT);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(xQueueMAG, &ul_previous_time, xHigherPriorityTaskWoken);
}

static void lv_config(void){
	
	lv_style_t style_config;
	lv_style_init(&style_config);
	lv_style_set_bg_color(&style_config, lv_color_black());
	
	inicio = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(inicio, handler_main, LV_EVENT_ALL, NULL);
	lv_obj_align(inicio, LV_ALIGN_TOP_LEFT, 10 , 10);
	
	lv_obj_set_height(inicio, LV_SIZE_CONTENT);
	lv_obj_set_size(inicio, 70, 30);
	
	labelInit = lv_label_create(inicio);
	lv_label_set_text(labelInit, "Inicio");
	lv_obj_center(labelInit);
	
	labelRaio = lv_label_create(lv_scr_act());
	lv_obj_align(labelRaio, LV_ALIGN_TOP_MID, 0, 80);
	
	lv_obj_set_style_text_font(labelRaio, &dseg70, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelRaio, lv_color_white(), LV_STATE_DEFAULT);
	int r = RADIUS * 1000;
	lv_label_set_text_fmt(labelRaio, "%03d", r);
	
	btnRaioUp = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btnRaioUp, handler_up_radius, LV_EVENT_ALL, NULL);
	lv_obj_align_to(btnRaioUp, labelRaio, LV_ALIGN_OUT_BOTTOM_RIGHT, -30, 20);
	lv_obj_set_height(btnRaioUp, LV_SIZE_CONTENT);
	lv_obj_set_size(btnRaioUp, 40, 40);
	labelBtnRaioUp = lv_label_create(btnRaioUp);
	lv_label_set_text(labelBtnRaioUp, "[  " LV_SYMBOL_UP " ]");
	lv_obj_center(labelBtnRaioUp);
	
	btnRaioDown = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btnRaioDown, handler_down_radius, LV_EVENT_ALL, NULL);
	lv_obj_align_to(btnRaioDown, labelRaio, LV_ALIGN_OUT_BOTTOM_LEFT, 30, 20);
	lv_obj_set_height(btnRaioDown, LV_SIZE_CONTENT);
	lv_obj_set_size(btnRaioDown, 40, 40);
	labelBtnRaioDown = lv_label_create(btnRaioDown);
	lv_label_set_text(labelBtnRaioDown, "[  " LV_SYMBOL_DOWN " ]");
	lv_obj_center(labelBtnRaioDown);
	
	raio = lv_label_create(lv_scr_act());
	lv_obj_align_to(raio,labelRaio, LV_ALIGN_OUT_LEFT_TOP, 30,-30);
	lv_style_set_text_font(&style_config, &lv_font_montserrat_24);
	lv_obj_add_style(raio, &style_config, 0);
	lv_obj_set_style_text_color(raio, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(raio, "Raio (mm)");
	
}

void lv_ex_btn_1(void) {
	
	static lv_style_t style;
	lv_style_init(&style);
	lv_style_set_bg_color(&style, lv_color_black());
	
	static lv_obj_t * label;
	
	labelClock = lv_label_create(lv_scr_act());
	lv_obj_align(labelClock, LV_ALIGN_TOP_RIGHT, 0 , 0);
	lv_obj_set_style_text_font(labelClock, &dseg30, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelClock, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelClock, "%02d:%02d:%02d", 0, 0 ,0);
	
	labelAc = lv_label_create(lv_scr_act());
	lv_obj_align(labelAc, LV_ALIGN_TOP_LEFT, 10 , 0);
	lv_style_set_text_font(&style, &lv_font_montserrat_24);
	lv_obj_add_style(labelAc, &style, 0);
	lv_obj_set_style_text_color(labelAc, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelAc, "%s", "----");
	
	/* Velocidade grande */
	labelVelocidadeAtual = lv_label_create(lv_scr_act());
	lv_obj_align(labelVelocidadeAtual, LV_ALIGN_LEFT_MID, 10 , -65);
	
	lv_obj_set_style_text_font(labelVelocidadeAtual, &dseg70, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelVelocidadeAtual, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelVelocidadeAtual, "%02d", 0);

	/* km/ h  */
	km_h = lv_label_create(lv_scr_act());
	lv_obj_align_to(km_h,labelVelocidadeAtual, LV_ALIGN_OUT_TOP_MID, -25, -15);
	lv_style_set_text_font(&style, &lv_font_montserrat_24);
	lv_obj_add_style(km_h, &style, 0);
	lv_obj_set_style_text_color(km_h, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(km_h, " km/h");
	
	/* Tempo da corrida */
	labelTime = lv_label_create(lv_scr_act());
	//lv_obj_align(labelTime, LV_ALIGN_TOP_LEFT, 20 , 0);
	lv_obj_align_to(labelTime, labelVelocidadeAtual, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
	lv_obj_set_style_text_font(labelTime, &dseg30, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelTime, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelTime, "%02d:%02d", 0, 0 );
	
	/* Distância percorrida */
	labelDistancia = lv_label_create(lv_scr_act());
	lv_obj_align_to(labelDistancia,labelTime, LV_ALIGN_OUT_BOTTOM_MID,	 -30, 10);
	lv_style_set_text_font(&style, &lv_font_montserrat_26);
	lv_obj_add_style(labelDistancia, &style, 0);
	lv_obj_set_style_text_color(labelDistancia, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelDistancia, "%d.%02d km",0,0);
	
	btn2 = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btn2, handler_v_med, LV_EVENT_ALL, NULL);
	lv_obj_align_to(btn2, labelClock, LV_ALIGN_BOTTOM_MID, 0, 50);
	//lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
	lv_obj_set_height(btn2, LV_SIZE_CONTENT);
	lv_obj_set_size(btn2, 70, 30);
	
	label = lv_label_create(btn2);
	lv_label_set_text(label, "Vmed");
	lv_obj_center(label);

	/*Velocidade atual*/
	btn3 = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btn3, handler_v_now, LV_EVENT_ALL, NULL);
	lv_obj_align_to(btn3, labelClock, LV_ALIGN_BOTTOM_MID, 0, 100);
	//lv_obj_add_flag(btn3, LV_OBJ_FLAG_CHECKABLE);
	lv_obj_set_height(btn3, LV_SIZE_CONTENT);
	lv_obj_set_size(btn3, 70, 30);
	
	lv_color_t color = lv_color_make(0, 255, 0);
	lv_obj_set_style_text_color(btn3, color, LV_STATE_DEFAULT);
	label = lv_label_create(btn3);
	lv_label_set_text(label, "Vnow");
	lv_obj_center(label);

	/* Config */
	config = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(config, handler_config, LV_EVENT_ALL, NULL);
	lv_obj_align_to(config, labelClock, LV_ALIGN_BOTTOM_MID, 0, 150);
	
	//lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
	lv_obj_set_height(config, LV_SIZE_CONTENT);
	lv_obj_set_size(config, 70, 30);

	label = lv_label_create(config);
	lv_label_set_text(label, "Config");
	lv_obj_center(label);


	/* Player button */

	play = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(play, handler_play, LV_EVENT_ALL, NULL);
	lv_obj_align(play, LV_ALIGN_BOTTOM_LEFT, 20 , -50);
	//lv_obj_add_style(play, &style, 0);
	//lv_obj_add_flag(play, LV_OBJ_FLAG_CHECKABLE);
	lv_obj_set_height(play, LV_SIZE_CONTENT);
	lv_obj_set_size(play, 50, 50);
	labelPlay = lv_label_create(play);
	lv_label_set_text(labelPlay, "[  " LV_SYMBOL_PLAY " ]");
	lv_obj_center(labelPlay);
	
	/* Pause button */
	pause = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(pause, handler_pause, LV_EVENT_ALL, NULL);
	lv_obj_align_to(pause, play , LV_ALIGN_OUT_RIGHT_TOP, 20 , 0);
	//lv_obj_add_style(pause, &style, 0);
	lv_obj_set_height(pause, LV_SIZE_CONTENT);
	lv_obj_set_size(pause, 50, 50);
	//lv_obj_add_flag(pause, LV_OBJ_FLAG_CHECKABLE);
	
	labelPause = lv_label_create(pause);
	lv_label_set_text(labelPause, "[ " LV_SYMBOL_PAUSE " ]");
	lv_obj_center(labelPause);

	/* Recomeçar button */
	refresh = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(refresh, handler_refresh, LV_EVENT_ALL, NULL);
	lv_obj_align_to(refresh, pause , LV_ALIGN_OUT_RIGHT_TOP, 20 , 0);
	//lv_obj_add_style(refresh, &style, 0);
	lv_obj_set_height(refresh, LV_SIZE_CONTENT);
	//lv_obj_add_flag(refresh, LV_OBJ_FLAG_CHECKABLE);
	lv_obj_set_size(refresh, 50, 50);

	labelRefresh = lv_label_create(refresh);
	lv_label_set_text(labelRefresh, "[ " LV_SYMBOL_REFRESH " ]");
	lv_obj_center(labelRefresh);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_lcd(void *pvParameters) {
	int px, py;
	minuto = 45;
	hora = 15;
	calendar rtc_initial = {2018, 3, 19, 12, hora, minuto ,1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);
	uint32_t current_hour, current_min, current_sec;
	rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
	lv_ex_btn_1();
	state = PAUSE;
	lv_color_t color = lv_color_make(0, 255, 0);
	lv_obj_set_style_text_color(pause, color, LV_STATE_DEFAULT);
	
	for (;;)  {
		if (xSemaphoreTake(xSemaphoreMAIN, 50))
		{
			lv_obj_clean(lv_scr_act());
			vTaskDelay(50);
			lv_ex_btn_1();
			lv_obj_set_style_text_color(pause, color, LV_STATE_DEFAULT);
			state = PAUSE;
		}
		if (xSemaphoreTake(xSemaphoreCONFIG, 50))
		{
			lv_obj_clean(lv_scr_act());
			vTaskDelay(50);
			lv_config();
		}
		lv_tick_inc(50);
		lv_task_handler();
	}
}

static void task_mag(void *pvParameters)
{
	MAG_init();
	rtt_init(RTT, 32);
	
	uint32_t ul_previous_time;
	bike_t bike;
	//static enum state_t state;
	bike.velocity = 0;
	bike.previus_velocity = 0;
	bike.avg_velocity = 0;
	bike.previus_avg_velocity;
	bike.distance = 0;
	uint32_t pulses = 0;
	float total_period = 0;
	float sensibility = 1;
	
	for (;;)
	{
		switch (state)
		{
			case PLAY:
				if (xQueueReceive(xQueueMAG, &ul_previous_time, (TickType_t) 100))
				{
					pulses++;
					float period = (float) ul_previous_time / 1024;
					rtt_init(RTT, 32);
					period /= 3600;
					float frequency = 1.0 / period;
					total_period += period;
					
					bike.previus_velocity = bike.velocity;
					
					bike.velocity = ceil(2 * PI * frequency * RADIUS / 1000);
					
					bike.distance = 2 * PI * RADIUS * pulses / 1000;
					bike.previus_avg_velocity = bike.avg_velocity;
					bike.avg_velocity = ceil(bike.distance / total_period);
					
					bike.acceleration = "----";
					
					if (flag_v){// flag_v = 1 -> velocidade média
						if (bike.avg_velocity > bike.previus_avg_velocity)
						{
							bike.acceleration = "Aum";
						}
						else if (bike.avg_velocity < bike.previus_avg_velocity)
						{
							bike.acceleration = "Dim";
						}
						lv_label_set_text_fmt(labelVelocidadeAtual, "%02d", (int) bike.avg_velocity);
					}else{// flag_v = 0 -> velocidade atual
						if (bike.velocity > bike.previus_velocity)
						{
							bike.acceleration = "Aum";
						}
						else if (bike.velocity < bike.previus_velocity)
						{
							bike.acceleration = "Dim";
						}
						lv_label_set_text_fmt(labelVelocidadeAtual, "%02d", (int) bike.velocity);
					}
					int aux = (int) 100*bike.distance;
					lv_label_set_text_fmt(labelDistancia, "%02d.%02d km",(int)bike.distance,(aux%100));
					lv_label_set_text_fmt(labelAc, "%s", bike.acceleration);

				}
				break;
			case PAUSE:
				vTaskDelay(100);
				break;
			case RESTART:
				bike.velocity = 0;
				bike.previus_velocity = 0;
				bike.avg_velocity = 0;
				bike.distance = 0;
				pulses = 0;
				total_period = 0;
				state = PAUSE;
				lv_color_t color = lv_color_make(0, 255, 0);
				lv_obj_set_style_text_color(pause, color, LV_STATE_DEFAULT);
				s = 0;
				min = 0;
				h = 0;
				lv_label_set_text_fmt(labelVelocidadeAtual, "%02d", (int) bike.velocity);
				lv_label_set_text_fmt(labelDistancia, "%02d.%02d km",0,0);
				lv_label_set_text_fmt(labelTime, "%02d:%02d", h, min );

				break;
			default:
				printf("Estado invalido!\n");
		}
	}
}

/************************************************************************/
/* configs                                                              */
/************************************************************************/

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

  uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
  rtt_sel_source(RTT, false);
  rtt_init(RTT, pllPreScale);
  
  if (rttIRQSource & RTT_MR_ALMIEN) {
	uint32_t ul_previous_time;
  	ul_previous_time = rtt_read_timer_value(RTT);
  	while (ul_previous_time == rtt_read_timer_value(RTT));
  	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
  }

  /* config NVIC */
  NVIC_DisableIRQ(RTT_IRQn);
  NVIC_ClearPendingIRQ(RTT_IRQn);
  NVIC_SetPriority(RTT_IRQn, 4);
  NVIC_EnableIRQ(RTT_IRQn);

  /* Enable RTT interrupt */
  if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
  else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
		  
}

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}

static void configure_lcd(void) {
	/**LCD pin configure on SPI*/
	pio_configure_pin(LCD_SPI_MISO_PIO, LCD_SPI_MISO_FLAGS);  //
	pio_configure_pin(LCD_SPI_MOSI_PIO, LCD_SPI_MOSI_FLAGS);
	pio_configure_pin(LCD_SPI_SPCK_PIO, LCD_SPI_SPCK_FLAGS);
	pio_configure_pin(LCD_SPI_NPCS_PIO, LCD_SPI_NPCS_FLAGS);
	pio_configure_pin(LCD_SPI_RESET_PIO, LCD_SPI_RESET_FLAGS);
	pio_configure_pin(LCD_SPI_CDS_PIO, LCD_SPI_CDS_FLAGS);
	
	ili9341_init();
	ili9341_backlight_on();
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT,
	};

	/* Configure console UART. */
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

/************************************************************************/
/* port lvgl                                                            */
/************************************************************************/

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
	ili9341_set_top_left_limit(area->x1, area->y1);   ili9341_set_bottom_right_limit(area->x2, area->y2);
	ili9341_copy_pixels_to_screen(color_p,  (area->x2 + 1 - area->x1) * (area->y2 + 1 - area->y1));
	
	/* IMPORTANT!!!
	* Inform the graphics library that you are ready with the flushing*/
	lv_disp_flush_ready(disp_drv);
}

void my_input_read(lv_indev_drv_t * drv, lv_indev_data_t*data) {
	int px, py, pressed;
	
	if (readPoint(&px, &py))
		data->state = LV_INDEV_STATE_PRESSED;
	else
		data->state = LV_INDEV_STATE_RELEASED; 
	
	data->point.x = py;
	data->point.y = 320-px;
}

void configure_lvgl(void) {
	lv_init();
	lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX);
	
	lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
	disp_drv.draw_buf = &disp_buf;          /*Set an initialized buffer*/
	disp_drv.flush_cb = my_flush_cb;        /*Set a flush callback to draw to the display*/
	disp_drv.hor_res = LV_HOR_RES_MAX;      /*Set the horizontal resolution in pixels*/
	disp_drv.ver_res = LV_VER_RES_MAX;      /*Set the vertical resolution in pixels*/

	lv_disp_t * disp;
	disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/
	
	/* Init input on LVGL */
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = my_input_read;
	lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);
}

static void MAG_init(void)
{
	pio_configure(MAG_PIO, PIO_INPUT, MAG_PIO_IDX_MASK, PIO_DEBOUNCE | PIO_DEFAULT);

	pio_handler_set(MAG_PIO,
					MAG_PIO_ID,
					MAG_PIO_IDX_MASK,
					PIO_IT_FALL_EDGE,
					&mag_callback);

	pio_enable_interrupt(MAG_PIO, MAG_PIO_IDX_MASK);
	pio_get_interrupt_status(MAG_PIO);
	
	NVIC_EnableIRQ(MAG_PIO_ID);
	NVIC_SetPriority(MAG_PIO_ID, 4); // Prioridade 4
}
/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* board and sys init */
	board_init();
	sysclk_init();
	configure_console();

	/* LCd, touch and lvgl init*/
	configure_lcd();
	ili9341_set_orientation(ILI9341_FLIP_Y | ILI9341_SWITCH_XY);
	configure_touch();
	configure_lvgl();
	
	xQueueMAG = xQueueCreate(32, sizeof(uint32_t));
	if (xQueueMAG == NULL)
		printf("falha em criar a queue do handler do sensor magnetico");

	/* Create task to control oled */
	if (xTaskCreate(task_lcd, "LCD", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create lcd task\r\n");
	}
	
	if (xTaskCreate(task_mag, "MAG", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create mag task\r\n");
	}
	
	xSemaphoreMAIN = xSemaphoreCreateBinary();
	if (xSemaphoreMAIN == NULL)
	printf("falha em criar o semaforo \n");
	
	xSemaphoreCONFIG = xSemaphoreCreateBinary();
	if (xSemaphoreCONFIG == NULL)
	printf("falha em criar o semaforo \n");
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){ }
}
