#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f3xx.h"
#include <math.h>

#define ARM_MATH_CM4
#include "arm_math.h"

//#include "lcd_init.h"
//#include "lvgl.h"
//#define LVGL_TIM_UPD 5 	//5 ms

//#include "lv_examples/lv_apps/demo/demo.h"
//#include "lv_examples/lv_apps/benchmark/benchmark.h"

#define OUTPUT		0x01UL
#define INPUT		0x00UL
#define AF 		    0x02UL
#define ANALOG		0x03UL
#define PULL_UP 	0x01UL
#define PULL_DOWN 	0x02UL
#define MS		    0x01UL
#define HS		    0x02UL

#define ADC_IN3		0x03UL


#define MOTORPULSELEN 	200
#define WAVELEN		    10000

#define HALLDELTAMAX	50000

void RCC_INIT(void);
void GPIO_INIT(void);
void EXTI_INIT(void);
void TIM_INIT(void);
void ADC_INIT(void);



//static lv_disp_buf_t disp_buf;
//static lv_color_t buf[LV_HOR_RES_MAX * 1];                     /*Declare a buffer for 10 lines*/
//void my_disp_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p);

//void LCD_init(void)
//{
//	lv_init();
//	
//	static lv_color_t buf[LV_HOR_RES_MAX * 1]; 
//	static lv_disp_buf_t disp_buf;
//	
//	lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 1);    /*Initialize the display buffer*/

//	lv_disp_drv_t disp_drv;               /*Descriptor of a display driver*/
//	lv_disp_drv_init(&disp_drv);          /*Basic initialization*/
//	disp_drv.flush_cb = my_disp_flush;    /*Set your driver function*/
//	disp_drv.buffer = &disp_buf;          /*Assign the buffer to the display*/
//	lv_disp_drv_register(&disp_drv);      /*Finally register the driver*/
//	
//	LCD_IOinit();
//}

//void my_disp_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p)
//{
//	LCD_SetArea(area->x1, area->y1, area->x2, area->y2);
//	
//	for(uint32_t i = 0; i < (area->x2 - area->x1) * (area->y2 - area->y1); i++){
//		LCD_Write_DATA((uint8_t)((color_p->full) >> 8), (uint8_t)(color_p->full)); // (high, low)
//		color_p++;
//	}
//	
//	lv_disp_flush_ready(disp);         /* Indicate you are ready with the flushing*/
//}


void PIDcalculateTask(void);
uint8_t softStart(uint16_t);

volatile uint8_t PIDcalculateFlag = 0;
volatile uint8_t pulseAllowed = 0;
volatile uint32_t motorPulse = 0;
volatile uint32_t HallDeltaTime = HALLDELTAMAX;

arm_pid_instance_f32 PID;
uint16_t PIDoutputMax;
uint16_t PIDoutputMin;


#endif
