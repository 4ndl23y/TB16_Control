#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f3xx.h"
#include <math.h>

#define ARM_MATH_CM4
#include "arm_math.h"

#define STATICTEST 
#ifdef STATICTEST               // for static testing define STATICTEST
    #include <assert.h>
#endif

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


#define MOTORPULSELEN   200     // length of output motor pulse
#define WAVELEN         10000   // T/2 = 10 ms for 50Hz
#define KF              800     // second zero cross wave detection error

#define HALLDELTAMAX	50000   // max valid Hall signal delta time (65535 MAX)

#define WACHDOGTIME     0x3F    // 3F max ~ 1s

void RCC_INIT(void);
void GPIO_INIT(void);
void EXTI_INIT(void);
void TIM_INIT(void);
void ADC_INIT(void);
void WACHDOG_INIT(void);

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
void KeyboardTask(void);
uint8_t keyArr[4][4];
// uint8_t softStart(uint16_t); //todo
volatile uint16_t EncPrescaler = 7;
volatile uint32_t HallDeltaTime = HALLDELTAMAX;

void saveEnding(void);


struct motorStruct
{
    volatile uint8_t 
        on: 1,                  // on/off
        PIDcalculate: 1,        // calculate new value
        softStart: 6;           // 
    volatile uint32_t Pulse; 
    volatile uint8_t Calculate;
    uint16_t RPM;
    arm_pid_instance_f32 PID;   
    uint16_t PIDoutMax;         // max output
    uint16_t PIDoutMin;         // min output
} motor;

struct menuStruct
{
    char Name[16];  // list element name
    uint16_t value;
    void *Func;
    struct menuStruct *Next; 
    struct menuStruct *Prev;
} menu;

enum{
    THREAD,
    ENDINGS,
    SETTINGS
};

//struct menuStruct Main[4] =     {
//                                "Thread", 0, NULL, Thread, NULL,
//                                "Endings", 0, NULL, Endings, NULL,
//                                "Settings", 0, NULL, Settings, NULL,
//                                };

//struct menuStruct Thread[20] =  {
//                                "M03", 30, NULL, NULL, NULL,
//                                "M04", 40, NULL, NULL, NULL,
//                                "M05", 50, NULL, NULL, NULL,
//                                "M06", 60, NULL, NULL, NULL,
//                                "M07", 70, NULL, NULL, NULL,
//                                "", 1, NULL, NULL, NULL,
//                                "", 1, NULL, NULL, NULL,
//                                "", 1, NULL, NULL, NULL,
//                                "", 1, NULL, NULL, NULL,
//                                "", 1, NULL, NULL, NULL,
//                                "", 1, NULL, NULL, NULL,
//                                "", 1, NULL, NULL, NULL,
//                                "", 1, NULL, NULL, NULL,
//                                "", 1, NULL, NULL, NULL,
//                                "", 1, NULL, NULL, NULL,
//                                "", 1, NULL, NULL, NULL,
//                                "", 1, NULL, NULL, NULL,
//                                "", 1, NULL, NULL, NULL,
//                                "", 1, NULL, NULL, NULL,
//                                "", 1, NULL, NULL, NULL,
//                                "", 1, NULL, NULL, NULL
//                                };

//struct menuStruct Endings[4] =  {
//                                "Set_X1", 0, NULL, NULL, NULL,
//                                "Set_X2", 0, NULL, NULL, NULL,
//                                "Set_Y1", 0, NULL, NULL, NULL,
//                                "Set_Y2", 0, NULL, NULL, NULL,
//                                };

//struct menuStruct Settings[6] = {
//                                "Disp", 1, NULL, NULL, NULL,
//                                "...", 1, NULL, NULL, NULL,
//                                "Set_kP", 1, NULL, NULL, NULL,
//                                "Set_kI", 1, NULL, NULL, NULL,
//                                "Set_kD", 1, NULL, NULL, NULL,
//                                "Reset_PID", 1, NULL, NULL, NULL
//                                };

#ifdef STATICTEST   // static test
    //
    static_assert((MOTORPULSELEN < WAVELEN), "MOTORPULSELEN is to long");
    //
    static_assert((HALLDELTAMAX < 65636), "HALLDELTAMAX > 65535");
#endif

#endif


/*
    Menu----+
            |Thread-----+
            |           |M03
            |           |M04
            |           |...
            |           |M40
            |           |Input()
            |
            |
            |Endings----+
            |           |Set_X1
            |           |Set_X2
            |           |Set_Y1
            |           |Set_Y2
            |
            |
            |Settings---+    
                        |Disp
                        |...
                        |kp
                        |ki
                        |kd
                        |...

*/

