/*
    todo:
        - add WACHDOG timer;
        - add and check stepper_y;
*/


#include "main.h"

int main()
{	
    RCC_INIT();
    GPIO_INIT();
    ADC_INIT();
    TIM_INIT();
    EXTI_INIT();

    __enable_irq ();
    
    ADC1->CR |= ADC_CR_ADSTART; // Start conversion
    TIM2->CR1 |= TIM_CR1_CEN;   // Hall input capture start
    TIM15->CR1 |= TIM_CR1_CEN;  // Hall wachdog timer start
    TIM3->CR1 |= TIM_CR1_CEN;   // Encoder timer start
    
    motor.PID.Kp = 0.7;
    motor.PID.Ki = 0.0;
    motor.PID.Kd = 0.0;
    motor.PIDoutMax = 5000;
    motor.PIDoutMin = 0;
    arm_pid_init_f32 (&motor.PID, 1);	// DSP lib PID init

//	LCD_init();
//	TIM16->CR1 |= TIM_CR1_CEN;	//lvgl tim enable
//	
//	LCD_SetArea(10, 20, 200, 170);	// screen test
//	for(uint32_t i = 0; i < (176*220); i++){
//		LCD_Write_DATA(0x0d, 0x62); 
//	}

//	demo_create();
    
//	lv_obj_t * set_current = lv_spinbox_create(lv_scr_act(), NULL);
//	lv_spinbox_set_digit_format(set_current, 4, 1);
//	lv_spinbox_set_range(set_current,1,3000);
//	lv_obj_set_pos(set_current,0,100);
//	lv_spinbox_set_style(set_current, LV_SPINBOX_STYLE_BG, &main_style);
//	lv_group_add_obj(power_group,set_current);

    for(uint32_t i = 0; i < 1000000; i++); // wait
    WACHDOG_INIT();

    while(1)
    {


        // if(PIDcalculateFlag){
        //     PIDcalculateTask();
        //     PIDcalculateFlag = 0;
        // }

//		KeyboardTask();
        
//		lv_task_handler();
        WWDG->CR |= (0x40 + WACHDOGTIME) << WWDG_CR_T_Pos;         // reset Wachdog tim
    }
}

void PIDcalculateTask() 
{
    float32_t RPMerror;
    uint16_t PIDoutput = 0;
    uint16_t RPMset = 0;
    
    if(ADC1->ISR & ADC_ISR_EOC){
        RPMset = ADC1->DR;				// reading POT value
        ADC1->ISR &= ~ADC_ISR_EOC;
    }

    RPMset > 10 ? (motor.on = 1) : (motor.on = 0);	// check ON/OFF

    RPMerror = HallDeltaTime - 30000000 / (RPMset + 1);
    
    RPMerror < 0 ? (motor.on = 0) : 0; 	// check error direction 
    
    PIDoutput = (uint16_t)(arm_pid_f32(&motor.PID, RPMerror));
    
    if(PIDoutput > motor.PIDoutMax){				// check MIN/MAX
        PIDoutput = motor.PIDoutMax;
                motor.PID.state[2] = motor.PIDoutMax;
    }
    else if(PIDoutput < motor.PIDoutMin){
        PIDoutput = motor.PIDoutMin;
                motor.PID.state[2] = motor.PIDoutMin;
    }
    
    motor.Pulse = PIDoutput;
}

/*
void KeyboardTask()         // TODO (make union keyArr)
{
    int8_t numPad = -1; 
    static uint8_t keyMode = 0; 

    if(keyArr[0][0]) numPad = 1;
    if(keyArr[0][1]) numPad = 4;
    if(keyArr[0][2]) numPad = 7;
    if(keyArr[0][3]) numPad = 10; // for #

    if(keyArr[1][0]) numPad = 2;
    if(keyArr[1][1]) numPad = 5;
    if(keyArr[1][2]) numPad = 8;
    if(keyArr[1][3]) numPad = 0;
    
    if(keyArr[2][0]) numPad = 3;
    if(keyArr[2][1]) numPad = 6;
    if(keyArr[2][2]) numPad = 9;
    if(keyArr[2][3]) numPad = 11; // for * 

    if(keyArr[3][0]) keyMode = 0;
    if(keyArr[3][1]) keyMode = 1;
    if(keyArr[3][2]) keyMode = 2;
    if(keyArr[3][3]) keyMode = 3;

    switch(keyMode){
        case 0: EncPrescaler = numPad; TIM3->ARR = EncPrescaler;  break;
        case 1: EncPrescaler += numPad; TIM3->ARR = EncPrescaler; break;
        case 2: EncPrescaler -= numPad; TIM3->ARR = EncPrescaler; break;
        case 3: break;
    }



    for(uint8_t i = 0; i < 4; i++){
        for(uint8_t j = 0; j < 4; j++){
            keyArr[i][j] = 0;
        }
    }
}
*/
// #ifdef __cplusplus
// extern "C" {
// #endif

void EXTI1_IRQHandler()		            // Zero Cross detected
{
    if(motor.on){
        if(GPIOB->IDR & GPIO_IDR_1){    // First wave pulse
            TIM17->CCR1 = (uint16_t)(WAVELEN - motor.Pulse - MOTORPULSELEN);
            TIM17->ARR = (uint16_t)(WAVELEN - motor.Pulse);
        }else{                          // Second wave pulse
            TIM17->CCR1 = (uint16_t)(WAVELEN - motor.Pulse - KF - MOTORPULSELEN);
            TIM17->ARR = (uint16_t)(WAVELEN - motor.PIDoutMin - KF);
        }
        TIM17->CNT = 0;                 // Set counter to 0
        TIM17->EGR |= TIM_EGR_UG;		// Update
        TIM17->CR1 |= TIM_CR1_CEN; 		// Start motorPulse timer
    }
    motor.Calculate = 1;
    EXTI->PR |= EXTI_PR_PR1;
}

/*
void EXTI9_5_IRQHandler()               // FIXME: rewrite keyboard handler code (EXTI9_5 and EXTI15_10)
{
    switch(GPIOB->ODR){
        case GPIO_ODR_4: keyArr[0][0] = 1; break;   // Main[THREAD].Next[1];
        case GPIO_ODR_5: keyArr[0][1] = 1; break;
        case GPIO_ODR_6: keyArr[0][2] = 1; break;
        case GPIO_ODR_7: keyArr[0][3] = 1; break;
    }
    EXTI->PR |= EXTI_PR_PR8;
}

void EXTI15_10_IRQHandler()             // FIXME: rewrite keyboard handler code (EXTI9_5 and EXTI15_10)
{
    if(EXTI->PR & EXTI_PR_PR13){
        switch(GPIOB->ODR){
            case GPIO_ODR_4: keyArr[1][0] = 1; break;
            case GPIO_ODR_5: keyArr[1][1] = 1; break;
            case GPIO_ODR_6: keyArr[1][2] = 1; break;
            case GPIO_ODR_7: keyArr[1][3] = 1; break;
        }
        EXTI->PR |= EXTI_PR_PR13;
    }

    if(EXTI->PR & EXTI_PR_PR14){
        switch(GPIOB->ODR){
            case GPIO_ODR_4: keyArr[2][0] = 1; break;
            case GPIO_ODR_5: keyArr[2][1] = 1; break;
            case GPIO_ODR_6: keyArr[2][2] = 1; break;
            case GPIO_ODR_7: keyArr[2][3] = 1; break;
        }
        EXTI->PR |= EXTI_PR_PR14;
    }

    if(EXTI->PR & EXTI_PR_PR15){
        switch(GPIOB->ODR){
            case GPIO_ODR_4: keyArr[3][0] = 1; break;
            case GPIO_ODR_5: keyArr[3][1] = 1; break;
            case GPIO_ODR_6: keyArr[3][2] = 1; break;
            case GPIO_ODR_7: keyArr[3][3] = 1; break;
        }
        EXTI->PR |= EXTI_PR_PR15;
    }
}
*/

void TIM2_IRQHandler()                  // Hall sensor handler
{
    HallDeltaTime = TIM2->CCR1;         // get hall input capture (delta time)
    TIM2->CNT = 0;                      // set hall timer to 0
    TIM15->CNT = 0;				        // reset hall wachdog tim15
    TIM2->SR &= ~TIM_SR_CC1OF;
    TIM2->SR &= ~TIM_SR_UIF;
}

void TIM3_IRQHandler()		            // Encoder tim handler
{
    static uint8_t pulse = 0;

    if(pulse){				            // Stepper pulse
        GPIOA->BSRR = GPIO_BSRR_BS_3;
        pulse = 0;
    }else{
        GPIOA->BSRR = GPIO_BSRR_BR_3;
        pulse = 1;
    }

    if(TIM3->CR1 & TIM_CR1_DIR){        // Change stepper direction
        GPIOA->BSRR = GPIO_BSRR_BR_4;   // Change direction
    }else{
        GPIOA->BSRR = GPIO_BSRR_BS_4;	
    }

    TIM3->SR &= ~TIM_SR_CC2IF;
    //TIM3->SR &= ~TIM_SR_UIF;
}

void TIM15_IRQHandler()		            // Hall wachdog timer
{
    HallDeltaTime = HALLDELTAMAX;
    TIM15->SR &= ~(TIM_SR_CC1IF 		//FIXME: DATASHEET! cc1f or cc2f? any difference
                 | TIM_SR_CC2IF
                 | TIM_SR_UIF);
}

//void TIM16_IRQHandler()		        // LVGL ticks
//{
//	lv_tick_inc(LVGL_TIM_UPD);
//	TIM16->SR &= ~TIM_SR_UIF;
//}

// #ifdef __cplusplus
// }
// #endif

void RCC_INIT()
{		
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;          // For 48 MHz < SystemCoreClock <= 72 MHz
    FLASH->ACR |= FLASH_ACR_PRFTBE;             // Enable buffer
    
    RCC->CR |= RCC_CR_HSION;
    while((RCC->CR & RCC_CR_HSIRDY) == 0);
    RCC->CR &= ~RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_PLLRDY) != 0); 
    
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_DIV2; 		// Set HSI as PLL source
    RCC->CFGR |= RCC_CFGR_PLLMUL16			    // x16
              | RCC_CFGR_HPRE_DIV1              // AHB PRE = 1
              | RCC_CFGR_PPRE1_DIV2             // APB1 PRESCKALER = 2
              | RCC_CFGR_PPRE2_DIV1;	        // APB2 PRESCKALER = 1
    
    RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV1;      // ADC12 clk = PLL / 1
    
    RCC->CR |= RCC_CR_PLLON; 
    while((RCC->CR & RCC_CR_PLLRDY) == 0); 
    RCC->CFGR |= RCC_CFGR_SW_PLL;               // PLL selected as system clock
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    SystemCoreClockUpdate();
    
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->AHBENR |= RCC_AHBENR_ADC12EN;

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}

void GPIO_INIT()
{
    //____Stepper_X__GPIOA3_GPIOA4_GPIOA5__________________________________________________________
    GPIOA->MODER |= OUTPUT << GPIO_MODER_MODER3_Pos     //STEP_x
                 | OUTPUT << GPIO_MODER_MODER4_Pos      //DIR_x
                 | OUTPUT << GPIO_MODER_MODER5_Pos;     //EN_x
    GPIOA->OSPEEDR |= HS << GPIO_OSPEEDER_OSPEEDR3_Pos;

    /*
    //____Stepper_Y_GPIOA3_GPIOA4_GPIOA5___(TODO)_______________________________________________
    GPIOA->MODER |= OUTPUT << GPIO_MODER_MODER3_Pos     //STEP_y
                 | OUTPUT << GPIO_MODER_MODER4_Pos      //DIR_y
                 | OUTPUT << GPIO_MODER_MODER5_Pos;     //EN_y
    GPIOA->OSPEEDR |= HS << GPIO_OSPEEDER_OSPEEDR3_Pos;
    */

    // //____Keyboard_GPIOB4_GPIOB5_GPIOB6_GPIOB7_GPIOB8_GPIOC13_GPIOC14_GPIOC15______________________
    // GPIOB->MODER &= ~GPIO_MODER_MODER4;                 // GPIOB4 reset (Alternate Func by default 10)
    // GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR4;          //    (Medium Speed by default 01)

    // GPIOB->MODER |= OUTPUT << GPIO_MODER_MODER4_Pos     // a0
    //              | OUTPUT << GPIO_MODER_MODER5_Pos      // a1
    //              | OUTPUT << GPIO_MODER_MODER6_Pos      // a2
    //              | OUTPUT << GPIO_MODER_MODER7_Pos;     // a3
}

void EXTI_INIT()
{
    //_____Zero_Cross_interrupt_by_falling_and_rising__EXTI1(GPIOB1)_______________________________
    GPIOB->MODER &= ~GPIO_MODER_MODER1;                 // ZeroCross interrupt (INPUT) 
    EXTI->FTSR |= EXTI_FTSR_TR1;                        // by falling
    EXTI->RTSR |= EXTI_RTSR_TR1;                        //    and by rising
    EXTI->IMR |= EXTI_IMR_IM1;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PB; 
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_SetPriority(EXTI1_IRQn, 1);	
/*
    //____Keyboard_GPIOB4_GPIOB5_GPIOB6_GPIOB7_GPIOB8_GPIOC13_GPIOC14_GPIOC15______________________
    GPIOB->MODER &= ~GPIO_MODER_MODER8;                 // b0 interrupt (INPUT)
    GPIOC->MODER &= ~(GPIO_MODER_MODER13                // b1 interrupt (INPUT)
                 | GPIO_MODER_MODER14                   // b2 interrupt (INPUT)
                 | GPIO_MODER_MODER15);                 // b3 interrupt (INPUT)

    EXTI->RTSR |= EXTI_RTSR_TR8;                        // by rising
    EXTI->IMR |= EXTI_IMR_IM8;
    EXTI->RTSR |= EXTI_RTSR_TR13;                       
    EXTI->IMR |= EXTI_IMR_IM13;
    EXTI->RTSR |= EXTI_RTSR_TR14;                       
    EXTI->IMR |= EXTI_IMR_IM14;
    EXTI->RTSR |= EXTI_RTSR_TR15;                       
    EXTI->IMR |= EXTI_IMR_IM15;

    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI8_PB;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI14_PC;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PC; 

    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_SetPriority(EXTI9_5_IRQn, 3);	
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(EXTI15_10_IRQn, 3);
*/
}

void TIM_INIT()
{
    //_____Hall_sensor_input_capture_mode_ch1(GPIOA0)______________________________________________
    GPIOA->MODER |= AF << GPIO_MODER_MODER0_Pos;
    GPIOA->AFR[0] |= 0x1 << GPIO_AFRL_AFRL0_Pos;	//AF1
    
    TIM2->PSC = (SystemCoreClock / 1000000) - 1;    // 1MHz 1us
    TIM2->ARR = 60000 - 1;                   // max 50000us = 50 ms					
    TIM2->CCMR1 |= 0x01 << TIM_CCMR1_CC1S_Pos;      // TIM2_CCR1 linked to the TI1 input (01)
    TIM2->CCMR1 |= TIM_CCMR1_IC1F;                   // filter 
    TIM2->CCER |= TIM_CCER_CC1P;                    // active edge of transition
    TIM2->DIER |= TIM_DIER_CC1IE;                   // interrupt enable	
    TIM2->CCER |= TIM_CCER_CC1E;                    // capture compare enable
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1);
    
    //_____Encoder_mode_tim3_ch1(GPIOA6)_ch2(GPIOA7)_______________________________________________
    GPIOA->MODER &= ~GPIO_MODER_MODER6;             // INPUT
    GPIOA->MODER &= ~GPIO_MODER_MODER7;

    GPIOA->PUPDR |= PULL_UP << GPIO_PUPDR_PUPDR6_Pos; 
    GPIOA->PUPDR |= PULL_UP << GPIO_PUPDR_PUPDR7_Pos;


    GPIOA->AFR[0] = 0x02 << GPIO_AFRL_AFRL6_Pos;    // AF2 TIM3_ch1
    GPIOA->AFR[0] = 0x02 << GPIO_AFRL_AFRL7_Pos;    // AF2 TIM3_ch2
    
    TIM3->PSC = 0;					
    TIM3->ARR = 20 - 1;						        // 2000 - encoder line number
    TIM3->CR1 &= ~TIM_CR1_ARPE;                     // permanently ARR update 
    TIM3->CCMR1 |= 0x01 << TIM_CCMR1_CC1S_Pos;      // IC1 mapped on TI1 
    TIM3->CCMR1 |= 0x01 << TIM_CCMR1_CC2S_Pos;      // IC2 mapped on TI2 
    TIM3->SMCR |= 0x3 << TIM_SMCR_SMS_Pos;          // counting on TI1 and TI2 edges (11)
    //TIM3->SMCR |= TIM_SMCR_ETF_1;                   // filter (0010)
    TIM3->DIER |= TIM_DIER_CC1IE;                   // interrupt enable
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 2);
    
    // //_____Stepper_opm_mode_no_output_ch1_______________________________________________________
    // TIM4->PSC = (SystemCoreClock / 1000000) - 1;   // 1MHz 1us	
    // TIM4->ARR = 100 - 1;                           // FIXME: add define 
    // TIM4->CR1 &= ~TIM_CR1_ARPE;                    // permanently ARR update 		
    // TIM4->DIER |= TIM_DIER_CC1IE;                  // interrupt enable
    // NVIC_EnableIRQ(TIM4_IRQn);
    // NVIC_SetPriority(TIM4_IRQn, 1);

    //_____HALL_wachdog_counter_mode_no_output_________________________________________________
    TIM15->PSC = (SystemCoreClock / 1000000) - 1;   // 1MHz 1us	
    TIM15->ARR = HALLDELTAMAX - 1;		
    TIM15->DIER |= TIM_DIER_CC2IE;                  // interrupt enable
    NVIC_EnableIRQ(TIM15_IRQn);
    NVIC_SetPriority(TIM15_IRQn, 1);
    
//	//_____LVGL_counter_mode_no_output__________________________________________________________
//	TIM16->PSC = 64000 - 1;				
//	TIM16->ARR = LVGL_TIM_UPD - 1;		
//	TIM16->DIER |= TIM_DIER_CC2IE;		
//	NVIC_EnableIRQ(TIM16_IRQn);
//	NVIC_SetPriority(TIM16_IRQn, 1);

    //_____Motor_Pulse_onePulse_mode_ch1(GPIOB9)___________________________________________________
    GPIOB->MODER |= AF << GPIO_MODER_MODER9_Pos;
    GPIOB->OSPEEDR |= HS << GPIO_OSPEEDER_OSPEEDR9_Pos;
    GPIOB->PUPDR |= PULL_DOWN << GPIO_PUPDR_PUPDR9_Pos;
    GPIOB->AFR[1] |= 0x1 << GPIO_AFRH_AFRH1_Pos;        //AF1 
    
    TIM17->PSC = (SystemCoreClock / 1000000) - 1;       // 1MHz 1us	
    TIM17->CCR1 = WAVELEN - 1;                          // time delay before toggle
    TIM17->ARR = WAVELEN + MOTORPULSELEN - 1;           // pulse length = ARR - CCR
    //TIM17->CR1 |= TIM_CR1_URS;                        // interrupt only by overflow
    TIM17->EGR |= TIM_EGR_UG;
    TIM17->CR1 |= TIM_CR1_OPM;
    TIM17->CCMR1 |= TIM_CCMR1_OC1PE;                    // preload enable
    TIM17->CCMR1 |= TIM_CCMR1_OC1M;                     // 111: output compare pwm2 mode
    //TIM17->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM17->CCER |= TIM_CCER_CC1E;                       // compare output channel 1 
    TIM17->BDTR |= TIM_BDTR_MOE;
}

void WACHDOG_INIT()
{
    //_____Wachdog_timer____________________________________________________________________________
    // T(wwdg) = T(pclk) * 4096 * 2^WDGTB *(T[5:0]) = 1/64000000 * 4096 * 2^8 * 0X3F = 1032 ms
    //WWDG->CFR |= 0x7F << WWDG_CFR_W;              // refresh allowed period
    WWDG->CFR |= 0x2 << WWDG_CFR_WDGTB_Pos;             // downcounter prescaler  10 -> div4 
    WWDG->CR |= (0x40 + WACHDOGTIME) << WWDG_CR_T_Pos;  // reset is produced when WWDG_CR_T < 0x40 
    WWDG->CR |= WWDG_CR_WDGA;                       // Wachdog downcounter enable
}

void ADC_INIT() // FIXME: check
{
    //_____Speed_set_pot_ADC1_CH3_regular conversion____________________________________________
    GPIOA->MODER |= ANALOG << GPIO_MODER_MODER2_Pos;
    RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV1;
    
    ADC1->CR |= ADC_CR_ADCAL;				
    while ((ADC1->CR & ADC_CR_ADCAL) != 0);	
    
    ADC1->CFGR |= ADC_CFGR_CONT; 
    ADC1->SQR1 &= ~ADC_SQR1_L;                  // regular sequence length 0 = 1 channel
    ADC1->SQR1 |= 0x3 << ADC_SQR1_SQ1_Pos;      // start convertion with channel 3
    ADC1->SMPR1 |= 0x7 << ADC_SMPR1_SMP3_Pos;   // channel 3 sampling time 111 = 601,5 clk
    ADC1->CR |= ADC_CR_ADEN;				
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);
    ADC1->CR |= ADC_CR_ADSTART;

}

