
#include "main.h"



int main()
{	
	RCC_INIT();
	GPIO_INIT();
	ADC_INIT();
	TIM_INIT();
	EXTI_INIT();
	__enable_irq ();
	
	ADC1->CR |= ADC_CR_ADSTART; 	// start conversion
	TIM2->CR1 |= TIM_CR1_CEN;	// hall start
	TIM3->CR1 |= TIM_CR1_CEN;	// encoder start
	
	PID.Kp = 1.0;
	PID.Ki = 0.0;
	PID.Kd = 0.0;
	PIDoutputMax = 4000;
	PIDoutputMin = 0;
	arm_pid_init_f32 (&PID, 1);	// DSP lib PID init
	
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

	while(1)
	{
		if(PIDcalculateFlag){
			PIDcalculateTask();
			PIDcalculateFlag = 0;
		}
		
//		lv_task_handler();
	}
}

void PIDcalculateTask() 
{
	float32_t RPMerror;
	uint16_t PIDoutput;
	uint16_t RPMset;
	
	uint16_t HALL = HallDeltaTime; // print 
	
	if(ADC1->ISR & ADC_ISR_EOC){
		RPMset = ADC1->DR;				// reading POT value
		ADC1->ISR &= ~ADC_ISR_EOC;
	}
	RPMset > 20 ? (pulseAllowed = 1) : (pulseAllowed = 0);	// check ON/OFF
	
	RPMerror = (float32_t)(RPMset) - (float32_t)((4095 * 1000) / (HallDeltaTime + 1));
	RPMerror < 0 ? (RPMerror = 0, pulseAllowed = 0) : 0; 	// check error direction 
	
	PIDoutput = (uint16_t)(arm_pid_f32(&PID, RPMerror));
	
	if(PIDoutput > PIDoutputMax){				// check MIN/MAX
		PIDoutput = PIDoutputMax;
                PID.state[2] = PIDoutputMax;
	}
	else if(PIDoutput < PIDoutputMin){
		PIDoutput = PIDoutputMin;
                PID.state[2] = PIDoutputMin;
	}
	
	motorPulse = PIDoutput;
}

#ifdef __cplusplus
extern "C" {
#endif

void EXTI1_IRQHandler()		// zero cross detected
{
	if(pulseAllowed){
		TIM17->CCR1 = (uint16_t)(WAVELEN - motorPulse - MOTORPULSELEN);
		TIM17->ARR = (uint16_t)(WAVELEN - motorPulse);
		TIM17->CNT = 0;
		TIM17->EGR |= TIM_EGR_UG;		// update 
		TIM17->CR1 |= TIM_CR1_CEN; 		// start motorPulse timer
	}
	PIDcalculateFlag = 1;
	EXTI->PR |= EXTI_PR_PR1;
}

void TIM2_IRQHandler()		// FIXME: 
{
	HallDeltaTime = TIM2->CCR1;		// get hall input capture (delta time)
	TIM15->CNT = 0;				// reset hall wachdog tim15
	TIM2->SR &= ~TIM_SR_CC1OF;
	TIM2->SR &= ~TIM_SR_UIF;
}

void TIM3_IRQHandler()		// change stepper direction
{
	(TIM3->CR1 & TIM_CR1_DIR) ? (GPIOA->BSRR = GPIO_BSRR_BR_4) : (GPIOA->BSRR = GPIO_BSRR_BS_4); // change direction
	TIM3->SR &= ~TIM_SR_CC2IF;
	//TIM3->SR &= ~TIM_SR_UIF;
}

void TIM15_IRQHandler()		// Hall wachdog timer
{
	HallDeltaTime = HALLDELTAMAX;
	TIM15->SR &= ~(TIM_SR_CC1IF 		//FIXME: DATASHEET! cc1f or cc2f? any difference
			| TIM_SR_CC2IF
			| TIM_SR_UIF);
}

//void TIM16_IRQHandler()		// lvgl ticks
//{
//	lv_tick_inc(LVGL_TIM_UPD);
//	TIM16->SR &= ~TIM_SR_UIF;
//}

#ifdef __cplusplus
}
#endif

void RCC_INIT()
{		
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_2; /// FIXME: LATENCY_2
	FLASH->ACR |= FLASH_ACR_PRFTBE;
	
	RCC->CR |= RCC_CR_HSION;
	while((RCC->CR & RCC_CR_HSIRDY) == 0);
	RCC->CR &= ~RCC_CR_PLLON;
	while((RCC->CR & RCC_CR_PLLRDY) != 0); 
	
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_DIV2; 		// set HSI as PLL source
	RCC->CFGR |= RCC_CFGR_PLLMUL16			// x16
		  | RCC_CFGR_HPRE_DIV1			// AHB PRE = 1
		  | RCC_CFGR_PPRE1_DIV2			// APB1 PRESCKALER = 2
		  | RCC_CFGR_PPRE2_DIV1;		// APB2 PRESCKALER = 1
	
	RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV1;		// ADC12 clk = PLL / 1
	
	RCC->CR |= RCC_CR_PLLON; 
	while((RCC->CR & RCC_CR_PLLRDY) == 0); 
	RCC->CFGR |= RCC_CFGR_SW_PLL;			// PLL selected as system clock
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
	GPIOA->MODER |= OUTPUT << GPIO_MODER_MODER3_Pos	//STEP
		     | OUTPUT << GPIO_MODER_MODER4_Pos		//DIR
		     | OUTPUT << GPIO_MODER_MODER5_Pos;		//ENABLE
	GPIOA->OSPEEDR |= MS << GPIO_OSPEEDER_OSPEEDR3_Pos;
}

void EXTI_INIT()
{
	GPIOB->MODER &= ~GPIO_MODER_MODER1; 	// ZeroCross interrupt (INPUT) 
	EXTI->FTSR |= EXTI_FTSR_TR1;
	EXTI->RTSR |= EXTI_RTSR_TR1;
	EXTI->IMR |= EXTI_IMR_IM1;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PB; 
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_SetPriority(EXTI1_IRQn, 1);	
}

void TIM_INIT()
{
	//_____Hall_sensor_input_capture_mode_ch1(PA0)______________________________________________
	GPIOA->MODER |= AF << GPIO_MODER_MODER0_Pos;
	GPIOA->AFR[0] |= 0x1 << GPIO_AFRL_AFRL0_Pos;	//AF1
	
	TIM2->PSC = 64 - 1;				// 1MHz 1us
	TIM2->ARR = 60000 - 1;				// max 60000us = 60 ms					
	TIM2->CCMR1 |= 0x01 << TIM_CCMR1_CC1S_Pos;	// TIM2_CCR1 linked to the TI1 input (01)
	TIM2->CCMR1 |= TIM_CCMR1_IC1F_0
		    | TIM_CCMR1_IC1F;			// filter 
	TIM2->CCER |= TIM_CCER_CC1P;			// active edge of transition
	TIM2->DIER |= TIM_DIER_CC1IE;			// interrupt enable	
	TIM2->CCER |= TIM_CCER_CC1E;			// capture compare enable
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 1);
	
	//_____Encoder_encoder_mode_ch1(PA6)_ch2(PA7)_______________________________________________
	GPIOA->MODER &= ~GPIO_MODER_MODER6;		//INPUT
	GPIOA->MODER &= ~GPIO_MODER_MODER7;
	
	TIM3->PSC = 0;					
	TIM3->ARR = 2000 - 1;
	TIM3->SMCR |= 0x2 << TIM_SMCR_SMS_Pos;		// counting on TI1 edges only (010)
	TIM3->SMCR |= TIM_SMCR_ETF_1;			// filter (0010)
	TIM3->CCMR1 |= 0x01 << TIM_CCMR1_CC1S_Pos;	// TI1FP1 mapped on TI1 
	TIM3->DIER |= TIM_DIER_CC2IE;			// interrupt enable
	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 2);
	
	//_____HALL_wachdog_counter_mode_no_output_________________________________________________
	TIM15->PSC = 64 - 1;				
	TIM15->ARR = HALLDELTAMAX - 1;		
	TIM15->DIER |= TIM_DIER_CC2IE;			// interrupt enable
	NVIC_EnableIRQ(TIM15_IRQn);
	NVIC_SetPriority(TIM15_IRQn, 1);
	
//	//_____LVGL_counter_mode_no_output__________________________________________________________
//	TIM16->PSC = 64000 - 1;				
//	TIM16->ARR = LVGL_TIM_UPD - 1;		
//	TIM16->DIER |= TIM_DIER_CC2IE;		
//	NVIC_EnableIRQ(TIM16_IRQn);
//	NVIC_SetPriority(TIM16_IRQn, 1);
	
	//_____Motor_Pulse_onePulse_mode_ch1(PB9)___________________________________________________
	GPIOB->MODER |= AF << GPIO_MODER_MODER9_Pos;
	GPIOB->OSPEEDR |= HS << GPIO_OSPEEDER_OSPEEDR9_Pos;
	GPIOB->PUPDR |= PULL_DOWN << GPIO_PUPDR_PUPDR9_Pos;
	GPIOB->AFR[1] |= 0x1 << GPIO_AFRH_AFRH1_Pos; 	//AF1
	
	TIM17->PSC = 64 - 1;				// 
	TIM17->CCR1 = WAVELEN - 1;			// time delay before toggle
	TIM17->ARR = WAVELEN + MOTORPULSELEN - 1;	// pulse length = ARR - CCR
	//TIM17->CR1 |= TIM_CR1_URS;			// interrupt only by overflow
	TIM17->EGR |= TIM_EGR_UG;
	TIM17->CR1 |= TIM_CR1_OPM;
	TIM17->CCMR1 |= TIM_CCMR1_OC1PE;		// preload enable
	TIM17->CCMR1 |= TIM_CCMR1_OC1M;			// 111: output compare pwm2 mode
	//TIM17->CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM17->CCER |= TIM_CCER_CC1E;			// compare output channel 1 
	TIM17->BDTR |= TIM_BDTR_MOE;
}

void ADC_INIT()
{
	GPIOA->MODER |= ANALOG << GPIO_MODER_MODER2_Pos;
	RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV1;
	
	ADC1->CR |= ADC_CR_ADCAL;				
	while ((ADC1->CR & ADC_CR_ADCAL) != 0);	
	
	ADC1->CFGR |= ADC_CFGR_CONT; 
	ADC1->SQR1 &= ~ADC_SQR1_L;			// regular sequence length 0 = 1 channel
	ADC1->SQR1 |= 0x3 << ADC_SQR1_SQ1_Pos;		// start convertion with channel 3
	ADC1->SMPR1 |= 0x7 << ADC_SMPR1_SMP3_Pos;	// channel 3 sampling time 111 = 601,5 clk
	ADC1->CR |= ADC_CR_ADEN;				
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);
	ADC1->CR |= ADC_CR_ADSTART;

}
