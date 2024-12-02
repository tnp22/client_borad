/********************************************************************************/
/* ir_remocon.c                                                                 */
/* STM32F407ZGT6                                                                */
/* (Lee ChangWoo HL2IRW  hl2irw@daum.net 010-8573-6860)                 	*/
/* stm32f4x_test								*/
/********************************************************************************/
#include "../../hwdefs.h"
#include "../prototype.h"

#define RDATA				PA_INPUT(8)
#define REMOTE_ID			0

unsigned char remote_count;
unsigned char remocon_status = 0;
unsigned short remocon_time;
unsigned int remocon_recv = 0;
unsigned char remote_count = 0;
unsigned char ir_ret,old_ir,ir_time;



void ir_remocon_init (void)
{
      NVIC_InitTypeDef NVIC_InitStructure;
      TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
      TIM_ICInitTypeDef TIM1_ICInitStructure;
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
      GPIO_Init_Pin(GPIOA,GPIO_Pin_8,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);
      GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1);
      TIM_TimeBaseStructure.TIM_Prescaler = 167;	// 168MHz / (167 + 1) = 1MHz
      TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
      TIM_TimeBaseStructure.TIM_Period = 10000;		// 10 msec
      TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
      TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
      TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S = 01
      TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
      TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
      TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
      TIM1_ICInitStructure.TIM_ICFilter = 0x03;
      TIM_ICInit(TIM1, &TIM1_ICInitStructure);
      TIM_ITConfig(TIM1,TIM_IT_Update | TIM_IT_CC1,ENABLE);
      TIM_Cmd(TIM1,ENABLE );
      NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
      NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
      ir_ret = 0;
      old_ir = 0;
      ir_time = 0;
}


void TIM1_UP_TIM10_IRQHandler (void)
{
      if (TIM_GetITStatus(TIM1,TIM_IT_Update) == SET) {
	 if (remocon_status & 0x80) {
	    remocon_status &= ~(0x10);
	    if ((remocon_status & 0x0F) == 0x00) remocon_status |= 1 << 6;
	    if ((remocon_status & 0x0F) < 14) {
	       remocon_status++;
	    } else {
	       remocon_status &= ~(1 << 7);
	       remocon_status &= 0xF0;
	    }
	 }
      }
      TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
}


void TIM1_CC_IRQHandler (void)
{
      if (TIM_GetITStatus(TIM1,TIM_IT_CC1) == SET) {
	 if (RDATA) {
	    TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Falling);				// CC1P = 1
	    TIM_SetCounter(TIM1,0);
	    remocon_status |= 0x10;
	 } else {
	    remocon_time = TIM_GetCapture1(TIM1);
	    TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Rising);				// CC1P = 0
	    if (remocon_status & 0x10) {
 	       if (remocon_status & 0x80) {
		  if ((remocon_time > 300) && (remocon_time < 800)) {			// 560us
		     remocon_recv <<= 1;
		     remocon_recv |= 0;
		  } else {
		     if ((remocon_time > 1400) && (remocon_time < 1800)) {		// 1680us
			remocon_recv <<= 1;
			remocon_recv |= 1;
		     } else {
		        if ((remocon_time > 2200) && (remocon_time < 2600)) {		// 2500us = 2.5ms
			   remote_count++;
			   remocon_status &= 0xF0;
			}
		     }
	          }
	       } else {
		  if ((remocon_time > 4200) && (remocon_time < 4700)) {	// 4500us = 4.5ms
		     remocon_status |= 1 << 7;
		     remote_count = 0;
		  }
	       }
	    }
	    remocon_status &= ~(1 << 4);
	 }
      }
      TIM_ClearITPendingBit(TIM1,TIM_IT_CC1);
}


unsigned char remocon_scan (void)
{
      unsigned char status = 0;
      unsigned char t1,t2;
      if (remocon_status & (1 << 6)) {
	 t1 = remocon_recv >> 24;
	 t2 = (remocon_recv >> 16) & 0xff;
 	 if (((t1 == (unsigned char)~t2)) && ((t1 == REMOTE_ID) || (t1 == 0x08))) {
	    t1 = remocon_recv >> 8;
	    t2 = remocon_recv;
	    if (t1 == (unsigned char)~t2) status = t1;
         }
	 if ((status == 0) || ((remocon_status & 0x80) == 0)) {
	    remocon_status &= ~(1 << 6);
	    remote_count = 0;
	 }
      }
      return status;
}


void ir_process (void)
{
      char *str = 0;
      ir_time++;
      if (ir_time >= 5) {
         ir_time = 0;
	 ir_ret = remocon_scan();
	 if ((ir_ret) && (ir_ret != old_ir)) {
	    old_ir = ir_ret;
	    switch (ir_ret) {
               case 0xA2:
               	 str = "CH- ";
               	 break;
	       case 0x62:
	       	 str = "CH ";
	       	 break;
	       case 0xE2:
	       	 str = "CH+ ";
	       	 break;
	       case 0x22:
	       	 str = "RR  ";
	       	 break;
	       case 0x02:
	       	 str = "FF  ";
	       	 break;
	       case 0xC2:
	       	 str = "PLAY";
	       	 break;
	       case 0xE0:
	       	 str = "VOL-";
	       	 break;
	       case 0xA8:
	       	 str = "VOL+";
	       	 break;
	       case 0x90:
	       	 str = "EQ  ";
	       	 break;
	       case 0x68:
	       	 str = "0   ";
	       	 break;
	       case 0x98:
	      	 str = "100+";
	      	 break;
	       case 0xB0:
	       	 str = "200+";
	       	 break;
	       case 0x30:
	       	 str = "1   ";
	       	 break;
	       case 0x18:
	       	 str = "2   ";
	       	 break;
	       case 0x7A:
	       	 str = "3   ";
	       	 break;
	       case 0x10:
	       	 str = "4   ";
	       	 break;
	       case 0x38:
	       	 str = "5   ";
	       	 break;
	       case 0x5A:
	       	 str = "6   ";
	       	 break;
	       case 0x42:
	       	 str = "7   ";
	       	 break;
	       case 0x4A:
	       	 str = "8   ";
	       	 break;
	       case 0x52:
	       	 str = "9   ";
	       	 break;
	    }
	    lcd_printf(1,2,"IR_RECV: %.2X Key %s ",ir_ret,str);
	 }
      }
}



























