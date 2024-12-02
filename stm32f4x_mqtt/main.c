/********************************************************************************/
/* main.c                                                                       */
/* STM32F407ZGT6                                                                */
/* (Lee ChangWoo HL2IRW  hl2irw@daum.net 010-8573-6860)                 	*/
/* stm32f4x_test								*/
/********************************************************************************/
#include "hwdefs.h"
#include "source/prototype.h"
#include "source/color.h"


volatile unsigned short tick,jiffes;
volatile unsigned char time_led,read_key,old_key,remote;


FLASH_Status FLASHStatus = FLASH_COMPLETE;


extern volatile unsigned char rxck1,rxck2,rxck3,rx_led,tx_led;
extern volatile unsigned short rxcnt1,rxcnt2,rxcnt3;



void wait_ms (unsigned short delay)
{
      unsigned short old_jiffes;
      jiffes = 0;
      old_jiffes = 0;
      while (jiffes < delay) {
      	    if (old_jiffes != jiffes) {
      	       old_jiffes = jiffes;
               /* Reload IWDG counter */
               IWDG_ReloadCounter();
            }
      }
}


char hex2dec (const char ch)
{
      if (ch <= '9') return (ch - '0');
      if (ch >= 'a' && ch <= 'f') return (ch - 'a' + 10);
      if (ch >= 'A' && ch <= 'F') return (ch - 'A' + 10);
      return 0;
}


char dec2hex (const char ch)
{
      if (ch <= 9) return (ch + '0');
      if (ch >= 10 && ch <= 15) return (ch + 'A' - 10);
      return 0;
}


void Periph_Configuration (void)
{
      /* Enable GPIO clocks */
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG, ENABLE);
      /* Enable USART1 clock */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
      /* Enable USART2 clock */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
      /* Enable USART3 clock */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
      /* Enable PWR and BKP clocks */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_AHB1Periph_BKPSRAM | RCC_AHB1Periph_SRAM1 | RCC_AHB1Periph_SRAM2, ENABLE);
}


void GPIO_Configuration (void)
{
      GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
      GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
      GPIO_Init_Pin(GPIOA,TXD2,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);
      GPIO_Init_Pin(GPIOA,RXD2,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);

      GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
      GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
      GPIO_Init_Pin(GPIOA,TXD1,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);
      GPIO_Init_Pin(GPIOA,RXD1,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);

      GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
      GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);
      GPIO_Init_Pin(GPIOB,TXD3,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);
      GPIO_Init_Pin(GPIOB,RXD3,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);

      GPIO_Init_Pin(GPIOC,WIFI_PGM_PIN,GPIO_Speed_100MHz,GPIO_Mode_Out_PP);
      
      GPIO_Init_Pin(GPIOF,WIFI_RST_PIN,GPIO_Speed_100MHz,GPIO_Mode_Out_PP);
      GPIO_Init_Pin(GPIOF,LED0,GPIO_Speed_100MHz,GPIO_Mode_Out_PP);
      GPIO_Init_Pin(GPIOF,LED1,GPIO_Speed_100MHz,GPIO_Mode_Out_PP);
      GPIO_Init_Pin(GPIOG,TXEN,GPIO_Speed_100MHz,GPIO_Mode_Out_PP);
      WIFI_PGM = 1;
      WIFI_RST = 0;
      LED_OUT0 = 1;
      LED_OUT1 = 1;
      TXEN_485 = 0;
      WIFI_RST = 1;
}


#ifdef VECT_TAB_RAM
/* vector-offset (TBLOFF) from bottom of SRAM. defined in linker script */
extern unsigned int _isr_vectorsram_offs;
void NVIC_Configuration (void)
{
      /* Set the Vector Table base location at 0x20000000 +_isr_vectorsram_offs */
      NVIC_SetVectorTable(NVIC_VectTab_RAM, (unsigned int)&_isr_vectorsram_offs);
}
#else
extern unsigned int _isr_vectorsflash_offs;
void NVIC_Configuration (void)
{
      /* Set the Vector Table base location at 0x08000000 +_isr_vectorsflash_offs */
      NVIC_SetVectorTable(NVIC_VectTab_FLASH, (unsigned int)&_isr_vectorsflash_offs);
}
#endif /* VECT_TAB_RAM */

extern void lcd_info (void);

RAMFUNC void SysTick_Handler (void)
{
      static unsigned short cnt = 0;
      static unsigned char flip = 0;
      cnt++;
      if (cnt >= 500) {
         cnt = 0;
         /* alive sign */
         if (flip) {
            time_led = ON;
         } else {
            time_led = OFF;
         }
      	 flip = !flip;
      }
      tick++;
      jiffes++;
      if (rxcnt1) rxck1++;
      if (rxcnt2) rxck2++;
      if (rxcnt3) rxck3++;
}


void watch_dog_init (void)
{
      /* Enable write access to IWDG_PR and IWDG_RLR registers */
      IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
      /* IWDG counter clock: LSI/32 */
      IWDG_SetPrescaler(IWDG_Prescaler_32);
      /* Set counter reload value to obtain 250ms IWDG TimeOut. */
      IWDG_SetReload(256);
      /* Reload IWDG counter */
      IWDG_ReloadCounter();
      /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
      IWDG_Enable();
}


int main (void)
{
      RCC_ClocksTypeDef RCC_Clocks;
      /* System Clocks Configuration */
      Periph_Configuration();
      /* NVIC configuration */
      NVIC_Configuration();
      /* Configure the GPIO ports */
      GPIO_Configuration();
      RCC_GetClocksFreq(&RCC_Clocks);
      /* Setup SysTick Timer for 1 millisecond interrupts, also enables Systick and Systick-Interrupt */
      if (SysTick_Config(SystemCoreClock / 1000)) {
         /* Capture error */
         while (1);
      }
      /* 4 bit for pre-emption priority, 0 bits for subpriority */
      NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
      if (FLASH_OB_GetRDP() != SET) {
         FLASH_Unlock();                           // this line is critical!
         FLASH_OB_Unlock();
         FLASH_OB_RDPConfig(OB_RDP_Level_1);
         FLASH_OB_Launch();                        // Option Bytes programming
         FLASH_OB_Lock();
         FLASH_Lock();
         NVIC_SystemReset();
      }
      serial_init();
      LCD_Init();
      init_i2c_24xx();
      ADC_Config();
      Dac1_Init();
      key_init();
      set_color(WHITE);
      lcd_printf(1,1,"Program_Start.");
      Dac1_Set_Voltage(0);
      while (1) {
            if (tick) {
               tick = 0;
               /* Reload IWDG counter */
               IWDG_ReloadCounter();
               read_key = key_read();
               if (old_key != read_key) {
               	  old_key = read_key;
               	  lcd_printf(1,7,"KEY READ %.2X",read_key);
               }
               adc_process();
               serial_check();
               if (remote == 0) {
                  if (time_led == ON) LED_OUT0 = 0;else LED_OUT0 = 1;
                  if ((tx_led) || (rx_led)) LED_OUT1 = 0;
                  if ((tx_led == 0) && (rx_led == 0)) LED_OUT1 = 1;
               }
            }
      }
}
