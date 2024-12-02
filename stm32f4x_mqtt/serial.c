/********************************************************************************/
/* serial.c                                                                     */
/* STM32F407ZGT6                                                                */
/* (Lee ChangWoo HL2IRW  hl2irw@daum.net 010-8573-6860)                 	*/
/* stm32f4x_test								*/
/********************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "hwdefs.h"
#include "source/prototype.h"

#define SOH				0x01
#define STX				0x02
#define ETX				0x03
#define EOT				0x04
#define ENQ				0x05
#define ACK				0x06
#define NAK				0x15


#define BASE64_PAD                      (char)'='
#define BASE64DE_FIRST                  (char)'+'
#define BASE64DE_LAST                   (char)'z'
#define BASE64_ENCODE_OUT_SIZE(s)       ((unsigned short)((((s) + 2) / 3) * 4 + 1))
#define BASE64_DECODE_OUT_SIZE(s)       ((unsigned short)(((s) / 4) * 3))

/* BASE 64 encode table */
static const char base64en[64] = {
      'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
      'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
      'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
      'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
      'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
      'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
      'w', 'x', 'y', 'z', '0', '1', '2', '3',
      '4', '5', '6', '7', '8', '9', '+', '/',
};


/* ASCII order for BASE 64 decode, 255 in unused character */
static const unsigned char base64de[128] = {
      /*  nul, soh, stx, etx, eot, enq, ack, bel, */
          255, 255, 255, 255, 255, 255, 255, 255,
      /*  bs,  ht,  nl,  vt,  np,  cr,  so,  si, */
          255, 255, 255, 255, 255, 255, 255, 255,
      /*  dle, dc1, dc2, dc3, dc4, nak, syn, etb, */
          255, 255, 255, 255, 255, 255, 255, 255,
      /*  can,  em, sub, esc,  fs,  gs,  rs,  us, */
          255, 255, 255, 255, 255, 255, 255, 255,
      /*  sp, '!', '"', '#', '$', '%', '&', ''', */
          255, 255, 255, 255, 255, 255, 255, 255,
      /*  '(', ')', '*', '+', ',', '-', '.', '/', */
          255, 255, 255,  62, 255, 255, 255,  63,
      /* '0', '1', '2', '3', '4', '5', '6', '7', */
          52,  53,  54,  55,  56,  57,  58,  59,
      /* '8', '9', ':', ';', '<', '=', '>', '?', */
          60,  61, 255, 255, 255, 255, 255, 255,
      /* '@', 'A', 'B', 'C', 'D', 'E', 'F', 'G', */
          255,   0,   1,  2,   3,   4,   5,    6,
      /* 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', */
          7,   8,   9,  10,  11,  12,  13,  14,
      /* 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', */
          15,  16,  17,  18,  19,  20,  21,  22,
      /* 'X', 'Y', 'Z', '[', '\', ']', '^', '_', */
          23,  24,  25, 255, 255, 255, 255, 255,
      /* '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', */
          255,  26,  27,  28,  29,  30,  31,  32,
      /* 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', */
          33,  34,  35,  36,  37,  38,  39,  40,
      /* 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', */
          41,  42,  43,  44,  45,  46,  47,  48,
      /* 'x', 'y', 'z', '{', '|', '}', '~', del, */
          49,  50,  51, 255, 255, 255, 255, 255
};


#define ADDR_FLASH_SECTOR_0     	((unsigned int)0x08000000)	/* Sector 0, 16 Kbyte */
#define ADDR_FLASH_SECTOR_1     	((unsigned int)0x08004000)	/* Sector 1, 16 Kbyte */
#define ADDR_FLASH_SECTOR_2     	((unsigned int)0x08008000)	/* Sector 2, 16 Kbyte */
#define ADDR_FLASH_SECTOR_3     	((unsigned int)0x0800C000)	/* Sector 3, 16 Kbyte */
#define ADDR_FLASH_SECTOR_4     	((unsigned int)0x08010000)	/* Sector 4, 64 Kbyte */
#define ADDR_FLASH_SECTOR_5     	((unsigned int)0x08020000)	/* Sector 5, 128 Kbyte */
#define ADDR_FLASH_SECTOR_6     	((unsigned int)0x08040000)	/* Sector 6, 128 Kbyte */
#define ADDR_FLASH_SECTOR_7     	((unsigned int)0x08060000)	/* Sector 7, 128 Kbyte */
#define ADDR_FLASH_SECTOR_8     	((unsigned int)0x08080000)	/* Sector 8, 128 Kbyte */
#define ADDR_FLASH_SECTOR_9     	((unsigned int)0x080A0000)	/* Sector 9, 128 Kbyte */
#define ADDR_FLASH_SECTOR_10    	((unsigned int)0x080C0000)	/* Sector 10, 128 Kbyte */
#define ADDR_FLASH_SECTOR_11    	((unsigned int)0x080E0000)	/* Sector 11, 128 Kbyte */


volatile unsigned short rxcnt1,txcnt1,maxtx1,rxcnt2,txcnt2,maxtx2,rxcnt3,txcnt3,maxtx3;
volatile unsigned char rxck1,rxck2,rxck3,rx_led,tx_led,this_id;
volatile unsigned int flash_address;
unsigned int flash_buff[256];


char rxbuff1[256],txbuff1[256],rxbuff2[256],txbuff2[256];
char rxbuff3[2048],txbuff3[2048],net_rx[2048],tcp_tx[2048];
unsigned char USART1_PORT,USART3_PORT;
char sbuff[256],message[17],rx_message[17];

volatile unsigned char dhcp_disable,mqtt_mode,keep_alive_time;
volatile unsigned short mqtt_port,pwr_error,tcp_length,rx_length,rx_offset,reboot_count,tcp_txcnt,
                        tcp_rxcnt,rx_topic_length,ssid_error,status_time,mqtt_connect_count,mqtt_check_cnt,
                        send_count,mqtt_sub_count;
volatile unsigned int time_out,main_time,ap_mode_time,time_req,m_ip,m_gw,m_nm;
volatile unsigned char gid,reset_flag,pwr_flag,mode_flag,ip_flag,mux_flag,server_flag,ssid_flag,send_flag,
                       send_data_flag,reboot_flag,wifi_power,mqtt_connect,mqtt_send,tcp_send,host_flag,
                       ssid_check,mqtt_protocol,ap_mode,wifi_ch,ntp_flag,ntp_req_flag,mqtt_cfg_flag,
                       mqtt_connect_flag,mqtt_sub_flag;
char ssid[32],passwd[32],host_url[64],ntp_url[64],login_id[32],login_pw[32],tx_topic[32],rx_topic[32];
volatile unsigned char led1,led2,buz;
volatile unsigned short dac;

extern FLASH_Status FLASHStatus;
extern volatile short adc_voltage1,adc_voltage2,adc_temperature;
extern volatile unsigned char read_key,remote;
extern volatile unsigned short send_dac;




void flash_read (void)
{
      unsigned int idx;
      FLASH_Unlock();
      flash_address = (unsigned int)0x08004000;
      for (idx=0;idx<256;idx++) {
          flash_buff[idx] = (*(volatile unsigned int *)(flash_address + (idx * 4)));
      }
      FLASH_Lock();
}


void flash_write (void)
{
      unsigned int idx;
      flash_address = (unsigned int)0x08004000;
      __disable_irq();
      FLASH_Unlock();
      FLASHStatus = FLASH_EraseSector(FLASH_Sector_1, VoltageRange_3);
      for (idx=0;idx<256;idx++) {
          FLASH_ProgramWord(flash_address + (idx * 4),flash_buff[idx]);
      }
      FLASH_Lock();
      __enable_irq();
}


void write_wifi_data (void)
{
      flash_buff[0] = wifi_power;
      flash_buff[1] = dhcp_disable;
      flash_buff[2] = keep_alive_time;
      flash_buff[3] = mqtt_mode;
      flash_buff[4] = mqtt_port;
      flash_buff[5] = ap_mode;
      flash_buff[6] = m_ip;
      flash_buff[7] = m_gw;
      flash_buff[8] = m_nm;
      memcpy((char *)&flash_buff[9],ssid,32);
      memcpy((char *)&flash_buff[17],passwd,32);
      memcpy((char *)&flash_buff[25],host_url,64);
      memcpy((char *)&flash_buff[41],ntp_url,64);
      memcpy((char *)&flash_buff[57],login_id,32);
      memcpy((char *)&flash_buff[65],login_pw,32);
      memcpy((char *)&flash_buff[73],tx_topic,32);
      memcpy((char *)&flash_buff[81],rx_topic,32);
      flash_buff[90] = 0x68666805;
      flash_write();
}


void read_wifi_data (void)
{
      unsigned int check_wifi;
      flash_read();
      wifi_power = flash_buff[0];
      dhcp_disable = flash_buff[1];
      keep_alive_time = flash_buff[2];
      mqtt_mode = flash_buff[3];
      mqtt_port = flash_buff[4];
      ap_mode = flash_buff[5];
      m_ip = flash_buff[6];
      m_gw = flash_buff[7];
      m_nm = flash_buff[8];
      memcpy(ssid, (char *)&flash_buff[9],32);
      memcpy(passwd, (char *)&flash_buff[17],32);
      memcpy(host_url, (char *)&flash_buff[25],64);
      memcpy(ntp_url, (char *)&flash_buff[41],64);
      memcpy(login_id, (char *)&flash_buff[57],32);
      memcpy(login_pw, (char *)&flash_buff[65],32);
      memcpy(tx_topic, (char *)&flash_buff[73],32);
      memcpy(rx_topic, (char *)&flash_buff[81],32);
      check_wifi = flash_buff[90];
      if (check_wifi != 0x68666805) {
         wifi_power = 40;
         dhcp_disable = 0;
         keep_alive_time = 60;
         mqtt_mode = 0;
         mqtt_port = 1883;
         ap_mode = 1;
         m_ip = 0;
         m_gw = 0;
         m_nm = 0;
      }
}


unsigned short base64_encode (const unsigned char* in, unsigned short inlen, char* out)
{
      short s;
      unsigned short i;
      unsigned short j;
      unsigned char c;
      unsigned char l;
      s = 0;
      l = 0;
      for (i=j=0;i<inlen;i++) {
          c = in[i];
          switch (s) {
             case 0:
               s = 1;
               out[j++] = base64en[(c >> 2) & 0x3F];
               break;
             case 1:
               s = 2;
               out[j++] = base64en[((l & 0x3) << 4) | ((c >> 4) & 0xF)];
               break;
             case 2:
               s = 0;
               out[j++] = base64en[((l & 0xF) << 2) | ((c >> 6) & 0x3)];
               out[j++] = base64en[c & 0x3F];
               break;
          }
          l = c;
      }
      switch (s) {
         case 1:
           out[j++] = base64en[(l & 0x3) << 4];
           out[j++] = BASE64_PAD;
           out[j++] = BASE64_PAD;
           break;
         case 2:
           out[j++] = base64en[(l & 0xF) << 2];
           out[j++] = BASE64_PAD;
           break;
      }
      out[j] = 0;
      return j;
}


unsigned short base64_decode (const char* in, unsigned short inlen, unsigned char* out)
{
      unsigned short i;
      unsigned short j;
      unsigned char c;
      if (inlen & 0x3) {
         return 0;
      }
      for (i=j=0;i<inlen;i++) {
          if (in[i] == BASE64_PAD) {
             break;
          }
          if (in[i] < BASE64DE_FIRST || in[i] > BASE64DE_LAST) {
             return 0;
          }
          c = base64de[(unsigned char)in[i]];
          if (c == 255) {
                        return 0;
          }
          switch (i & 0x3) {
             case 0:
               out[j] = (c << 2) & 0xFF;
               break;
             case 1:
               out[j++] |= (c >> 4) & 0x3;
               out[j] = (c & 0xF) << 4;
               break;
             case 2:
               out[j++] |= (c >> 2) & 0xF;
               out[j] = (c & 0x3) << 6;
               break;
             case 3:
               out[j++] |= c;
               break;
          }
      }
      return j;
}


void USART1_IRQHandler (void)
{
      if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
         /* Read one byte from the receive data register */
         rxbuff1[rxcnt1] = USART_ReceiveData(USART1);
         //USART_SendData(USART3, rxbuff1[rxcnt1]);
         //rxcnt1 = 0;
         rxcnt1++;
         rx_led = 1;
         rxck1 = 0;
         USART_ClearITPendingBit(USART1, USART_IT_RXNE);
      }
      if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
         /* Write one byte to the transmit data register */
      	 if (txcnt1 < maxtx1) {
            USART_SendData(USART1, txbuff1[txcnt1]);
      	    txcnt1++;
      	 } else {
      	    /* Disable the USART1 Transmit interrupt */
      	    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
      	    USART_ITConfig(USART1, USART_IT_TC, ENABLE);
      	 }
      	 USART_ClearITPendingBit(USART1, USART_IT_TXE);
      }
      if (USART_GetITStatus(USART1, USART_IT_TC) != RESET) {
      	 USART_ITConfig(USART1, USART_IT_TC, DISABLE);
      	 USART_ClearITPendingBit(USART1, USART_IT_TC);
         tx_led = 0;
      }
}


void USART2_IRQHandler (void)
{
      if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
         /* Read one byte from the receive data register */
         rxbuff2[rxcnt2] = USART_ReceiveData(USART2);
         rxcnt2++;
         rx_led = 1;
         rxck2 = 0;
         USART_ClearITPendingBit(USART2, USART_IT_RXNE);
      }
      if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
         /* Write one byte to the transmit data register */
      	 if (txcnt2 < maxtx2) {
            USART_SendData(USART2, txbuff2[txcnt2]);
      	    txcnt2++;
      	 } else {
      	    /* Disable the USART2 Transmit interrupt */
      	    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
      	    USART_ITConfig(USART2, USART_IT_TC, ENABLE);
      	 }
      	 USART_ClearITPendingBit(USART2, USART_IT_TXE);
      }
      if (USART_GetITStatus(USART2, USART_IT_TC) != RESET) {
      	 USART_ITConfig(USART2, USART_IT_TC, DISABLE);
      	 USART_ClearITPendingBit(USART2, USART_IT_TC);
         tx_led = 0;
         TXEN_485 = 0;
      }
}


void USART3_IRQHandler (void)
{
      if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
         /* Read one byte from the receive data register */
         rxbuff3[rxcnt3] = USART_ReceiveData(USART3);
         USART_SendData(USART1, rxbuff3[rxcnt3]);
         //rxcnt3 = 0;
         rxcnt3++;
         rx_led = 1;
         rxck3 = 0;
         USART_ClearITPendingBit(USART3, USART_IT_RXNE);
      }
      if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET) {
         /* Write one byte to the transmit data register */
      	 if (txcnt3 < maxtx3) {
            USART_SendData(USART3, txbuff3[txcnt3]);
      	    txcnt3++;
      	 } else {
      	    /* Disable the USART3 Transmit interrupt */
      	    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
      	    USART_ITConfig(USART3, USART_IT_TC, ENABLE);
      	 }
      	 USART_ClearITPendingBit(USART3, USART_IT_TXE);
      }
      if (USART_GetITStatus(USART3, USART_IT_TC) != RESET) {
      	 USART_ITConfig(USART3, USART_IT_TC, DISABLE);
      	 USART_ClearITPendingBit(USART3, USART_IT_TC);
         tx_led = 0;
      }
}


void tx_enable1 (unsigned char max)
{
      maxtx1 = max;
      txcnt1 = 0;
      USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
      tx_led = 1;
}


void tx_enable2 (unsigned char max)
{
      TXEN_485 = 1;
      maxtx2 = max;
      txcnt2 = 0;
      USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
      tx_led = 1;
}


void tx_enable3 (unsigned char max)
{
      maxtx3 = max;
      txcnt3 = 0;
      USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
      tx_led = 1;
}


void receive_check1 (void)
{
      unsigned char bcc,idx;
      if (rxcnt1) {
         if ((rxbuff1[0] == STX) && (rxbuff1[5] == ETX) && ((rxbuff1[1] == this_id) || (rxbuff1[1] == 33))) {
	    bcc = rxbuff1[0];
	    for (idx=1;idx<6;idx++) bcc = bcc ^ rxbuff1[idx];
            if (bcc == rxbuff1[6]) {
	       if ((rxbuff1[2] == 0xFF) && (rxbuff1[3] == 0xFF) && (rxbuff1[4] == 0xFF)) {
	       	  NVIC_SystemReset();
	       }
	    }
         }
         bzero(rxbuff1,rxcnt1);
         rxcnt1 = 0;
      }
      rx_led = 0;
}


void receive_check2 (void)
{
      unsigned char bcc,idx;
      if (rxcnt2) {
         if ((rxbuff2[0] == STX) && (rxbuff2[5] == ETX) && ((rxbuff2[1] == this_id) || (rxbuff2[1] == 33))) {
	    bcc = rxbuff2[0];
	    for (idx=1;idx<6;idx++) bcc = bcc ^ rxbuff2[idx];
            if (bcc == rxbuff2[6]) {
	       if ((rxbuff2[2] == 0xFF) && (rxbuff2[3] == 0xFF) && (rxbuff2[4] == 0xFF)) {
	       	  NVIC_SystemReset();
	       }
	    }
         }
         bzero(rxbuff2,rxcnt2);
         rxcnt2 = 0;
      }
      rx_led = 0;
}


void receive_check3 (void)
{
      unsigned char bcc,idx;
      if (rxcnt3) {
         if ((rxbuff3[0] == STX) && (rxbuff3[5] == ETX) && ((rxbuff3[1] == this_id) || (rxbuff3[1] == 33))) {
	    bcc = rxbuff3[0];
	    for (idx=1;idx<6;idx++) bcc = bcc ^ rxbuff3[idx];
            if (bcc == rxbuff3[6]) {
	       if ((rxbuff3[2] == 0xFF) && (rxbuff3[3] == 0xFF) && (rxbuff3[4] == 0xFF)) {
	       	  NVIC_SystemReset();
	       }
	    }
         }
         bzero(rxbuff3,rxcnt3);
         rxcnt3 = 0;
      }
      rx_led = 0;
}


void uasrt_send (char *buff,int length)
{
      while (tx_led);
      bzero(txbuff1,256);
      memcpy(txbuff1,buff,length);
      tx_enable1(length);
}


void s_printf (char *form,...)
{
      va_list argptr;
      va_start(argptr,form);
      vsprintf(sbuff,form,argptr);
      uasrt_send(sbuff,strlen(sbuff));
      va_end(argptr);
}


void send_server_data (unsigned char ack)
{
      unsigned short idx,encode;
      unsigned char bcc;
      tcp_tx[0] = STX;
      tcp_tx[1] = 0x41;
      tcp_tx[2] = ack;
      switch (ack) {
         case 0x10:
         case 0x30:
         case 0x31:
         case 0x32:
         case 0x33:
         case 0x34:
         case 0x35:
         case 0x50:
           break;
         case 0x11:
           tcp_tx[3] = wifi_power;
           tcp_tx[4] = dhcp_disable;
           tcp_tx[5] = keep_alive_time;
           tcp_tx[6] = mqtt_mode;
           tcp_tx[7] = ap_mode;
           tcp_tx[8] = (m_ip >> 24) & 0xFF;
           tcp_tx[9] = (m_ip >> 16) & 0xFF;
           tcp_tx[10] = (m_ip >> 8) & 0xFF;
           tcp_tx[11] = (m_ip >> 0) & 0xFF;
           tcp_tx[12] = (m_gw >> 24) & 0xFF;
           tcp_tx[13] = (m_gw >> 16) & 0xFF;
           tcp_tx[14] = (m_gw >> 8) & 0xFF;
           tcp_tx[15] = (m_gw >> 0) & 0xFF;
           tcp_tx[16] = (m_nm >> 24) & 0xFF;
           tcp_tx[17] = (m_nm >> 16) & 0xFF;
           tcp_tx[18] = (m_nm >> 8) & 0xFF;
           tcp_tx[19] = (m_nm >> 0) & 0xFF;
           tcp_tx[20] = (mqtt_port >> 8) & 0xFF;
           tcp_tx[21] = mqtt_port & 0xFF;
           break;
         case 0x12:
           memcpy(tcp_tx + 3, ssid, 32);
           memcpy(tcp_tx + 35, passwd, 32);
           break;
         case 0x13:
           memcpy(tcp_tx + 3, host_url, 64);
           break;
         case 0x14:
           memcpy(tcp_tx + 3, ntp_url, 64);
           break;
         case 0x15:
           memcpy(tcp_tx + 3, login_id, 32);
           memcpy(tcp_tx + 35, login_pw, 32);
           break;
         case 0x16:
           memcpy(tcp_tx + 3, tx_topic, 32);
           memcpy(tcp_tx + 35, rx_topic, 32);
           break;
      }
      tcp_tx[67] = ETX;
      bcc = tcp_tx[0];
      for (idx=1;idx<68;idx++) bcc = bcc ^ tcp_tx[idx];
      tcp_tx[68] = bcc;
      tcp_txcnt = 69;
      txcnt3 = base64_encode((unsigned char*)tcp_tx, tcp_txcnt, txbuff3);
      encode = BASE64_ENCODE_OUT_SIZE(tcp_txcnt);
      if (txcnt3 != (encode - 1)) {
         //txcnt3 = 0;
      }
      tcp_length = txcnt3;
      memcpy(tcp_tx,txbuff3,tcp_length);
      tcp_send = 1;
      send_flag = 1;
}


void send_board_data (void)
{
      unsigned short idx,encode;
      unsigned char bcc;
      tcp_tx[0] = STX;
      tcp_tx[1] = 0x42;
      tcp_tx[2] = (adc_temperature >> 8) & 0xFF;
      tcp_tx[3] = adc_temperature & 0xFF;
      tcp_tx[4] = (adc_voltage1 >> 8) & 0xFF;
      tcp_tx[5] = adc_voltage1 & 0xFF;
      tcp_tx[6] = (adc_voltage2 >> 8) & 0xFF;
      tcp_tx[7] = adc_voltage2 & 0xFF;
      tcp_tx[8] = (dac >> 8) & 0xFF;
      tcp_tx[9] = send_dac & 0xFF;
      tcp_tx[10] = read_key;
      tcp_tx[11] = led1;
      tcp_tx[12] = led2;
      tcp_tx[13] = buz;
      tcp_tx[14] = ETX;
      bcc = tcp_tx[0];
      for (idx=1;idx<15;idx++) bcc = bcc ^ tcp_tx[idx];
      tcp_tx[15] = bcc;
      tcp_txcnt = 16;
      txcnt3 = base64_encode((unsigned char*)tcp_tx, tcp_txcnt, txbuff3);
      encode = BASE64_ENCODE_OUT_SIZE(tcp_txcnt);
      if (txcnt3 != (encode - 1)) {
         //txcnt3 = 0;
      }
      tcp_length = txcnt3;
      memcpy(tcp_tx,txbuff3,tcp_length);
      mqtt_send = 1;
      send_flag = 1;
}


void rx_check (char *rxbuff, unsigned short rxcnt)
{
      unsigned short idx,decode;
      unsigned char bcc;
      tcp_rxcnt = base64_decode(rxbuff, rxcnt, (unsigned char *)net_rx);
      decode = BASE64_DECODE_OUT_SIZE(rxcnt);
      if (decode) {
         if ((net_rx[0] == STX) && (net_rx[1] == 0x41) && (net_rx[67] == ETX) && (tcp_rxcnt >= 69)) {
            bcc = net_rx[0];
            for (idx=1;idx<68;idx++) bcc = bcc ^ net_rx[idx];
            if (bcc == net_rx[68]) {
               ap_mode_time = 0;
               switch (net_rx[2]) {
                  case 0x10:
                  case 0x11:
                  case 0x12:
                  case 0x13:
                  case 0x14:
                  case 0x15:
                  case 0x16:
                    send_server_data(net_rx[2]);
                    break;
                  case 0x50:
                    reboot_flag = 1;
                    send_server_data(0x50);
                    break;
                  case 0x20:
                    wifi_power = net_rx[3];
                    dhcp_disable = net_rx[4];
                    keep_alive_time = net_rx[5];
                    mqtt_mode = net_rx[6];
                    ap_mode = net_rx[7];
                    m_ip = (net_rx[8] & 0xFF) << 24 | (net_rx[9] & 0xFF) << 16 | (net_rx[10] & 0xFF) << 8 | (net_rx[11] & 0xFF);
                    m_gw = (net_rx[12] & 0xFF) << 24 | (net_rx[13] & 0xFF) << 16 | (net_rx[14] & 0xFF) << 8 | (net_rx[15] & 0xFF);
                    m_nm = (net_rx[16] & 0xFF) << 24 | (net_rx[17] & 0xFF) << 16 | (net_rx[18] & 0xFF) << 8 | (net_rx[19] & 0xFF);
                    mqtt_port = (unsigned short)(((net_rx[20] & 0xFF) << 8) + (net_rx[21] & 0xFF));
                    send_server_data(0x30);
                    break;
                  case 0x21:
                    memcpy(ssid, net_rx + 3, 32);
                    memcpy(passwd, net_rx + 35, 32);
                    send_server_data(0x31);
                    break;
                  case 0x22:
                    memcpy(host_url, net_rx + 3, 64);
                    send_server_data(0x32);
                    break;
                  case 0x23:
                    memcpy(ntp_url, net_rx + 3, 64);
                    send_server_data(0x33);
                    break;
                  case 0x24:
                    memcpy(login_id, net_rx + 3, 32);
                    memcpy(login_pw, net_rx + 35, 32);
                    send_server_data(0x34);
                    break;
                  case 0x25:
                    memcpy(tx_topic, net_rx + 3, 32);
                    memcpy(rx_topic, net_rx + 35, 32);
                    send_server_data(0x35);
                    write_wifi_data();
                    break;
               }
            }
         }
         if ((net_rx[0] == STX) && (net_rx[1] == 0x42) && (net_rx[23] == ETX) && (tcp_rxcnt >= 25)) {
            bcc = net_rx[0];
            for (idx=1;idx<24;idx++) bcc = bcc ^ net_rx[idx];
            if (bcc == net_rx[24]) {
               ap_mode_time = 0;
               remote = net_rx[2];
               led1 = net_rx[3];
               led2 = net_rx[4];
               buz = net_rx[5];
               dac = (net_rx[6] & 0xFF) << 8 | (net_rx[7] & 0xFF);
               memcpy(rx_message,net_rx + 8,16);
               send_board_data();
               if (remote) {
               	  if (led1) LED_OUT0 = 0;else LED_OUT0 = 1;
               	  if (led2) LED_OUT1 = 0;else LED_OUT1 = 1;
               	  if (buz) BEEP = 1;else BEEP = 0;
               	  Dac1_Set_Voltage(dac);
               	  rx_message[16] = 0;
               	  message[16] = 0;
               	  if (strcmp(rx_message,message)) {
               	     memcpy(message,rx_message,16);
               	     lcd_printf(1,8,"RX:                  ");
               	     lcd_printf(1,8,"RX: %s ",message);
                  }
               }
            }
         }
      }
}


void clear_flag (void)
{
     pwr_flag = 0;
     mode_flag = 0;
     ssid_flag = 0;
     ip_flag = 0;
     mux_flag = 0;
     server_flag = 0;
     mqtt_cfg_flag = 0;
     mqtt_connect_flag = 0;
     mqtt_sub_flag = 0;
     send_flag = 0;
     ntp_req_flag = 0;
}


void send_data (void)
{
      if (time_out == 0) {
         if (reset_flag) {
            clear_flag();
            sprintf(txbuff3, "AT+RST\r\n");
            mqtt_connect = 0;
            time_out = 1000;
         } else {
            if (pwr_flag) {
               if (wifi_power > 80) wifi_power = 80;
               if (wifi_power < 3) wifi_power = 3;
               if (pwr_error >= 2) {
               	   pwr_error = 0;
               	   wifi_power = 40;
               }
               if (wifi_power < 40) wifi_power = 40;
               sprintf(txbuff3, "AT+RFPOWER=%d\r\n",wifi_power);// 40~80;	old -> min 3
               time_out = 200;
            } else {
               if (mode_flag) {
                  if (ap_mode) {
                     sprintf(txbuff3, "AT+CWMODE=2\r\n");       //1 Station, 2 AP, 3 AP+Station
                  } else {
                     sprintf(txbuff3, "AT+CWMODE=1\r\n");       //1 Station, 2 AP, 3 AP+Station
                  }
                  time_out = 200;
               } else {
                  if (host_flag) {
                     sprintf(txbuff3, "AT+CWHOSTNAME=\"LEEp\"\r\n");	//Host
                     time_out = 200;
                  } else {
                     if (ssid_flag) {
                        if (ap_mode) {
                           sprintf(txbuff3, "AT+CWSAP=\"APLEE\",\"1234567890\",%d,3\r\n",wifi_ch);	//AP Name
                        } else {
                           sprintf(txbuff3, "AT+CWJAP=\"%s\",\"%s\"\r\n",ssid,passwd);
                        }
                        time_out = 5000;
                     } else {
                        if (ip_flag) {
                           if (ap_mode) {
                              sprintf(txbuff3, "AT+CIPAP=\"192.168.1.254\",\"192.168.1.254\",\"255.255.255.0\"\r\n");
                              time_out = 500;
                           } else {
                              if (dhcp_disable) {
                                 sprintf(txbuff3, "AT+CIPSTA=\"%d.%d.%d.%d\",\"%d.%d.%d.%d\",\"%d.%d.%d.%d\"\r\n", // Ip, GateWay, NetMask
                                 (m_ip >> 0) & 0xFF,(m_ip >> 8) & 0xFF,(m_ip >> 16) & 0xFF,(m_ip >> 24) & 0xFF,
                                 (m_gw >> 0) & 0xFF,(m_gw >> 8) & 0xFF,(m_gw >> 16) & 0xFF,(m_gw >> 24) & 0xFF,
                                 (m_nm >> 0) & 0xFF,(m_nm >> 8) & 0xFF,(m_nm >> 16) & 0xFF,(m_nm >> 24) & 0xFF);
                                 time_out = 500;
                              } else {
                                 ip_flag = 0;
                                 time_out = 0;
                                 mux_flag = 1;
                              }
                           }
                        } else {
                           if (mux_flag) {
                              sprintf(txbuff3, "AT+CIPMUX=1\r\n"); // 0 Single, 1 Mult
                              time_out = 200;
                           } else {
                              if (server_flag) {
                                 sprintf(txbuff3, "AT+CIPSERVER=1,11000\r\n");     // 0 Disable 1 enable, Port
                                 time_out = 200;
                              } else {
                                 if (mqtt_cfg_flag) {
                                    sprintf(txbuff3, "AT+MQTTUSERCFG=0,%d,\"LEEp\",\"%s\",\"%s\",0,0,\"\"\r\n",
                                                   mqtt_mode+1,login_id,login_pw);	// Name
                                    time_out = 200;
                                 } else {
                                    if (mqtt_connect_flag) {
                                       sprintf(txbuff3, "AT+MQTTCONN=0,\"%s\",%d,1\r\n",host_url,mqtt_port);
                                       time_out = 5000;
                                    } else {
                                       if (mqtt_sub_flag) {
                                          mqtt_connect = 1;
                                          sprintf(txbuff3, "AT+MQTTSUB=0,\"%s\",1\r\n",rx_topic);
                                          time_out = 5000;
                                       } else {
                                          if (send_flag) {
                                             if (ap_mode) {
                                                sprintf(txbuff3, "AT+CIPSEND=%d,%d\r\n",gid,tcp_length);
                                                send_data_flag = 1;
                                                time_out = 100;
                                             } else {
                                             	if (mqtt_send) {
                                                   if (mqtt_connect) {
                                                      sprintf(txbuff3, "AT+MQTTPUBRAW=0,\"%s\",%d,0,0\r\n",tx_topic,txcnt3);
                                                      send_data_flag = 1;
                                                      mqtt_send = 0;
                                                      time_out = 100;
                                                   }
                                                }
                                                if (tcp_send) {
                                                   tcp_send = 0;
                                                   sprintf(txbuff3, "AT+CIPSEND=%d,%d\r\n",gid,tcp_length);
                                                    send_data_flag = 1;
                                                    time_out = 100;
                                                }
                                             }
                                             send_flag = 0;
                                          } else {
                                             if (ntp_flag) {
                                                sprintf(txbuff3, "AT+CIPSNTPCFG=1,8,\"%s\"\r\n",ntp_url);
                                                time_out = 200;
                                             } else {
                                                if (ntp_req_flag)  {
                                                   sprintf(txbuff3, "AT+CIPSNTPTIME?\r\n");
                                                   time_out = 1000;
                                                }
                                             }
                                          }
                                       }
                                    }
                                 }
                              }
                           }
                        }
                     }
                  }
               }
            }
         }
         if (time_out) {
            txcnt3 = strlen(txbuff3);
            tx_enable3(txcnt3);
            bzero(rxbuff3,2048);
            rxcnt3 = 0;
         }
      }
}


void send_mode (void)
{
      unsigned short idx;
      unsigned char bcc;
      txbuff3[0] = STX;
      txbuff3[1] = this_id;
      txbuff3[2] = 0x41;
      txbuff3[3] = 0x20;
      txbuff3[4] = 0x06;
      txbuff3[5] = 0x20;
      txbuff3[6] = 0x20;
      txbuff3[7] = ETX;
      bcc = txbuff3[0];
      for (idx=1;idx<8;idx++) bcc = bcc ^ txbuff3[idx];
      txbuff3[8] = bcc;
      tx_enable3(9);
}


void receive_check_wifi (void)
{
      unsigned short idx,pos;
      unsigned char bcc,wait_wifi,rxok;
      if (rxcnt3 >= 2048) rxcnt3 = 0;
      if (rxcnt3) {
      	 rxok = 0;
         if ((rxbuff3[0] == STX) && (rxbuff3[1] == this_id) && (rxbuff3[5] == ETX) && (rxcnt3 >= 7)) {
            bcc = rxbuff3[0];
            for (idx=1;idx<6;idx++) bcc = bcc ^ rxbuff3[idx];
            if (bcc == rxbuff3[6]) {
               if ((rxbuff3[2] == 0xFF) && (rxbuff3[3] == 0xFF) && (rxbuff3[4] == 0xFF)) {
                  NVIC_SystemReset();
               } else {
                  send_mode();
                  ap_mode = 1;
                  reset_flag = 1;
               }
               time_out = 100;
            }
         }
         // 2.x.x
         if ((rxbuff3[rxcnt3 - 3] == 0x0D) && (rxbuff3[rxcnt3 - 2] == 0x0A) && (rxbuff3[rxcnt3 - 1] == '>') && (rxcnt3 >= 4) && (rxok == 0)) {
            if (send_data_flag) {
               send_data_flag = 0;
               memcpy(txbuff3,tcp_tx,tcp_length);
               tx_enable3(tcp_length);
               time_out = 10;
               main_time = 0;
               //s_printf("Boot1 SEND %d \r\n",tcp_length);
            }
            rxok = 1;
         }
	 //1.x.x
	 if ((rxbuff3[rxcnt3 - 4] == 0x0D) && (rxbuff3[rxcnt3 - 3] == 0x0A) && (rxbuff3[rxcnt3 - 2] == '>') && (rxbuff3[rxcnt3 - 1] == 0x20) && (rxcnt3 >= 5) && (rxok == 0)) {
	    if (send_data_flag) {
               send_data_flag = 0;
               memcpy(txbuff3,tcp_tx,tcp_length);
	       tx_enable3(tcp_length);
	       time_out = 10;
	       main_time = 0;
	       //s_printf("Boot2 SEND %d \r\n",tcp_length);
	    }
	    rxok = 1;
	 }
         //AT+CIPSEND=, AT+MQTTPUBRAW=
         if (((strncmp(rxbuff3,"AT+CIPSEND=",11) == 0) || (strncmp(rxbuff3,"AT+MQTTPUBRAW=",14) == 0)) && (rxok == 0)) {
            if (rxbuff3[rxcnt3 - 1] == '>') {
               if (send_data_flag) {
                  send_data_flag = 0;
                  memcpy(txbuff3,tcp_tx,tcp_length);
                  tx_enable3(tcp_length);
                  bzero(rxbuff3, rxcnt3);
                  rxcnt3 = 0;
                  time_out = 0;
                  main_time = 0;
               }
            }
            rxok = 1;
         }
         //\r\n+IPD     1.xx    2.xx,3.xx +\r\n
         if (((rxbuff3[0] == 0x0D) && (rxbuff3[1] == 0x0A) && (strncmp(rxbuff3 + 2,"+IPD",4) == 0)) && (rxok == 0)) {
            //RL+IPD,0,92:
            if (rxbuff3[6] == ',') {
               if (rxbuff3[8] == ',') {
                  if (rxbuff3[10] == ':') {
                     rx_length = rxbuff3[9] - 0x30;
                     rx_offset = 11;
                  } else if (rxbuff3[11] == ':') {
                     rx_length = (rxbuff3[9] - 0x30) * 10 + (rxbuff3[10] - 0x30);
                     rx_offset = 12;
                  } else if (rxbuff3[12] == ':') {
                     rx_length = (rxbuff3[9] - 0x30) * 100 + (rxbuff3[10] - 0x30) * 10 + (rxbuff3[11] - 0x30);
                     rx_offset = 13;
                  } else if (rxbuff3[13] == ':') {
                     rx_length = (rxbuff3[9] - 0x30) * 1000 + (rxbuff3[10] - 0x30) * 100 + (rxbuff3[11] - 0x30) * 10 + (rxbuff3[12] - 0x30);
                     rx_offset = 14;
                  }
                  if (rxcnt3 >= (rx_offset + rx_length)) {
                     gid = rxbuff3[7] - 0x30;
                     rx_check(rxbuff3 + rx_offset,rx_length);
                     time_out = 0;
                     main_time = 0;
                     bzero(rxbuff3, rxcnt3);
                     rxcnt3 = 0;
                  }
               }
            }
            rxok = 1;
         }
         //mqtt
         if ((strncmp(rxbuff3,"+MQTTSUBRECV", 12) == 0) && (rxok == 0)) {
            pos = 17 + rx_topic_length;
            if (rxbuff3[pos] == ',') {
               if (rxbuff3[pos + 2] == ',') {
                  rx_length = rxbuff3[pos + 1] - 0x30;
                  rx_offset = pos + 3;
               } else if (rxbuff3[pos + 3] == ',') {
                  rx_length = (rxbuff3[pos + 1] - 0x30) * 10 + (rxbuff3[pos + 2] - 0x30);
                  rx_offset = pos + 4;
               } else if (rxbuff3[pos + 4] == ',') {
                  rx_length = (rxbuff3[pos + 1] - 0x30) * 100 + (rxbuff3[pos + 2] - 0x30) * 10 + (rxbuff3[pos + 3] - 0x30);
                  rx_offset = pos + 5;
               } else if (rxbuff3[pos + 5] == ',') {
                  rx_length = (rxbuff3[pos + 1] - 0x30) * 1000 + (rxbuff3[pos + 2] - 0x30) * 100 + (rxbuff3[pos + 3] - 0x30) * 10 + (rxbuff3[pos + 4] - 0x30);
                  rx_offset = pos + 6;
               }
               if (rxcnt3 >= (rx_offset + rx_length)) {
                  rx_check(rxbuff3 + rx_offset,rx_length);
                  bzero(rxbuff3, rxcnt3);
                  rxcnt3 = 0;
                  time_out = 0;
                  main_time = 0;
               }
            }
            rxok = 1;
         }
         if (((rxbuff3[rxcnt3 - 2] == 0x0D) && (rxbuff3[rxcnt3 - 1] == 0x0A)) && (rxok == 0)) {
            main_time = 0;
            if ((strncmp(rxbuff3 + (rxcnt3 - 7),"ready",5) == 0) && (rxok == 0)) {
               time_out = 0;
               reset_flag = 0;
               pwr_flag = 1;
               rxok = 1;
            }
            if ((strncmp(rxbuff3 + (rxcnt3 - 11),"busy p...",4) == 0) && (rxok == 0)) {
               wait_wifi = 1;
               rxok = 1;
            }
            //WIFI CONNECTED\r\nWIFI GOT IP\r\n		1.x.x
            if ((strncmp(rxbuff3,"WIFI CONNECTED",14) == 0) && (ap_mode == 0) && (rxok == 0)) {
               if (strncmp(rxbuff3 + 16,"WIFI GOT IP",11) == 0) {
               	  if (ssid_flag) {
               	     ssid_check =  1;
               	  }
               }
               rxok = 1;
            }
            if (((strncmp(rxbuff3 + (rxcnt3 - 7),"ERROR",5) == 0) || (strncmp(rxbuff3 + (rxcnt3 - 6),"FAIL",4) == 0)) && (rxok == 0)) {
               if ((ssid_flag) && (rxok == 0)) {
                  ssid_error++;
                  if (ssid_error >= 5) {
                     ssid_error = 0;
                     dhcp_disable = 0;
                     keep_alive_time = 60;
                     mqtt_mode = 0;
                     mqtt_port = 1883;
                     ap_mode = 1;
                     reset_flag = 1;
                     time_out = 0;
                  }
                  rxok = 1;
               }
               if ((mqtt_connect_flag) && (rxok == 0)) {
                  mqtt_connect_count++;
                  if (mqtt_connect_count >= 5) {
                     mqtt_connect_count = 0;
                     reset_flag = 1;
                     main_time = 0;
                     mqtt_check_cnt++;
                     if (mqtt_check_cnt >= 2) {
                        mqtt_check_cnt = 0;
                        ap_mode = 1;
                        time_out = 0;
                     }
                  }
                  rxok = 1;
               }
               if ((mqtt_sub_flag) && (rxok == 0)) {
                  mqtt_sub_count++;
                  if (mqtt_sub_count >= 10) {
                     mqtt_sub_count = 0;
                     reset_flag = 1;
                     time_out = 0;
                  }
                  rxok = 1;
               }
               if ((send_flag) && (rxok == 0)) {
                  send_count++;
                  if (send_count >= 20) {
                     send_count = 0;
                     reset_flag = 1;
                     time_out = 0;
                  }
                  rxok = 1;
               }
               if ((pwr_flag) && (rxok == 0)) {
                  pwr_error++;
                  rxok = 1;
               }
            } else {
               if ((rxbuff3[rxcnt3 - 4] == 'O') && (rxbuff3[rxcnt3 - 3] == 'K')) {
                  if ((rxbuff3[rxcnt3 - 6] == 0x0D) && (rxbuff3[rxcnt3 - 5] == 0x0A)) {
                     rxok = 0;
                     if ((strncmp(rxbuff3,"+MQTTPUB",8) == 0) && (strncmp(rxbuff3,"SEND",4) == 0) && (rxok == 0)) {
                        if (send_flag) {
                           send_flag = 0;
                           send_count = 0;
                        }
                        rxok = 1;
                     }
                     if (((rxbuff3[0] == 0x0D) && (rxbuff3[1] == 0x0A) && (rxbuff3[2] == 'O') && (rxbuff3[3] == 'K')) && (rxok == 0)) {
                        //\r\nOK\r\n
                        if ((ap_mode) && (ssid_flag)) {
                           ssid_flag = 0;
                           ip_flag = 1;
                           time_out = 0;
                        }
                        if ((ap_mode == 0) && (ssid_check)) {
                           ssid_check = 0;
                           ssid_flag = 0;
                           //ip_flag = 1;	//Not use 2.xx
                           time_out = 0;
                        }
                        if (mqtt_sub_flag) {
                           mqtt_sub_flag = 0;
                           ntp_flag = 1;
                           time_out = 0;
                        }
                        rxok = 1;
                     }
                     if ((strncmp(rxbuff3,"AT+CWSAP",8) == 0) && (rxok == 0)) {	//1.xx
                        if ((ap_mode) && (ssid_flag)) {
                           ssid_flag = 0;
                           ip_flag = 1;
                           time_out = 0;
                        }
                     }
                     //WIFI CONNECTED\r\nWIFI GOT IP\r\nOK
                     //              \r\nWIFI GOT IP\r\nOK
                     if ((strncmp(rxbuff3 + rxcnt3 - 19,"WIFI GOT IP",11) == 0) && (rxok == 0))  {
                        //if (strncmp(rxbuff3 + rxcnt3 - 35,"WIFI CONNECTED",11) == 0) {
                        if (ssid_flag) {
                           ssid_flag = 0;
                           ip_flag = 1;	//OK
                           time_out = 0;
                        } else {
                           if (dhcp_disable) {
                              if (ip_flag) {
                                 ip_flag = 0;
                                 mux_flag = 1;
                                 time_out = 0;
                              }
                           }
                        }
                        rxok = 1;
                     }
                     if ((strncmp(rxbuff3,"AT+CIPSERVER",12) == 0) && (rxok == 0)) {
                        if (server_flag) {
                           server_flag = 0;
                           if (ap_mode == 0) {
                              //host_flag = 1;
                              if (mqtt_protocol) {
                                 mqtt_cfg_flag = 1;
                              }
                           }
                           time_out = 0;
                        }
                        rxok = 1;
                     }
                     if ((strncmp(rxbuff3,"AT+RFPOWER",10) == 0) && (rxok == 0)) {
                        if (pwr_flag) {
                           pwr_flag = 0;
                           mode_flag = 1;
                           time_out = 0;
                        }
                        rxok = 1;
                     }
                     if ((strncmp(rxbuff3,"AT+CWMODE",9) == 0) && (rxok == 0)) {
                        if (mode_flag) {
                           mode_flag = 0;
                           if (ap_mode == 0) {
                              host_flag = 1;
                           } else {
                              ssid_flag = 1;
                           }
                           time_out = 0;
                        }
                     }
                     if (((strncmp(rxbuff3,"AT+CIPAP",8) == 0) || (strncmp(rxbuff3,"AT+CIPSTA",9) == 0)) && (rxok == 0)) {
                        if (ip_flag) {
                           ip_flag = 0;
                           mux_flag = 1;
                           time_out = 0;
                        }
                        rxok = 1;
                     }
                     if ((strncmp(rxbuff3,"AT+CIPMUX",9) == 0) && (rxok == 0)) {
                        if (mux_flag) {
                           mux_flag = 0;
                           server_flag = 1;
                           time_out = 0;
                        }
                        rxok = 1;
                     }
                     if ((strncmp(rxbuff3,"AT+MQTTUSERCFG",14) == 0) && (rxok == 0)) {
                        if (mqtt_cfg_flag) {
                           mqtt_cfg_flag = 0;
                           mqtt_connect_flag = 1;
                           time_out = 0;
                        }
                        rxok = 1;
                     }
                     if ((strncmp(rxbuff3,"+MQTTCONNECTED",14) == 0) && (rxok == 0)) {
                        if (mqtt_connect_flag) {
                           mqtt_connect_flag = 0;
                           mqtt_sub_flag = 1;
                           mqtt_check_cnt = 0;
                           mqtt_connect_count = 0;
                           time_out = 0;
                        }
                        rxok = 1;
                     }
                     if ((strncmp(rxbuff3,"AT+CIPSNTPCFG",13) == 0) && (rxok == 0)) {
                        if (ntp_flag) {
                           ntp_flag = 0;
                           ntp_req_flag = 1;
                           mqtt_sub_count = 0;
                        }
                        rxok = 1;
                     }
                     if ((strncmp(rxbuff3,"AT+CIPSNTPTIME",14) == 0) && (rxok == 0)) {
                        if (ntp_req_flag) {
                           ntp_req_flag = 0;
                        }
                        rxok = 1;
                     }
                     if ((strncmp(rxbuff3,"AT+CWHOSTNAME",13) == 0) && (rxok == 0)) {
                        if (host_flag) {
                           host_flag = 0;
                           if (ap_mode == 0) {
                              ssid_flag = 1;
                           }
                        }
                        rxok = 1;
                     }
                  }
               }
            }
         }
         if (rxcnt3) {
            if (wait_wifi == 0) {
               time_out = 0;
            }
            bzero(rxbuff3, rxcnt3);
            rxcnt3 = 0;
         }
      }
      rx_led = 0;
}


void serial_check (void)
{
      unsigned int max_time;
      if ((rxcnt1 != 0) && (rxck1 >= 3)) {
         rxck1 = 0;
         receive_check1();
      }
      if ((rxcnt2 != 0) && (rxck2 >= 3)) {
         rxck2 = 0;
         receive_check2();
      }
      if ((rxcnt3 != 0) && (rxck3 >= 5)) {
         rxck3 = 0;
         receive_check_wifi();
      }
      if (time_out == 0) {
         send_data();
      } else {
         time_out--;
      }
      main_time++;
      if (mqtt_connect) {
         max_time = 600000;
         time_req++;
         if (time_req >= 1800000) {
            time_req = 0;
            ntp_req_flag = 1;
         }
         status_time++;
         if (status_time >= 30000) {
            status_time = 0;
            send_board_data();
         }
      } else {
         max_time = 60000;
      }
      if (main_time >= max_time) {
         main_time = 0;
         reset_flag = 1;
      }
      if (reboot_flag) {
         reboot_count++;
         if (reboot_count >= 500) {
            reboot_count = 0;
            reboot_flag = 0;
            reset_flag = 1;
         }
      }
      if (ap_mode) {
      	 ap_mode_time++;
      	 if (ap_mode_time >= 600000) {
      	    ap_mode_time = 0;
      	    ap_mode = 0;
      	    reset_flag = 1;
      	 }
      }
}


void change_ap_mode (void)
{
     dhcp_disable = 0;
     keep_alive_time = 60;
     mqtt_mode = 0;
     mqtt_port = 1883;
     ap_mode = 1;
     reset_flag = 1;
     time_out = 0;
}


void serial_init (void)
{
      USART_InitTypeDef	USART_InitStructure;
      NVIC_InitTypeDef NVIC_InitStructure;
      USART1_PORT = 0;
      USART3_PORT = 0;
      rxcnt1 = 0;
      txcnt1 = 0;
      maxtx1 = 0;
      rxcnt2 = 0;
      txcnt2 = 0;
      maxtx2 = 0;
      rxcnt3 = 0;
      txcnt3 = 0;
      maxtx3 = 0;
      /* Enable the USART1 Interrupt */
      NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
      switch (USART1_PORT) {
         case 0:
           GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
           GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
           GPIO_Init_Pin(GPIOA,TXD1,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);
           GPIO_Init_Pin(GPIOA,RXD1,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);
           break;
         case 1:
           GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1);
           GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);
           GPIO_Init_Pin(GPIOB,GPIO_Pin_6,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);
           GPIO_Init_Pin(GPIOB,GPIO_Pin_7,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);
           break;
      }
      /* Enable the USART2 Interrupt */
      NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
      GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
      GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
      GPIO_Init_Pin(GPIOA,TXD2,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);
      GPIO_Init_Pin(GPIOA,RXD2,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);
      /* Enable the USART3 Interrupt */
      NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
      switch (USART3_PORT) {
      	 case 0:
           GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
           GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);
           /* Configure USART3 TX (PB10) */
           GPIO_Init_Pin(GPIOB,TXD3,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);
           /* Configure USART3 TX (PB10) */
           GPIO_Init_Pin(GPIOB,RXD3,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);
           break;
         case 1:
           GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_USART3);
           GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3);
           /* Configure USART3 TX (PC10) */
           GPIO_Init_Pin(GPIOC,GPIO_Pin_10,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);
           /* Configure USART3 RX (PC11) */
           GPIO_Init_Pin(GPIOC,GPIO_Pin_11,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);
           break;
         case 2:
           GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
           GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);
           /* Configure USART3 TX (PD8) */
           GPIO_Init_Pin(GPIOD,GPIO_Pin_8,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);
           /* Configure USART3 RX (PD9) */
           GPIO_Init_Pin(GPIOD,GPIO_Pin_9,GPIO_Speed_100MHz,GPIO_Mode_AF_PP_PU);
           break;
      }
      USART_InitStructure.USART_BaudRate = 115200;
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;
      USART_InitStructure.USART_StopBits = USART_StopBits_1;
      USART_InitStructure.USART_Parity = USART_Parity_No;
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
      USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
      USART_DeInit(USART1);
      /* Enable USART1 clock */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
      /* Configure USART1 */
      USART_Init(USART1, &USART_InitStructure);
      /* Enable USART1 Receive and Transmit interrupts */
      USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
      /* Enable the USART1 */
      USART_Cmd(USART1, ENABLE);
      USART_DeInit(USART2);
      /* Enable USART2 clock */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
      /* Configure USART2 */
      USART_Init(USART2, &USART_InitStructure);
      /* Enable USART2 Receive and Transmit interrupts */
      USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
      /* Enable the USART2 */
      USART_Cmd(USART2, ENABLE);
      USART_DeInit(USART3);
      /* Enable USART3 clock */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
      /* Configure USART3 */
      USART_Init(USART3, &USART_InitStructure);
      /* Enable USART3 Receive and Transmit interrupts */
      USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
      /* Enable the USART3 */
      USART_Cmd(USART3, ENABLE);
      read_wifi_data();
      this_id = 1;
      reset_flag = 1;
      rx_topic_length = strlen(rx_topic);
      mqtt_protocol = 1;
      wifi_ch = 5;
}