/********************************************************************************/
/* prototype.h                                                                  */
/* STM32F407ZGT6                                                                */
/* (Lee ChangWoo HL2IRW  hl2irw@daum.net 010-8573-6860)                 	*/
/* stm32f4x_test								*/
/********************************************************************************/

// i2c_app.c

extern unsigned char eeprom_read (unsigned char address);
extern void eeprom_write (unsigned char address,unsigned char data);
extern void init_i2c_24xx (void);

// key.c
#define KEY0				PE_INPUT(4)
#define KEY1				PE_INPUT(3)
#define KEY2				PE_INPUT(2)
#define KEY_UP				PA_INPUT(0)
#define BEEP				PF_OUTPUT(8)

#define K_UP				0x08
#define K_DOWN				0x02
#define K_RIGHT				0x01
#define K_LEFT				0x04

extern volatile unsigned char key_value,beep_on;

extern void beep_control (unsigned char ctl);
extern unsigned char key_read (void);
extern void key_init (void);

// lcd.c

typedef struct
{
      unsigned short width;
      unsigned short height;
      unsigned short id;
      unsigned char dir;
      unsigned short wramcmd;
      unsigned short setxcmd;
      unsigned short setycmd;
} _lcd_dev;

extern unsigned short foreground_color,background_color,maxx,maxy,max_col,max_row,eng_mode;
extern volatile unsigned char hangul_mode;
extern _lcd_dev lcddev;

extern void delay_us (unsigned short delay);
extern void LCD_DrawPoint (unsigned short x, unsigned short y);
extern void LCD_Fast_DrawPoint (unsigned short x, unsigned short y, unsigned short color);
extern void LCD_SSD_BackLightSet (unsigned char pwm);
extern void LCD_Display_Dir (unsigned char dir);
extern void LCD_Set_Window (unsigned short sx, unsigned short sy, unsigned short width, unsigned short height);
extern void LCD_Clear (unsigned short color);
extern void LCD_Fill (short sx, short sy, short ex, short ey, unsigned short color);
extern void LCD_Color_Fill (short sx, short sy, short ex, short ey, unsigned short *color);
extern void LCD_DrawLine (short x1, short y1, short x2, short y2);
extern void LCD_DrawRectangle (short x1, short y1, short x2, short y2);
extern void LCD_Draw_Circle (short x0, short y0,unsigned short r);
extern void set_color (unsigned short color);
extern void set_background_color (unsigned short color);
extern void set_pixel (unsigned short x, unsigned short y, unsigned short color);
extern unsigned short uni_to_kssm2 (unsigned short wchar);
extern unsigned short uni_to_kssm (unsigned short wchar);
extern unsigned short ks5601_to_kssm (unsigned short wchar);
extern void put_engxy (unsigned short x,unsigned short y,char pdata);
extern void put_hanxy (unsigned short x,unsigned short y,unsigned short pdata);
extern void lcd_printf (unsigned short x,unsigned short y,char *form,...);
extern void LCD_Init (void);

// random_disp.c
extern void random_display (void);

// touch_panel.c

#define CT_MAX_TOUCH			5
#define TP_PRES_DOWN			0x80
#define TP_CATH_PRES			0x40

typedef struct
{
      unsigned short x[CT_MAX_TOUCH];
      unsigned short y[CT_MAX_TOUCH];
      unsigned char sta;
      float xfac;
      float yfac;
      short xoff;
      short yoff;
      unsigned char touchtype;
} _m_tp_dev;

extern _m_tp_dev tp_dev;
extern void TP_Adjust (void);
extern unsigned char TP_Scan (unsigned char tp);
extern unsigned char TP_Init (void);
extern void TP_Drow_Touch_Point (unsigned short x,unsigned short y,unsigned short color);

// ir_remocon.c

extern void ir_remocon_init (void);
extern void ir_process (void);

// touch_pad.c
extern unsigned char TPAD_Init (unsigned char psc);
extern unsigned char TPAD_Scan (unsigned char mode);

// dac1.c
extern void Dac1_Set_Voltage (unsigned short voltage);
extern void Dac1_Init (void);
extern void change_ap_mode (void);

// adc.c
extern unsigned short convert_voltage (unsigned short adc_data);
extern short cal_temperature (unsigned short adc_data);
extern void ADC_Config (void);
extern void adc_process (void);

// box3.c
extern void bar (int x1, int y1, int x2, int y2, int color);
extern void norm_3d (int x1,int y1,int x2,int y2,int color);
extern void box_3d (int x1, int y1, int x2, int y2, int out_color, int in_color);
extern void vert_3d (int x1, int y1, int x2, int y2, int color);
extern void shade (int x1, int y1, int x2, int y2, int color);
extern void two_line_3d (int x1, int y1, int x2, int y2, int in_color, int out_color);
extern void double_3d (int x1, int y1, int x2, int y2, int color);
extern void round_3d (int x1, int y1, int x2, int y2, int color);
extern void inline_3d (int x1, int y1, int x2, int y2, int color);
extern void line_3d (int x1, int y1, int x2, int y2);
extern  void demo_box (void);
