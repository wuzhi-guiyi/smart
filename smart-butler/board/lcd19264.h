#ifndef __LCD19264_H
#define __LCD19264_H 	   
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"

#include "clock_config.h"
#include "pin_mux.h"


extern	gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput, 0,
    };
	
extern	gpio_pin_config_t psb_config = {
        kGPIO_DigitalOutput, 0,
    };
	
extern	gpio_pin_config_t E2_config = {
        kGPIO_DigitalOutput, 0,
    };
	
extern	gpio_pin_config_t E1_config = {
        kGPIO_DigitalOutput, 0,
    };
	
extern	gpio_pin_config_t rw_config = {
        kGPIO_DigitalOutput, 0,
    };
	
extern	gpio_pin_config_t rs_config = {
        kGPIO_DigitalOutput, 0,
    };
	
extern	gpio_pin_config_t rest_config = {
        kGPIO_DigitalOutput, 0,
    };


#define lcd19264_sid(a)	if (a)	\
					GPIO_WritePinOutput(BOARD_LED_GPIO,RW_PIN,1);\
					else		\
					GPIO_WritePinOutput(BOARD_LED_GPIO,RW_PIN,0);
					
#define lcd19264_reset(a)	if (a)	\
					GPIO_WritePinOutput(BOARD_LED_GPIO,RSTB_PIN,1);\
					else		\
					GPIO_WritePinOutput(BOARD_LED_GPIO,RSTB_PIN,0);
					
#define lcd19264_cs(a)	if (a)	\
					GPIO_WritePinOutput(BOARD_LED_GPIO,RS_PIN,1);\
					else		\
					GPIO_WritePinOutput(BOARD_LED_GPIO,RS_PIN,0);
					
#define lcd19264_sclk1(a)	if (a)	\
					GPIO_WritePinOutput(BOARD_LED_GPIO,E1_PIN,1);\
					else		\
					GPIO_WritePinOutput(BOARD_LED_GPIO,E1_PIN,0);

#define lcd19264_sclk2(a)	if (a)	\
					GPIO_WritePinOutput(BOARD_LED_GPIO,E2_PIN,1);\
					else		\
					GPIO_WritePinOutput(BOARD_LED_GPIO,E2_PIN,0);

#define lcd19264_psb(a)	if (a)	\
					GPIO_WritePinOutput(BOARD_LED_GPIO,PSB_PIN,1);\
					else		\
					GPIO_WritePinOutput(BOARD_LED_GPIO,PSB_PIN,0);
				
					
void DELAY_CLS(void);
void display(unsigned char fst, unsigned char snd);
void Check1(void);
void Check2(void);

void LCD_CLS1(void);
void LCD_CLS2(void);
void gotoxy(unsigned int x, unsigned int y);
void SendStr1(char *ptString);
void SendStr2(char *ptString);

void init_lcd(void);
void LCD_Test(void);	
void display_pic(unsigned char *pic);
					
void show_char(unsigned char x,unsigned char y,char *p);
					

void lcd_gpioinit(void);
void transfer_command_IC1(int data1);
void transfer_data_IC1(int data1);
void transfer_command_IC2(int data1);
void transfer_data_IC2(int data1);
void screen_shadow(void);
void clear_screen(void);
void display_32X32(int y,int x,char *dp);
void display_16X32(int y,int x,char *dp);
void display_192X64(char *dp);
void lcd_allinit(void);
void display_char(int y,int x,int char_total,char *dp);

void disp_kuang(void);
void Display_num(unsigned char x,unsigned char y,int num,char length);	//intÊý×ÖÏÔÊ¾º¯Êý
void Display_float_num(unsigned char x,unsigned char y,float num);

void Display_Data_Time(int *dt);
void Display_Test_Name(void);
void Lcd19264_Running_Init(void);



void lcd_test(void);				
					
void All_Screen_Clear(void);




					
					
					
#endif

					



