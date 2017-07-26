/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "board.h"
#include "fsl_lpuart.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "includes.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "fsl_port.h"
#include "fsl_common.h"




/****************************************Definitions*******************************************/
#define BOARD_LED_GPIO BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN

#define BOARD_WALLE_GPIO GPIOA
#define STOP_PIN 1U
#define MOVE_PIN 0U
#define LEFT_PIN 19U
#define RIGHT_PIN 16U
#define OPP_PIN 17U
#define DANCE_PIN 18U

#define BOARD_LCD_GPIO GPIOC
#define PSB_PIN 6U
#define E2_PIN 7U
#define E1_PIN 19U
#define RW_PIN 16U
#define RS_PIN 4U
#define RSTB_PIN 17U


#define DEMO_LPUART LPUART0
#define DEMO_LPUART_CLKSRC kCLOCK_CoreSysClk
#define DEMO_LPUART_CLK_FREQ CLOCK_GetFreq(kCLOCK_CoreSysClk)
#define DEMO_LPUART_IRQn LPUART0_IRQn
#define DEMO_LPUART_IRQHandler LPUART0_IRQHandler

#define DEMO_RING_BUFFER_SIZE 16

#define DHT11_FGPIO FGPIOC
#define DHT11_GPIO  GPIOC
#define DHT11_Port  PORTC
#define DHT11_Pin   18U

#define	DHT11_DQ_OUT_0  GPIO_WritePinOutput(DHT11_GPIO, DHT11_Pin, 0)
#define	DHT11_DQ_OUT_1  GPIO_WritePinOutput(DHT11_GPIO, DHT11_Pin, 1)
//read 1 or 0
#define	DHT11_DQ_IN     GPIO_ReadPinInput(DHT11_GPIO,DHT11_Pin)

#define action0 0x00	//空闲
#define action1 0x01
#define action2 0x02
#define action3 0x03
#define action4 0x04
#define action5 0x05
#define action6 0x06


#define BOARD_SW_GPIO BOARD_SW4_GPIO
#define BOARD_SW_PORT BOARD_SW4_PORT
#define BOARD_SW_GPIO_PIN BOARD_SW4_GPIO_PIN
#define BOARD_SW_IRQ BOARD_SW4_IRQ
#define BOARD_SW_IRQ_HANDLER BOARD_SW4_IRQ_HANDLER
#define BOARD_SW_NAME BOARD_SW4_NAME


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




/****************************************函数*******************************************/
void wifi_msg_program(void);
void lcd_show_program(void);

void DHT11_IO_IN(void);
void DHT11_IO_OUT(void);
void DHT11_Rst(void);
unsigned char DHT11_Check(void);
unsigned char DHT11_Read_Bit(void);
unsigned char DHT11_Read_Byte(void);
unsigned char DHT11_Read_Data(unsigned char *temp,unsigned char *humi);
unsigned char DHT11_Init(void);



					
/****************************************变量*******************************************/
unsigned char walle_state = 0;
unsigned char wifi_msg = 7;
unsigned char DHT11_temp = 0,DHT11_humi=0;

unsigned int alarm_times = 0;	//报警次数

uint8_t g_tipString[] =
    "Lpuart functional API interrupt example\r\nBoard receives characters then sends them out\r\nNow please input:\r\n";

uint8_t demoRingBuffer[DEMO_RING_BUFFER_SIZE];
volatile uint16_t txIndex; /* Index of the data to send out. */
volatile uint16_t rxIndex; /* Index of the memory to save new arrived data. */

uint8_t USART_RX_BUF[200];     //接收缓冲,最大USART_REC_LEN个字节.
uint16_t USART_RX_STA=0;       //接收状态标记	



	gpio_pin_config_t sw_config = {
        kGPIO_DigitalInput, 0,
    };
					
	gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput, 0,
    };
	
	gpio_pin_config_t psb_config = {
        kGPIO_DigitalOutput, 0,
    };
	
	gpio_pin_config_t E2_config = {
        kGPIO_DigitalOutput, 0,
    };
	
	gpio_pin_config_t E1_config = {
        kGPIO_DigitalOutput, 0,
    };
	
	gpio_pin_config_t rw_config = {
        kGPIO_DigitalOutput, 0,
    };
	
	gpio_pin_config_t rs_config = {
        kGPIO_DigitalOutput, 0,
    };
	
	gpio_pin_config_t rest_config = {
        kGPIO_DigitalOutput, 0,
    };
	
	gpio_pin_config_t stop_config = {
        kGPIO_DigitalOutput, 0,
    };
	
	gpio_pin_config_t move_config = {
        kGPIO_DigitalOutput, 0,
    };
	
	gpio_pin_config_t opp_config = {
        kGPIO_DigitalOutput, 0,
    };
	
	gpio_pin_config_t circle_config = {
        kGPIO_DigitalOutput, 0,
    };
	
	gpio_pin_config_t dance_config = {
        kGPIO_DigitalOutput, 0,
    };
	
	gpio_pin_config_t voice_config = {
        kGPIO_DigitalOutput, 0,
    };


/*!
 * @brief Main function
 */

/****************************************延时函数*******************************************/
void delay_ms(unsigned int ms)
{
    volatile uint32_t i = 0,j=0;
	for(j=0;j<ms;++j)
	{
		for (i = 0; i < 3000; ++i)
		{
			__asm("NOP"); /* delay */
		}
	}
}

void delay_us(unsigned int us)
{
    volatile uint32_t i = 0,j=0;
	for (j = 0; j < us; ++j)
	{
		for (i = 0; i < 3; ++i)
		{
			__asm("NOP"); /* delay */
		}
	}
}
//static void delay_ms(unsigned long xms)
//{
//	volatile uint32_t i = 0;
//	while(xms--)
//  {
//    for (i = 0; i < 100; ++i)
//    {
//      __asm("nop"); /* delay */
//    }
//  }
//}
//static void delay_us(unsigned long xus)
//{
//	volatile uint32_t i = 0;
//	while(xus--)
//  {
//    for (i = 0; i < 3; ++i)
//    {
//      __asm("nop"); /* delay */
//    }
//  }
//}

/****************************************LCD显示*******************************************/
void transfer_command_IC1(int data1)   
{
	int i;
	lcd19264_cs(1);
	for(i=0;i<5;i++)
	{
		lcd19264_sid(1);
		lcd19264_sclk1(0);
		lcd19264_sclk1(1);delay_us(1);
	}
	lcd19264_sid(0);
	lcd19264_sclk1(0);
	lcd19264_sclk1(1);delay_us(1);
	lcd19264_sid(0);
	lcd19264_sclk1(0);
	lcd19264_sclk1(1);delay_us(1);
	lcd19264_sid(0);
	lcd19264_sclk1(0);
	lcd19264_sclk1(1);delay_us(1);
	for(i=0;i<4;i++)
	{
		if(data1&0x80){lcd19264_sid(1);}
		else lcd19264_sid(0);
		lcd19264_sclk1(0);
		lcd19264_sclk1(1);delay_us(1);
	 	data1=data1<<1;
	}
	for(i=0;i<4;i++)
	{
		lcd19264_sid(0);
		lcd19264_sclk1(0);
		lcd19264_sclk1(1);delay_us(1);
	}
	for(i=0;i<4;i++)
	{
		if(data1&0x80){lcd19264_sid(1);}
		else lcd19264_sid(0);
		lcd19264_sclk1(0);
		lcd19264_sclk1(1);delay_us(1);
	 	data1=data1<<1;
	}
	for(i=0;i<4;i++)
	{
		lcd19264_sid(0);
		lcd19264_sclk1(0);
		lcd19264_sclk1(1);delay_us(1);
	}
	lcd19264_cs(0);
	delay_us(110);
}

void transfer_command_IC2(int data1)
{
	int i;
	lcd19264_cs(1);
	for(i=0;i<5;i++)
	{
		lcd19264_sid(1);
		lcd19264_sclk2(0);
		lcd19264_sclk2(1);delay_us(1);
	}
	lcd19264_sid(0);
	lcd19264_sclk2(0);
	lcd19264_sclk2(1);delay_us(1);
	lcd19264_sid(0);
	lcd19264_sclk2(0);
	lcd19264_sclk2(1);delay_us(1);
	lcd19264_sid(0);
	lcd19264_sclk2(0);
	lcd19264_sclk2(1);delay_us(1);
	for(i=0;i<4;i++)
	{
		if(data1&0x80){lcd19264_sid(1);}
		else lcd19264_sid(0);
		lcd19264_sclk2(0);
		lcd19264_sclk2(1);delay_us(1);
	 	data1=data1<<1;
	}
	for(i=0;i<4;i++)
	{
		lcd19264_sid(0);
		lcd19264_sclk2(0);
		lcd19264_sclk2(1);delay_us(1);
	}
	for(i=0;i<4;i++)
	{
		if(data1&0x80){lcd19264_sid(1);}
		else lcd19264_sid(0);
		lcd19264_sclk2(0);
		lcd19264_sclk2(1);delay_us(1);
	 	data1=data1<<1;
	}
	for(i=0;i<4;i++)
	{
		lcd19264_sid(0);
		lcd19264_sclk2(0);
		lcd19264_sclk2(1);delay_us(1);
	}
	lcd19264_cs(0);
	delay_us(110);
}

void transfer_data_IC1(int data1)
{
	int i;
	lcd19264_cs(1);
	for(i=0;i<5;i++)
	{
		lcd19264_sid(1);
		lcd19264_sclk1(0);
		lcd19264_sclk1(1);delay_us(1);
	}
	lcd19264_sid(0);
	lcd19264_sclk1(0);
	lcd19264_sclk1(1);delay_us(1);
	lcd19264_sid(1);
	lcd19264_sclk1(0);
	lcd19264_sclk1(1);delay_us(1);
	lcd19264_sid(0);
	lcd19264_sclk1(0);
	lcd19264_sclk1(1);delay_us(1);
	for(i=0;i<4;i++)
	{
		if(data1&0x80){lcd19264_sid(1);}
		else lcd19264_sid(0);
		lcd19264_sclk1(0);
		lcd19264_sclk1(1);delay_us(1);
	 	data1=data1<<1;
	}
	for(i=0;i<4;i++)
	{
		lcd19264_sid(0);
		lcd19264_sclk1(0);
		lcd19264_sclk1(1);delay_us(1);
	}
	for(i=0;i<4;i++)
	{
		if(data1&0x80){lcd19264_sid(1);}
		else lcd19264_sid(0);
		lcd19264_sclk1(0);
		lcd19264_sclk1(1);delay_us(1);
	 	data1=data1<<1;
	}
	for(i=0;i<4;i++)
	{
		lcd19264_sid(0);
		lcd19264_sclk1(0);
		lcd19264_sclk1(1);delay_us(1);
	}
	lcd19264_cs(0);
	delay_us(110);
}

void transfer_data_IC2(int data1)
{
	int i;
	lcd19264_cs(1);
	for(i=0;i<5;i++)
	{
		lcd19264_sid(1);
		lcd19264_sclk2(0);
		lcd19264_sclk2(1);delay_us(1);
	}
	lcd19264_sid(0);
	lcd19264_sclk2(0);
	lcd19264_sclk2(1);delay_us(1);
	lcd19264_sid(1);
	lcd19264_sclk2(0);
	lcd19264_sclk2(1);delay_us(1);
	lcd19264_sid(0);
	lcd19264_sclk2(0);
	lcd19264_sclk2(1);delay_us(1);
	for(i=0;i<4;i++)
	{
		if(data1&0x80){lcd19264_sid(1);}
		else lcd19264_sid(0);
		lcd19264_sclk2(0);
		lcd19264_sclk2(1);delay_us(1);
	 	data1=data1<<1;
	}
	for(i=0;i<4;i++)
	{
		lcd19264_sid(0);
		lcd19264_sclk2(0);
		lcd19264_sclk2(1);delay_us(1);
	}
	for(i=0;i<4;i++)
	{
		if(data1&0x80){lcd19264_sid(1);}
		else lcd19264_sid(0);
		lcd19264_sclk2(0);
		lcd19264_sclk2(1);delay_us(1);
	 	data1=data1<<1;
	}
	for(i=0;i<4;i++)
	{
		lcd19264_sid(0);
		lcd19264_sclk2(0);
		lcd19264_sclk2(1);delay_us(1);
	}
	lcd19264_cs(0);
	delay_us(110);
}

void clear_screen(void)
{
	int i,j;
	transfer_command_IC1(0x36);
	transfer_command_IC2(0x36);
	for(j=0;j<32;j++)
	{
		transfer_command_IC1(0x80+j);
		transfer_command_IC1(0x80);
		transfer_command_IC2(0x80+j);
		transfer_command_IC2(0x80);

		for(i=0;i<32;i++)
		{
			transfer_data_IC1(0x00);
			transfer_data_IC2(0x00);
		}
	}
}

void lcd_allinit(void)
{
	transfer_command_IC1(0x34);
	delay_ms(1);
	transfer_command_IC2(0x34);
	delay_ms(1);
	transfer_command_IC1(0x30);
	delay_ms(1);
	transfer_command_IC2(0x30);
	delay_ms(1);
	transfer_command_IC1(0x01);
	delay_ms(1);
	transfer_command_IC2(0x01);
	delay_ms(1);
	transfer_command_IC1(0x06);
	delay_ms(1);
	transfer_command_IC2(0x06);
	delay_ms(1);
	transfer_command_IC1(0x0c);
	delay_ms(1);
	transfer_command_IC2(0x0c);
	delay_ms(1);
}

void LCD_CLS1(void)
{
	transfer_command_IC1(0x30);  // 8-bit interface, Extended instruction
	transfer_command_IC1(0x01);
	delay_ms(5);
}

void LCD_CLS2(void)
{
	transfer_command_IC2(0x30);    // 8-bit interface, Extended instruction
	transfer_command_IC2(0x01);
	delay_ms(5);
}

void DELAY_CLS(void)
{
	delay_ms(1);
	LCD_CLS1();
	LCD_CLS2();
}

void SendStr1(char *ptString)
{
	while((*ptString)!='\0')
	{
		transfer_data_IC1( *ptString++);
//		delay_ms(10);
	}
}

void SendStr2(char *ptString)
{
	while((*ptString)!='\0')
	{
		transfer_data_IC2( *ptString++);
//		delay_ms(10);
	}
}

void gotoxy(unsigned int x, unsigned int y)
{
	switch(x)
	{
		case 1: transfer_command_IC1(0x80+y);break;
		case 2: transfer_command_IC1(0x90+y);break;
		case 3: transfer_command_IC2(0x80+y);break;
		case 4: transfer_command_IC2(0x90+y);break;
	}
}

void show_char(unsigned char x,unsigned char y,char *p)
{
	gotoxy(x,y);
	if(x>0&&x<3)
	{
		SendStr1(p);
	}
	else
	{
		SendStr2(p);
	}
	delay_ms(10);
}

void Lcd19264_Running_Init(void)
{
	lcd19264_reset(1);
	delay_ms(1);
 	lcd19264_psb(0);
	delay_ms(1);
	lcd_allinit();
	delay_ms(1);
}

void All_Screen_Clear(void)
{
	transfer_command_IC2(0x0C);delay_ms(1);
	transfer_command_IC1(0x01);delay_ms(1);
	transfer_command_IC2(0x01);delay_ms(1);
	transfer_command_IC1(0x01);delay_ms(1);
	transfer_command_IC2(0x01);delay_ms(1);
}

void Display_num(unsigned char x,unsigned char y,int num,char length)	//int数字显示函数
{
	char display_buf[7];
	memset(display_buf,0,sizeof(display_buf));
	snprintf(display_buf,7,"%d",num);
	show_char(x,y,display_buf);
}

void Display_float_num(unsigned char x,unsigned char y,float num)	//float数字显示函数
{
	char display_buf[13];
	memset(display_buf,0,sizeof(display_buf));
	sprintf(display_buf,"%7.3f",num);	
	show_char(x,y,display_buf);
}






/****************************************按键中断*******************************************/
volatile bool g_ButtonPress = false;

void BOARD_SW_IRQ_HANDLER(void)
{
    /* Clear external interrupt flag. */
    GPIO_ClearPinsInterruptFlags(BOARD_SW_GPIO, 1U << BOARD_SW_GPIO_PIN);
    /* Change state of button. */
    g_ButtonPress = true;
}

/****************************************串口中断*******************************************/
void DEMO_LPUART_IRQHandler(void)
{
    uint8_t data;
    if ((kLPUART_RxDataRegFullFlag)&LPUART_GetStatusFlags(DEMO_LPUART))
    {
        data = LPUART_ReadByte(DEMO_LPUART);

		if((USART_RX_STA&0x8000)==0)
		{
			if(USART_RX_STA&0x4000)
			{
				if(data!=0xff)USART_RX_STA=0;
				else USART_RX_STA|=0x8000;
			}
			else
			{
				if(data==0xff)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=data ;
					USART_RX_STA++;
					if(USART_RX_STA>(200-1))USART_RX_STA=0;
				}
			}
		}
    }
}



int main(void)
{
/*****************************************初始化********************************************/
	lpuart_config_t config;
	uint32_t uartClkSrcFreq;
	
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
	
	delay_ms(1000);

	PORT_SetPinInterruptConfig(BOARD_SW_PORT, BOARD_SW_GPIO_PIN, kPORT_InterruptFallingEdge);
    EnableIRQ(BOARD_SW_IRQ);
    GPIO_PinInit(BOARD_SW_GPIO, BOARD_SW_GPIO_PIN, &sw_config);
	
    /* Init output LED GPIO. */
    GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN, &led_config);
	GPIO_PinInit(BOARD_LCD_GPIO, PSB_PIN, &psb_config);
	GPIO_PinInit(BOARD_LCD_GPIO, E2_PIN, &E2_config);
	GPIO_PinInit(BOARD_LCD_GPIO, E1_PIN, &E1_config);
	GPIO_PinInit(BOARD_LCD_GPIO, RW_PIN, &rw_config);
	GPIO_PinInit(BOARD_LCD_GPIO, RS_PIN, &rs_config);
	GPIO_PinInit(BOARD_LCD_GPIO, RSTB_PIN, &rest_config);
	
	GPIO_PinInit(BOARD_WALLE_GPIO, STOP_PIN, &stop_config);
	GPIO_PinInit(BOARD_WALLE_GPIO, MOVE_PIN, &move_config);
	GPIO_PinInit(BOARD_WALLE_GPIO, LEFT_PIN, &opp_config);
	GPIO_PinInit(BOARD_WALLE_GPIO, RIGHT_PIN, &circle_config);
	GPIO_PinInit(BOARD_WALLE_GPIO, OPP_PIN, &dance_config);
	GPIO_PinInit(BOARD_WALLE_GPIO, DANCE_PIN, &voice_config);
	
	Lcd19264_Running_Init();
	
	while(DHT11_Init())															//DHT11初始化
	{

	}
/****************************************开机界面*******************************************/
	while(1)
	{
		if(g_ButtonPress)
		{
			GPIO_WritePinOutput(BOARD_LED_GPIO,BOARD_LED_GPIO_PIN,1);
			All_Screen_Clear();
			g_ButtonPress = false;
			
			uartClkSrcFreq = BOARD_DEBUG_UART_CLK_FREQ;
			DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR, BOARD_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE, uartClkSrcFreq);
			
			CLOCK_SetLpuartClock(0x1U);

			LPUART_GetDefaultConfig(&config);
			config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
			config.enableTx = true;
			config.enableRx = true;

			LPUART_Init(DEMO_LPUART, &config, DEMO_LPUART_CLK_FREQ);

			/* Send g_tipString out. */
		//    LPUART_WriteBlocking(DEMO_LPUART, g_tipString, sizeof(g_tipString) / sizeof(g_tipString[0]));

			/* Enable RX interrupt. */
			LPUART_EnableInterrupts(DEMO_LPUART, kLPUART_RxDataRegFullInterruptEnable);
			EnableIRQ(DEMO_LPUART_IRQn);

			break;
		}
		else
		{
			GPIO_WritePinOutput(BOARD_LED_GPIO,BOARD_LED_GPIO_PIN,0);
			show_char(1,1,"家居监测机器人--爸宝");
			show_char(2,1,"EEWORLD--NXP--KW41Z");
			show_char(3,5,"皈依");
			show_char(4,2,"395485316@qq.com");
		}
	}
	
/*****************************************主函数********************************************/
    while(1)
    {
		wifi_msg_program();
		DHT11_Read_Data(&DHT11_temp,&DHT11_humi);
		if((DHT11_temp>=0x28)||(DHT11_humi>=0x5a))	//温度大于40度或者湿度大于90
		{
			alarm_times++;
			if(alarm_times>=10)		//10S左右报警一次
			{
				alarm_times = 0;
				LPUART_WriteBlocking(DEMO_LPUART, &DHT11_temp, sizeof(DHT11_temp));
				LPUART_WriteBlocking(DEMO_LPUART, &DHT11_humi, sizeof(DHT11_humi));
			}
		}
		else
		{
			alarm_times = 0;
		}
		lcd_show_program();
		GPIO_WritePinOutput(BOARD_LED_GPIO,BOARD_LED_GPIO_PIN,1);
		delay_ms(500);
		GPIO_WritePinOutput(BOARD_LED_GPIO,BOARD_LED_GPIO_PIN,0);
		delay_ms(500);
	}
}


/****************************************爸宝动作*******************************************/
void Walle_stop(void)
{
	GPIO_WritePinOutput(BOARD_WALLE_GPIO,STOP_PIN,1);
	delay_ms(300);
	GPIO_WritePinOutput(BOARD_WALLE_GPIO,STOP_PIN,0);
}

void Walle_move(void)
{
	GPIO_WritePinOutput(BOARD_WALLE_GPIO,MOVE_PIN,1);
	delay_ms(300);
	GPIO_WritePinOutput(BOARD_WALLE_GPIO,MOVE_PIN,0);
}

void Walle_left(void)
{
	GPIO_WritePinOutput(BOARD_WALLE_GPIO,LEFT_PIN,1);
	delay_ms(300);
	GPIO_WritePinOutput(BOARD_WALLE_GPIO,LEFT_PIN,0);
}

void Walle_right(void)
{
	GPIO_WritePinOutput(BOARD_WALLE_GPIO,RIGHT_PIN,1);
	delay_ms(300);
	GPIO_WritePinOutput(BOARD_WALLE_GPIO,RIGHT_PIN,0);
}

void Walle_opp(void)
{
	GPIO_WritePinOutput(BOARD_WALLE_GPIO,OPP_PIN,1);
	delay_ms(300);
	GPIO_WritePinOutput(BOARD_WALLE_GPIO,OPP_PIN,0);
}

void Walle_dance(void)
{
	GPIO_WritePinOutput(BOARD_WALLE_GPIO,DANCE_PIN,1);
	delay_ms(300);
	GPIO_WritePinOutput(BOARD_WALLE_GPIO,DANCE_PIN,0);
}


/****************************************wifi控制*******************************************/
void wifi_msg_program(void)
{
	
	if(USART_RX_STA&0x8000)
	{
		wifi_msg = USART_RX_BUF[0];
		switch(wifi_msg)
		{
			case action0:
			{
				Walle_stop();
			} break;
			case action1:
			{
				Walle_move();
			} break;
			case action2:
			{
				Walle_left();
			} break;
			case action3:
			{
				Walle_right();
			} break;
			case action4:
			{
				Walle_opp();
			} break;
			case action5:
			{
				Walle_dance();
			} break;
			case action6:
			{
				LPUART_WriteBlocking(DEMO_LPUART, &DHT11_temp, sizeof(DHT11_temp));
				LPUART_WriteBlocking(DEMO_LPUART, &DHT11_humi, sizeof(DHT11_humi));
			} break;
			default: break;
		}
		All_Screen_Clear();
//		GPIO_TogglePinsOutput(BOARD_LED_GPIO,BOARD_LED_GPIO_PIN);
		if(wifi_msg!=action6) walle_state = wifi_msg;
		USART_RX_STA=0;
	}
}

/****************************************LCD显示*******************************************/
void lcd_show_program(void)
{
	show_char(1,0,"温度：");
	show_char(2,0,"湿度：");
	show_char(1,9,"摄氏度");
	show_char(2,9,"% 湿度");
	show_char(3,0,"爸宝状态：");
	show_char(4,0,"wifi信息：");
	
	Display_num(1,5,DHT11_temp,4);
	Display_num(2,5,DHT11_humi,4);
	
	switch(walle_state)
	{
		case 0x00:{show_char(3,5,"空闲");} break;
		case 0x01:{show_char(3,5,"前进");} break;
		case 0x02:{show_char(3,5,"左转");} break;
		case 0x03:{show_char(3,5,"右转");} break;
		case 0x04:{show_char(3,5,"后退");} break;
		case 0x05:{show_char(3,5,"跳舞");} break;
//		case 0x06:{show_char(3,5,"");} break;
		default: break;
	}
	
	switch(wifi_msg)
	{
		case 0x00:{show_char(4,5,"爸宝休息");} break;
		case 0x01:{show_char(4,5,"爸宝前进");} break;
		case 0x02:{show_char(4,5,"爸宝左转身");} break;
		case 0x03:{show_char(4,5,"爸宝右转身");} break;
		case 0x04:{show_char(4,5,"爸宝后退");} break;
		case 0x05:{show_char(4,5,"爸宝跳舞");} break;
		case 0x06:{show_char(4,5,"查询温湿度");} break;
		default:{show_char(4,5,"无wifi指令");} break;
	}
}



/****************************************DHT11*******************************************/
void DHT11_IO_IN(void)
{
	/* Input pin configuration */
	gpio_pin_config_t inconfig =
	{
		kGPIO_DigitalInput,
		0,
	};
	/* Input pin PORT configuration */
	port_pin_config_t config =
	{
		kPORT_PullUp,
		kPORT_FastSlewRate,
		kPORT_PassiveFilterDisable,
		kPORT_LowDriveStrength,
		kPORT_MuxAsGpio,
	};

	PORT_SetPinConfig(DHT11_Port, DHT11_Pin, &config);//config as pull-up
	GPIO_PinInit(DHT11_GPIO, DHT11_Pin, &inconfig);//config as input
}
//Set GPIO Direction
void DHT11_IO_OUT(void)
{
	/* Output pin configuration */
	gpio_pin_config_t outconfig =
	{
		kGPIO_DigitalOutput,
		0,
	};
	GPIO_PinInit(DHT11_GPIO, DHT11_Pin, &outconfig);//config as output
}

void DHT11_Rst(void)
{
	DHT11_IO_OUT();
	DHT11_DQ_OUT_0;
	delay_ms(30);
	DHT11_DQ_OUT_1;
	delay_us(30);
}

//等待DHT11的回应
//返回1:未检测到DHT11的存在
//返回0:存在
unsigned char DHT11_Check(void)
{
	unsigned char retry=0;
	DHT11_IO_IN();													//SET INPUT
	while (DHT11_DQ_IN&&retry<100)					//DHT11会拉低40~80us
	{
		retry++;
		delay_us(1);
	};
	if(retry>=100)return 1;
	else retry=0;
	while (!DHT11_DQ_IN&&retry<100)					//DHT11拉低后会再次拉高40~80us
	{
		retry++;
		delay_us(1);
	};
	if(retry>=100)return 1;
	return 0;
}

//从DHT11读取一个位
//返回值：1/0
unsigned char DHT11_Read_Bit(void)
{
 	unsigned char retry=0;
	while(DHT11_DQ_IN&&retry<100)						//等待变为低电平
	{
		retry++;
		delay_us(1);
	}
	retry=0;
	while(!DHT11_DQ_IN&&retry<100)					//等待变高电平
	{
		retry++;
		delay_us(1);
	}
	delay_us(40);														//等待40us
	if(DHT11_DQ_IN)return 1;
	else return 0;
}

//从DHT11读取一个字节
//返回值：读到的数据
unsigned char DHT11_Read_Byte(void)
{
	unsigned char i,dat;
	dat=0;
	for (i=0;i<8;i++)
	{
		dat<<=1;
		dat|=DHT11_Read_Bit();
	}
	return dat;
}

//从DHT11读取一次数据
//temp:温度值(范围:0~50°)
//humi:湿度值(范围:20%~90%)
//返回值：0,正常;1,读取失败
unsigned char DHT11_Read_Data(unsigned char *temp,unsigned char *humi)
{
 	unsigned char buf[5]={0};
	unsigned char i;
	DHT11_Rst();
	if(DHT11_Check()==0)
	{
		for(i=0;i<5;i++)												//读取40位数据
		{
			buf[i]=DHT11_Read_Byte();
		}
		if((buf[0]+buf[1]+buf[2]+buf[3])==buf[4])
		{
			*humi=buf[0];
			*temp=buf[2];
		}
	}else return 1;
	return 0;
}

//初始化DHT11的IO口 DQ 同时检测DHT11的存在
//返回1:不存在
//返回0:存在
unsigned char DHT11_Init(void)
{
	DHT11_IO_OUT();
	DHT11_DQ_OUT_1;
	DHT11_Rst();				//复位DHT11
	return DHT11_Check();		//等待DHT11的回应
}








/********************************************************************************** 皈依 395485316@qq.com *************************************************************************************/








