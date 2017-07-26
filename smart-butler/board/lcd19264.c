#include "lcd19264.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"

#include "clock_config.h"
#include "pin_mux.h"




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

//-----------transfer data to LCM---------------
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



//void display_32X32(int y,int x,char *dp)
//{
//	int i,j;
//	if(y>0&&y<3)
//	{
//		transfer_command_IC1(0x36);
//		delay_us(10);
//		transfer_command_IC1(0x80);
//		delay_us(10);
//		for(j=0;j<32;j++)
//		{
//			transfer_command_IC1(0x80+y-1+j);
//			transfer_command_IC1(0x80+x-1);
//			for(i=0;i<4;i++)
//			{
//				transfer_data_IC1(*dp);
//				dp++;
//			}
//		}
//	}
//	else
//	{
//		y-=2;
//		transfer_command_IC2(0x36);
//		delay_us(10);
//		transfer_command_IC2(0x80);
//		delay_us(10);
//		for(j=0;j<32;j++)
//		{
//			transfer_command_IC2(0x80+y-1+j);
//			transfer_command_IC2(0x80+x-1);
//			for(i=0;i<4;i++)
//			{
//				transfer_data_IC2(*dp);
//				dp++;
//			}
//		}
//	}
//}


//void display_16X32(int y,int x,char *dp)
//{
//	int i,j;
//	if(y>0&&y<3)
//	{
//		transfer_command_IC1(0x36);
//		delay_us(10);
//		transfer_command_IC1(0x80);
//		delay_us(10);
//		for(j=0;j<32;j++)
//		{
//			transfer_command_IC1(0x80+y-1+j);
//			transfer_command_IC1(0x80+x-1);
//			for(i=0;i<2;i++)
//			{
//				transfer_data_IC1(*dp);
//				dp++;
//			}
//		}
//	}
//	else
//	{
//		y-=2;
//		transfer_command_IC2(0x36);
//		delay_us(10);
//		transfer_command_IC2(0x80);
//		delay_us(10);
//		for(j=0;j<32;j++)
//		{
//			transfer_command_IC2(0x80+y-1+j);
//			transfer_command_IC2(0x80+x-1);
//			for(i=0;i<2;i++)
//			{
//				transfer_data_IC2(*dp);
//				dp++;
//			}
//		}
//	}
//}


//void display_192X64(char *dp)
//{
//	int i,j;
//	transfer_command_IC1(0x36);
//	delay_ms(1);
//	transfer_command_IC1(0x80);
//	delay_ms(1);
//	transfer_command_IC2(0x36);
//	delay_ms(1);
//	transfer_command_IC2(0x80);
//	delay_ms(1);
//	for(j=0;j<32;j++)
//	{
//		transfer_command_IC1(0x80+0+j);
//		transfer_command_IC1(0x80+0);
//		for(i=0;i<24;i++)
//		{
//			transfer_data_IC1(*dp);
//			dp++;
//		}
//	}
//	for(j=0;j<32;j++)
//	{
//		transfer_command_IC2(0x80+j);
//		transfer_command_IC2(0x80);
//		for(i=0;i<24;i++)
//		{
//			transfer_data_IC2(*dp);
//			dp++;
//		}
//	}
//}

void lcd_gpioinit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_11);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
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

//void display_char(int y,int x,int char_total,char *dp)
//{
//	int i;
//	if(y>0&&y<3)
//	{
//		transfer_command_IC1(0x30);
//		delay_ms(1);
//		delay_ms(1);
//		transfer_command_IC1(0x80+(y-1)*(0x10)+(x-1));
//		for(i=0;i<char_total;i++)
//		{
//			transfer_data_IC1(*dp);
//			dp++;
//			transfer_data_IC1(*dp);
//			dp++;
//		}
//	}
//	else
//	{
//		y-=2;
//		transfer_command_IC2(0x30);
//		delay_ms(1);
//		delay_ms(1);
//		transfer_command_IC2(0x80+(y-1)*(0x10)+(x-1));
//		for(i=0;i<char_total;i++)
//		{
//			transfer_data_IC2(*dp);
//			dp++;
//			transfer_data_IC2(*dp);
//			dp++;
//		}
//	}
//}







void DELAY_CLS()
{
	delay_ms(1);
	LCD_CLS1();
	LCD_CLS2();
}


void LCD_CLS1()
{
	transfer_command_IC1(0x30);  // 8-bit interface, Extended instruction
	transfer_command_IC1(0x01);
	delay_ms(5);
}


void LCD_CLS2()
{
	transfer_command_IC2(0x30);    // 8-bit interface, Extended instruction
	transfer_command_IC2(0x01);
	delay_ms(5);
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

//void LCD_Test(void)
//{
//	unsigned char i;
//	unsigned char TestData[][2] =
//	{
//		{0xff,0xff},	// Stars 2
//		{0x00,0x00},	// All
//		{0xff,0x00},	// None	
//		{0x00,0xFF},	// Virtical 2
//		{0x55,0x55},	// Horizontal 1	
//		{0xaa,0xaa},	// Horizontal 1	
//		{0xaa,0x55},	// Horizontal 1	
//	};
//	for (i = 0; i < 7; i++)
//	{
//		display(TestData[i][0], TestData[i][1]);     	
//		delay_ms(300);
//	}
//}


//void display(unsigned char fst, unsigned char snd)
//{
//	unsigned char x,y,k;
//	transfer_command_IC1(0x36);
//	transfer_command_IC2(0x36);
//	for(y=0;y<32;y++)
//	{
//		if((y%2)==1)
//			k=fst;
//		else
//			k=snd;
//		for(x=0;x<12;x++)
//		{
//			transfer_command_IC1(y+0x80);
//			transfer_command_IC1(x+0x80);
//			transfer_data_IC1(k);
//			transfer_data_IC1(k);
//		}
//	}

//	for(y=0;y<32;y++)
//	{
//		if((y%2)==1)
//			k=fst;
//		else
//			k=snd;
//		for(x=0;x<12;x++)
//		{
//			transfer_command_IC2(y+0x80);
//			transfer_command_IC2(x+0x80);
//			transfer_data_IC2(k);
//			transfer_data_IC2(k);
//		}
//	}
//	transfer_command_IC1(0x30);
//	transfer_command_IC2(0x30);
//}



//void cgram()
//{
//	unsigned char x,y;

//	for(y=0;y<32;y++)
//	{
//		for(x=0;x<12;x++)
//		{
//			transfer_command_IC1(0x34);
//			transfer_command_IC1(y+0x80);
//			transfer_command_IC1(x+0x80);
//			transfer_data_IC1(0x00);
//			transfer_data_IC1(0x00);
//			transfer_command_IC1(0x30);
//		}
//	}
//	for(y=0;y<32;y++)
//	{
//		for(x=0;x<12;x++)
//		{
//			transfer_command_IC2(0x34);
//			transfer_command_IC2(y+0x80);
//			transfer_command_IC2(x+0x80);
//			transfer_data_IC2(0x00);
//			transfer_data_IC2(0x00);
//			transfer_command_IC2(0x30);
//		}
//	}
//}



//void display_pic(unsigned char *pic) 
//{
//	unsigned char x,y;

//	for(y=0;y<32;y++)
//	{
//		for(x=0;x<12;x++)
//		{
//			transfer_command_IC1(0x36);
//			transfer_command_IC1(y+0x80);
//			transfer_command_IC1(x+0x80);
//			transfer_command_IC1(0x30);
//			transfer_data_IC1(*pic++);
//			transfer_data_IC1(*pic++);
//		}
//	}
//	for(y=0;y<32;y++)
//	{
//		for(x=0;x<12;x++)
//		{
//			transfer_command_IC2(0x36);
//			transfer_command_IC2(y+0x80);
//			transfer_command_IC2(x+0x80);
//			transfer_command_IC2(0x30);
//			transfer_data_IC2(*pic++);
//			transfer_data_IC2(*pic++);
//		}
//	}
//}


//void disp_kuang(void)
//{
//	unsigned char i,j;

//	transfer_command_IC1(0x36);
//	transfer_command_IC2(0x36);
//	for(i=0;i<12;i++)
//	{
//		transfer_command_IC1(0x80);
//		transfer_command_IC1(0x80+i);
//		transfer_data_IC1(0xff);
//		transfer_data_IC1(0xff);
//	}
//	for(j=1;j<32;j++)
//	{
//		for(i=0;i<12;i++)
//		{
//			transfer_command_IC1(0x80+j);
//			transfer_command_IC1(0x80+i);
//			if(i==0) {transfer_data_IC1(0x80);transfer_data_IC1(0x00); }
//			if(i==11) {transfer_data_IC1(0x00);transfer_data_IC1(0x01); }
//			else  {transfer_data_IC1(0x00);transfer_data_IC1(0x00); }
//		}
//	}

//	for(j=0;j<31;j++)
//	{
//		for(i=0;i<12;i++)
//		{
//			transfer_command_IC2(0x80+j);
//			transfer_command_IC2(0x80+i);
//			if(i==0) {transfer_data_IC2(0x80);transfer_data_IC2(0x00); }
//			if(i==11) {transfer_data_IC2(0x00);transfer_data_IC2(0x01); }
//			else  {transfer_data_IC2(0x00);transfer_data_IC2(0x00); }
//		}
//	}

//	for(i=0;i<12;i++)
//	{
//		transfer_command_IC2(0x9f);
//		transfer_command_IC2(0x90+i);
//		transfer_data_IC2(0xff);
//		transfer_data_IC2(0xff);
//	}
//	transfer_command_IC1(0x30);
//	transfer_command_IC2(0x30);
//}

//void CallBuiltinChar(void)
//{
//	unsigned int i,j;
//	// 0xA140 ~ 0xA15F
//	gotoxy(2,0);
//	for (i = 0; i < 12; i++)
//	{
//		transfer_data_IC1(0xA1);
//		transfer_data_IC1(0xc0 + i);
//	}
//	gotoxy(3,0);
//	for (j = 0; j < 36; j++)
//	{
//		transfer_data_IC2(0xA2);
//		transfer_data_IC2(0x45 + j);
//	}
//}

//char b[7];

//char * int_to_char(int a)
//{
//	memset(b,0,sizeof(b));
//	sprintf(b,"%d",a);
//	return b;
//}




void Lcd19264_Running_Init(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_11);
	delay_ms(1);
 	lcd19264_psb(0);
	delay_ms(1);
	lcd_allinit();
	delay_ms(1);
}


/*******************全屏清除函数*********************/
void All_Screen_Clear(void)
{
	transfer_command_IC2(0x0C);delay_ms(1);
	transfer_command_IC1(0x01);delay_ms(1);
	transfer_command_IC2(0x01);delay_ms(1);
	transfer_command_IC1(0x01);delay_ms(1);
	transfer_command_IC2(0x01);delay_ms(1);
}





/*******************显示日期函数*********************/
void Display_num(unsigned char x,unsigned char y,int num,char length)	//int数字显示函数
{
	char display_buf[7];
	memset(display_buf,0,sizeof(display_buf));
	snprintf(display_buf,7,"%d",num);
	switch(length)
	{
		case 1:{show_char(x,y,"  ");} break;
		case 2:{show_char(x,y,"    ");} break;
		case 3:{show_char(x,y,"      ");} break;
		case 4:{show_char(x,y,"        ");} break;
		default: break;
	}
	show_char(x,y,display_buf);
}



void Display_float_num(unsigned char x,unsigned char y,float num)	//float数字显示函数
{
	char display_buf[13];
	memset(display_buf,0,sizeof(display_buf));
	sprintf(display_buf,"%7.3f",num);	
	show_char(x,y,display_buf);
}



void Display_Data_Time(int *dt)
{
	switch(YL_result_machine_setup.language)
	{
		case LANGUAGE_SETUP_CHINESE:{show_char(2,0,"日期："); show_char(3,0,"时间：");} break;
		case LANGUAGE_SETUP_ENGLISH:{show_char(2,0,"Date："); show_char(3,0,"time：");} break;
		case LANGUAGE_SETUP_FRENCH:{show_char(2,0,"Date："); show_char(3,0,"le temps：");} break;
		case LANGUAGE_SETUP_GERMAN:{show_char(2,0,"Datum："); show_char(3,0,"die zeit：");} break;
		case LANGUAGE_SETUP_ITALIAN:{show_char(2,0,"Data："); show_char(3,0,"il tempo：");} break;
		case LANGUAGE_SETUP_JAPANESE:{show_char(2,0,"日付："); show_char(3,0,"時間：");} break;
		default: break;
	}

	Display_num(2,3,*(dt),2);
	gotoxy(2,5);
	transfer_data_IC1(0xa9);
	transfer_data_IC1(0xa6);
	Display_num(2,7,*(dt+1),2);
	gotoxy(2,8);
	transfer_data_IC1(0xa9);
	transfer_data_IC1(0xa6);
	Display_num(2,10,*(dt+2),2);
	Display_num(3,5,*(dt+3),2);
	gotoxy(3,6);
	transfer_data_IC2(0xa9);
	transfer_data_IC2(0xa6);
	Display_num(3,7,*(dt+4),2);
}

/*******************显示名称函数*********************/
extern char Tname_Edit[8];

extern char *Name_Member[90];
extern int Dtime_Setup[5];
extern unsigned char Setup_Line_Num;


void Display_Test_Name(void)
{
	unsigned char i;
	switch(YL_result_machine_setup.language)
	{
		case LANGUAGE_SETUP_CHINESE:{show_char(1,3,"实验名称编辑");} break;
		case LANGUAGE_SETUP_ENGLISH:{show_char(1,3,"Name editor");} break;
		case LANGUAGE_SETUP_FRENCH:{show_char(1,3,"nom editor");} break;
		case LANGUAGE_SETUP_GERMAN:{show_char(1,3,"name - editor");} break;
		case LANGUAGE_SETUP_ITALIAN:{show_char(1,3,"nome editore");} break;
		case LANGUAGE_SETUP_JAPANESE:{show_char(1,3,"名エディタ");} break;
		default: break;
	}

	gotoxy(2,0);
	transfer_data_IC1(0xa9);
	transfer_data_IC1(0xb0);
	for(i=0;i<10;i++)
	{
		transfer_data_IC1(0xa9);
		transfer_data_IC1(0xa4);
	}
	transfer_data_IC1(0xa9);
	transfer_data_IC1(0xb4);
	gotoxy(3,0);
	transfer_data_IC2(0xa9);
	transfer_data_IC2(0xa6);
	gotoxy(3,11);
	transfer_data_IC2(0xa9);
	transfer_data_IC2(0xa6);
	gotoxy(4,0);
	transfer_data_IC2(0xa9);
	transfer_data_IC2(0xb8);
	for(i=0;i<10;i++)
	{
		transfer_data_IC2(0xa9);
		transfer_data_IC2(0xa4);
	}
	transfer_data_IC2(0xa9);
	transfer_data_IC2(0xbc);
	for(i=0;i<8;i++)
	{
		show_char(3,i+2,Name_Member[Tname_Edit[i]]);
	}
}







//void lcd_test(void)
//{

////	unsigned char line_num[4];

//	while(1)
//	{
//		Display_Data_Time(Dtime_Setup);
//		
//		
//		
//		
//		
//		
//		
////		Display_num(1,5,12345);
////		memset(b,0,sizeof(b));
////		snprintf(b,sizeof(b),"%d",i);
////		show_char(1,5,b);
////		gotoxy(1,4);
////				transfer_data_IC1(0xa9);
////				transfer_data_IC1(0xa7);
////				gotoxy(2,4);
////				transfer_data_IC1(0xa9);
////				transfer_data_IC1(0xc7);  
////				gotoxy(3,4);
////				transfer_data_IC2(0xa9);
////				transfer_data_IC2(0xa7);
////				gotoxy(4,4);
////				transfer_data_IC2(0xa9);
////				transfer_data_IC2(0xa7);
////		gotoxy(1,0x0b);
////				transfer_data_IC1(0xa9);
////				transfer_data_IC1(0xa7);
////				gotoxy(2,0x0b);
////				transfer_data_IC1(0xa9);
////				transfer_data_IC1(0xcc);  
////				gotoxy(3,0x0b);
////				transfer_data_IC2(0xa9);
////				transfer_data_IC2(0xa7);
////				gotoxy(4,0x0b);
////				transfer_data_IC2(0xa9);
////				transfer_data_IC2(0xa7);
////				gotoxy(2,5);
////				transfer_data_IC1(0xa1);
////				transfer_data_IC1(0xf4);
////				gotoxy(2,0x0a);
////				transfer_data_IC1(0xa1);
////				transfer_data_IC1(0xf4);
////				
////				show_char(1,0,"设置菜单");
////				line_num[1] = Setup_Line_Num;
////				switch(line_num[1])
////				{
////					case 0:{line_num[0]=9;line_num[2]=1;line_num[3]=2;} break;
////					case 8:{line_num[0]=7;line_num[2]=9;line_num[3]=0;} break;
////					case 9:{line_num[0]=8;line_num[2]=0;line_num[3]=1;} break;
////					default:{line_num[0]=line_num[1]-1;line_num[2]=line_num[1]+1;line_num[3]=line_num[1]+2;} break;
////				}
////				show_char(1,6,Display_Setup_List0_one_words(line_num[0]));
////				show_char(2,6,Display_Setup_List0_one_words(line_num[1]));
////				show_char(3,6,Display_Setup_List0_one_words(line_num[2]));
////				show_char(4,6,Display_Setup_List0_one_words(line_num[3]));
//				


////		strcpy(words,"欢迎使用");
//////		clear_screen();
//////		transfer_command_IC1(0x01);
//////		delay_ms(1);
//////		transfer_command_IC2(0x01);
//////		delay_ms(1);
////		show_char(1,1,words);
////		show_char(4,4,"卤素水分测试仪");
////		gotoxy(2,0);
////		transfer_data_IC1(0xa1);
////		transfer_data_IC1(0xfa);
////		transfer_data_IC1(0xa1);
////		transfer_data_IC1(0xfa);
////		transfer_data_IC1(0xa1);
////		transfer_data_IC1(0xfa);
////		gotoxy(2,9);
////		transfer_data_IC1(0xa1);
////		transfer_data_IC1(0xfb);
////		transfer_data_IC1(0xa1);
////		transfer_data_IC1(0xfb);
////		transfer_data_IC1(0xa1);
////		transfer_data_IC1(0xfb);
////		display_char(1,2,4,"欢迎使用");
////		display_char(4,5,7,"卤素水分测试仪");
////		waitkey();
////		transfer_command_IC1(0x01);
////		delay_ms(1);
////		transfer_command_IC2(0x01);
////		delay_ms(1);
////		display_char(1,1,6,"安徽优力电子");
////		display_char(2,5,6,"安徽优力电子");
//////		display_char(2,1,5,"2222222222");
//////		display_char(3,1,5,"3333333333");
////		display_char(4,1,8,"深圳市艾可特电子");
////		waitkey();

////		clear_screen();
////		waitkey();
////		transfer_command_IC1(0x01);
////		delay_ms(1);
////		transfer_command_IC2(0x01);
////		delay_ms(1);
////		display_32X32(1,5,yun1);
////		display_32X32(1,7,xing1);
////		display_16X32(3,3,char_R);
////		display_16X32(3,4,char_U);
////		display_16X32(3,5,char_N);
////		display_16X32(3,6,char_N);
////		display_16X32(3,7,char_I);
////		display_16X32(3,8,char_N);
////		display_16X32(3,9,char_G);
////		waitkey();
////		clear_screen();
////		display_192X64(bmp1);
////		waitkey();
////		//clear_screen();
////		display_192X64(bmp2);
////		waitkey();
////		//clear_screen();
////		display_192X64(bmp3);
////		waitkey();
////		//clear_screen();
////		display_192X64(bmp4);
////		waitkey();
////		clear_screen();



////			disp_kuang();
////		delay_ms(100);
////		DELAY_CLS(); 

////		display(0xaa,0x11);
////		DELAY_CLS();

////		show_char(1,0,"你知道我在等你吗？");
////		show_char(2,2,"你如果真的在乎我！");
////		gotoxy(3,0);
////		transfer_data_IC2(0xc1);
////		transfer_data_IC2(0xe3);

////		transfer_data_IC2(0xd2);
////		transfer_data_IC2(0xbc);

////		transfer_data_IC2(0xb7);
////		transfer_data_IC2(0xa1);

////		transfer_data_IC2(0xc8);
////		transfer_data_IC2(0xfe);

////		transfer_data_IC2(0xcb);
////		transfer_data_IC2(0xc1);

////		transfer_data_IC2(0xce);
////		transfer_data_IC2(0xe9);/*?CEE9*/

////		transfer_data_IC2(0xc2);
////		transfer_data_IC2(0xbd);/*?C2BD*/

////		transfer_data_IC2(0xc6);
////		transfer_data_IC2(0xe2);/*?C6E2*/

////		transfer_data_IC2(0xb0);
////		transfer_data_IC2(0xc6);/*?B0C6*/

////		transfer_data_IC2(0xbe);
////		transfer_data_IC2(0xc1);/*?BEC1*/

////		transfer_data_IC2(0xca);
////		transfer_data_IC2(0xb0);/*?CAB0*/

////		transfer_data_IC2(0xb0);
////		transfer_data_IC2(0xdb);/* ?B0DB*/
//////		delay_ms(500);

////		gotoxy(4,0);
////		SendStr2("你在南方的艳阳里");
////		delay_ms(500);
////		gotoxy(4,10);
////		transfer_data_IC2(0xd5);
////		transfer_data_IC2(0xfd);/*?*/
////		gotoxy(4,11);
////		transfer_data_IC2(0xb3);
////		transfer_data_IC2(0xa3);/*?*/
////		delay_ms(500);
////		DELAY_CLS(); 

////		LCD_Test();
////		LCD_CLS1();
////		LCD_CLS2();

////		gotoxy(1,0);
////		for(i=0;i<72;i++)
////		{  
////		transfer_data_IC1(0x01+i);
////		}
////		gotoxy(3,0);
////		for(i=0;i<72;i++)
////		{  
////		transfer_data_IC2(0x41+i);
////		}
////		delay_ms(500);
////		DELAY_CLS();


////		gotoxy(1,0);
////		SendStr1("大雪纷飞"); 
////		gotoxy(2,0);
////		SendStr1("我在北方的寒夜里"); 
////		gotoxy(3,0);
////		SendStr2("四季如春"); 
////		gotoxy(4,0);
////		SendStr2("如果天黑之前来得及"); 
////		delay_ms(500);
////		DELAY_CLS(); 

////		CallBuiltinChar();
////		gotoxy(1, 0);
////		SendStr1("我要忘了你的眼睛"); 
////		transfer_command_IC1(0x0E); //????
//////		transfer_command_IC1(0x0E); //????
//////		transfer_command_IC1(0x0E); //????
////		gotoxy(1,3);
//////		transfer_command_IC1(0x0E);
////		gotoxy(1,4);
//////		transfer_command_IC1(0x0E);
////		gotoxy(1,5);
//////		for(i=0;i<4;i++)
//////		{  		
//////		transfer_command_IC1(0x14+i);
////		delay_ms(300);
//////		}
////		transfer_command_IC1(0x0C); //????
////		delay_ms(300);
////		delay_ms(600);
////		DELAY_CLS(); 
//	
//	}
//}






