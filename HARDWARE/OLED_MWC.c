
/************************
MS5611.c
20141112
Dammstanger
// ########################################
// #  i2c OLED display funtion primitives #
// ########################################
*****************************/

#include "OLED_MWC.h"
#include "I2C_1.h"
#include <math.h>
#include "usart.h"
#include <stdio.h>
#include "SysTick.h"

//#define SUPPRESS_OLED_I2C_128x64LOGO
char DispData[10] ={0};
const uint8_t myFont[][5];
//const uint8_t LOGO[];
static char buffer; 
unsigned char CHAR_FORMAT = 0;      // use to INVERSE characters


////////////////////////////////////////////////////////////////////
//  void LCDattributesBold() {/*CHAR_FORMAT = 0b01111111; */}
//  void LCDattributesReverse() {CHAR_FORMAT = 0b01111111; }
//  void LCDattributesOff() {CHAR_FORMAT = 0; }
//  void LCDalarmAndReverse() {LCDattributesReverse(); }
////////////////////////////////////////////////////////////////////


/********************************
函数：OLED_MWC_send_cmd
功能：送入命令
输入：命令值
输出：-
修改：20141112
***********************************/
bool OLED_MWC_send_cmd(uint8_t command)
{
	bool retval;
  retval = IIC1_WriteByte(OLED_ADDRESS, 0x80, (uint8_t)command);
	return retval;
}

/********************************
函数：OLED_MWC_send_cmd
功能：送入命令
输入：命令值
输出：-
修改：20141112
***********************************/
void OLED_MWC_send_byte(uint8_t val) {
	IIC1_WriteByte(OLED_ADDRESS, 0x40, (uint8_t)val);
}


/********************************
函数：OLED_MWC_send_cmd
功能：送入命令
输入：命令值
输出：-
修改：20141112
***********************************/
bool  OLED_MWC_init(void){
	bool retval;
	retval=OLED_MWC_send_cmd(0xae);    //display off
	if(retval==FALSE) return retval;
	OLED_MWC_send_cmd(0xa4);          //SET All pixels OFF
	//  OLED_MWC_send_cmd(0xa5);            //SET ALL pixels ON
	Delay_ms(50);
	OLED_MWC_send_cmd(0x20);            //Set Memory Addressing Mode
	OLED_MWC_send_cmd(0x02);            //Set Memory Addressing Mode to Page addressing mode(RESET)
	//  OLED_MWC_send_cmd(0xa0);      //colum address 0 mapped to SEG0 (POR)*** wires at bottom
	OLED_MWC_send_cmd(0xa1);    //colum address 127 mapped to SEG0 (POR) ** wires at top of board
	//  OLED_MWC_send_cmd(0xC0);            // Scan from Right to Left (POR)         *** wires at bottom
	OLED_MWC_send_cmd(0xC8);          // Scan from Left to Right               ** wires at top
	OLED_MWC_send_cmd(0xa6);            // Set WHITE chars on BLACK backround
	//  OLED_MWC_send_cmd(0xa7);            // Set BLACK chars on WHITE backround
	OLED_MWC_send_cmd(0x81);            // Setup CONTRAST CONTROL, following byte is the contrast Value
	OLED_MWC_send_cmd(0xaf);            // contrast value between 1 ( == dull) to 256 ( == bright)
	//  OLED_MWC_send_cmd(0xd3);            // Display Offset :
	//  OLED_MWC_send_cmd(0x0);            // 0
	//  Delay_ms(20);
	//  OLED_MWC_send_cmd(0x40);            // Display start line [0;63] -> [0x40;0x7f]
	//  Delay_ms(20);
	#ifdef DISPLAY_FONT_DSIZE
	OLED_MWC_send_cmd(0xd6);            // zoom
	OLED_MWC_send_cmd(0x01);            // on
	#else
	//    OLED_MWC_send_cmd(0xd6);            // zoom
	//    OLED_MWC_send_cmd(0x00);            // off
	#endif
	Delay_ms(20);
	OLED_MWC_send_cmd(0xaf);          //display on
	OLED_MWC_Clear();
	Delay_ms(20);
	return TRUE;
}

/********************************
函数：OLED_MWC_send_cmd
功能：送入命令
输入：命令值
输出：-
修改：20141112
***********************************/
void OLED_MWC_Clear(void)
{
  u16 i;
//  for(i=0;i<8;i++){
//    OLED_MWC_set_XY(0,i);
//    OLED_MWC_send_string(LINE_FILL_STRING);
//  }
  OLED_MWC_send_cmd(0xa6);              //Set Normal Display
  OLED_MWC_send_cmd(0xae);              // Display OFF
  OLED_MWC_send_cmd(0x20);              // Set Memory Addressing Mode
  OLED_MWC_send_cmd(0x00);              // Set Memory Addressing Mode to Horizontal addressing mode
  OLED_MWC_send_cmd(0xb0);              // set page address to 0
  OLED_MWC_send_cmd(0X40);              // Display start line register to 0
  OLED_MWC_send_cmd(0);                 // Set low col address to 0
  OLED_MWC_send_cmd(0x10);              // Set high col address to 0
  for(i=0; i<1024; i++) {           // fill the display's RAM with graphic... 128*64 pixel picture
     OLED_MWC_send_byte(0);  // clear
  }
  OLED_MWC_send_cmd(0x81);              // Setup CONTRAST CONTROL, following byte is the contrast Value... always a 2 byte instruction
  OLED_MWC_send_cmd(200);               // Here you can set the brightness 1 = dull, 255 is very bright
  OLED_MWC_send_cmd(0xaf);              // display on
}

/********************************
函数：OLED_MWC_set_XY
功能：设置显示坐标
输入：列值：1-21  21个字符 行值：1-8	共8行
输出：-
修改：20141112
备注：列地址是这样的：|0_低地址_f|________|________|________|________|________|________|________| 128个点
						高地址0															高地址7						

***********************************/
void OLED_MWC_set_XY(u8 col, u8 row) 
{   u8 l=0,h=0;     										//  Not used in MW V2.0 but its here anyway!
	col-=1;
	row-=1;
	OLED_MWC_send_cmd(0xb0+row);                //set page address
//	OLED_MWC_send_cmd(0x00+(8*col&0x0f));       //set low col address
//	OLED_MWC_send_cmd(0x10+((8*col>>4)&0x0f));  //set high col address
	col*=6;
	l=col%0x0f;
	h=col/0x10;
	OLED_MWC_send_cmd(l);
	OLED_MWC_send_cmd(0x10|h);
//	OLED_MWC_send_cmd(0x2f);  //实现了右移
}


/********************************
函数：OLED_MWC_set_line
功能：设置显示起始行
输入：起始行值1-8	共8行
输出：-
修改：20141112
***********************************/
void OLED_MWC_set_line(u8 row)
{   								// goto the beginning of a single row, compattible with LCD_CONFIG
	row=row-1;
	OLED_MWC_send_cmd(0xb0+row);     //set page address
	OLED_MWC_send_cmd(0);            //set low col address
	OLED_MWC_send_cmd(0x10);         //set high col address
}

/********************************
函数：OLED_MWC_send_char
功能：显示一个字符
输入：字符的ASCII码
输出：-
修改：20141112
***********************************/
void OLED_MWC_send_char(unsigned char ascii){
  unsigned char i;
  for(i=0;i<5;i++){
    buffer = myFont[ascii - 32][i]; // call the macro to read ROM byte and put it in buffer
    buffer ^= CHAR_FORMAT;  // apply
    OLED_MWC_send_byte(buffer);
  }
  OLED_MWC_send_byte(CHAR_FORMAT);    // the gap
}



/********************************
函数：OLED_MWC_send_string
功能：显示字符串
输入：字符数组地址
输出：-
修改：20141112
***********************************/
void OLED_MWC_send_string(const char *string){  // Sends a string of chars untill null terminator
  unsigned char i=0;
  while(*string){
    for(i=0;i<5;i++){
      buffer = myFont[(*string)- 32][i]; // call the macro to read the ROM ASCII table
      buffer ^= CHAR_FORMAT;
      OLED_MWC_send_byte((unsigned char)buffer);
    }
    OLED_MWC_send_byte(CHAR_FORMAT);    // the gap
    string++;
  }
}


void PrintDebugDat_OLED(int dat1,int dat2,int dat3,int dat4,char *str)
{
//	sprintf(DispData,"%4d %4d %4d %4d",dat1,dat2,dat3,dat4);
//	OLED_MWC_set_XY(1,2);
//	OLED_MWC_send_string(DispData);
	
	sprintf(DispData,"%4d %4d",dat1,dat2);
	OLED_MWC_set_XY(1,2);
	OLED_MWC_send_string(DispData);
}



#if 0
/********************************
函数：OLED_MWC_send_logo
功能：//logo闪烁显示logo
输入：-
输出：-
修改：20141112
***********************************/

void OLED_MWC_send_logo(void){
	unsigned char i,j;
	u16 k;
	OLED_MWC_send_cmd(0xa6);              //Set Normal Display
	OLED_MWC_send_cmd(0xae);      // Display OFF
	OLED_MWC_send_cmd(0x20);              // Set Memory Addressing Mode
	OLED_MWC_send_cmd(0x00);              // Set Memory Addressing Mode to Horizontal addressing mode
	OLED_MWC_send_cmd(0xb0);              // set page address to 0
	OLED_MWC_send_cmd(0X40);              // Display start line register to 0
	OLED_MWC_send_cmd(0);                 // Set low col address to 0
	OLED_MWC_send_cmd(0x10);              // Set high col address to 0
	for(k=0; k<1024; k++) 
	{          // fill the display's RAM with graphic... 128*64 pixel picture
		buffer = LOGO[k];
		OLED_MWC_send_byte(buffer);
	}
	OLED_MWC_send_cmd(0x81);             // Setup CONTRAST CONTROL, following byte is the contrast Value... always a 2 byte instruction
	OLED_MWC_send_cmd(0x0);              // Set contrast value to 0
	OLED_MWC_send_cmd(0xaf);           // display on
	for(j=0; j<2; j++)
	{
		for(i=0x01; i<0xff; i++)
		{
		  OLED_MWC_send_cmd(0x81);         // Setup CONTRAST CONTROL, following byte is the contrast Value
		  OLED_MWC_send_cmd(i);            // Set contrast value
		  Delay_ms(1);
		}
		for(i=0xff; i>0x01; i--)
		{
		  OLED_MWC_send_cmd(0x81);         // Setup CONTRAST CONTROL, following byte is the contrast Value
		  OLED_MWC_send_cmd(i);            // Set contrast value
		  Delay_ms(1);
		}
	}
	OLED_MWC_init();
//	OLED_MWC_Clear();
}



/********************************
函数：OLED_MWC_Put_Logo
功能：//永久显示Logo
输入：-
输出：-
修改：20141112
***********************************/

void OLED_MWC_Put_Logo(void){
  unsigned int i;
  OLED_MWC_send_cmd(0xa6);              //Set Normal Display
  OLED_MWC_send_cmd(0xae);              // Display OFF
  OLED_MWC_send_cmd(0x20);              // Set Memory Addressing Mode
  OLED_MWC_send_cmd(0x00);              // Set Memory Addressing Mode to Horizontal addressing mode
  OLED_MWC_send_cmd(0xb0);              // set page address to 0
  OLED_MWC_send_cmd(0X40);              // Display start line register to 0
  OLED_MWC_send_cmd(0);                 // Set low col address to 0
  OLED_MWC_send_cmd(0x10);              // Set high col address to 0
  for(i=0; i<1024; i++) {           // fill the display's RAM with graphic... 128*64 pixel picture
    buffer = LOGO[i];
    OLED_MWC_send_byte(buffer);
  }
  OLED_MWC_send_cmd(0x81);              // Setup CONTRAST CONTROL, following byte is the contrast Value... always a 2 byte instruction
  OLED_MWC_send_cmd(250);               // Here you can set the brightness 1 = dull, 255 is very bright
  OLED_MWC_send_cmd(0xaf);              // display on
}
#endif
/********************************
字库
修改：20141112
***********************************/
const uint8_t myFont[][5] = { // Refer to "Times New Roman" Font Database... 5 x 7 font
  { 0x00,0x00,0x00,0x00,0x00},
  { 0x00,0x00,0x4F,0x00,0x00}, //   (  1)  ! - 0x0021 Exclamation Mark
  { 0x00,0x07,0x00,0x07,0x00}, //   (  2)  " - 0x0022 Quotation Mark
  { 0x14,0x7F,0x14,0x7F,0x14}, //   (  3)  # - 0x0023 Number Sign
  { 0x24,0x2A,0x7F,0x2A,0x12}, //   (  4)  $ - 0x0024 Dollar Sign
  { 0x23,0x13,0x08,0x64,0x62}, //   (  5)  % - 0x0025 Percent Sign
  { 0x36,0x49,0x55,0x22,0x50}, //   (  6)  & - 0x0026 Ampersand
  { 0x00,0x05,0x03,0x00,0x00}, //   (  7)  ' - 0x0027 Apostrophe
  { 0x00,0x1C,0x22,0x41,0x00}, //   (  8)  ( - 0x0028 Left Parenthesis
  { 0x00,0x41,0x22,0x1C,0x00}, //   (  9)  ) - 0x0029 Right Parenthesis
  { 0x14,0x08,0x3E,0x08,0x14}, //   ( 10)  * - 0x002A Asterisk
  { 0x08,0x08,0x3E,0x08,0x08}, //   ( 11)  + - 0x002B Plus Sign
  { 0x00,0x50,0x30,0x00,0x00}, //   ( 12)  , - 0x002C Comma
  { 0x08,0x08,0x08,0x08,0x08}, //   ( 13)  - - 0x002D Hyphen-Minus
  { 0x00,0x60,0x60,0x00,0x00}, //   ( 14)  . - 0x002E Full Stop
  { 0x20,0x10,0x08,0x04,0x02}, //   ( 15)  / - 0x002F Solidus
  { 0x3E,0x51,0x49,0x45,0x3E}, //   ( 16)  0 - 0x0030 Digit Zero
  { 0x00,0x42,0x7F,0x40,0x00}, //   ( 17)  1 - 0x0031 Digit One
  { 0x42,0x61,0x51,0x49,0x46}, //   ( 18)  2 - 0x0032 Digit Two
  { 0x21,0x41,0x45,0x4B,0x31}, //   ( 19)  3 - 0x0033 Digit Three
  { 0x18,0x14,0x12,0x7F,0x10}, //   ( 20)  4 - 0x0034 Digit Four
  { 0x27,0x45,0x45,0x45,0x39}, //   ( 21)  5 - 0x0035 Digit Five
  { 0x3C,0x4A,0x49,0x49,0x30}, //   ( 22)  6 - 0x0036 Digit Six
  { 0x01,0x71,0x09,0x05,0x03}, //   ( 23)  7 - 0x0037 Digit Seven
  { 0x36,0x49,0x49,0x49,0x36}, //   ( 24)  8 - 0x0038 Digit Eight
  { 0x06,0x49,0x49,0x29,0x1E}, //   ( 25)  9 - 0x0039 Dight Nine
  { 0x00,0x36,0x36,0x00,0x00}, //   ( 26)  : - 0x003A Colon
  { 0x00,0x56,0x36,0x00,0x00}, //   ( 27)  ; - 0x003B Semicolon
  { 0x08,0x14,0x22,0x41,0x00}, //   ( 28)  < - 0x003C Less-Than Sign
  { 0x14,0x14,0x14,0x14,0x14}, //   ( 29)  = - 0x003D Equals Sign
  { 0x00,0x41,0x22,0x14,0x08}, //   ( 30)  > - 0x003E Greater-Than Sign
  { 0x02,0x01,0x51,0x09,0x06}, //   ( 31)  ? - 0x003F Question Mark
  { 0x32,0x49,0x79,0x41,0x3E}, //   ( 32)  @ - 0x0040 Commercial At
  { 0x7E,0x11,0x11,0x11,0x7E}, //   ( 33)  A - 0x0041 Latin Capital Letter A
  { 0x7F,0x49,0x49,0x49,0x36}, //   ( 34)  B - 0x0042 Latin Capital Letter B
  { 0x3E,0x41,0x41,0x41,0x22}, //   ( 35)  C - 0x0043 Latin Capital Letter C
  { 0x7F,0x41,0x41,0x22,0x1C}, //   ( 36)  D - 0x0044 Latin Capital Letter D
  { 0x7F,0x49,0x49,0x49,0x41}, //   ( 37)  E - 0x0045 Latin Capital Letter E
  { 0x7F,0x09,0x09,0x09,0x01}, //   ( 38)  F - 0x0046 Latin Capital Letter F
  { 0x3E,0x41,0x49,0x49,0x7A}, //   ( 39)  G - 0x0047 Latin Capital Letter G
  { 0x7F,0x08,0x08,0x08,0x7F}, //   ( 40)  H - 0x0048 Latin Capital Letter H
  { 0x00,0x41,0x7F,0x41,0x00}, //   ( 41)  I - 0x0049 Latin Capital Letter I
  { 0x20,0x40,0x41,0x3F,0x01}, //   ( 42)  J - 0x004A Latin Capital Letter J
  { 0x7F,0x08,0x14,0x22,0x41}, //   ( 43)  K - 0x004B Latin Capital Letter K
  { 0x7F,0x40,0x40,0x40,0x40}, //   ( 44)  L - 0x004C Latin Capital Letter L
  { 0x7F,0x02,0x0C,0x02,0x7F}, //   ( 45)  M - 0x004D Latin Capital Letter M
  { 0x7F,0x04,0x08,0x10,0x7F}, //   ( 46)  N - 0x004E Latin Capital Letter N
  { 0x3E,0x41,0x41,0x41,0x3E}, //   ( 47)  O - 0x004F Latin Capital Letter O
  { 0x7F,0x09,0x09,0x09,0x06}, //   ( 48)  P - 0x0050 Latin Capital Letter P
  { 0x3E,0x41,0x51,0x21,0x5E}, //   ( 49)  Q - 0x0051 Latin Capital Letter Q
  { 0x7F,0x09,0x19,0x29,0x46}, //   ( 50)  R - 0x0052 Latin Capital Letter R
  { 0x46,0x49,0x49,0x49,0x31}, //   ( 51)  S - 0x0053 Latin Capital Letter S
  { 0x01,0x01,0x7F,0x01,0x01}, //   ( 52)  T - 0x0054 Latin Capital Letter T
  { 0x3F,0x40,0x40,0x40,0x3F}, //   ( 53)  U - 0x0055 Latin Capital Letter U
  { 0x1F,0x20,0x40,0x20,0x1F}, //   ( 54)  V - 0x0056 Latin Capital Letter V
  { 0x3F,0x40,0x38,0x40,0x3F}, //   ( 55)  W - 0x0057 Latin Capital Letter W
  { 0x63,0x14,0x08,0x14,0x63}, //   ( 56)  X - 0x0058 Latin Capital Letter X
  { 0x07,0x08,0x70,0x08,0x07}, //   ( 57)  Y - 0x0059 Latin Capital Letter Y
  { 0x61,0x51,0x49,0x45,0x43}, //   ( 58)  Z - 0x005A Latin Capital Letter Z
  { 0x00,0x7F,0x41,0x41,0x00}, //   ( 59)  [ - 0x005B Left Square Bracket
  { 0x02,0x04,0x08,0x10,0x20}, //   ( 60)  \ - 0x005C Reverse Solidus
  { 0x00,0x41,0x41,0x7F,0x00}, //   ( 61)  ] - 0x005D Right Square Bracket
  { 0x04,0x02,0x01,0x02,0x04}, //   ( 62)  ^ - 0x005E Circumflex Accent
  { 0x40,0x40,0x40,0x40,0x40}, //   ( 63)  _ - 0x005F Low Line
  { 0x01,0x02,0x04,0x00,0x00}, //   ( 64)  ` - 0x0060 Grave Accent
  { 0x20,0x54,0x54,0x54,0x78}, //   ( 65)  a - 0x0061 Latin Small Letter A
  { 0x7F,0x48,0x44,0x44,0x38}, //   ( 66)  b - 0x0062 Latin Small Letter B
  { 0x38,0x44,0x44,0x44,0x20}, //   ( 67)  c - 0x0063 Latin Small Letter C
  { 0x38,0x44,0x44,0x48,0x7F}, //   ( 68)  d - 0x0064 Latin Small Letter D
  { 0x38,0x54,0x54,0x54,0x18}, //   ( 69)  e - 0x0065 Latin Small Letter E
  { 0x08,0x7E,0x09,0x01,0x02}, //   ( 70)  f - 0x0066 Latin Small Letter F
  { 0x06,0x49,0x49,0x49,0x3F}, //   ( 71)  g - 0x0067 Latin Small Letter G
  { 0x7F,0x08,0x04,0x04,0x78}, //   ( 72)  h - 0x0068 Latin Small Letter H
  { 0x00,0x44,0x7D,0x40,0x00}, //   ( 73)  i - 0x0069 Latin Small Letter I
  { 0x20,0x40,0x44,0x3D,0x00}, //   ( 74)  j - 0x006A Latin Small Letter J
  { 0x7F,0x10,0x28,0x44,0x00}, //   ( 75)  k - 0x006B Latin Small Letter K
  { 0x00,0x41,0x7F,0x40,0x00}, //   ( 76)  l - 0x006C Latin Small Letter L
  { 0x7C,0x04,0x18,0x04,0x7C}, //   ( 77)  m - 0x006D Latin Small Letter M
  { 0x7C,0x08,0x04,0x04,0x78}, //   ( 78)  n - 0x006E Latin Small Letter N
  { 0x38,0x44,0x44,0x44,0x38}, //   ( 79)  o - 0x006F Latin Small Letter O
  { 0x7C,0x14,0x14,0x14,0x08}, //   ( 80)  p - 0x0070 Latin Small Letter P
  { 0x08,0x14,0x14,0x18,0x7C}, //   ( 81)  q - 0x0071 Latin Small Letter Q
  { 0x7C,0x08,0x04,0x04,0x08}, //   ( 82)  r - 0x0072 Latin Small Letter R
  { 0x48,0x54,0x54,0x54,0x20}, //   ( 83)  s - 0x0073 Latin Small Letter S
  { 0x04,0x3F,0x44,0x40,0x20}, //   ( 84)  t - 0x0074 Latin Small Letter T
  { 0x3C,0x40,0x40,0x20,0x7C}, //   ( 85)  u - 0x0075 Latin Small Letter U
  { 0x1C,0x20,0x40,0x20,0x1C}, //   ( 86)  v - 0x0076 Latin Small Letter V
  { 0x3C,0x40,0x30,0x40,0x3C}, //   ( 87)  w - 0x0077 Latin Small Letter W
  { 0x44,0x28,0x10,0x28,0x44}, //   ( 88)  x - 0x0078 Latin Small Letter X
  { 0x0C,0x50,0x50,0x50,0x3C}, //   ( 89)  y - 0x0079 Latin Small Letter Y
  { 0x44,0x64,0x54,0x4C,0x44}, //   ( 90)  z - 0x007A Latin Small Letter Z
  { 0x00,0x08,0x36,0x41,0x00}, //   ( 91)  { - 0x007B Left Curly Bracket
  { 0x00,0x00,0x7F,0x00,0x00}, //   ( 92)  | - 0x007C Vertical Line
  { 0x00,0x41,0x36,0x08,0x00}, //   ( 93)  } - 0x007D Right Curly Bracket
  { 0x02,0x01,0x02,0x04,0x02}, //   ( 94)  ~ - 0x007E Tilde
  { 0x3E,0x55,0x55,0x41,0x22}, //   ( 95)  C - 0x0080 <Control>
  { 0x00,0x00,0x00,0x00,0x00}, //   ( 96)    - 0x00A0 No-Break Space
  { 0x00,0x00,0x79,0x00,0x00}, //   ( 97)  ! - 0x00A1 Inverted Exclamation Mark
  { 0x18,0x24,0x74,0x2E,0x24}, //   ( 98)  c - 0x00A2 Cent Sign
  { 0x48,0x7E,0x49,0x42,0x40}, //   ( 99)  L - 0x00A3 Pound Sign
  { 0x5D,0x22,0x22,0x22,0x5D}, //   (100)  o - 0x00A4 Currency Sign
  { 0x15,0x16,0x7C,0x16,0x15}, //   (101)  Y - 0x00A5 Yen Sign
  { 0x00,0x00,0x77,0x00,0x00}, //   (102)  | - 0x00A6 Broken Bar
  { 0x0A,0x55,0x55,0x55,0x28}, //   (103)    - 0x00A7 Section Sign
  { 0x00,0x01,0x00,0x01,0x00}, //   (104)  " - 0x00A8 Diaeresis
  { 0x00,0x0A,0x0D,0x0A,0x04}, //   (105)    - 0x00AA Feminine Ordinal Indicator
  { 0x08,0x14,0x2A,0x14,0x22}, //   (106) << - 0x00AB Left-Pointing Double Angle Quotation Mark
  { 0x04,0x04,0x04,0x04,0x1C}, //   (107)    - 0x00AC Not Sign
  { 0x00,0x08,0x08,0x08,0x00}, //   (108)  - - 0x00AD Soft Hyphen
  { 0x01,0x01,0x01,0x01,0x01}, //   (109)    - 0x00AF Macron
  { 0x00,0x02,0x05,0x02,0x00}, //   (110)    - 0x00B0 Degree Sign
  { 0x44,0x44,0x5F,0x44,0x44}, //   (111) +- - 0x00B1 Plus-Minus Sign
  { 0x00,0x00,0x04,0x02,0x01}, //   (112)  ` - 0x00B4 Acute Accent
  { 0x7E,0x20,0x20,0x10,0x3E}, //   (113)  u - 0x00B5 Micro Sign
  { 0x06,0x0F,0x7F,0x00,0x7F}, //   (114)    - 0x00B6 Pilcrow Sign
  { 0x00,0x18,0x18,0x00,0x00}, //   (115)  . - 0x00B7 Middle Dot
  { 0x00,0x40,0x50,0x20,0x00}, //   (116)    - 0x00B8 Cedilla
  { 0x00,0x0A,0x0D,0x0A,0x00}, //   (117)    - 0x00BA Masculine Ordinal Indicator
  { 0x22,0x14,0x2A,0x14,0x08}, //   (118) >> - 0x00BB Right-Pointing Double Angle Quotation Mark
  { 0x17,0x08,0x34,0x2A,0x7D}, //   (119) /4 - 0x00BC Vulgar Fraction One Quarter
  { 0x17,0x08,0x04,0x6A,0x59}, //   (120) /2 - 0x00BD Vulgar Fraction One Half
  { 0x30,0x48,0x45,0x40,0x20}, //   (121)  ? - 0x00BE Inverted Question Mark
  { 0x42,0x00,0x42,0x00,0x42}, //   (122)    - 0x00BF Bargraph - 0
  { 0x7E,0x42,0x00,0x42,0x00}, //   (123)    - 0x00BF Bargraph - 1
  { 0x7E,0x7E,0x00,0x42,0x00}, //   (124)    - 0x00BF Bargraph - 2
  { 0x7E,0x7E,0x7E,0x42,0x00}, //   (125)    - 0x00BF Bargraph - 3
  { 0x7E,0x7E,0x7E,0x7E,0x00}, //   (126)    - 0x00BF Bargraph - 4
  { 0x7E,0x7E,0x7E,0x7E,0x7E}, //   (127)    - 0x00BF Bargraph - 5
};

#if 0
/********************************
// logo....Multiwill
修改：20141112
***********************************/
const uint8_t LOGO[] = { 
    0x00, 0x00, 0x02, 0xFE, 0xFE, 0x0E, 0xFC, 0xF8, 0xC0, 0x00, 0xC0, 0xF8, 0xFC, 0x0E, 0xFE, 0xFE,
    0xFE, 0x02, 0x00, 0x00, 0x30, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x30, 0xF0, 0xF0, 0x00, 0x00, 0x00,
    0x02, 0xFE, 0xFE, 0x00, 0x00, 0x00, 0x30, 0xF8, 0xFE, 0x30, 0x30, 0x30, 0x00, 0x00, 0x30, 0xF6,
    0xF6, 0x00, 0x00, 0x00, 0x02, 0x06, 0x1E, 0xFE, 0xFE, 0xC2, 0x00, 0xC2, 0xFE, 0x7E, 0xFE, 0xC2,
    0x00, 0xC2, 0xFE, 0xFE, 0x3E, 0x06, 0x02, 0x00, 0x30, 0xF6, 0xF6, 0x00, 0x00, 0x00, 0x00, 0x30,
    0xF6, 0xF6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x10, 0x1F, 0x1F, 0x10, 0x00, 0x83, 0x9F, 0x9F, 0x9F, 0x83, 0x80, 0x90, 0x9F, 0x9F,
    0x9F, 0x10, 0x00, 0x00, 0x00, 0x0F, 0x1F, 0x18, 0x18, 0x18, 0x0C, 0x1F, 0x1F, 0x10, 0x00, 0x00,
    0x10, 0x1F, 0x1F, 0x10, 0x00, 0x80, 0x80, 0x8F, 0x9F, 0x98, 0x9E, 0x8F, 0x80, 0x80, 0x90, 0x1F,
    0x1F, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x1F, 0x1E, 0x1F, 0x03, 0x00, 0x07, 0x1F,
    0x1E, 0x1F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x1F, 0x1F, 0x10, 0x00, 0x00, 0x00, 0x10,
    0x1F, 0x1F, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xE0, 0xF8, 0x3C, 0x0E, 0x07, 0x03, 0x03, 0x01, 0x81, 0xC1, 0xC1, 0xC1, 0xC1, 0x81, 0x01,
    0x03, 0x03, 0x07, 0x0E, 0x3C, 0xF8, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0,
    0xF8, 0x3C, 0x0E, 0x07, 0x03, 0x03, 0x01, 0x81, 0xC1, 0xC1, 0xC1, 0xC1, 0x81, 0x01, 0x03, 0x03,
    0x07, 0x0E, 0x3C, 0xF8, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x3E, 0xF8, 0xC0,
    0xFC, 0x0E, 0xFC, 0xC0, 0xF8, 0x3E, 0x06, 0x00, 0xFE, 0xFE, 0x00, 0x06, 0xFF, 0xFF, 0x86, 0x86,
    0x00, 0xFF, 0xFF, 0x0C, 0x06, 0x06, 0xFE, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1F, 0x7F, 0xF0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x07, 0x0F, 0x0C, 0x0C, 0x0F, 0x07, 0x00,
    0x00, 0x00, 0x80, 0xC0, 0xF0, 0x7F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F,
    0x7F, 0xF0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x07, 0x0F, 0x0C, 0x0C, 0x0F, 0x07, 0x00, 0x00, 0x00,
    0x80, 0xC0, 0xF0, 0x7F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0xC0, 0xC0, 0xC0, 0xC1, 0xC1,
    0x80, 0x80, 0x00, 0x01, 0x01, 0x00, 0x00, 0xC0, 0xC1, 0xC1, 0xC0, 0xC0, 0xC0, 0xC1, 0xC1, 0xC1,
    0x80, 0x01, 0x01, 0x00, 0x00, 0x00, 0x81, 0x81, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x80, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x07, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
    0x07, 0x03, 0x03, 0x01, 0x03, 0x07, 0x0E, 0x1C, 0x38, 0xF0, 0xE0, 0xE0, 0xF0, 0x38, 0x1C, 0x0E,
    0x07, 0x03, 0x01, 0x03, 0x03, 0x07, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x07, 0x03,
    0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFE, 0xFF, 0x07, 0x03, 0x01, 0x01, 0xE1, 0xE1, 0xE1,
    0xE3, 0xE7, 0xE7, 0xE6, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x71, 0x71, 0x71, 0x71, 0x7B, 0x7F,
    0x3F, 0x1F, 0x00, 0x00, 0x00, 0x0F, 0x1F, 0x3F, 0x39, 0x71, 0x71, 0x71, 0xE3, 0xE7, 0xE7, 0x86,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x80, 0xC0, 0xE0, 0x70, 0x30, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
    0x38, 0x30, 0x70, 0xE0, 0xF0, 0xB8, 0x1C, 0x0E, 0x07, 0x03, 0x01, 0x01, 0x03, 0x07, 0x0E, 0x1C,
    0xB8, 0xF0, 0xE0, 0x70, 0x30, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x38, 0x30,
    0x70, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x0F, 0x0E, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C,
    0x0E, 0x0F, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x0F, 0x0F, 0x1E, 0x1C, 0x1C, 0x1C, 0x1C, 0x1E, 0x0F, 0x0F, 0x07,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xFE, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0xFC, 0xCC, 0xCC, 0xFC, 0x78, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE,
    0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0xFC, 0xCC, 0xCC, 0xFC, 0x78, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x04,
    0x04, 0x04, 0x04, 0x04, 0x04, 0x08, 0x10, 0xE0, 0x00, 0x00, 0x80, 0x40, 0x20, 0x20, 0x20, 0x40,
    0x80, 0x00, 0x60, 0x80, 0x00, 0x00, 0x00, 0x80, 0x60, 0x00, 0xF0, 0x08, 0x04, 0x04, 0x04, 0x08,
    0xF0, 0x00, 0x00, 0xE0, 0x5C, 0x44, 0x44, 0x44, 0x84, 0x04, 0x00, 0x00, 0x10, 0x08, 0x04, 0x04,
    0x04, 0x8C, 0x70, 0x00, 0x00, 0x30, 0x48, 0x84, 0x84, 0x84, 0x48, 0x30, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x07, 0x0F, 0x1C, 0x38, 0x30, 0x70, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60,
    0x70, 0x30, 0x38, 0x1C, 0x0F, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x07, 0x0F, 0x1C, 0x38, 0x30, 0x70, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x70, 0x30,
    0x38, 0x1C, 0x0F, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x20,
    0x20, 0x20, 0x20, 0x20, 0x20, 0x10, 0x08, 0x07, 0x00, 0x00, 0x0F, 0x12, 0x22, 0x22, 0x22, 0x12,
    0x0B, 0x00, 0x00, 0x01, 0x0E, 0x30, 0x0E, 0x01, 0x00, 0x00, 0x0F, 0x10, 0x20, 0x20, 0x20, 0x10,
    0x0F, 0x00, 0x00, 0x08, 0x10, 0x20, 0x20, 0x20, 0x10, 0x0F, 0x00, 0x00, 0x20, 0x30, 0x28, 0x24,
    0x22, 0x21, 0x20, 0x00, 0x00, 0x0E, 0x11, 0x20, 0x20, 0x20, 0x11, 0x0E, 0x00, 0x00, 0x00, 0x00

};

#endif
