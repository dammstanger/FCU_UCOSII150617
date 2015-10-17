//20141001
#ifndef __OLED_MWC_H
#define __OLED_MWC_H

#include "Project_cfg.h"
#include "stm32f10x.h"


#define OLED_ADDRESS 0x3C<<1	 // OLED at address 0x3C in 7bit	//ÓëHMC5883µØÖ·ÖØ¸´

bool OLED_MWC_init(void);
void OLED_MWC_Clear(void);
void OLED_MWC_set_XY(u8 col, u8 row);        //  Not used in MW V2.0 but its here anyway!
void OLED_MWC_set_line(u8 row);   // goto the beginning of a single row, compattible with LCD_CONFIG
void OLED_MWC_send_char(unsigned char ascii);
void OLED_MWC_send_string(const char *string);  // Sends a string of chars untill null terminator
void OLED_MWC_send_logo(void);
void OLED_MWC_Put_Logo(void);
void PrintDebugDat_OLED(int dat1,int dat2,int dat3,int dat4,char *str);



#endif


