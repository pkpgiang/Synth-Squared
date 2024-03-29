/*
 * keybd.h
 *
 *  Created on: Feb 2, 2023
 *      Author: phili
 */

#ifndef INC_KEYBD_H_
#define INC_KEYBD_H_

#include "usbh_hid_keybd.h"

static  const  uint8_t  HID_KEYBRD_Key[] =
{
  '\0',  '`',  '1',  '2',  '3',  '4',  '5',  '6',
  '7',  '8',  '9',  '0',  '-',  '=',  '\0', '\r',
  '\t',  'q',  'w',  'e',  'r',  't',  'y',  'u',
  'i',  'o',  'p',  '[',  ']',  '\\',
  '\0',  'a',  's',  'd',  'f',  'g',  'h',  'j',
  'k',  'l',  ';',  '\'', '\0', '\n',
  '\0',  '\0', 'z',  'x',  'c',  'v',  'b',  'n',
  'm',  ',',  '.',  '/',  '\0', '\0',
  '\0',  '\0', '\0', ' ',  '\0', '\0', '\0', '\0',
  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
  '\0',  '\0', '\0', '\0', '\0', '\r', '\0', '\0',
  '\0', '\0', '\0', '\0', '\0', '\0',
  '\0',  '\0', '7',  '4',  '1',
  '\0',  '/',  '8',  '5',  '2',
  '0',   '*',  '9',  '6',  '3',
  '.',   '-',  '+',  '\0', '\n', '\0', '\0', '\0', '\0', '\0', '\0',
  '\0',  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
  '\0', '\0', '\0', '\0'
};

static  const  uint8_t  HID_KEYBRD_Codes[] =
{
  0,     0,    0,    0,   31,   50,   48,   33,
  19,   34,   35,   36,   24,   37,   38,   39,       /* 0x00 - 0x0F */
  52,    51,   25,   26,   17,   20,   32,   21,
  23,   49,   18,   47,   22,   46,    2,    3,       /* 0x10 - 0x1F */
  4,    5,    6,    7,    8,    9,   10,   11,
  43,  110,   15,   16,   61,   12,   13,   27,       /* 0x20 - 0x2F */
  28,   29,   42,   40,   41,    1,   53,   54,
  55,   30,  112,  113,  114,  115,  116,  117,       /* 0x30 - 0x3F */
  118,  119,  120,  121,  122,  123,  124,  125,
  126,   75,   80,   85,   76,   81,   86,   89,       /* 0x40 - 0x4F */
  79,   84,   83,   90,   95,  100,  105,  106,
  108,   93,   98,  103,   92,   97,  102,   91,       /* 0x50 - 0x5F */
  96,  101,   99,  104,   45,  129,    0,    0,
  0,    0,    0,    0,    0,    0,    0,    0,       /* 0x60 - 0x6F */
  0,    0,    0,    0,    0,    0,    0,    0,
  0,    0,    0,    0,    0,    0,    0,    0,       /* 0x70 - 0x7F */
  0,    0,    0,    0,    0,  107,    0,   56,
  0,    0,    0,    0,    0,    0,    0,    0,       /* 0x80 - 0x8F */
  0,    0,    0,    0,    0,    0,    0,    0,
  0,    0,    0,    0,    0,    0,    0,    0,       /* 0x90 - 0x9F */
  0,    0,    0,    0,    0,    0,    0,    0,
  0,    0,    0,    0,    0,    0,    0,    0,       /* 0xA0 - 0xAF */
  0,    0,    0,    0,    0,    0,    0,    0,
  0,    0,    0,    0,    0,    0,    0,    0,       /* 0xB0 - 0xBF */
  0,    0,    0,    0,    0,    0,    0,    0,
  0,    0,    0,    0,    0,    0,    0,    0,       /* 0xC0 - 0xCF */
  0,    0,    0,    0,    0,    0,    0,    0,
  0,    0,    0,    0,    0,    0,    0,    0,       /* 0xD0 - 0xDF */
  58,   44,   60,  127,   64,   57,   62,  128        /* 0xE0 - 0xE7 */
};


void get_all_keys(HID_KEYBD_Info_TypeDef *info, int* key_tracker)
{
	int internal_key_tracker[16] = {0};
	for(int i=0; i<16; i++){
		uint8_t key = HID_KEYBRD_Key[HID_KEYBRD_Codes[info->keys[i]]];
		if (key == '\0'){
			for(int i = 0; i < 16; i++){
				key_tracker[i] = internal_key_tracker[i];
			}
			break;
		}
		switch(key){
			case 'z':
				internal_key_tracker[0] = 1;
						break;
			case 's':
				internal_key_tracker[1] = 1;
						break;
			case 'x':
				internal_key_tracker[2] = 1;
						break;
			case 'c':
				internal_key_tracker[3] = 1;
						break;
			case 'f':
				internal_key_tracker[4] = 1;
						break;
			case 'v':
				internal_key_tracker[5] = 1;
						break;
			case 'g':
				internal_key_tracker[6] = 1;
						break;
			case 'b':
				internal_key_tracker[7] = 1;
						break;
			case 'n':
				internal_key_tracker[8] = 1;
						break;
			case 'j':
				internal_key_tracker[9] = 1;
						break;
			case 'm':
				internal_key_tracker[10] = 1;
						break;
			case 'k':
				internal_key_tracker[11] = 1;
						break;
			case ',':
				internal_key_tracker[12] = 1;
						break;
			case 'l':
				internal_key_tracker[13] = 1;
						break;
			case '.':
				internal_key_tracker[14] = 1;
						break;
			case '/':
				internal_key_tracker[15] = 1;
						break;
			default:
				break;
		}
	}
	for(int i = 0; i < 16; i++){
					key_tracker[i] = internal_key_tracker[i];
				}
}

#endif /* INC_KEYBD_H_ */
