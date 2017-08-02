/*оНДЯВЕР ХЛОСКЭЯНБ МЮ ГЮДЮММНИ МНФЙЕ Я ХЯОНКЭГНБЮМХЕЛ РЮИЛЕПЮ/ЯВЕРВХЙЮ TC1*/
/*git22*/
#pragma once

#include <avr/interrupt.h>

#define	PORT_Freq	PINC		//оНПР БЕМРХКЪРНПЮ
#define	PIN_Freq	1			//оНПР БЕМРХКЪРНПЮ

volatile unsigned int g_seconds = 0;

//мЮЯРПНИЙЮ МЮ ОНДЯВЕР ЯЕЙСМД ЦКНАЮКЭМНИ ОЕПЕЛЕММНИ g_seconds
void setup_TC1() {
	TCCR1A = 0;				//мЮЯРПНИЙЮ РЮИЛЕПЮ 1, ЙЮМЮКЮ ю
	TCCR1B = 0x5;			//оПЕДДЕКХРЕКЭ CLK/1024;
	OCR1A = 0x3D09;			//оПЕПШБЮМХЕ ПЮГ Б ЯЕЙСМДС МЮ ВЮЯРНРЕ 16MцЖ
	TIMSK1 = 0x2;			//гЮОСЯЙ РЮИЛЕПЮ ОН ЯНБОЮДЕМХЧ 1ю
	sei();					//пЮГПЕЬЮЕЛ ОПЕПШБЮМХЪ (ГЮОПЕЫЮЕЛ: cli(); )
}

//нАПЮАНРВХЙ ОПЕПШБЮМХЪ ОН ЯНБОЮДЕМХЧ 1ю
//яВЕРВХЙ БПЕЛЕМХ
ISR(TIMER1_COMPA_vect)
{
  if (g_seconds > 2) 
  {
    g_seconds = 0;
  }
  else 
  {
    g_seconds++;
  }  
  TCNT1 = 0;		//нАМСКЕМХЕ ПЕЦХЯРПЮ ЯВЕРВХЙЮ TCNT1
}

//гЮЛЕП ХЛОСКЭЯНБ МЮ ОНПРЕ PORT_Freq Х МНФЙЕ PIN_Freq Я ОЕПХНДНЛ
int freq(unsigned int period, unsigned int time_start, unsigned int time_metering)		//period - ОЕПХНД ГЮЛЕПЮ ХЛОСКЭЯНБ Б ЯЕЙСМДЮУ, time_start - БПЕЛЪ МЮВЮКЮ ГЮЛЕПЮ, time_metering - ДКХРЕКЭМНЯРЭ ГЮЛЕПЮ
{
	int frequency=0;
	int prev=0, real=0;
	while ((g_seconds >= time_start ) &(g_seconds <= time_start + time_metering))
	{
		{
			real = (PORT_Freq & PIN_Freq);
			if (real != prev)
			{
				frequency++;
				//USART_Transmit('/');
				//Serial.println('*');
			}
			prev = real;
		}
	}
	return (int)frequency/2;
}

//лЦМНБЕММШИ ГЮЛЕП ХЛОСКЭЯНБ МЮ ОНПРЕ PORT_Freq Х МНФЙЕ PIN_Freq 
int freq(unsigned int time_metering)		 //time_metering - ДКХРЕКЭМНЯРЭ ГЮЛЕПЮ
{
	g_seconds = 0;
	TCNT1 = 0;
	int frequency=0;
	int prev=0, real=0;
	while (g_seconds < time_metering)
	{
		{
			real = (PORT_Freq & PIN_Freq);
			if (real != prev)
			{
				frequency++;
			}
			prev = real;
		}
	}
	return (int)frequency/2;
}