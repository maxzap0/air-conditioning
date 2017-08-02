/*Подсчет импульсов на заданной ножке с использованием таймера/счетчика TC1*/
/*git22*/
#pragma once

#include <avr/interrupt.h>

#define	PORT_Freq	PINC		//Порт вентилятора
#define	PIN_Freq	1			//Порт вентилятора

volatile unsigned int g_seconds = 0;

//Настройка на подсчет секунд глобальной переменной g_seconds
void setup_TC1() {
	TCCR1A = 0;				//Настройка таймера 1, канала А
	TCCR1B = 0x5;			//Предделитель CLK/1024;
	OCR1A = 0x3D09;			//Прерывание раз в секунду на частоте 16MГц
	TIMSK1 = 0x2;			//Запуск таймера по совпадению 1А
	sei();					//Разрешаем прерывания (запрещаем: cli(); )
}

//Обработчик прерывания по совпадению 1А
//Счетчик времени
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
  TCNT1 = 0;		//Обнуление регистра счетчика TCNT1
}

//Замер импульсов на порте PORT_Freq и ножке PIN_Freq с периодом
int freq(unsigned int period, unsigned int time_start, unsigned int time_metering)		//period - период замера импульсов в секундах, time_start - время начала замера, time_metering - длительность замера
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

//Мгновенный замер импульсов на порте PORT_Freq и ножке PIN_Freq 
int freq(unsigned int time_metering)		 //time_metering - длительность замера
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