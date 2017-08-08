
/*
Copyright (c) 2015, AO "NTC"RADAR", by 42 sektor.
*/
#ifndef timer_radar_h
#define timer_radar_h
 
#include "Arduino.h"
 
class timer_radar
{
  public:
    timer_radar(int dimension);
    boolean start(long timer, long impulse);
	boolean blink(unsigned long timer_blink);
	boolean one_start(boolean input_init, unsigned long one_timer); 
	float average(float value, int quantityvalue);
	
  private:
     long _impulse;
	 long _timer;
	 unsigned long _previous_time=0;
	 unsigned long _previous_time_blink;
	 unsigned long _previous_time_avarage;
	 unsigned long _dimension;
	 unsigned long _blink;
	 boolean _init_blink;
	 boolean _init_one_start;
	 int _quantityvalue;
	 float _value;
	 int _count=0;
	 float _sumvalue=0;
	 float _sumquantityvalue=0;
	 boolean _average_one=true;
};
 
#endif