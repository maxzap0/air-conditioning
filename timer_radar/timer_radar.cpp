
#include "Arduino.h"
#include "timer_radar.h"

/*
  Copyright (c) 2015, AO "NTC"RADAR", by 42 sektor.
*/

timer_radar::timer_radar(int dimension)
{
 _dimension=dimension;
 boolean _init_blink=0;
 boolean _init_one_start=0;
}


boolean timer_radar::start(long timer, long impulse) 
{
	/* 
		
	*/
	unsigned long _current_time = micros();
	switch (_dimension) {
		case 1:
			_current_time = micros();
		break;
		case 2:
			_current_time = millis();
		break;
		case 3:
			_current_time = millis()/1000;
		break;
		case 4:
			_current_time = millis()/60000;
		break;
	} 
	
	_timer=timer;
	_impulse=impulse;
	boolean _init;
	    
	if (_impulse<0) {
		_impulse=0;
	}	
	if (_timer<0) {
		_timer=0;
	}
		
   if (_current_time - _previous_time >= _timer) {
		_previous_time=_current_time;
		_init=1;
    }
	  
   if (_init==1 and _current_time - _previous_time >= _impulse){
		_init=0;
   }
  
     return _init;
	 
}

boolean timer_radar::blink(unsigned long timer_blink)
{
	/*
	
	*/
	unsigned long _current_time_blink = millis();
	switch (_dimension) {
		case 1:
			_current_time_blink = micros();
		break;
		case 2:
			_current_time_blink = millis();
		break;
		case 3:
			_current_time_blink = millis()/1000;
		break;
		case 4:
			_current_time_blink = millis()/60000;
		break;
	}
	
	_timer=timer_blink;
	
	if (_current_time_blink - _previous_time_blink >= _timer) {
	  	_previous_time_blink=_current_time_blink;
        if (_init_blink==0) 
			_init_blink=1;
		else
			_init_blink=0;	
	}
	
	return _init_blink;
}

boolean timer_radar::one_start(boolean input_init, unsigned long one_timer){
	unsigned long _current_time_one_start = millis();
	switch (_dimension) {
		case 1:
			_current_time_one_start = micros();
		break;
		case 2:
			_current_time_one_start = millis();
		break;
		case 3:
			_current_time_one_start = millis()/1000;
		break;
		case 4:
			_current_time_one_start = millis()/60000;
		break;
	}
	
	boolean _input_init=input_init;
	unsigned long _one_timer=one_timer;
	
	if (_current_time_one_start>=_one_timer and input_init==1) 
		_init_one_start=1;
	else
		_init_one_start=0;

	return _init_one_start;
}

float timer_radar::average(float value, int quantityvalue){
	
	if (_average_one==true){
		_sumquantityvalue=value;
		_average_one=false;
	}
	
	_quantityvalue=quantityvalue;
	_value=value;
	_count=_count+1;
	_sumvalue=_sumvalue+_value;
	
	 if (_count==_quantityvalue) {
		_sumquantityvalue=_sumvalue/_count;
		_sumvalue=0;
		_count=0;
		
    }
	return _sumquantityvalue;
}
