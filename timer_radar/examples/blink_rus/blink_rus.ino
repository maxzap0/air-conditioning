/*
  include the library timer_radar
  #########################################
  rus: подключаем библиотеку timer_radar
*/

#include <timer_radar.h>

/*
  create a variable to pass a parameter dimension:
      1-microseconds,
      2-milliseconds,
      3-seconds,
      4-minutes.
  #########################################
  rus: создаём переменную с параметром размерности
         1-микросекунды
         2-миллесекунды
         3-секунды
         4-минуты.
*/
timer_radar abc(3);
boolean tinit;

void setup() {

  digitalWrite(13, OUTPUT);

}

void loop() {
  /*
    tinit variable takes the value of 1 or 0
    at the specified (2) The period of time
    #########################################
    rus: переменная tinit принимает значение 1 или 0
    через заданный (2) промежуток времени

  */
  tinit = abc.blink(2);
  digitalWrite(13, tinit);

}
