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
boolean rinit=1;

void setup() {

  digitalWrite(13, OUTPUT);

}

void loop() {
  /*
    tinit variable is set to 1 by the time 5, если rinit=1
    #########################################
    rus: переменная тинит принимает значение 1 через время 5,
         если rinit=1
  */
  tinit = abc.one_start(rinit,5);
  digitalWrite(13, tinit);

}
