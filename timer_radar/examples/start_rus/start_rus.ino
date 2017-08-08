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
timer_radar abc(2);
boolean tinit;

void setup() {

  digitalWrite(13, OUTPUT);

}

void loop() {
  /*
    tinit variable is set to 1 for the period 600, the rest of 0.
    repeats every 1000.time
    #########################################
    rus: Переменная тинит принимает значение 1 на время 600, остальное время 0 
    повторяется каждые 1000.

  */
  tinit = abc.start(1000,600);
  digitalWrite(13, tinit);

}
