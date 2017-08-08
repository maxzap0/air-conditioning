#include <timer_radar.h>

timer_radar abc(2); //обозначаем класс

int average_rand;

void setup() {
  Serial.begin(9600);
}

void loop() {
  average_rand=random(110); //присваеваем случайное число
  /*
  Усредняем случайное число.
  average(average_rand, 200):
   average_rand - значение,
   200 - количество усредняемых значений
  */
  Serial.println(abc.average(average_rand, 200)); 

}
