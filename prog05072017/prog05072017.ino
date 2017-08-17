#include <EEPROM.h>
#include <timer_radar.h>
#include <DHT.h>
#include <Wire.h> //библиотека I2C
#include "frequency.h"

#define pcf1 0x38 // первый сегиент индикатора
#define pcf2 0x39 //
#define pcf3 0x3A //
#define pcf4 0x3B // последний сегмент индикатора
#define pcf5 0x3C // ДИПы
#define pcf6 0x3D // 

#define dac1 0x2C // первый DAC
#define dac2 0x2D //
#define dac3 0x2E //
#define dac4 0x2F // последний DAC

#define A_PORT PIND //Порт ножки A энкодера
#define A_pin 6     //Пин ножки А энкодера
#define B_PORT PIND //Порт ножки B энкодера
#define B_pin 7     //Пин ножки B энкодера

#define user_reg 60 //диапазон регулировки вентилятора пользователем
#define user_reg_min 70 //Минимальный уровень скорости вентилятора для пользователя

#define compressor1_port PORTD//Определение порта компрессора
#define compressor1_pin 3
#define compressor2_port PIND
#define compressor2_pin 5

#define button1_port PINB  //Определение кнопки компрессора
#define button1_pin 0
#define button2_port PINB  //Определение кнопки компрессора
#define button2_pin 1

#define thermostat_port PORTC//Определение порта термостата
#define thermostat_pin 6

#define time_fan 1500

/*Датчик температуры*/
DHT dht(A1, DHT22); //ДТ1
DHT dht2(A2, DHT22); //ДТ2

/*Замер скорости работы программы*/
uint16_t prev_millis_speed = 0; // предыдущее время
uint16_t cur_millis_speed = 0; // текущее время
uint16_t time_millis = 0; // время прошедшее после включения

/*Работа энкодера*/
byte A = 1; // Первоначальное сосояние A
byte B = 1; // Первоначальное состояние B
byte pinLast = 1; // Служебная переменная
bool enc_init=false; //Регистр вращения энкодера

/*Считывание датчик температуры */
byte count_cycle_temp = 0;

/*Таймер мигания дисплея*/
timer_radar blink_disp(2);

/*Функция change_val*/
bool change_enc_init;                   // регистр функции change_val
unsigned long int change_time_prev;     // для запоминания времени
bool change_val_int;                    // регистр функции change_val

/*Переменные оборотов вентилятора*/
uint16_t rpm_1_cold;      //первый холодный вентилятор, текущее значение оборотов
uint16_t rpm_1_cold_dac;  //Значение выдаваемое на dac первого холодного венттидятора
uint16_t rpm_2_cold;      //второй холодный вентилятор, текущее значение оборотов
uint16_t rpm_2_cold_dac;  //Значение выдаваемое на dac второго холодного венттидятора
uint16_t rpm_1_hot;       //Первый горячий вентилятор
uint16_t rpm_1_hot_dac;   //Значение выдаваемое на dac первого горячего венттидятора
uint16_t rpm_2_hot;       //Второй горячий вентилятор
uint16_t rpm_2_hot_dac;   //Значение выдаваемое на dac второго горячего венттидятора

/*Замер скорости работы вентиляторов*/
int *rpm_fan[4];      
int mass[4] = {0,}; 
int i;

/*Переменные считаные с реле*/
bool fan1;          //состояние 1 горячего вентилятора
bool fan2;          //состояние 2 горячего вентилятора
bool low_press_1;   //реле низкого давоения 1
bool high_press_1;  //реле высокого давления 1
bool low_press_2;   //реле низкого давления 2
bool high_press_2;  //реле высокого давления 2

/*Переменные рабочего режима*/
byte indicator = true;         //выбор режима отображения. 1-температура 2-обороты
byte user_rpm = 50;            //число оборотов вентиляторов в процентах для пользователя
int user_rpm_k1;               //обороты вентилятора 1 прибавляемые/убавляемые пользователям
int user_rpm_k2;               //обороты вентилятора 2 прибавляемые/убавляемые пользователям
float t_real;                  //температура реальная с датчика 1
float t_wish=25;               //требуемая температура
bool k1_button;                //кнопка первого компрессора 
bool k2_button;                //кнопка второго компрессора
bool k1_init;                  //разрешение на включение первого крмпрессора
bool k2_init;                  //разрешение на включение первого крмпрессора
bool k1_action;                //переменная работы первого компрессора
bool k2_action;                //переменная работы второго компрессора
bool temp_init;                //Хранит состояние термостата (нужно ли включить или нет)
unsigned long fan1_last_time;  //Переменная хранения времени после работы вентилятора перед запуском компрессора
bool fan1_time;                //Регистр прошедшего времени после работы первого вентилятора


/* Временные переменные *//////////////////////////////////////////////////////////////////////////////////////////  

/**///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup(){
  Serial.begin(115200); // скорость com-порта
  Wire.begin(); // подключение к шине wire
  /*Установка пинов энкодера*/
  pinMode(6, INPUT_PULLUP);  // установка пина на вход, подтяжка к +5В
  pinMode(7, INPUT_PULLUP);  // установка пина на вход, подтяжка к +5В
  /*Установка пинов датчика температуры*/
  pinMode(8, INPUT_PULLUP);  // установка пина на вход, подтяжка к +5В
  /*Установка пина кнопки энкодера*/
  pinMode(A3, INPUT_PULLUP); // установка пина на вход, подтяжка к +5В
  /*Установка портов компрессора как выходы*/
  pinMode(3, OUTPUT);        //компрессор 1
  pinMode(5, OUTPUT);        //компрессор 2

  /*Считывание первоначальных значение EEPROM*/
  rpm_1_cold_dac = EEPROM.read(1);  //значение rpm первого холодного вентилятора
  rpm_2_cold_dac = EEPROM.read(2);  //значение rpm второго холодного вентилятора
  rpm_1_hot_dac = EEPROM.read(3);   //значение rpm первого горячего вентилятора
  rpm_2_hot_dac = EEPROM.read(4);   //значение rpm первого горячего вентилятора

  /*Настройка измерителя чистоты*/
  setup_TC1(20);
  for (i=0; i<4; i++) {rpm_fan[i]=&mass[i];}

 /*Временные установки*//////////////////////////////////////////////////////////////////////////////////////////
 
}

void loop(){

  /*Замер скорости работы программы*/
  //prev_millis_speed = millis(); // время на начало программы
  /************************************/

  /*Считывание состояния реле и кнопок*/
  read_relay();

  /*Считывание оборотов вентиляторов*/
  freq(rpm_fan, 5, 1);
  rpm_1_cold=*rpm_fan[1]; //первый холодный
  rpm_2_cold=*rpm_fan[3]; //второй холодный
  rpm_1_hot=*rpm_fan[0];  //первый горячий 
  rpm_2_hot=*rpm_fan[2];  //первый горячий
 

  //Считывание режима dip переключаталей и переход в нужный режим
  switch (read_dip()) {
      case 15 :  //Рабочий режим
          work();
        break;
      case 14 : //настройка 1го холодного вентилятора
        fan_setting1();
      break;
      case 13:  //настройка 2го холодного вентилятора
        fan_setting2();
      break;
      case 11:  //настройка 1го горяего вентилятора
        fan_setting3();
      break;
      case 7:  //настройка 2го горяего вентилятора
        fan_setting4();
      break;
      case 0:  //вывод температуры со второго датчика темепературы
        write_display_temp(temp_metr(200,0));
      break;
     default : //если комбинация дипов не встретилось выводим 'no'
        seg7_write(pcf4, 0, 0);
        seg7_write(pcf3, 'n', 0);
        seg7_write(pcf2, ' ', 0);
        seg7_write(pcf1, ' ', 0);
  } 

  /*Замер скорости работы программы*/
  //cur_millis_speed = millis(); // время на конец программы
  //Serial.println(cur_millis_speed - prev_millis_speed); // вывод времени исполнения программы

  
/*  compressor1_port |=(k1_action<<compressor1_pin);*/
  
  /************************************/
}

/*Рабочий режим работы программы*/
void work() {

  bitRead(PINC, 3) ? indicator=1 : indicator = 2;   //Считывание значения переключателя 

  dac_write(dac1, user_rpm_k1);                            //Включение вентиляторов
  dac_write(dac2, user_rpm_k2);
  dac_write(dac3, rpm_1_hot_dac);
  dac_write(dac4, rpm_2_hot_dac);  

  t_real=temp_metr(100, 1);                                      //считывание значение температуры

  user_rpm_k1 = rpm_1_cold_dac + ((user_rpm-50)*(user_reg/50)); //Изменение оборотов пользователем на 1м холод вентиляторе
  user_rpm_k2 = rpm_2_cold_dac + ((user_rpm-50)*(user_reg/50)); //Изменение оборотов пользователем на 2м холод вентиляторе

  switch (indicator) {                                          //Выбор режима: температура или обороты
      case 1: {
        user_rpm = encoder_read(user_rpm, 1);                   //Изменение значения оборотов
        change_val(user_rpm,user_rpm, 0);                       //Индикация оборотов на дисплее

        if (user_rpm>100) {                                     //Защита от выхода за пределы регулировки оборотов
          user_rpm=100;
        } else if (user_rpm<1) {
          user_rpm=1;
        }

        if (user_rpm_k1<user_reg_min) {                         //Защита от выхода за перделы значений вентилятора 1 холодного
          user_rpm_k1=user_reg_min;
        } else if (user_rpm_k1>255) {
          user_rpm_k1=255;
        }

         if (user_rpm_k2<user_reg_min) {                        //Защита от выхода за перделы значений вентилятора 2 холодного
          user_rpm_k2=user_reg_min;
        } else if (user_rpm_k2>255) {
          user_rpm_k2=255;
        }      
        } break;  

      case 2: {                                                       //Режим температуры
       t_wish=encoder_read(t_wish,0.5);                               //изменение темепературы крутилкой
       change_val(t_real,t_wish,1);                                   //индикация температуры на дисплее
       if (t_wish>35) {                                               //Защита от выхода за пределы
         t_wish=35;
       } else if (t_wish<18.5) {
         t_wish=18;
       }
       break;
      }
  }

  if ((t_real-t_wish)>0) {                                 //Если температура выше желаемой, пишем регистр термостата
    temp_init = true;
  } else {
    temp_init = false;
  }
  thermostat_port |=(temp_init<<thermostat_pin);          //Выдача значения на ногу термостата

  
  /*if (!fan1 & k1_action) {                                //Если не сработало реле вентилятора и компрессор работает
    k1_init=true;                                         //Разрешаем включение 
  } else if(fan1) {                                       //Если не сработало реле вентилятора разрешаем включение
    k1_init=true; 
  } else {                                                //Если реле сработало запрещаем включение
    k1_init=false;
  }

  
  if (!k1_button & !fan1) {                               //Если нажата кнопка и не вкл реле вентилятора
    if (millis()-fan1_last_time>time_fan) {               //Если прошло время Разрешаем работу
      fan1_time=true;                                     
    }
  } else if (!k1_button & fan1) {                         //Если кнопка нажата а вентилятор выключен разрешаем работу
    fan1_time=true;
  } else {                                                //Иначе запрещаем работу
    fan1_last_time = millis();
    fan1_time=false;
  }


  if (!k1_button & fan1_time & k1_init & temp_init) {
    k1_action = true;
  } else {k1_action = false;}*/

  if (!k1_button) {                                     //Если нажата кнопка
    k1_action=true;                                     //Включаем компрессор
  }

  if (!fan1){                                                                      //Если включён вентилятор 
    //Serial.print(fan1_last_time); Serial.print(" "); Serial.println(millis());                                 
    if (millis()-fan1_last_time>time_fan) {                                        //Если прошло время 
      k1_action=true;                                                              //Включаем компрессор
    } else {
      k1_action=false;
    }
  } else {
    fan1_last_time=millis(); 
  }

  if (!fan1 & fan1_time) {
    k1_action=true;} 

  if (!temp_init) {                                       //Если не сработал термостат
    k1_action = false;                                    //выключаем компрессор
  }

  if (k1_button) {                                        //Если кнопка не нажата 
    k1_action=false;                                      //выключаем компрессор
  }

  /*if (!k1_action) {
    fan1_last_time=millis();
  } */

  k1_action ? fan1_time=true:fan1_time=false;

  Serial.println(k1_action);
}

/*Функция настройки вентилятора первого холодного*/
void fan_setting1 () {
  rpm_1_cold_dac=encoder_read(rpm_1_cold_dac, 1);       //Если крутим энкодер меняем значение dac
  if (rpm_1_cold_dac>255) {                             //Защита от выхода за пределы значений dac
    rpm_1_cold_dac=255;
  } else if (rpm_1_cold_dac<1) {
    rpm_1_cold_dac=1;
  }
  if (change_val(rpm_1_cold, rpm_1_cold_dac, 0)) {      //Если функция изменения (мигание дисплеем) значения закончила работу (отдала true) 
    EEPROM.write(1, rpm_1_cold_dac);                    //Пишем значение в EEPROM                
  }
  dac_write(dac1, rpm_1_cold_dac);                      //Вывод текущего значения на dac
}

/*Функция настройки вентилятора второго холодного*/
void fan_setting2 () {
  rpm_2_cold_dac=encoder_read(rpm_2_cold_dac, 1);       //Если крутим энкодер меняем значение dac
  if (rpm_2_cold_dac>255) {                             //Защита от выхода за пределы значений dac
    rpm_2_cold_dac=255;
  } else if (rpm_2_cold_dac<1) {
    rpm_2_cold_dac=1;
  }
  if (change_val(rpm_2_cold, rpm_2_cold_dac, 0)) {      //Если функция изменения (мигание дисплеем) значения закончила работу (отдала true) 
    EEPROM.write(2, rpm_2_cold_dac);                    //Пишем значение в EEPROM
  }
  dac_write(dac2, rpm_2_cold_dac);                      //Вывод текущего значения на dac
}

/*Функция настройки вентилятора первого горячего*/
void fan_setting3 () {
  rpm_1_hot_dac=encoder_read(rpm_1_hot_dac, 1);       //Если крутим энкодер меняем значение dac
  if (rpm_1_hot_dac>255) {                             //Защита от выхода за пределы значений dac
    rpm_1_hot_dac=255;
  } else if (rpm_1_hot_dac<1) {
    rpm_1_hot_dac=1;
  }
  if (change_val(rpm_1_hot, rpm_1_hot_dac, 0)) {      //Если функция изменения (мигание дисплеем) значения закончила работу (отдала true) 
    EEPROM.write(3, rpm_1_hot_dac);                    //Пишем значение в EEPROM
  }
  dac_write(dac3, rpm_1_hot_dac);                      //Вывод текущего значения на dac
}

/*Функция настройки вентилятора первого горячего*/
void fan_setting4 () {
  rpm_2_hot_dac=encoder_read(rpm_2_hot_dac, 1);       //Если крутим энкодер меняем значение dac
  if (rpm_2_hot_dac>255) {                             //Защита от выхода за пределы значений dac
    rpm_2_hot_dac=255;
  } else if (rpm_2_hot_dac<1) {
    rpm_2_hot_dac=1;
  }
  if (change_val(rpm_2_hot, rpm_2_hot_dac, 0)) {      //Если функция изменения (мигание дисплеем) значения закончила работу (отдала true) 
    EEPROM.write(4, rpm_2_hot_dac);                    //Пишем значение в EEPROM
  }
  dac_write(dac4, rpm_2_hot_dac);                      //Вывод текущего значения на dac
}

/*Функция считывания значений dip-переключателей, возвращает значение dip-переключателей*/
byte read_dip () {
  return (0b1111 & read_pcf(pcf5)); //считываем первые четыре байта pcf
  //
}  

/*Функция считывания значений реле и кнопок компрессора*/
void read_relay() {
  fan1 = ( 0b01 & read_pcf(pcf6) );                               //считывание реле первого вентилятора 
  fan2 = ( 0b10 & read_pcf(pcf6) );                               //считывание реле второго вентилятора
  low_press_1 =  ( 0b10000 & read_pcf(pcf5)    );                 //считывание реле низкого давления 1
  high_press_1 = ( 0b100000 & read_pcf(pcf5)   );                 //считывание реле высокого давления 1
  low_press_2 =  ( 0b1000000 & read_pcf(pcf5)  );                 //считывание реле низкого давления 2
  high_press_2 = ( 0b10000000 & read_pcf(pcf5) );                 //считывание реле высокого давления 2
  k1_button=      bitRead(button1_port, button1_pin);             //считывание кнопки компрессора 1
  k2_button=      bitRead(button1_port, button1_pin);             //считывание кнопки компрессора 2
}

/*Функция вывода на 7seg дисплей */
// Формат seg7_write(адрес, значение, точка)
void seg7_write(byte addr, unsigned char val, byte dot){
  unsigned char _val_bit;
  switch (val)
  {
    // перевод символа в двоичный
    case 0:
      _val_bit = B00111111;
      break;
    case 1:
      _val_bit = B00000110;
      break;
    case 2:
      _val_bit = B01011011;
      break;
    case 3:
      _val_bit = B01001111;
      break;
    case 4:
      _val_bit = B01100110;
      break;
    case 5:
      _val_bit = B01101101;
      break;
    case 6:
      _val_bit = B01111101;
      break;
    case 7:
      _val_bit = B00000111;
      break;
    case 8:
      _val_bit = B01111111;
      break;
    case 9:
      _val_bit = B01101111;
      break;
    case 'a':
      _val_bit = B01110111;
      break;
    case 'b':
      _val_bit = B01111100;
      break;
    case 'c':
      _val_bit = B00111001;
      break;
    case 'd':
      _val_bit = B01011110;
      break;
    case 'e':
      _val_bit = B01111001;
      break;
    case 'f':
      _val_bit = B01110001;
      break;
    case 'g':
      _val_bit = B00111101;
      break;
    case 'h':
      _val_bit = B01110110;
      break;
    case 'i':
      _val_bit = B00000110;
      break;
    case 'j':
      _val_bit = B00001110;
      break;
    case 'l':
      _val_bit = B00111000;
      break;
    case 'n':
      _val_bit = B01010100;
      break;
    case 'p':
      _val_bit = B01110011;
      break;
    case 'q':
      _val_bit = B01100111;
      break;
    case 'r':
      _val_bit = B01010000;
      break;
    case 's':
      _val_bit = B01101101;
      break;
    case 't':
      _val_bit = B01111000;
      break;
    case 'u':
      _val_bit = B00111110;
      break;
    case 'y':
      _val_bit = B01101110;
      break;
    case ' ':
      _val_bit = B00000000;
      break;
    case '-':
      _val_bit = B01000000;
      break;
    case '*':
      _val_bit = B01100011;
      break;
  }
  Wire.beginTransmission(addr); // начало передачи по адресу
  Wire.write(_val_bit); // вывод значения
  dot? Wire.write(_val_bit |= 1 << 7):Wire.write(_val_bit |= 0 << 7); // вывод точки
  Wire.endTransmission(); // конец передачи
}

//Выключение дисплея. 
void display_hide(byte init){
    if (init) {
      seg7_write(pcf1, ' ', 0);
      seg7_write(pcf2, ' ', 0);
      seg7_write(pcf3, ' ', 0);
      seg7_write(pcf4, ' ', 0);
    } else {
      ;
    }
 }

//Мигание дисплея. Принимает значение выводимое на дисплей и вид измерения (false - температура, true - обороты)
void display_blink(float val, bool init) {
  if (blink_disp.blink(50)) {     //Частота мерцания
      init ? write_display_temp(val) : write_display_rpm ((int)val);
  } else {
      display_hide(1);
  }
}

/*Функция вывода температуры на семисегментный дисплей*/
//Принимает значение температуры float
void write_display_temp(float val) {
  float _fraction = (val - ((int)val))*10; //вычисление дробной части
  //если значение меньше -10
  if (val<=-10) {
    seg7_write(pcf1,'-',0);                 //в первый разряд -
    seg7_write(pcf2, abs (val / 10), 0);    //второй разряд первая цифра
    seg7_write(pcf3, abs ((int)val % 10), 1); //третий разряд вторая цифра
    seg7_write(pcf4, abs(_fraction), 0);      //четвёртый разряд дробная часть
  }
  else if (val<0) {
    seg7_write(pcf1,'-',0);
    seg7_write(pcf2, 0, 0);
    seg7_write(pcf3, abs(val), 1);
    seg7_write(pcf4, abs(_fraction), 0);
  } 
  else if (val<10) {
    seg7_write(pcf1, 0, 0);
    seg7_write(pcf2, 0, 0);
    seg7_write(pcf3, val, 1);
    seg7_write(pcf4, _fraction, 0);
  }
  else if (val>=10) {
    seg7_write(pcf1,0,0);
    seg7_write(pcf2, (val / 10), 0);
    seg7_write(pcf3, ((int)val % 10), 1);
    seg7_write(pcf4, (_fraction), 0);
  } else {
    seg7_write(pcf1, 'e', 0);
    seg7_write(pcf2, 'r', 0);
    seg7_write(pcf3, 'r', 0);
    seg7_write(pcf4, ' ', 0);
  }
}

/*Функция вывода температуры на семисегментный дисплей*/
//Принимает значение температуры float
void write_display_rpm(uint16_t val){
  if (val<10) {
    seg7_write(pcf1, 0, 0);
    seg7_write(pcf2, 0, 0);
    seg7_write(pcf3, 0, 0);
    seg7_write(pcf4, val, 0);
  }
  else if (val < 100) {
    seg7_write(pcf1, 0, 0);
    seg7_write(pcf2, 0, 0);
    seg7_write(pcf3, val / 10, 0);
    seg7_write(pcf4, val % 10, 0);
  } 
  else if (val <1000) {
    seg7_write(pcf1, 0, 0);
    seg7_write(pcf2, val / 100, 0);
    seg7_write(pcf3, (val % 100)/10, 0);
    seg7_write(pcf4, val % 10, 0);
  }
  else if (val <10000) {
    seg7_write(pcf1, val / 1000, 0);
    seg7_write(pcf2, (val % 1000)/100, 0);
    seg7_write(pcf3, (val % 100)/10, 0);
    seg7_write(pcf4, val%10, 0);
  }
}

/*Функция инкримента(декремента) энкодера*/
// Принимает значение и возвращает ++ или -- этого значения, k - значение прибавляемое/убавляемое
float encoder_read(float init_value, float k){
  float _count = 0;
  //Считываем состояние ножек энкодера
  A = bitRead (A_PORT, A_pin);
  B = bitRead (B_PORT, B_pin);

  // Если А изменил состояние первым то прибавляем, в противном случает убавляем
  if (A != pinLast)
  {
    if (B != A)
    {
      _count = _count + k; 
      enc_init=true;      //регистр врщения энкодера
    }
    else
    {
      _count = _count - k; 
      enc_init=true;      //регистр вращения энкодера
    }
  } else {enc_init=false;}

  pinLast = A; // Присваеваем последнее значение A
  return init_value + _count; // Возвращаем изменённое значение
}

/*Считывание датчика температуры определённое количество циклов*/
// Принимает колличество циклов, и номер датчика (1-основной, 0 - дополнительный) возвращает показние термометра
float temp_metr(uint16_t cycle, bool dht_numb){
  if (count_cycle_temp > cycle)
  {
    // если прошло больше циклов cчитываем значение с определённого датчика, если нет, инкремент счётчика
    return dht_numb ? dht.readTemperature() : dht2.readTemperature();
    count_cycle_temp = 0;
  }
  else
  {
    count_cycle_temp++;
  }
}
/*Функция отправки значения на ЦАП*/
void dac_write(uint8_t addr, uint16_t val) { //Принимает адрес и значение от 0 до 255
  Wire.beginTransmission(addr); //начало передачи по адресу 44
  Wire.write(byte(0x00)); //байт инструкции
  Wire.write(val);        //отправка значения
  Wire.endTransmission(); //конец передачи
}

/*Функция считывания переключателей*/  //принимает адрес pcf и колличество считаных бит, возвращает считаный байт
byte read_pcf(int addr) {
 byte _val;
  Wire.requestFrom(addr, 8);    //запрос на чтение адреc, кол-во байт. Колличество байт не работает(
   while(Wire.available())            //если еcть что читать
   {                                  //читаем
     _val = Wire.read();    
   }
 return _val;
}

/*Функция мигания дисплея во время выбора значения*/
// Отдаёт true, когда дисплей перестаёт мигать сur_val - текущее значение, dis_val - выбранное значение, init - вид измерения (true - температура, false - обороты)
bool change_val(float cur_val, float dis_val, bool init) {
  if (enc_init==true) {           //Если сработад энкодер
    change_enc_init=true;         //пишем регистр
    change_time_prev=millis();    //запоминаем время
  }

  if (change_enc_init==true) {                                                  //если сработал регистр
    if (millis()-change_time_prev>700) {                                        //если прошло время                                        
      init ? write_display_temp(cur_val) : write_display_rpm ((int)cur_val);    //выводим текущие показания
      if (change_val_int==true) {                                               //регистр, для вывода значения функции один раз
        change_val_int=false;                                                   //меняем регистр                                                   
        return true;                                                            //функция отдаёт true один раз
      }
    } else {                                                                    //если не прошло время
      display_blink(dis_val, init);                                             //мигаем дисплеем
      change_val_int=true;                                                      
    }
  } else {                                                                      //если регистр не сработал выводим текщие показания
    init ? write_display_temp(cur_val): write_display_rpm ((int)cur_val);
  }
  return false;                                                                 //всё вермя отдаём false
} 

/*

*/
