/*Дипы
0000 - рабочий режим
1000 - настройка первого холодного вентилятора
0100 - настройка второго холодного вентилятора
0010 - наатройка первого горячего вентилятора
0001 - настройка втрого горячего вентилятора
1111 - температура со второго датчика темпретуры
1100 - индикация ошибок
*/

#include <EEPROM.h>
#include <timer_radar.h>
#include <DHT.h>
#include <Wire.h>      //библиотека I2C
#include "frequency.h"

#define pcf1 0x38      // первый сегиент индикатора
#define pcf2 0x39      //
#define pcf3 0x3A      //
#define pcf4 0x3B      // последний сегмент индикатора
#define pcf5 0x3C      // ДИПы
#define pcf6 0x3D      // 

#define dac1 0x2C      // первый DAC
#define dac2 0x2D      //
#define dac3 0x2E      //
#define dac4 0x2F      // последний DAC

#define A_PORT PIND    //Порт ножки A энкодера
#define A_pin 6        //Пин ножки А энкодера
#define B_PORT PIND    //Порт ножки B энкодера
#define B_pin 7        //Пин ножки B энкодера

#define user_reg 60     //диапазон регулировки вентилятора пользователем
#define user_reg_min 70 //Минимальный уровень скорости вентилятора для пользователя

#define compressor1_port PORTD//Определение порта компрессора
#define compressor1_pin 3
#define compressor2_port PORTD
#define compressor2_pin 5

#define button1_port PINB  //Определение кнопки компрессора
#define button1_pin 0
#define button2_port PINB  //Определение кнопки компрессора
#define button2_pin 1

#define thermostat_port PORTC//Определение порта термостата
#define thermostat_pin 6

#define time_fan 1300

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
bool contr_f;       //реле контроля фаз
bool k1_off;        //неисправность 1го компрессора
bool k2_off;        //неисправность 2го компрессора
bool k1_overload;   //перегрузка 1го компрессора
bool k2_overload;   //перегрузка 2го компрессора  

/*Переменные рабочего режима*/
byte indicator = true;         //выбор режима отображения. 1-температура 2-обороты
byte user_rpm = 50;            //число оборотов вентиляторов в процентах для пользователя
int user_rpm_k1;               //обороты вентилятора 1 прибавляемые/убавляемые пользователям
int user_rpm_k2;               //обороты вентилятора 2 прибавляемые/убавляемые пользователям
float t_real;                  //температура реальная с датчика 1
float t_wish=25;               //требуемая температура
bool k1_button;                //кнопка первого компрессора 
bool k2_button;                //кнопка второго компрессора
bool k1_init;                  //регистр включённого первого крмпрессора
bool k2_init;                  //регистр включённого второго крмпрессора
bool k1_action;                //переменная работы первого компрессора
bool k2_action;                //переменная работы второго компрессора
bool temp_init;                //Хранит состояние термостата (нужно ли включить или нет)
unsigned long fan1_last_time;  //Переменная хранения времени после работы вентилятора перед запуском компрессора
unsigned long fan2_last_time;  //Переменная хранения времени после работы вентилятора перед запуском компрессора

/* Временные переменные *//////////////////////////////////////////////////////////////////////////////////////////  

/**///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup(){
  Serial.begin(115200); // скорость com-порта
  Wire.begin(); // подключение к шине wire
  /*Установка пинов энкодера*/
  pinMode(6, INPUT_PULLUP);  // установка пина на вход, подтяжка к +5В
  pinMode(7, INPUT_PULLUP);  // установка пина на вход, подтяжка к +5В
  /*Установка пинов кнопок компрессора*/
  pinMode(8, INPUT_PULLUP);  // установка пина на вход, подтяжка к +5В
  pinMode(9, INPUT_PULLUP);
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
  /*prev_millis_speed = millis(); // время на начало программы*/
  /************************************/

  /*Считывание состояния реле и кнопок*/
  read_relay();

  /*считывание значение температуры*/
  t_real=temp_metr(100, 1);                                      

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
      case 12:  //Режим индикации ошибок
        err();
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

 /* Wire.beginTransmission(pcf1); // начало передачи по адресу
  Wire.write(0b11111110); // вывод значения
  Wire.endTransmission(); // конец передачи
*/
  /*Замер скорости работы программы*/
  /* cur_millis_speed = millis(); // время на конец программы
   Serial.println(cur_millis_speed - prev_millis_speed); // вывод времени исполнения программы*/
  
  /************************************/
}

/*Индикация ошибок*/
void err() {
  int _err_code;

  _err_code = 0;                         //Изначально код ошибки 0
  
  !low_press_1  ? _err_code = 1:0;       //код 1: низкое давление в контуре 1
  !low_press_2  ? _err_code = 2:0;       //код 2: низкое давление в контуре 2
  !high_press_1 ? _err_code = 3:0;       //код 3: высокое давление в контуре 1
  !high_press_2 ? _err_code = 4:0;       //код 4: высокое давление в контуре 2
  !k1_off       ? _err_code = 5:0;       //код 5: выключени 1 компрессор
  !k2_off       ? _err_code = 6:0;       //код 6: выключен 2 компрессор
  !contr_f      ? _err_code = 7:0;       //код 7: сработало реле контроля фаз
  !k1_overload  ? _err_code = 8:0;       //код 8: перегрузка 1 компрессора
  !k2_overload  ? _err_code = 9:0;       //код 9: перегрузка 2 компрессора

  if (!k1_overload & !high_press_1) {    //код 10: перегрузка 1 компрессора и высокое давление в контуре
    _err_code = 10;
  }

  if (!k2_overload & !high_press_2) {    //код 11: перегрузка 2 компрессора и высокое давление в контуре
    _err_code = 11;
  }

  if (!high_press_1 & fan1) {            //код 12: высокое давление в 1м контуре и не работате 1й вентилятор
    _err_code = 12;
  }

  if (!high_press_2 & fan2) {            //код 13: высокое давление в 2м контуре и не работате 2й вентилятор
    _err_code = 13;
  }

  if (!high_press_1 & (t_real>35)) {     //код 14: высокое давление в 1 контуре и превышена температура
    _err_code = 14;
  }

  if (!high_press_2 & (t_real>35)) {     //код 15: высокое давление в 2 контуре и превышена температура
    _err_code = 15;
  }

  write_display_err(_err_code);          //Вывод ошибки на дисплей
}

/*Рабочий режим работы программы*/
void work() {

  bitRead(PINC, 3) ? indicator = 1 : indicator = 2;        //Считывание значения переключателя 

  dac_write(dac1, user_rpm_k1);                            //Включение вентиляторов
  dac_write(dac2, user_rpm_k2);
  dac_write(dac3, rpm_1_hot_dac);
  dac_write(dac4, rpm_2_hot_dac);  

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

  compressors_action();
}

/*Логика работы компрессоров*/
void compressors_action () {

  !k1_button ? k1_action=true : 0;                               //Если нажата кнопка включаем компрессор
  !k2_button ? k2_action=true : 0;

  if (k1_action) {                                               //Если работает компрессор
    if (!fan1) {                                                 //Если включён вентилятор
      if (millis()-fan1_last_time>time_fan) {                    //Если прошло время 
        k1_action=true;                                          //включаем компрессор
      } else {                                                   //иначе
        k1_init ? 0:k1_action=false;                             //выключаем компрессор, если он не работал
      }
    } 
  }

  if (k2_action) {                                               
    if (!fan2) {                                                 
      if (millis()-fan2_last_time>time_fan) {                     
        k2_action=true;                                          
      } else {                                                  
        k2_init ? 0:k2_action=false;                             
      }
    } 
  }

  !temp_init ? k1_action = false:0;                              //Если не сработал термостат выключаем компрессор
  !temp_init ? k2_action = false:0;

  k1_button ? k1_action=false:0;                                 //Если кнопка не нажата выключаем компрессор
  k2_button ? k2_action=false:0;

  (fan1 || k1_button || !temp_init) ? fan1_last_time=millis():0; //Если не включён вентилятор или кнопка не нажата или не сработал термостат запоминаем секунды
  (fan2 || k2_button || !temp_init) ? fan2_last_time=millis():0;

  k1_action ? k1_init=true: k1_init=false;                       //Если компрессор работает, запоминаем переменную
  k2_action ? k2_init=true: k2_init=false;

  k1_action ? compressor1_port |=(1<<compressor1_pin) : compressor1_port &= ~(1 << compressor1_pin); //Включение компрессоров в зависимости от переменной
  k2_action ? compressor2_port |=(1<<compressor2_pin) : compressor2_port &= ~(1 << compressor2_pin); 
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
  fan1        =  ( 0b1 & read_pcf(pcf6)        );                //считывание реле первого вентилятора 
  fan2        =  ( 0b10 & read_pcf(pcf6)       );                //считывание реле второго вентилятора
  contr_f     =  ( 0b100 & read_pcf(pcf6)      );                //считывание реле конроля фаз
  k1_off      =  ( 0b1000 & read_pcf(pcf6)     );                //неисправность компрессора 1
  k2_off      =  ( 0b10000 & read_pcf(pcf6)    );                //неисправность компрессора 2
  k1_overload =  ( 0b100000 & read_pcf(pcf6)   );                //перегрузка 1го компрессора
  k2_overload =  ( 0b1000000 & read_pcf(pcf6)  );                //перегрузка 2го компрессора

  low_press_1 =  ( 0b10000 & read_pcf(pcf5)    );                 //считывание реле низкого давления 1
  high_press_1 = ( 0b100000 & read_pcf(pcf5)   );                 //считывание реле высокого давления 1
  low_press_2 =  ( 0b1000000 & read_pcf(pcf5)  );                 //считывание реле низкого давления 2
  high_press_2 = ( 0b10000000 & read_pcf(pcf5) );                 //считывание реле высокого давления 2

  k1_button=      bitRead(button1_port, button1_pin);             //считывание кнопки компрессора 1
  k2_button=      bitRead(button2_port, button2_pin);             //считывание кнопки компрессора 2
}

/*Функция вывода на 7seg дисплей */
// Формат seg7_write(адрес, значение, точка)
void seg7_write(byte addr, unsigned char val, byte dot){
  unsigned char _val_bit;
  switch (val)
  {
    // перевод символа в двоичный
    case 0:
      _val_bit = B11000000;
      break;
    case 1:
      _val_bit = B11111001;
      break;
    case 2:
      _val_bit = B10100100;
      break;
    case 3:
      _val_bit = B10110000;
      break;
    case 4:
      _val_bit = B10011001;
      break;
    case 5:
      _val_bit = B10010010;
      break;
    case 6:
      _val_bit = B10000010;
      break;
    case 7:
      _val_bit = B11111000;
      break;
    case 8:
      _val_bit = B10000000;
      break;
    case 9:
      _val_bit = B10010000;
      break;
    case 'a':
      _val_bit = B10001000;
      break;
    case 'b':
      _val_bit = B10000011;
      break;
    case 'c':
      _val_bit = B11000110;
      break;
    case 'd':
      _val_bit = B10100001;
      break;
    case 'e':
      _val_bit = B10000110;
      break;
    case 'f':
      _val_bit = B10001110;
      break;
    case 'g':
      _val_bit = B11000010;
      break;
    case 'h':
      _val_bit = B01110110;
      break;
    case 'i':
      _val_bit = B11111001;
      break;
    case 'j':
      _val_bit = B11110001;
      break;
    case 'l':
      _val_bit = B11000111;
      break;
    case 'n':
      _val_bit = B10101011;
      break;
    case 'p':
      _val_bit = B10001100;
      break;
    case 'q':
      _val_bit = B10011000;
      break;
    case 'r':
      _val_bit = B10101111;
      break;
    case 's':
      _val_bit = B10010010;
      break;
    case 't':
      _val_bit = B10000111;
      break;
    case 'u':
      _val_bit = B11000001;
      break;
    case 'y':
      _val_bit = B10010001;
      break;
    case ' ':
      _val_bit = B11111111;
      break;
    case '-':
      _val_bit = B10111111;
      break;
    case '*':
      _val_bit = B10011100;
      break;
  }
  Wire.beginTransmission(addr); // начало передачи по адресу
  Wire.write(_val_bit); // вывод значения
  dot? Wire.write(_val_bit &=~(1<<7)):Wire.write(_val_bit |=(1<<7)); // вывод точки
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

/*Функция вывода оборотов на семисегментный дисплей*/
//Принимает значение оборотов
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

/*Функция вывода номера ошибки на семисегментный дисплей*/
//Принимает значение ошиибки
void write_display_err(uint16_t val) {
  if (val<10) {
    seg7_write(pcf1, 'e', 0);
    seg7_write(pcf2, 0, 0);
    seg7_write(pcf3, 0, 0);
    seg7_write(pcf4, val, 0);
  }
  else if (val < 100) {
    seg7_write(pcf1, 'e', 0);
    seg7_write(pcf2, 0, 0);
    seg7_write(pcf3, val / 10, 0);
    seg7_write(pcf4, val % 10, 0);
  } 
  else if (val <1000) {
    seg7_write(pcf1, 'e', 0);
    seg7_write(pcf2, val / 100, 0);
    seg7_write(pcf3, (val % 100)/10, 0);
    seg7_write(pcf4, val % 10, 0);
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
