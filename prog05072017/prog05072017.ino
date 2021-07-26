/*Значение переключателей
0000 - рабочий режим
1000 - настройка первого холодного вентилятора
0100 - настройка второго холодного вентилятора
0010 - наcтройка первого горячего вентилятора
1100 - настройка второго горячего вентилятора
1110 - температура со второго датчика температуры
1010 - индикация ошибок
0001 - настройка таблицы оборотов от температуры
1001 - версия программного обеспечения
*/

#include <EEPROM.h>
#include <timer_radar.h>
#include <DHT.h>
#include <Wire.h>                 //библиотека I2C
#include "frequency.h"

#define version_numb 1.00         //Версия программного обеспечения

#define pcf1 0x38                 // первый сегиент индикатора
#define pcf2 0x39                 //
#define pcf3 0x3A                 //
#define pcf4 0x3B                 // последний сегмент индикатора
#define pcf5 0x3C                 // ДИПы
#define pcf6 0x3D                 // 

#define dac1 0x2C                 // первый DAC
#define dac2 0x2E                 //
#define dac3 0x2D                 //
#define dac4 0x2F                 // последний DAC

#define dac_mode 'I2C'            //режим вывода на цифро-аналог преобразователь (I2C или SPI)

#define latchPort PORTB           //Порт clock для SPI
#define latchPin 2

#define dataPort PORTB            //Порт clock для SPI
#define dataPin 3

#define A_PORT PIND               //Порт ножки A энкодера
#define A_pin 6                   //Пин ножки А энкодера
#define B_PORT PIND               //Порт ножки B энкодера
#define B_pin 7                   //Пин ножки B энкодера

#define user_reg 130              //Диапазон регулировки вентилятора пользователем
#define user_reg_min 30           //Минимальный уровень скорости вентилятора для пользователя
#define volts 0.0376              //Вольт на одно деление dac

#define compressor1_port PORTD    //Определение порта компрессора
#define compressor1_pin 3
#define compressor2_port PORTD
#define compressor2_pin 5

#define button1_port PINB         //Определение кнопки компрессора
#define button1_pin 0
#define button2_port PINB         //Определение кнопки компрессора
#define button2_pin 1

#define thermostat_port PORTC     //Определение порта термостата
#define thermostat_pin 6

#define clockPort PORTB           //Порт clock для SPI
#define clockPin 5

#define time_fan1 20000           //Время после которого при работе вентилятора включится компрессор 1 (ms)
#define time_fan2 22000           //Время после которого при работе вентилятора включится компрессор 1 (ms)
               

/*Датчик температуры*/
DHT dht(A1, DHT22);               //ДТ1  //основной
DHT dht2(A2, DHT22);              //ДТ2  //дополнительный

/*Замер скорости работы программы*/
uint16_t prev_millis_speed = 0;   // предыдущее время
uint16_t cur_millis_speed = 0;    // текущее время
uint16_t time_millis = 0;         // время прошедшее после включения

uint16_t cur_time;
uint16_t last_time;

/*Работа энкодера*/
unsigned char A;                       // Первоначальное сосояние A
unsigned char B;                       // Первоначальное состояние B
byte pinLast = 1;                      // Служебная переменная
byte pinLastB =1;
bool enc_init=false;                   // Регистр вращения энкодера

/*Считывание датчик температуры */
uint16_t count_cycle_temp_in = 0;
uint16_t count_cycle_temp_out = 0;

/*Считывание ДИПов*/
byte count_cycle_read_relay = 0;

/*Таймер мигания дисплея*/
timer_radar blink_disp(2);

/*Функция change_val*/
bool change_enc_init;                  // регистр функции change_val
unsigned long int change_time_prev;    // для запоминания времени
bool change_val_int;                   // регистр функции change_val

int blink_on_count=0; //счётчик мигания дисплея

/*Переменные прерывания с pcf*/
bool reg_pcf=true;     //регистр прерывания от pcf
uint16_t read_dip_int; //значение переключателей dip

/*Переменные оборотов вентилятора*/
uint16_t rpm_1_cold;                   //первый холодный вентилятор, текущее значение оборотов
uint16_t rpm_1_cold_dac;               //Значение выдаваемое на dac первого холодного венттидятора
float    rpm_1_cold_volt;
uint16_t rpm_2_cold;                   //второй холодный вентилятор, текущее значение оборотов
uint16_t rpm_2_cold_dac;               //Значение выдаваемое на dac второго холодного венттидятора
float    rpm_2_cold_volt;
uint16_t rpm_1_hot;                    //Первый горячий вентилятор
uint16_t rpm_1_hot_dac;                //Значение выдаваемое на dac первого горячего венттидятора
float    rpm_1_hot_volt;
uint16_t rpm_2_hot;                    //Второй горячий вентилятор
uint16_t rpm_2_hot_dac;                //Значение выдаваемое на dac второго горячего венттидятора
int min_rpm = 28;                      //Минимальная скорость вращения вентиляторов, примерно соответствующее 1В

float    rpm_2_hot_volt;

/*Замер скорости работы вентиляторов*/
int *rpm_fan[4];      
int mass[4] = {0,}; 
int i;
int time_mettering=2; //продолжительность измерения оборотов

/*Переменные считаные с реле*/
bool fan1;                     //состояние 1 горячего вентилятора
bool fan2;                     //состояние 2 горячего вентилятора
bool low_press_1;              //реле низкого давоения 1
bool high_press_1;             //реле высокого давления 1
bool low_press_2;              //реле низкого давления 2
bool high_press_2;             //реле высокого давления 2
bool contr_f;                  //реле контроля фаз
bool k1_off;                   //неисправность 1го компрессора
bool k2_off;                   //неисправность 2го компрессора
bool k1_overload;              //перегрузка 1го компрессора
bool k2_overload;              //перегрузка 2го компрессора  
bool open_alert;               //открытая крышка

/*Переменные рабочего режима*/
byte indicator = true;         //выбор режима отображения. 1-температура 2-обороты
byte user_rpm = 50;            //число оборотов вентиляторов в процентах для пользователя
int user_rpm_k1;               //обороты вентилятора 1 прибавляемые/убавляемые пользователям
int user_rpm_k2;               //обороты вентилятора 2 прибавляемые/убавляемые пользователям
float t_real;                  //температура реальная с датчика 1
int t_out;                     //температура реальная с датчика 2
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
int err_byte=0;                //колличество ошибок
int err_array[9];              //колличество возможных ошибок
int err_time=0;                //время отображения ошибки
int b1=0, b2=0, b3=0;          //биты для режима SPI. хранят значение выдаваемое на dac
int t_min=14;                  //минимальная температура включения компрессора
int t_adjustment=35;           //температура с которой будут применяться обороты вентилятора  
//int t_max=55;                  //температура выше которой не будет применяться максимальный коэфициент

int temp_array[ ] = {35, 40, 45, 50, 55, 60}; //Массив со "ступенями" температур, на которых будут изменяться обороты
int rpm_array[ ] = { };                       //Массив со "ступенями" оборотов, соответствующий массиву температур
int temp_num=0;                               //Текущий индекс массива temp_array и rpm_array

/* Временные переменные *//////////////////////////////////////////////////////////////////////////////////////////

int val_dac=255;

/**////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup(){

  //Для первоначального установления оборотов вентилятора, раскоментировать при первой прошивке
  //                 35   40   45   50   55   60        //температура
 /* int _k_rpm[  ] = {210, 220, 230, 240, 250, 255};    //первоначальная таблица поправочных коэфициэнтов увелиения работы вентилятора
  for (int _i=(sizeof(temp_array)/2)-1; _i>=0; _i--) {  //перебираем коэфициенты                                                           
    EEPROM.write(_i+4, _k_rpm[_i]);                     //записываем в память
  }
  EEPROM.write(1, 150);                                 //записываем в память rpm 1 холодного вентилятора
  EEPROM.write(2, 150);                                 //записываем в память rpm 2 холодного вентилятора*/

  Serial.begin(115200);              // скорость com-порта
  Wire.begin();                      // подключение к шине wire

  /*Установка пинов энкодера*/
  pinMode(6, INPUT_PULLUP);          // установка пина на вход, подтяжка к +5В
  pinMode(7, INPUT_PULLUP);          // установка пина на вход, подтяжка к +5В
  /*Установка пинов кнопок компрессора*/
  //pinMode(8, INPUT_PULLUP);        // установка пина на вход, подтяжка к +5В
  //pinMode(9, INPUT_PULLUP);
  /*Установка пина кнопки энкодера*/
  pinMode(A3, INPUT_PULLUP);         // установка пина на вход, подтяжка к +5В
  /*Установка портов компрессора как выходы*/
  pinMode(3, OUTPUT);                //компрессор 1
  pinMode(5, OUTPUT);                //компрессор 2 

  attachInterrupt(0, pcf_int, CHANGE); // привязываем 0-е прерывание к функции
  
  //SPI РЕЖИМ, раскомментировать. I2C закоментировать
 /* pinMode(10, OUTPUT);               //latch
    pinMode(11, OUTPUT);               //data
    pinMode(13, OUTPUT);               //clock*/

  //pinMode(A2, OUTPUT);               //раскоментировать, если вместо ДТ2 нужен выход термостата.

  //t_real = 0;                        //определяем первоначальные значения температуры
  //t_out = 0;

  /*Считывание первоначальных значение EEPROM*/
  rpm_1_cold_dac = EEPROM.read(1);                 //значение rpm первого холодного вентилятора
  rpm_2_cold_dac = EEPROM.read(2);                 //значение rpm второго холодного вентилятора
  rpm_1_hot_dac  = EEPROM.read(3);                 //значение rpm первого горячего вентилятора
  rpm_2_hot_dac  = EEPROM.read(4);                 //значение rpm первого горячего вентилятора
  t_wish         = (EEPROM.read(100)+100)/10;      //Считывание последней температуры, прибаляем 100 и делим на 10 т.к. значение хранится только до 255

  /*Считывание первоначальных показаний EEPROM для таблицы значений оборотов*/
  for (int _i=0; _i<=(((sizeof(temp_array))/2)-1); _i++) {  
    rpm_array[_i]=EEPROM.read(_i+4);
  }

  /*Настройка измерителя частоты*/
  setup_TC1(10);                                   //Период замера в секундах
  for (int i=0; i<4; i++) {rpm_fan[i]=&mass[i];}

 /*Временные установки*//////////////////////////////////////////////////////////////////////////////////////////
 
}

void loop(){

  /*Замер скорости работы программы*/
  /*prev_millis_speed = millis(); // время на начало программы*/
  /************************************/

  /*Считывание состояния реле и кнопок*/
  if (reg_pcf==true) {                 //если сработал регистр прерывания
     read_relay();                     //считываем состояние реле
     read_dip_int = read_dip();        //считываем дип переключатели
     !open_alert ? read_dip_int=16:0;  //если открыта крышка
     reg_pcf=0;                        //обнуляем регистр

     read_pcf(pcf5);                   //считывание с PCF для обнуления прерывания
     read_pcf(pcf6); 
  }
 
  /*считывание значение температуры*/
  t_real=temp_metr_in(350); 
  t_out=temp_metr_out(450);  
  //t_out=35;

  /*считывание кнопок копрессора*/
  read_k_button();

  /*Выбор режима работы*/
  switch (read_dip_int) {
         case 16:                                   //открытая крышка
            open_cap();                             
           break; 
         case 15 :                                  //Рабочий режим
             work();
           break;
         case 14 :                                  //настройка 1го холодного вентилятора
             fan_setting1();
           break;
         case 13:                                   //настройка 2го холодного вентилятора
             fan_setting2();
           break;
         case 10:                                   //Режим индикации ошибок
             err();
           break;
         case 11:                                   //настройка 1го горячего вентилятора
             fan_setting3();
           break;
         case 12:                                   //настройка 2го горячего вентилятора
             fan_setting4();
           break;
         case 8:                                    //вывод температуры со второго датчика темепературы
             write_display_temp(temp_metr_out(50));
           break;
         case 7:                                    //настройка таблицы оборотов вентилятора
             rpm_hot_adjustment_manual();
           break;
         case 6:
            version();                              //Отображение версии встроенного ПО
           break;
         default :                                  //если комбинация дипов не встретилось выводим 'no'
             seg7_write(pcf4, 0, 0);
             seg7_write(pcf3, 'n', 0);
             seg7_write(pcf2, ' ', 0);
             seg7_write(pcf1, ' ', 0);     
     }

  /*Замер скорости работы программы*/
  /* cur_millis_speed = millis(); // время на конец программы
  //Serial.println(cur_millis_speed - prev_millis_speed); // вывод времени исполнения программы*/

}

/*Индикация ошибок*/
void err() {

  int _err_code;                            //Создаём переменную ошибок
  _err_code = 0;                            //Изначально код ошибки 0

  err_byte=0b000000000;                     //Обнуляем ошибки                     

  !low_press_1  ? err_byte|=(1<<0):0;       //код 1: низкое давление в контуре 1
  !low_press_2  ? err_byte|=(1<<1):0;       //код 2: низкое давление в контуре 2
  !high_press_1 ? err_byte|=(1<<2):0;       //код 3: высокое давление в контуре 1
  !high_press_2 ? err_byte|=(1<<3):0;       //код 4: высокое давление в контуре 2
  !k1_off       ? err_byte|=(1<<4):0;       //код 5: выключен 1 компрессор
  !k2_off       ? err_byte|=(1<<5):0;       //код 6: выключен 2 компрессор
  !contr_f      ? err_byte|=(1<<6):0;       //код 7: сработало реле контроля фаз
  !k1_overload  ? err_byte|=(1<<7):0;       //код 8: перегрузка 1 компрессора
  !k2_overload  ? err_byte|=(1<<8):0;       //код 9: перегрузка 2 компрессора

  if (err_byte==0) {write_display_err(0);}                //изначально код ошибки 0

  for ( int _err_count=0; _err_count<=8; _err_count++) {  //Перебираем ошибки
    if (((err_byte >> _err_count) & 1)==1) {              //Если бит=1, значит ошибка  
      err_array[_err_count] = true;                       //Записываем бит ошибки
      while (err_time<1000) {                             //Пока не прошло время
        write_display_err(_err_count+1);                  //Выводим код ошибки
        err_time++;                                       //Прибавляем время
      }
      err_time=0;                                         //Обнуляем время
    } else {err_array[_err_count]=false;}                 //Если не ошибка пишем 0
  }
}

//Функция для открытой крышки
void open_cap() {

 //Отображение и регулировка температуры(используется для 1го кондицонера, т.к. не правильно смонтирован датчик открытия крышки)
  t_wish=encoder_read(t_wish,0.5);                           //Изменение темепературы крутилкой
  if (change_val(t_real,t_wish,1)) {                         //Индикация температуры на дисплее
      EEPROM.write(100,(t_wish*10-100));                     //Записываем в память значение *10-100 для того, что бы значение не превышало 255
  }                              
  if (t_wish>35) {                                           //Защита от выхода за пределы
      t_wish=35;
  } else if (t_wish<18) {
      t_wish=18;
  }

  /*Выводит "0PEn" на индикатор*/  //Закоментировано, т.к. в первом кондиционере это не работает
/*  seg7_write(pcf4, 'n', 0);
  seg7_write(pcf3, 'e', 0);
  seg7_write(pcf2, 'p', 0);
  seg7_write(pcf1, 0, 0);*/

  /*Отключаем вентиляторы */
  dac_write(dac1, 0);
  dac_write(dac2, 0);
  dac_write(dac3, 0);
  dac_write(dac4, 0);

  /*Отключаем компрессоры*/
  k1_action = 0;
  k2_action = 0;
}

/*Настройка таблицы оборотов с панели управления*/
void rpm_hot_adjustment_manual() {
  
  bitRead(PINC, 3) ? indicator = 1 : indicator = 2;                        //Считываем положение кнопки (температура или обороты)

  if (temp_num<=0) {                                                       //Защита от выхода за пределы индексов текущего элемента массива
    temp_num=0;
  } else if (temp_num>=(sizeof(temp_array)/2)-1) {                         
    temp_num=(sizeof(temp_array)/2)-1;
  }

  if (indicator==2) {                                                      //Если энкодер нажат
    temp_num=encoder_read(temp_num, 1);                                    //Выбираем температуру
    write_display_temp(temp_array[temp_num]);                              //Выводим её на дисплей

  } else if (indicator==1) {                                               //Если энкодер не нажат
    rpm_array[temp_num]=encoder_read(rpm_array[temp_num], 2);              //Изменяем значение оборотов ввыводиое на DAC
    if (  change_val(rpm_1_hot, (rpm_array[temp_num]*volts), 2)   ) {      //Отображаем обороты, мигаем при изменении, и если изменились 
      EEPROM.write(temp_num+4, rpm_array[temp_num]);                       //Записываем в память по текущему адресу
    }
    dac_write(dac1, rpm_array[temp_num]);                                  //Выдаём значение на dac
    dac_write(dac2, rpm_array[temp_num]);

    if (rpm_array[temp_num]>=255) {                                        //Защита от выхода за пределы значения выдаваемого на DAC
    rpm_array[temp_num]=255;
    } else if (rpm_array[temp_num] <= min_rpm) {
    rpm_array[temp_num]=min_rpm;                                           //28 значение соответствующее (примерно) 1В
    }
  }

  rpm_read();
}

/*Работа горячих вентиляторов*/
void rpm_hot_adjustment() {
 int _size = (sizeof(temp_array)/2)-1;                             // Колличество элементов в масиве температур

  if (t_out<=temp_array[0]) {                                      // Если наружняя температура меньше или равна первой в массиве
    dac_write(dac1, rpm_array[0]);                                 // На DAC минимальное значение  из таблицы
    dac_write(dac2, rpm_array[0]);                                 // 
  } else if ( t_out >= temp_array[_size] ) {                       // Если температура больше или равна максимальной из таблицы 
    dac_write(dac1, rpm_array[_size]);                             // На DAC максимальное значение из таблицы
    dac_write(dac2, rpm_array[_size]);                             //
  } else {                                                         // Иаче
      for (int _i=_size; _i>=0; _i--) {                            // Перебираем значения из таблицы
        if (  t_out>=temp_array[_i] & t_out<temp_array[_i+1]  ) {  // Если текущая температура в пределах регулирования
           dac_write(dac1, rpm_array[_i]);                         // Выдаём на dac значение из таблицы
           dac_write(dac2, rpm_array[_i]);                         // 
        }
      }
  }
}

/*Рабочий режим работы программы*/
void work() {

  bitRead(PINC, 3) ? indicator = 1 : indicator = 2;              //Считывание значения переключателя

  dac_write(dac3, user_rpm_k1);                                  //Включение холодных вентиляторов
  dac_write(dac4, user_rpm_k2);

  rpm_hot_adjustment();                                          //Работа горячих вентиляторов


  user_rpm_k1 = rpm_1_cold_dac + ((user_rpm-50)*(user_reg/50));  //Изменение оборотов пользователем на 1м холод вентиляторе
  user_rpm_k2 = rpm_2_cold_dac + ((user_rpm-50)*(user_reg/50));  //Изменение оборотов пользователем на 2м холод вентиляторе

  if (user_rpm_k1>255) {                                         //Защита от выхода за перделы значений вентилятора 1 холодного
    user_rpm_k1 = 255;  }                  
  else if (user_rpm_k1<user_reg_min) {
    user_rpm_k1 = user_reg_min;
  }

  if (user_rpm_k2>255) {                                         //Защита от выхода за перделы значений вентилятора 2 холодного
    user_rpm_k2 = 255;  }
  else if (user_rpm_k2<user_reg_min) {
    user_rpm_k2 = user_reg_min;
  }
  

  switch (indicator) {                                           //Выбор режима: температура или обороты
      case 1: {
        user_rpm = encoder_read(user_rpm, 1);                    //Изменение значения оборотов
        change_val(user_rpm,user_rpm, 0);                        //Индикация оборотов на дисплее

        if (user_rpm>100) {                                      //Защита от выхода за пределы регулировки оборотов
          user_rpm=100;
        } else if (user_rpm<1) {
          user_rpm=1;
        }

        if (user_rpm_k1<user_reg_min) {                          //Защита от выхода за перделы значений вентилятора 1 холодного
          user_rpm_k1=user_reg_min;
        } else if (user_rpm_k1>=255) {
          user_rpm_k1=255;
        }

         if (user_rpm_k2<user_reg_min) {                         //Защита от выхода за перделы значений вентилятора 2 холодного
          user_rpm_k2=user_reg_min;
        } else if (user_rpm_k2>=255) {
          user_rpm_k2=255;
        }      
        }
         break;  


      case 2: {                                                   //Режим температуры
       t_wish=encoder_read(t_wish,0.5);                           //Изменение темепературы крутилкой
       if (change_val(t_real,t_wish,1)) {                         //Индикация температуры на дисплее
          EEPROM.write(100,(t_wish*10-100));                      //Записываем в память значение *10-100 для того, что бы значение не превышало 255
       }                              
       if (t_wish>35) {                                           //Защита от выхода за пределы
         t_wish=35;
       } else if (t_wish<18) {
         t_wish=18;
       }
       break;
      }

  }

  if (t_real>(t_wish+0.5)) {                                      //Если температура выше желаемой, пишем регистр термостата
    temp_init = true;
  }

  if (t_real<(t_wish-0.5)) {                                      //Если температура ниже желаемый выкл термостат
    temp_init = false;
  }         

  temp_init ? PORTC |= (1 << 2) : PORTC &= ~ (1 << 2);            //Выдача значения на ногу термостата

  compressors_action();
}

/*Логика работы компрессоров*/
void compressors_action() {

  if (!k1_button) {                          //Если нажата кнопка
    if (millis()-fan1_last_time>time_fan1) { //И прошло время
      k1_action = true;                      //Включаем компрессор
    } else {                                 //Если время не прошло - выключаем компрессор
      k1_action = false;
    }
  } else {                                   //Если кнопка не нажата
      k1_action = false;                     //выключаем компрессор
      fan1_last_time = millis();             //Запоминаем последнее время
  }


  if (!k2_button) {                          //Если нажата кнопка
    if (millis()-fan2_last_time>time_fan2) { //И прошло время
      k2_action = true;                      //Включаем компрессор
    } else {                                 //Если время не прошло - выключаем компрессор
      k2_action = false;
    }
  } else {                                   //Если кнопка не нажата
      k2_action = false;                     //выключаем компрессор
      fan2_last_time = millis();             //Запоминаем последнее время
  }


  if (!temp_init) {                                            //Если не сработал термостат 
      k1_action = false; k2_action;                            //Выключаем компрессоры
      fan1_last_time = millis(); fan2_last_time = millis();}   //Запоминаем последнее время
 
  !low_press_1  ? k1_action=false:0;          //низкое давление в контуре 1 выключаем компрессор
  !low_press_2  ? k2_action=false:0;          //низкое давление в контуре 2 выключаем компрессор
  !high_press_1 ? k1_action=false:0;          //высокое давление в контуре 1 выключаем компрессор
  !high_press_2 ? k2_action=false:0;          //высокое давление в контуре 2 выключаем компрессор
  !k1_off       ? k1_action=false:0;          //неисправность 1го компрессора выключаем компрессор
  !k2_off       ? k2_action=false:0;          //неисправность 2го компрессора выключаем компрессор
  !contr_f      ? k1_action=false:0;          //сработало реле контроля фаз выключаем 1й компрессор
  !contr_f      ? k2_action=false:0;          //сработало реле контроля фаз выключаем 2й компрессор
  !k1_overload  ? k1_action=false:0;          //перегрузка 1 компрессора выключаем 1 компрессор
  !k2_overload  ? k2_action=false:0;          //перегрузка 2 компрессора выключем 2 коспрессор


  if (t_out < t_min) {                       //Если температура меньше минимальной не включаем кондеционер
    k1_action = false;
    k2_action = false;
  }

  k1_action ? compressor1_port |=(1<<compressor1_pin) : compressor1_port &= ~(1 << compressor1_pin); //Включение компрессоров в зависимости от переменной
  k2_action ? compressor2_port |=(1<<compressor2_pin) : compressor2_port &= ~(1 << compressor2_pin); 


}

/*Функция настройки вентилятора первого холодного*/
void fan_setting1 () {
  rpm_1_cold_dac=encoder_read(rpm_1_cold_dac, 2);             //Если крутим энкодер меняем значение dac
  if (rpm_1_cold_dac>255) {                                   //Защита от выхода за пределы значений dac
    rpm_1_cold_dac=255;
  } else if (rpm_1_cold_dac<=min_rpm) {
    rpm_1_cold_dac=min_rpm;
  }
  if (change_val(rpm_1_cold, rpm_1_cold_dac*volts, 2)) {      //Если функция изменения (мигание дисплеем) значения закончила работу (отдала true) 
    EEPROM.write(1, rpm_1_cold_dac);                          //Пишем значение в EEPROM                
  }

  dac_write(dac3, rpm_1_cold_dac);                            //Вывод текущего значения на dac
  rpm_read();                                                 //считывание оборотов вентиляторов

} 

/*Функция настройки вентилятора второго холодного*/
void fan_setting2 () {
  rpm_2_cold_dac=encoder_read(rpm_2_cold_dac, 2);             //Если крутим энкодер меняем значение dac
  if (rpm_2_cold_dac>=255) {                                  //Защита от выхода за пределы значений dac
    rpm_2_cold_dac=255;
  } else if (rpm_2_cold_dac<=min_rpm) {
    rpm_2_cold_dac=min_rpm;
  }
  if (change_val(rpm_2_cold, rpm_2_cold_dac*volts, 2)) {      //Если функция изменения (мигание дисплеем) значения закончила работу (отдала true) 
    EEPROM.write(2, rpm_2_cold_dac);                          //Пишем значение в EEPROM
  }
  dac_write(dac4, rpm_2_cold_dac);                            //Вывод текущего значения на dac
  rpm_read();                                                 //считывание оборотов вентиляторов

}

/*Функция настройки вентилятора первого горячего*/
void fan_setting3 () {

  rpm_1_hot_dac=encoder_read(rpm_1_hot_dac, 2);               //Если крутим энкодер меняем значение dac
  if (rpm_1_hot_dac>=255) {                                   //Защита от выхода за пределы значений dac
    rpm_1_hot_dac=255;
  } else if (rpm_1_hot_dac<=min_rpm) {
    rpm_1_hot_dac=min_rpm;
  }
  if (change_val(rpm_1_hot, rpm_1_hot_dac*volts, 2)) {        //Если функция изменения (мигание дисплеем) значения закончила работу (отдала true)   
    EEPROM.write(3, rpm_1_hot_dac);                           //Пишем значение в EEPROM
  }
  dac_write(dac1, rpm_1_hot_dac);                             //Вывод текущего значения на dac
  rpm_read();                                                 //считывание оборотов вентиляторов
}

/*Функция настройки вентилятора первого горячего*/
void fan_setting4 () {
  rpm_2_hot_dac=encoder_read(rpm_2_hot_dac, 2);               //Если крутим энкодер меняем значение dac

  if (rpm_2_hot_dac>=255) {                                   //Защита от выхода за пределы значений dac
    rpm_2_hot_dac=255;
  } else if (rpm_2_hot_dac<=min_rpm) {
    rpm_2_hot_dac=min_rpm;
  }
  if (change_val(rpm_2_hot, rpm_2_hot_dac*volts, 2)) {        //Если функция изменения (мигание дисплеем) значения закончила работу (отдала true) 
    EEPROM.write(4, rpm_2_hot_dac);                           //Пишем значение в EEPROM
  }
  dac_write(dac2, rpm_2_hot_dac);                             //Вывод текущего значения на dac
  rpm_read();                                                 //считывание оборотов вентиляторов
}

/*Функция считывания значений dip-переключателей, возвращает значение dip-переключателей*/
byte read_dip () {
  return (0b1111 & read_pcf(pcf5));                                //считываем первые три бита pcf
  //
}  

/*Функция считывания значений реле*/
void read_relay() {

    fan1         = ( 0b1 & read_pcf(pcf6)         );                //считывание реле первого вентилятора 
    fan2         = ( 0b10 & read_pcf(pcf6)        );                //считывание реле второго вентилятора
    contr_f      = ( 0b100 & read_pcf(pcf6)       );                //считывание реле конроля фаз
    k1_off       = ( 0b1000 & read_pcf(pcf6)      );                //неисправность компрессора 1
    k2_off       = ( 0b10000 & read_pcf(pcf6)     );                //неисправность компрессора 2
    k1_overload  = ( 0b100000 & read_pcf(pcf6)    );                //перегрузка 1го компрессора
    k2_overload  = ( 0b1000000 & read_pcf(pcf6)   );                //перегрузка 2го компрессора
    open_alert   = ( 0b10000000 & read_pcf(pcf6)  );                //открытая крышка 

    low_press_1  = !( 0b10000 & read_pcf(pcf5)    );                //считывание реле низкого давления 1
    high_press_1 = !( 0b100000 & read_pcf(pcf5)   );                //считывание реле высокого давления 1
    low_press_2  = !( 0b1000000 & read_pcf(pcf5)  );                //считывание реле низкого давления 2
    high_press_2 = !( 0b10000000 & read_pcf(pcf5) );                //считывание реле высокого давления 2
}

/*Функция считывания кнопок компрессора*/
void read_k_button() {
    k1_button    = !bitRead(button1_port, button1_pin);             //считывание кнопки компрессора 1
    k2_button    = !bitRead(button2_port, button2_pin);             //считывание кнопки компрессора 2
}

//Функция вывода на 7seg дисплей Формат seg7_write(адрес, значение, точка)
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

  Wire.beginTransmission(addr);                                       // Начало передачи по адресу
  Wire.write(_val_bit);                                               // Вывод символа
  dot? Wire.write(_val_bit &=~(1<<7)):Wire.write(_val_bit |=(1<<7));  // Вывод точки
  Wire.endTransmission();                                             // Конец передачи
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

//Мигание дисплея. Принимает значение выводимое на дисплей и вид измерения (0 - обороты, 1 - температура)
void display_blink(float val, bool init) {
  
  uint8_t _value_on=180;                 //Период включения дисплея
  uint8_t _value_off=220;                //Период выклюяения дисплея

  if (blink_on_count <= _value_on) {     //Если счётчик меньше значения включения
    if (init==0) {                       //В соответсвии с типом отображения
      write_display_rpm ((int)val);      //Выводим значение
    } else if (init==1) {
      write_display_temp(val);
    } else if (init==2) {
      write_display_rpm ((int)val);
    }
  } 
  else {                                 //Если счётчик значения больше
      display_hide(1);                   //Скрываем дисплей
  }
  blink_on_count++;                      //Инкримент счётчика
  if (blink_on_count>_value_off) {       //Обнуление счётчика
    blink_on_count=0;
  }
}

/*Функция вывода температуры на семисегментный дисплей*/
//Принимает значение температуры float
void write_display_temp(float val) {
  float _fraction = (val - ((int)val))*10;      //вычисление дробной части
  
  if (val<=-10) {                               //если значение меньше -10
    seg7_write(pcf1,'-',0);                     //в первый разряд "-"
    seg7_write(pcf2, abs (val / 10), 0);        //второй разряд первая цифра
    seg7_write(pcf3, abs ((int)val % 10), 1);   //третий разряд вторая цифра
    seg7_write(pcf4, abs(_fraction), 0);        //четвёртый разряд дробная часть
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
  } else {                                         //В противном случае
    seg7_write(pcf1, 'e', 0);                      //Ошибка
    seg7_write(pcf2, 'r', 0);
    seg7_write(pcf3, 'r', 0);
    seg7_write(pcf4, ' ', 0);
  }
}

/*Функция вывода оборотов на семисегментный дисплей*/
//Принимает значение оборотов
void write_display_rpm(uint16_t val){

  if (val<10) {                                 //Если значение оборотов  меньше 10
    seg7_write(pcf1, 0, 0);                     //в первый разряд "0"
    seg7_write(pcf2, 0, 0);                     //во второй разряд "0"
    seg7_write(pcf3, 0, 0);                     //в трертий разряд "0"
    seg7_write(pcf4, val, 0);                   //в четвертый разряд значение
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

  if (val<10) {                                   //Если код ошибки меньше 10
    seg7_write(pcf1, 'e', 0);                     //В первый разряд выводим "e"
    seg7_write(pcf2, 0, 0);                       //Во второй разряд выводим "0"
    seg7_write(pcf3, 0, 0);                       //В третий разряд выводим "0"
    seg7_write(pcf4, val, 0);                     //В четвёртый разряд выводим значение
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
  
  if (!A && pinLast)                         //Если А изменил состояние первым 
  {
    if (B)
    {
      _count = _count - k;                   //То прибавляем 
      enc_init=true;                         //Регистр вращения энкодера
    }
    else                                     //В противном случает 
    {
      _count = _count + k;                   //Убавляем
      enc_init=true;                         //Регистр вращения энкодера
    }    
  } else {enc_init=false;}

  pinLast = A;                               //Присваеваем последнее значение A

  return init_value + _count;                //Возвращаем изменённое значение
}


/*Считывание датчика температуры внутреннего определённое количество циклов*/
// Принимает колличество циклов возвращает показание термометра
float temp_metr_in(uint16_t cycle) {
  // если прошло больше циклов cчитываем значение с определённого датчика, если нет, инкремент счётчика
  if (count_cycle_temp_in > cycle)
  {
    return dht.readTemperature();
    count_cycle_temp_in = 0;
  }
  else
  {
    count_cycle_temp_in++;
  }
}

/*Считывание датчика температуры наружнего определённое количество циклов*/
// Принимает колличество циклов возвращает показание термометра
float temp_metr_out(uint16_t cycle) {
  // если прошло больше циклов cчитываем значение с определённого датчика, если нет, инкремент счётчика
  if (count_cycle_temp_out > cycle)
  {
    return dht2.readTemperature();
    count_cycle_temp_out = 0;
  }
  else
  {
    count_cycle_temp_out++;
  }
}

/*Функция отправки значения на ЦАП*/
//Принимает адрес и значение от 0 до 255
void dac_write(uint8_t addr, uint16_t val) { 
  
  if (dac_mode=='I2C') {                //Если I2C режим

    Wire.beginTransmission(addr);       //Начало передачи по адресу 44
    Wire.write(byte(0x00));             //Байт инструкции
    Wire.write(val);                    //Отправка значения
    Wire.endTransmission();             //Конец передачи

  } else if (dac_mode=='SPI') {         //Если SPI режим

    addr==dac1 ? b1=(val*2)+512:0;      //Если dac1, пишем в b1 значение, выдаваемое на dac
    addr==dac2 ? b2=(val*2)+512:0;      //.....
    addr==dac3 ? b3=(val*2)+512:0;      //Если dac2, пишем в b2 значение, выдаваемое на dac

    latchPort &= ~ (1<<latchPin);       //"Защёлка'' низкий уровень

      //пишем в порт значение
      shiftOutmy(b3<<2);
      shiftOutmy(b2<<4 | b3>>6);
      shiftOutmy(b1<<6 | b2>>4);
      shiftOutmy(b1>>2);

    latchPort |= (1<<latchPin);          //"Защёлка" высокий уровень, выдаём значение

  }
}

/*Функция считывания переключателей*/  
//Принимает адрес pcf 
byte read_pcf(int addr) {
 byte _val;
  Wire.requestFrom(addr, 8);          //Запрос на чтение адреc, кол-во байт. 
   while(Wire.available())            //Если еcть что читать
   {                                  
     _val = Wire.read();              //Читаем в переменную
   }
 return _val;
}

/*Функция мигания дисплея во время выбора значения*/
// Отдаёт true, когда дисплей перестаёт мигать сur_val - текущее значение, dis_val - выбранное значение, init - вид измерения (0 - температура, 1 - обороты, 3-комбинирован)
bool change_val(float cur_val, float dis_val, int init) {
  if (enc_init==true) {                                            //Если сработал энкодер
    change_enc_init=true;                                          //пишем регистр
    change_time_prev=millis();                                     //запоминаем время
  }

  if (change_enc_init==true) {                                     //если сработал регистр
    if (millis()-change_time_prev>2000) {                          //если прошло время                                        
      if (init==0) {                                               //В зависимости от init выбираем формат вывода
        write_display_rpm((int)cur_val);
      } else if (init==1) {
        write_display_temp(cur_val);
      } else if (init==2) {
        write_display_rpm(cur_val);
      }
      if (change_val_int==true) {                                   //регистр, для вывода значения функции один раз
        change_val_int=false;                                       //меняем регистр                                                   
        return true;                                                //функция отдаёт true один раз
      }
    } else {                                                        //если не прошло время
      display_blink(dis_val, init);                                 //мигаем дисплеем
      change_val_int=true;                                                      
    }
  } else {                                                          //если регистр не сработал выводим текущие показания
    if (init==0) {                                                  //В зависимости от init выбираем формат вывода
      write_display_rpm((int)cur_val);
    } else if (init==1) {
      write_display_temp(cur_val);
    } else if (init==2) {
      write_display_rpm(cur_val);
    }

  }
  return false;                                                      //всё время отдаём false
} 

/*Ускорение работы функции ShiftOut - выводит бит информации в порт последовательно. val - бит информации*/
void shiftOutmy(uint8_t val) {
  uint8_t i;
  for (i = 0; i < 8; i++)  {
    val & (1 << i) ? dataPort |= (1<<dataPin) : dataPort &= ~(1<<dataPin);
    clockPort |= (1<<clockPin);
    clockPort &= ~ (1 << clockPin);   
  }
}

/*Считывание оборотов вентилятора*/
void rpm_read () {
  freq(rpm_fan, 4, time_mettering);
  rpm_1_cold=*rpm_fan[0]*60/time_mettering;            //первый холодный в минуту
  rpm_2_cold=*rpm_fan[2]*60/time_mettering;            //второй холодный в минуту
  rpm_1_hot=*rpm_fan[1]*60/time_mettering;             //первый горячий в минуту
  rpm_2_hot=*rpm_fan[3]*60/time_mettering;             //второй горячий в минуту
}

/*Отображение версии ПО*/
void version () {
  float _fraction = (version_numb - ((int)version_numb))*10;            //вычисление дробной части десятые
  float _fraction2 = (version_numb*10) - ((int)(version_numb*10));      //вычисление дробной части сотые
  seg7_write(pcf1, 'u', 0);                                             //Первый сегмент
  seg7_write(pcf2, version_numb, 1);                                    //Второй сегмент
  seg7_write(pcf3, abs(_fraction), 0);                                  //Третий сегмент
  seg7_write(pcf4, abs(_fraction2*10), 0);                              //Четвёртый сегмент
}

/*Функция обработки прерывания от INT pcf8573*/
void pcf_int() {
  reg_pcf=true;           //Регистр срабатывания прерывания
  //
}

/*

*/
