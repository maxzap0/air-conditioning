#include <EEPROM.h>
#include <timer_radar.h>
#include <DHT.h>
#include <Wire.h> //библиотека I2C
#include <frequency.h>

#define pcf1 0x20 // первый сегиент индикатора
#define pcf2 0x21 //
#define pcf3 0x22 //
#define pcf4 0x23 // последний сегмент индикатора
#define pcf5 0x24 // ДИПы
#define pcf6 0x25 // 

#define dac1 0x2C // первый DAC
#define dac2 0x2D //
#define dac3 0x2E //
#define dac4 0x2F // последний DAC

#define A_PORT PIND //Порт ножки A энкодера
#define A_pin 6     //Пин ножки А энкодера
#define B_PORT PIND //Порт ножки B энкодера
#define B_pin 7     //Пин ножки B энкодера

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
bool change_enc_init;       // регистр функции change_val
uint16_t change_time_prev;  // для запоминания времени
bool change_val_int;        // регистр функции change_val

/*Переменные оборотов вентилятора*/
uint16_t rpm_1_cold; //первый холодный вентилятор, текущее значение оборотов
uint16_t rpm_1_cold_dac; //Значение выдаваемое на dac первого холодного венттидятора
uint16_t rpm_2_cold;      //второй холодный вентилятор, текущее значение оборотов
uint16_t rpm_2_cold_dac; //Значение выдаваемое на dac второго холодного венттидятора

/*Замер скорости работы вентиляторов*/
int *rpm_fan[] = new int[4];


/* Временные переменные *//////////////////////////////////////////////////////////////////////////////////////////  



/**///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup(){
  Serial.begin(115200); // скорость com-порта
  Wire.begin(); // подключение к шине wire
  /*Установка пинов энкодера*/
  pinMode(6, INPUT_PULLUP); // установка пина на вход, подтяжка к +5В
  pinMode(7, INPUT_PULLUP); // установка пина на вход, подтяжка к +5В
  /*Установка пинов датчика температуры*/
  pinMode(8, INPUT_PULLUP); // установка пина на вход, подтяжка к +5В

  /*Считывание первоначальных значение EEPROM*/
  rpm_1_cold_dac = EEPROM.read(1); //значение rpm первого холодного вентилятора
  rpm_2_cold_dac = EEPROM.read(2); //значение rpm второго холодного вентилятора

  /*Настройка измерителя чистоты*/
  setup_TC1(20);

 /*Временные установки*//////////////////////////////////////////////////////////////////////////////////////////
 
}

void loop(){

  /*Замер скорости работы программы*/
  prev_millis_speed = millis(); // время на начало программы
  /************************************/

  freq(&rpm_fan, 10, 1);

  //Считывание режима dip переключаталей и переход в нужный режим
  switch (read_dip()) {
      case 15 :  //Рабочий режим
         
        break;
      case 14 : //настройка 1го холодного вентилятора
        fan_setting1();  
      break;
      case 13:  //настройка 2го холодного вентилятора
        fan_setting2();
      break;

     default :
     ;
  } 

 
  /*Замер скорости работы программы*/
  //cur_millis_speed = millis(); // время на конец программы
  //Serial.println(cur_millis_speed - prev_millis_speed); // вывод времени исполнения программы
  
  /************************************/
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

/*Функция считывания значений dip-переключателей, возвращает значение dip-переключателей*/
byte read_dip () {
  return (0b1111 & read_pcf(pcf5)); //считываем первые четыре байта pcf
  //
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
    change_enc_init=true;        //пишем регистр
    change_time_prev=millis();   //запоминаем время
  }

  if (change_enc_init==true) {                                                           //если сработал регистр
    if (millis()-change_time_prev>700) {                                               //если прошло время
      init ? write_display_temp(cur_val) : write_display_rpm ((int)cur_val);    //выводим текущие показания
      if (change_val_int==true) {                                                         //регистр, для вывода значенияфункции один раз
        change_val_int=false;                                                             //меняем регистр
        return true;                                                            //функция отдаёт true один раз
      }
    } else {                                                                    //если не прошло время
      display_blink(dis_val, init);
      change_val_int=true;                                             //мигаем дисплеем
    }
  } else {                                                                      //если регистр не сработал выводим текщие показания
    init ? write_display_temp(cur_val): write_display_rpm ((int)cur_val);
  }
  return false;                                                                  //всё вермя отдаём false
} 

/*
Sketch uses 7,456 bytes (24.3%) of program storage space. Maximum is 30,720 bytes.
Global variables use 550 bytes (26.9%) of dynamic memory, leaving 1,498 bytes for local variables. Maximum is 2,048 bytes.


*/
