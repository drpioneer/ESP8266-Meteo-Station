/*--------------------------------------------------------------------------------------------
 *  Метеостанция на базе ESP8266 ( LoLin D1 mini / WeMos D1 mini )
 *  http://arduino.ru/forum/proekty/meteostantsiya-dlya-narodnogo-monitoringa#comment-551457
 *  Станция собирает информацию от датчиков и передает информацию на сервер нородного мониторинга
 *  При первом запуске или отсутствии сети создается точка доступа на 5 секунд (нужно успеть
 *  подключиться). В WEB-интерфейсе можно просканировать сети и подключится к имеющейся либо
 *  вписать название сети вручную, после чего на сайт начинают отправлятся данные, а в SERIAL
 *  выводится отладочная информация. 
 *  Дополнительно станция отсылает на сервер информацию о напряжении питания контроллера и уровне сигнала Wifi.
 *  Все показания и статистику на графиках можно смотреть в приложениях под разные платформы, в том числе Android, IOS.
 *-------------------------------------------------------------------------------------------- */
#include <FS.h>
#include "ESP8266WiFi.h"
#include "DNSServer.h"
#include "ESP8266WebServer.h"
#include "DallasTemperature.h"
#include "WiFiManager.h"                                    // источник библиотеки https://github.com/tzapu/WiFiManager
#include <Wire.h>
#include "Adafruit_BMP280.h"                                // библиотека для работы с датчиком BMP280
#include "Adafruit_AHTX0.h"                                 // библиотека для работы с датчиком AHT20
#include "SparkFun_SCD4x_Arduino_Library.h"                 // библиотека для работы с датчиком SCD40 http://librarymanager/All#SparkFun_SCD4x

#define PIN_1WIRE_VCC             D6                        // user defined 1WIRE sensor VCC-pin for DS18B20
#define PIN_1WIRE_DAT             D7                        // user defined 1WIRE sensor DAT-pin for DS18B20
#define PIN_1WIRE_GND             D8                        // user defined 1WIRE sensor GND-pin for DS18B20 (!!!для старта МК уровень на этом PINе должен быть LOW!!!)

/*
#define PIN_I2C_GND               D1                        // user defined I2C sensor VCC-pin for AHT20&BMP280 // SCD40 припаяный
#define PIN_I2C_VCC               D2                        // user defined I2C sensor SDA-pin for AHT20&BMP280
#define PIN_I2C_CLK               D3                        // user defined I2C sensor SCL-pin for AHT20&BMP280
#define PIN_I2C_SDA               D4                        // user defined I2C sensor GND-pin for AHT20&BMP280
*/

#define PIN_I2C_SDA               D2                        // user defined I2C sensor SDA-pin for AHT20&BMP280 // AHT20+BMP280 припаяный
#define PIN_I2C_CLK               D4                        // user defined I2C sensor SCL-pin for AHT20&BMP280
#define PIN_I2C_VCC               D1                        // user defined I2C sensor VCC-pin for AHT20&BMP280
#define PIN_I2C_GND               D3                        // user defined I2C sensor GND-pin for AHT20&BMP280

/*
#define PIN_I2C_SDA               D2                        // user defined I2C sensor SDA-pin for AHT20&BMP280 // AHT20+BMP280 на проводе
#define PIN_I2C_CLK               D3                        // user defined I2C sensor SCL-pin for AHT20&BMP280
#define PIN_I2C_VCC               D1                        // user defined I2C sensor VCC-pin for AHT20&BMP280
#define PIN_I2C_GND               D4                        // user defined I2C sensor GND-pin for AHT20&BMP280
*/
#define COM_PORT_SPEED            115200                    // скорость COM-порта отладочного терминала
#define SLEEP_TIME                15                        // время сна в минутах
#define ATTEMPTS                  10                        // количество попыток инициализации датчиков
#define TEMPERATURE_PRECISION     10                        // точность бит для DS18B20. Если глючит или врет -> уменьшить до 9

ADC_MODE                          (ADC_VCC);                // настройка АЦП на измерение напряжения питания микроконтроллера
OneWire                           oneWire(PIN_1WIRE_DAT);
SCD4x                             mySensor;                 // объявление объекта для работы с датчиком SCD4x
DallasTemperature                 ds18b20(&oneWire);        // объявление объекта для работы с DS18B20 с использованием библиотеки DallasTemperature
Adafruit_AHTX0                    aht;                      // объявление объекта для работы с датчиком AHT20
Adafruit_Sensor                   *aht_humidity, *aht_temperature;
Adafruit_BMP280                   bmx;                      // объявление объекта для работы с датчиком BMP280
String                            hostname = "";            // уникальное имя метеостанции в формате: "ESP+MAC-адрес" (выглядит как ESPAABBCCDDEEFF)

// --------------------------------------------------------------------------------------------
void WifiManStart() {                                       // процедура начального подключения к wifi, если не знает к чему подцепить - создает точку доступа ESP8266 и настроечную таблицу
  WiFiManager wifiManager;                                  // подробнее: https://github.com/tzapu/WiFiManager
  wifiManager.setTimeout (1);                               // устанавливает время ожидания до выключения портала конфигурации полезно, чтобы все повторилось или пошло спать в секундах
  wifiManager.setDebugOutput(true);                         // вывод отладочных сообщений от wifi-менеджера
  wifiManager.setMinimumSignalQuality(1);                   // минимальное качество сигнала в % для попытки соединения
  if (!wifiManager.autoConnect("meteostation"))             // когда интернет отсутствует -> включаем точку доступа "meteostation" на 5 секунд и если не успели подключиться тогда ->
    Serial.println("Not connect in allotted time");
  Serial.println("Connected...");
}

// --------------------------------------------------------------------------------------------
void setup() {                                              // настройка оборудования
  Serial.begin (COM_PORT_SPEED);                            // инициализация последовательного порта для отладки на заданной скорости
  Serial.println("\r\n\r\n\r\n***** MeteoStation awaked ******");
  Serial.println("\r\n***** >>> WIFI STAGE <<< *****");
  WifiManStart();                                           // запуск wifi
  hostname="ESP"+WiFi.macAddress();                         // формирование уникального имени метеостанции
  hostname.replace(":","");
  WiFi.hostname(hostname);                                  // представление имени хоста метеостанции для wifi
  //WiFiManager(resetSettings());                           // сброс настроек wifi
  Serial.println(WiFi.macAddress());
  Serial.println(WiFi.localIP());
  Serial.println("MeteoStation ID: "+hostname);
}

// ------------------расчёт значений---------------------
String Calculate(char chr, float temper, float humid) {
  if (temper >= -40 && temper <= +50 && humid > 0) {        // при допустимых условиях для расчёта точки росы ->
    const float k = 273.15;                                 // температура 0°C, выраженная в градусах Кельвина
    const float a = 17.62, b = 243.12;                      // коэффициенты зависимости удерживаемой в воздухе влаги при температуре от -40 до +50°C
    const float p = 6.112;                                  // коэффициент зависимости давления насыщенного пара от температуры
    const float r = 461.52;                                 // удельная газовая постоянная для водяного пара
    float       v = (a * temper)/(b + temper);              // влияние температуры воздуха на способность удерживать водяной пар
    float       g = v + log(humid / 100);                   // привязка температуры воздуха ко влажности для оценки точки росы
    float     tdp = (b * g) / (a - g);                      // расчёт точки росы по упрощённой формуле Магнуса
    float  absHmd = p * exp(v) * humid * 2.1674 / (k + temper);                // расчет абсолютной влажности v.1
    //float  absHmd = humid * 10 * ((p * 100.0 * exp(v)) / (r * (temper + k)));  // расчет абсолютной влажности v.2
    String   text = "";
    text = "#D" + String(chr) + "#" + String(tdp)     + "#°C Dew point\r\n#H";
    text = text + String(chr) + "#" + String(absHmd) + "#g/m3 Absolute humidity\r\n";
    return (text);
  }
  return "";
}

String Measure(void) {
  String            buf = "";                               // динамический текстовый буфер для подготовки сообщения
  uint8_t      attempts = 0;                                // счётчик попыток инициализации датчиков
  float          humAHT = 0;                                // буфер для значения влажности от AHT20
  float          tmpAHT = 0;                                // буфер для значения температуры воздуха от AHT20
  float          tmpBMx = 0;                                // буфер для значения температуры воздуха от BMP280
  float          prsBMx = 0;                                // буфер для значения атмосферного давления от BMP280
  float          tmpAVR = 0;                                // буфер для значения усреднённой температуры воздуха
  uint16_t       co2SCD = 0;
  float          tmpSCD = 0;
  float          hmdSCD = 0;
  bool             real = true;                             // признак реальных значений

  Serial.println("\r\n***** >>> MEASUREMENTS STAGE <<< *****");

// -----------------------начало работы датчиками I2C------------------------------
  pinMode      (PIN_I2C_GND, OUTPUT);
  digitalWrite (PIN_I2C_GND, LOW);                          // подключение I2C датчиков к GND
  pinMode      (PIN_I2C_VCC, OUTPUT);
  digitalWrite (PIN_I2C_VCC, HIGH);                         // подключение I2C датчиков к VCC 
  Wire.begin   (PIN_I2C_SDA, PIN_I2C_CLK);                  // инициализация шины I2C с пользовательскими параметрами

// -----------------------опрос BMP280------------------------------
  attempts = ATTEMPTS;
  while (!bmx.begin() && (attempts == 0))                   // попытки инициализации BMP280
    attempts--;
  if (bmx.begin()) {                                        // инициализация BMP280
    Serial.println("Sensor BMP280 success init");
    tmpBMx = bmx.readTemperature();                         // чтение температуры в градусах Цельсия из BMP280
    prsBMx = bmx.readPressure();                            // чтение давления в Паскалях из BMP280

    if (isnan(prsBMx))                                      // когда отсутствует датчик давления BMP280 ->
      real = false;                                         // запрет дальнейших расчетов на основе полученных данных
    else {
      prsBMx = prsBMx / 133.33F;                            // пересчёт атмосферного давления из Паскалей в мм ртутного столба
      buf = buf + "#PB#" + String(prsBMx) + "#mmHg Atmospheric pressure BMP280\r\n";
    }

    if (isnan(tmpBMx))                                      // когда отсутствует датчик температуры BMP280 ->
      real = false;                                         //запрет дальнейших расчетов на основе полученных данных
    else {
      tmpAVR = (tmpAVR + tmpBMx);                           // накопление данных о температуре воздуха
      buf = buf + "#TB#" + String(tmpBMx) + "#°C Air temperature BMP280\r\n";
    }

  } else {
    real = false;                                           // запрет дальнейших расчетов на основе полученных данных
    Serial.println("Sensor BMP280 not detected, сheck I2C interface & sensor address");
  }

// -----------------------опрос AHT20------------------------------
  attempts = ATTEMPTS;
  while (!aht.begin() && (attempts == 0))                   // попытки инициализации AHT20
    attempts--;
  if (aht.begin()) {                                        // инициализация AHT20
    Serial.println("Sensor AHT20 success init");
    sensors_event_t   humid, temp;
    aht.getEvent(&humid, &temp);                            // обновление данных об относительной влажности и температуре из AHT20
    tmpAHT = temp.temperature;                              // извлечение температуры в градусах Цельсия
    humAHT = humid.relative_humidity;                       // извлечение относительной влажности в процентах

    if (isnan(tmpAHT))                                      // когда отсутствует датчик температуры AHT20 ->
      real = false;                                         // запрет дальнейших расчетов на основе полученных данных
    else {
      tmpAVR = (tmpAVR + tmpAHT);                           // накопление данных о температуре воздуха
      buf = buf + "#TA#" + String(tmpAHT) + "#°C Air temperature AHT20\r\n";
    }
    if (isnan(humAHT))                                      // когда отсутствует датчик влажности AHT20 ->
      real = false;                                         // запрет дальнейших расчетов на основе полученных данных
    else
      buf = buf + "#HA#" + String(humAHT) + "#%rH Relative humidity AHT20\r\n";

  } else {
    real = false;                                           // запрет дальнейших расчетов на основе полученных данных
    Serial.println("Sensor AHT20 not detected, сheck I2C interface & sensor address");
  }

// -----------------------опрос SCD40------------------------------
  attempts = ATTEMPTS;
  while (!mySensor.begin() && (attempts == 0))              // попытки инициализации SCD40
    attempts--;
  if (mySensor.begin()) {                                   // инициализация SCD40
    Serial.println("Sensor SCD40 success init");
    for (uint8_t i=0; i < 10000; i++) {
      if (mySensor.readMeasurement()) {                     // ожидание свежих значений от SCD40
        co2SCD = mySensor.getCO2();
        tmpSCD = mySensor.getTemperature();
        hmdSCD = mySensor.getHumidity();
        buf = buf + "#DS#" + String(co2SCD) + "#ppm CO2 concentration SCD40\r\n";
        buf = buf + "#TS#" + String(tmpSCD) + "#°C Air temperature SCD40\r\n";
        buf = buf + "#HS#" + String(hmdSCD) + "#%rH Relative humidity SCD40\r\n";

        break;
      }
    }
  } else { Serial.println("Sensor SCD40 not detected, сheck I2C interface & sensor address"); }

// -----------------------окончание работы датчиками I2C------------------------------
  pinMode      (PIN_I2C_GND, INPUT);                        // обесточивание I2C датчиков
  pinMode      (PIN_I2C_VCC, INPUT);
    
// -----------------------расчётные значения------------------------------
  if (abs(tmpBMx - tmpAHT) > 1)                             // при большой разнице между значениями с датчиков температуры воздуха AHT20+BMP280 ->
    real = false;                                           // запрет расчетов на основе полученных данных от AHT20+BMP280

  if (real) {                                               // когда все значения от AHT20 + BMP280 корректны ->
    tmpAVR = tmpAVR / 2;                                    // усреднение температуры воздуха
    buf = buf + "#TZ#" + String(tmpAVR) + "#°C Air temperature\r\n";
    buf = buf + Calculate('Z', tmpAVR, humAHT);
  } else {                                                  // используем значения от SCD40, когда значения от AHT20 + BMP280 НЕ корректны ->
    buf = buf + (Calculate('Z', tmpSCD, hmdSCD));
  }

// -----------------------работа с DS18B20----------------------------
  pinMode      (PIN_1WIRE_GND, OUTPUT);
  digitalWrite (PIN_1WIRE_GND, LOW);                        // подключение 1WIRE датчиков к GND
  pinMode      (PIN_1WIRE_VCC, OUTPUT);
  digitalWrite (PIN_1WIRE_VCC, HIGH);                       // подключение 1WIRE датчиков к VCC

  ds18b20.begin();                                          // инициализация DS18B20
  DeviceAddress tempDeviceAddress;
  uint8_t numberOf1Wire = ds18b20.getDeviceCount();         // определение количества подключенных датчиков DS18B20 к шине 1WIRE
  if (numberOf1Wire > 0) {                                  // инициализация DS18B20
    Serial.println("Sensor DS18B20 detected");
    for (uint8_t i=0; i < numberOf1Wire; i++) {             // перебор обнаруженных датчиков DS18B20
      if (ds18b20.getAddress(tempDeviceAddress,i))
        ds18b20.setResolution(tempDeviceAddress,TEMPERATURE_PRECISION);  // настройка точности обнаруженных датчиков DS18B20
      ds18b20.requestTemperatures();                        // проведение измерений DS18B20
      float tmpDS18 = ds18b20.getTempCByIndex(i);
      if (!isnan(tmpDS18))
        buf = buf + "#T" + String(i+1) + "#" + String(tmpDS18) + "#°C Temperature DS18B20\r\n"; // чтение температуры с конкретного датчика DS18B20
    }
  } else { Serial.println("Sensor DS18B20 not detected, сheck 1-Wire interface"); }

  pinMode      (PIN_1WIRE_GND, INPUT);                      // обесточивание 1WIRE датчиков
  pinMode      (PIN_1WIRE_VCC, INPUT);

// ---------------------значения ESP8266-------------------------
  long dBm = WiFi.RSSI();
  if (dBm > 0)  buf = buf + "#WF#-120#dBm Wifi is missing\r\n";
    else  buf = buf + "#WF#" + String(dBm) + "#dBm Wifi level\r\n";

  float vcc = ESP.getVcc();
  if (!isnan(vcc)) {
    vcc = (vcc + 300) / 1000;                               // напряжения питания ESP8266
    if (vcc > 0)
      buf = buf + "#VCC#" + String(vcc) + "#V Supply voltage ESP8266\r\n";
  }

  buf = buf + "##\r\n";                                     // признак окончания посылки на narodmon.ru
  return buf;
}

// --------------------------------------------------------------------------------------------
bool SendToNarodmon() {                                     // формирование и отправка пакета
  WiFiClient client;
  String msg = "#" + hostname + "\r\n" + Measure();         // id станции + показания датчиков
  Serial.print (msg);
  if (!client.connect("narodmon.ru",8283)) {                // когда попытка подключения не удалась ->
    Serial.println("connection failed"); 
    return false;                                           // возврат FALSE (неудачная посылка)
  } else {                                                  // когда подключение удалось ->
    Serial.println("\r\n***** >>> SENDING STAGE <<< *****");
    client.print(msg);                                      // отправка данных
    delay(100);
    while (client.available()) {                            // если прилетит ответ -> 
      String line=client.readStringUntil('\r');
      Serial.print("Answer from server: " + line);          // вывод ответа в Serial
    }
  }
  return true;                                              // возврат TRUE (успешная посылка)
}

// --------------------------------------------------------------------------------------------
void loop() {
  if (WiFi.status()==WL_CONNECTED) {                        // при наличии wifi подключения к сети ->
    if (SendToNarodmon())                                   // формирование и отправка посылки на narodmon.ru
      Serial.println ("Successful sending!");
  } else {                                                  // при отсутствии wifi подключения к сети ->
    Serial.println("WIFI connection failed"); 
    String msg = "#" + hostname + "\r\n" + Measure();       // формирование отчёта
    Serial.print (msg);
  }
  Serial.print ("\r\n***** >>> SNOOZING STAGE <<< *****\r\n\r\n_ _ _ zzzzZZZZ _ _ _\r\n\r\n***** MeteoStation snoozing ******\r\n");
  ESP.deepSleep(SLEEP_TIME * 60e6);                         // сон (в мс), по окончании которого активируется pin D0 -> RESET МК и программа стартует заново
}
