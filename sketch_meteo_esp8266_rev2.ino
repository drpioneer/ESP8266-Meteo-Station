/* --------------------------------------------------------------------------------------------
 *  Метеостанция на отладочной плате WeMos D1R2 & mini (Lolin D1R2 & mini) на основе ESP8266
 *  http://arduino.ru/forum/proekty/meteostantsiya-dlya-narodnogo-monitoringa#comment-551457
 *  Станция получает информацию от датчиков: температуры воздуха-DS18B20; атмосферного давления-BME280/BMP280; 
 *  направления ветра (магнитный энкодер)-AS5600; скорости ветра (датчик Холла) -OH137.
 *  Программа автоматически определяет наличие датчиков и шлет от них информацию на сервер.
 *  При первом запуске или отсутствии сети создается точка доступа на 5 секунд (нужно успеть
 *  подключиться). В WEB-интерфейсе можно просканировать сети и подключится к имеющейся либо
 *  вписать название сети вручную, после чего на сайт начинают отправлятся данные, а в SERIAL
 *  выводится уникальный ID станции, который потом привяжется к сайту. 
 *  Дополнительно станция отсылает на сервер информацию о напряжении VCC и уровне сигнала Wifi в %.
 *  Все показания и статистику на графиках можно смотреть в приложениях под разные платформы,
 *  в том числе Android, IOS.
 *  Для отладки без Wifi удобно закоментировать 63 и 208 строки.
 *  Последняя редакция: 20230801
-------------------------------------------------------------------------------------------- */
#include <FS.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>                                    // источник библиотеки https://github.com/tzapu/WiFiManager
#include <DallasTemperature.h>
#include <Wire.h>
#include <iarduino_Pressure_BMP.h>                          // библиотека для работы с датчиками атмосферного давления BMP180 или BMP280
#include <AS5600.h>
#include <AHTxx.h>                                          // AHT10/AHT15/AHT20/AHT21/AHT25/AM2301B/AM2311B Digital Humidity & Temperature Sensor https://github.com/enjoyneering/AHTxx

#define COM_PORT_SPEED            115200                    // скорость COM-порта отладочного терминала
#define POSTING_INTERVAL          300000                    // интервал между отправками данных в миллисекундах (5 минут)
#define OH137                     D3                        // GPIO для приёма данных от OH137
#define DS18B20                   D6                        // GPIO для приёма данных от DS18B20
#define POWER_SENSOR              D7                        // GPIO для включения питания всех сенсоров (OH137, DS18B20, BMP280, AHT20, AS5600)
#define TEMPERATURE_PRECISION     10                        // точность бит для DS18B20. Если глючит или врет -> уменьшить до 9
#define SLEEP_TIME                930e6                     // время сна 930*10^6мс = 15мин

OneWire               oneWire(DS18B20);
DallasTemperature     ds18b20(&oneWire);                    // объявление объекта для работы с DS18B20 с использованием библиотеки DallasTemperature
iarduino_Pressure_BMP bmpx80;                               // объявление объекта для работы с BMP180 с использованием библиотеки iarduino_Pressure_BMP
AS5600                as5600;                               // объявление объекта для работы с AS5600 с использованием библиотеки as5600
AHTxx aht20(AHTXX_ADDRESS_X38, AHT2x_SENSOR);               // sensor address, sensor type

int numberOfDevices               = 0;                      // счётчик подключенных датчиков DS18B20
unsigned long lastConnectionTime  = 0;                      // время последней передачи данных
String hostname                   = "";                     // уникальное имя метеостанции в формате: "ESP+MAC-адрес" (выглядит как ESPAABBCCDDEEFF)
byte bmx_not_found                = false;                  // флаг присутствия датчика BMPx80
byte as56_not_found               = false;                  // флаг присутствия датчика AS5600
byte aht_not_found                = false;                  // флаг присутствия датчика AHTx0
float period                      = 0;                      // длительность периода сигнала от OH137
float frequency                   = 0;                      // частота вращения от OH137
float currentTemp                 = 0;                      // текущая температура считанная с конкретного датчика температуры
float averageTemp                 = 0;                      // усреднённая температура со всех датчиков температуры
ADC_MODE(ADC_VCC);                                          // замер напряжения VCC внутри МК

// --------------------------------------------------------------------------------------------
void WifiManStart()                                         // процедура начального подключения к Wifi
{                                                           // если не знает к чему подцепить - создает точку доступа ESP8266 и настроечную таблицу
    WiFiManager wifiManager;                                // подробнее: https://github.com/tzapu/WiFiManager
    wifiManager.setTimeout(1);                              // устанавливает время ожидания до выключения портала конфигурации полезно, чтобы все повторилось или пошло спать в секундах
    wifiManager.setDebugOutput(true);                       // вывод отладочных сообщений от WiFi-менеджера
    wifiManager.setMinimumSignalQuality(1);                 // минимальное качество сигнала в % для попытки соединения
    if (!wifiManager.autoConnect("meteostation"))           // когда интернет отсутствует -> включаем точку доступа "meteostation" на 5 секунд и если не успели подключиться тогда ->
    {
        Serial.println("Failed to connect and hit timeout");
        //ESP.deepSleep(SLEEP_TIME);                          // сон 15 мин, по окончании сна активируется pin D0, подключенный к RESET МК -> программа стартует заново
    }
    Serial.println("Connected...");
}

// --------------------------------------------------------------------------------------------
void setup()                                                // настройка оборудования
{
    Serial.begin(COM_PORT_SPEED);                // инициализация последовательного порта для отладки на заданной скорости

    // инициализация OH137
    pinMode(OH137,INPUT);                                   // контакт приёма данных о скорости ветра от OH137

    // включение питания сенсоров
    pinMode(POWER_SENSOR,OUTPUT);
    digitalWrite(POWER_SENSOR,HIGH);                        // активация питания всех датчиков

    // инициализация DS18B20
    DeviceAddress tempDeviceAddress;
    ds18b20.begin();                                        // инициализация DS18B20
    numberOfDevices=ds18b20.getDeviceCount();               // определение количества подключенных DS18B20 к шине
    for (int i=0;i<numberOfDevices;i++)                     // настройка точности всех обнаруженных DS18B20
        if (ds18b20.getAddress(tempDeviceAddress,i)) ds18b20.setResolution(tempDeviceAddress,TEMPERATURE_PRECISION);
    ds18b20.requestTemperatures();                          // проведение измерений DS18B20
    Serial.println("\r\n***** Start of meteostation *****");
    Serial.println("Number of sensors DS18B20 found: "+String(numberOfDevices));

    // инициализация BMPx80
    bmpx80.begin();                                         // инициализация BMPx80. Текущая высота принимается за 0м.
    bmpx80.read(1);                                         // проведение измерений BMPx80
    if (!bmpx80.read(1))                                    // когда BMPx80 не представлен ->
    {
        Serial.println("Sensor BMPx80 not detected, сheck I2C interface and address");
        bmx_not_found=true;
    } else                                                  // когда BMPx80 обнаружен ->
        Serial.println("Detected sensor BMP"+String(bmpx80.type));

    // инициализация AS5600
    as5600.begin(4);
    as5600.setDirection(AS5600_CLOCK_WISE);
    if (!as5600.isConnected())                              // когда AS5600 не представлен ->
    {
        Serial.println("Sensor AS5600 not detected, сheck I2C interface and address");
        as56_not_found=true;
    } else                                                  // когда AS5600 обнаружен ->
        Serial.println("Detected sensor AS5600");

    // инициализация AHTx0
    if (!aht20.begin())                                     // когда AHTxx не представлен ->
    {
        Serial.println("Sensor AHT20  not detected, сheck I2C interface and address");
        aht_not_found=true;
    } else                                                  // когда AHTxx обнаружен ->
        Serial.println("Detected sensor AHT20");

    // инициализация WiFi
    WifiManStart();                                         // запуск WiFi
    hostname="ESP"+WiFi.macAddress();                       // формирование уникального имени метеостанции
    hostname.replace(":","");
    WiFi.hostname(hostname);                                // представление имени хоста метеостанции для WiFi
    //WiFiManager(resetSettings());                           // сброс настроек wifi
    Serial.println(WiFi.macAddress());
    Serial.println(WiFi.localIP());
    Serial.println("NarodMon ID: "+hostname);
    lastConnectionTime=millis()-POSTING_INTERVAL+15000;     // передача на народный мониторинг через 15 сек.
}

// --------------------------------------------------------------------------------------------
bool SendToNarodmon()                                       // формирование и отправка пакета
{
    DeviceAddress tempDeviceAddress;
    WiFiClient client;
    String buf="#"+hostname+"\n";                           // id станции

    for (int i=0;i<numberOfDevices;i++)                     // перебор датчиков температуры DS18B20
    {
        ds18b20.getAddress(tempDeviceAddress,i);
        currentTemp=ds18b20.getTempCByIndex(i);
        averageTemp=averageTemp+currentTemp;
        buf=buf+"#TEMP"+String(i+1)+"#"+String(currentTemp)+"#Температура (град.Цельсия) DS18B20 id:";   // чтение температуры с конкретного датчика DS18B20
        for (uint8_t i=0;i<8;i++)
        {
            if (tempDeviceAddress[i]<16) buf=buf+"0";       // адрес конкретного датчика DS18B20
            buf=buf+String(tempDeviceAddress[i],HEX);
        }
        buf=buf+"\n";
    }
    averageTemp=averageTemp/numberOfDevices;

    period=pulseIn(OH137,HIGH)+pulseIn(OH137,LOW);          // сумма длительности высокого и низкого уровней OH137
    if (period!=0)
    {
        frequency=1000000/period;                           // подсчёт частоты с учётом времени в микросекундах
        buf=buf+"#TACHO#"+String(frequency/2)+"#Скорость ветра (м/сек) OH137\n";   // скорость OH137
    } else 
        buf=buf+"#TACHO#0.00#Скорость ветра (м/сек) OH137\n";                      // нулевая скорость от OH137

    if (as56_not_found==false)                              // при наличии AS5600 ->
        buf=buf+"#ANGLE#"+String(as5600.rawAngle()*AS5600_RAW_TO_DEGREES)+"#Направление ветра (град. от направления на Север) AS5600\n"; // угол (азимут) AS5600

    if (aht_not_found==false)                               // при наличии AHTxx ->
    {
        currentTemp=aht20.readTemperature();
        averageTemp=(averageTemp+currentTemp)/2;
        buf=buf+"#TEMPA#"+String(currentTemp)+"#Температура (град.Цельсия) AHT20\n";    // температура AHTxx
        buf=buf+"#HUMID#"+String(aht20.readHumidity())+"#Влажность (%) AHT20\n";        // влажность AHTxx
    }

    if (bmx_not_found==false)                               // при наличии BMPx80 ->
    {
        currentTemp=bmpx80.temperature;
        averageTemp=(averageTemp+currentTemp)/2;
        buf=buf+"#TEMPB#"+String(currentTemp)+"#Температура (град.Цельсия) BMP"+String(bmpx80.type)+"\n";   // температура BMPx80
        buf=buf+"#PRESS#"+String(bmpx80.pressure)+"#Давление (мм рт.ст.) BMP"+String(bmpx80.type)+"\n";     // давление BMPx80
    }

    buf=buf+"#TEMPZ#"+String(averageTemp)+"#Усреднённая температура (град.Цельсия) со всех датчиков\n";     // средняя температура со всех датчиков

    buf=buf+"#VCC#"+String(float ((ESP.getVcc()+350))/1000)+"#Питание контроллера (В) ESP8266\n";           // напряжение питания ESP8266

    int wifiRSSI=(WiFi.RSSI()+100)*2;
    wifiRSSI=constrain(wifiRSSI,0,100);
    buf=buf+"#WIFI#"+String(wifiRSSI)+"#Уровень сигнала WiFi (%)"+String(WiFi.SSID())+"\n";                 // уровень сигнала WiFi 
    buf=buf+"##\n";                                         // формирование окончания посылки

    Serial.print(buf);

    if (!client.connect("narodmon.ru",8283))                // когда попытка подключения не удалась ->
    {
        Serial.println("connection failed"); 
        return false;                                       // возврат FALSE (неудачная посылка)
    }
    else                                                    // когда подключение удалось ->
    {
        client.print(buf);                                  // отправка данных
        delay(100);
        while (client.available())
        {
            String line=client.readStringUntil('\r');       // если что-то прилетит в ответ -> отобразить в Serial
            Serial.print("Answer from server: "+line);
        }
    }
    return true;                                            // возврат TRUE (успешная посылка)
}

// --------------------------------------------------------------------------------------------
void loop()
{
    //if (WiFi.status()==WL_CONNECTED)                        // при наличии WiFi-подключения к сети ->
    {
        if (SendToNarodmon())                               // формирование и отправка пакета на narodmon.ru
        {
            Serial.print (millis());
            digitalWrite(POWER_SENSOR,LOW);                 // отключение питания сенсоров
        }
        ESP.deepSleep(SLEEP_TIME);                          // сон 15 мин, по окончании сна активируется pin D0, подключенный к RESET МК -> программа стартует заново
    }
}
