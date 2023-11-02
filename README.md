# ESP8266-Meteo-Station
Метеостанция на базе отладочной платы WeMos D1R2 & mini (Lolin D1R2 & mini) на основе ESP8266.
В коде используются идеи различных авторов, найденные на просторах интернета, и сторонние библиотеки для работы с климатическими датчиками.
Код написан в среде Arduino IDE.

При работе метеостанция получает информацию от датчиков:
* температуры воздуха - DS18B20 https://amperkot.ru/msk/catalog/germetichnyiy_datchik_ds18b20_1_metr__bez_parazitnogo_rezhima-39162651.html
* атмосферного температуры, влажности и давления - AHT20 + BMP280 https://amperkot.ru/msk/catalog/modul_datchika_temperaturyi_vlazhnosti_i_davleniya_na_chipah_aht20bmp280_s_interfeysom_i2c-39385782.html
* направления ветра - AS5600 https://amperkot.ru/msk/catalog/magnitnyiy_enkoder_na_datchike_holla_as5600_12bit-39166160.html
* скорости ветра - OH137 https://amperkot.ru/msk/catalog/oh137_odnopolyarnyiy_tsifrovoy_datchik_holla-39870913.html

Программа автоматически определяет наличие датчиков и шлет от них информацию на сервер. При первом запуске или отсутствии сети создается точка доступа на 5 секунд (нужно успеть подключиться). В WEB-интерфейсе можно просканировать сети и подключиться к имеющейся либо вписать название сети вручную, после чего на сайт начинают отправлятся данные, а в SERIAL выводится уникальный ID станции, который потом привяжется к сайту. Дополнительно станция отсылает на сервер информацию о напряжении VCC и уровне сигнала Wifi в %. Все показания и статистику на графиках можно смотреть в приложениях под разные платформы, в том числе Android, IOS. Для отладки без Wifi удобно закоментировать 63 и 208 строки.

Корпусные детали для метеостанции: https://r2akt.ru/%d0%bf%d1%80%d0%be%d0%b5%d0%ba%d1%82%d1%8b/%d0%bc%d0%b5%d1%82%d0%b5%d0%be%d0%be%d1%81%d1%82%d0%b0%d0%bd%d1%86%d0%b8%d1%8f/
