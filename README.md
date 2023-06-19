# SmartHome-MQTT-ESP32
An IoT side project utilizing MQTT protocol and ESP32 to enable control of air conditioning and light switches via a micro-computer. This repo houses the code and documentation for creating a smarter, more automated home environment.
## 需求
需求資源:ESP32x1,可用的MQTT Broker Server,伺服馬達x2,(選用)鋰電池x1
請安裝Arduino並且將libraries資料夾中的內容存入Arduino對應的libraries資料夾內。
將defines.h中的SSID改成和MQTT Broker同個區域網路下的WiFi熱點SSID，和對應的WiFi密碼。
WiFiMQTT.ino內請更改MQTT Broker之IP或URL。
伺服馬達之pin腳位為:GPIO3,GPIO4。