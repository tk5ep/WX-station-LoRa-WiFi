#include "MQTT_utils.h"
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>           // by Nick O'Leary
#include "settings.h"
#include "logger.h"
#include "display.h"

extern logging::Logger  logger;
extern float humi;
extern float press;
extern float tempC;
extern byte rain1hAPRS;
extern int winDirAvg_2min;
extern float windSpeed_avg2m;
extern int windgustDir;
extern float windgustSpeed;
extern bool RS485WindSpeedSensorTimeout;
extern bool RS485WindDirSensorTimeout;
extern float rain1hmm;
extern volatile float rain24hmm;
extern float batteryVoltage;

WiFiClient client;
PubSubClient mqttclient(client);

namespace MQTT_Utils {
    /**************************************************************************
     * connecting to MQTT brocker
     **************************************************************************/
    #ifdef WITH_MQTT
    void connect(){
        mqttclient.setServer(mqtt_broker, mqtt_port);
        //mqttclient.setKeepAlive(300);
        uint8_t retries = 5;                             // number of connection tries
        while (!mqttclient.connected()) {
            String client_id = "WX-station";
            logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "MQTT", "Client %s connects MQTT broker %s", client_id.c_str(),mqtt_broker);
            if (mqttclient.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
                logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "MQTT", "Connected to broker %s",mqtt_broker);
            } else {
                logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "MQTT", "Connection failed with state %s",String(mqttclient.state()));
//                delay(1000);
            }
            retries--;

            if (retries == 0) {
                logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "MQTT", "Connection to broker failed, trying again later !");
                //while (1); // if we want to stop
                break;
            }
        }
    }

    /**************************************************************************
     * publishing on MQTT brocker
     **************************************************************************/
    void publish() {
        connect();
        if (mqttclient.connected()) {
            logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "MQTT", "Publishing latest MQTT datas");
            char buffer[6];
            sprintf(buffer, "%.2f", tempC);                           // convert float to string before publishing
            mqttclient.publish(TOPIC_TEMP, buffer , mqtt_retained );  // publish topic to broker Last parameter "retained" is 0 or 1
            sprintf(buffer, "%.2f", humi);                            // convert float to string before publishing
            mqttclient.publish(TOPIC_HUMI, buffer, mqtt_retained  );
            sprintf(buffer, "%.1f", press);
            mqttclient.publish(TOPIC_PRESS, buffer, mqtt_retained  );
            sprintf(buffer, "%.2f", batteryVoltage);
            mqttclient.publish(TOPIC_BATTERYVOLTAGE, buffer, mqtt_retained  );
        
            #ifdef WITH_WIND
                sprintf(buffer, "%d", winDirAvg_2min);
                mqttclient.publish(TOPIC_WINDDIR, buffer,mqtt_retained);
                sprintf(buffer, "%.2f", windSpeed_avg2m);
                mqttclient.publish(TOPIC_WINDSPEED, buffer,mqtt_retained);
                sprintf(buffer, "%d", windgustDir);
                mqttclient.publish(TOPIC_GUSTDIR, buffer,mqtt_retained);
                sprintf(buffer, "%.2f", windgustSpeed);
                mqttclient.publish(TOPIC_GUSTSPEED, buffer,mqtt_retained);
                // report if the RS485 sensors are responding
                if (RS485WindSpeedSensorTimeout) mqttclient.publish(TOPIC_WINDSPEEDSENSOR, "0" ,mqtt_retained);
                else mqttclient.publish(TOPIC_WINDSPEEDSENSOR, "1" ,mqtt_retained);
                if (RS485WindDirSensorTimeout) mqttclient.publish(TOPIC_WINDDIRSENSOR, "0" ,mqtt_retained);
                else mqttclient.publish(TOPIC_WINDDIRSENSOR, "1" ,mqtt_retained);
            #endif

            #ifdef WITH_RAIN
                sprintf(buffer, "%.2f", rain1hmm);                  // convert float to string before publishing
                mqttclient.publish(TOPIC_RAIN1H, buffer , mqtt_retained  );
                sprintf(buffer, "%.2f", rain24hmm);               // convert float to string before publishing
                mqttclient.publish(TOPIC_RAIN24H, buffer , mqtt_retained  );
                #endif
            mqttclient.disconnect();
        }
        else {}
    }
    #endif
}