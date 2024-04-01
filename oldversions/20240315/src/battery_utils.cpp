#include "battery_utils.h"
#include "settings.h"
#include "pins_config.h"

float adcReadingTransformation = (3.30/4096);
//float voltageDividerCorrection = 0.4; //0.288
// for External Voltage Measurement (MAX = 15Volts !!!)
//float R1 = 100.000; //in Kilo-Ohms
//float R2 = 27.000; //in Kilo-Ohms
//float readingCorrection = 0.125;
//float multiplyCorrection = 0.035;

namespace BATTERY_Utils {
// read the battery voltage thru the inner 100k/100k divider connected to pin IO35
    float checkBattery() { 
        int sample;
        int sampleSum = 0;
        for (int i=0; i<100; i++) {
            sample = analogRead(ADC_PIN);
            sampleSum += sample;
            delayMicroseconds(50); 
        }
        return (2 * (sampleSum/100) * adcReadingTransformation) + OnBoardDividerCorrection;
    }
/*
    float checkExternalVoltage() {
        int sample;
        int sampleSum = 0;
        for (int i=0; i<100; i++) {
            sample = analogRead(Config.externalVoltagePin);
            sampleSum += sample;
            delayMicroseconds(50); 
        }
        return ((((sampleSum/100)* adcReadingTransformation) + readingCorrection) * ((R1+R2)/R2)) - multiplyCorrection;
    }
*/
}