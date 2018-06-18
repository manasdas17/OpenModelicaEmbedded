#ifndef MODELPLUG_H
#define MODELPLUG_H


#if defined(_MSC_VER)
    //  Microsoft VC++
    #define EXPORT __declspec(dllexport)
#else
    //  GCC
    #define EXPORT __attribute__((visibility("default")))
#endif

extern "C" {

EXPORT void* boardConstructor(char* port,bool showCapabilitites,int samplingMs,int baudRate,bool dtr);
EXPORT void boardDestructor(void* object);

EXPORT void updateBoard(int id);
EXPORT int getBoardId(void* object);


EXPORT double readAnalogPin  (int pin, double min, double max, double init, int id, int adcResolution);
EXPORT int    readDigitalPin (int pin, int init, int id);
EXPORT void   writeAnalogPin (int pin, int id,double value);
EXPORT void   writeDigitalPin(int pin, int id,int value);
EXPORT void   writeServoPin  (int pin, int id,double value, int min, int max);

}

#endif