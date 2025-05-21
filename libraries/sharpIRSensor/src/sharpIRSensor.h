#ifndef SHARP_IR
#define SHARP_IR

#include <Arduino.h>

    const int SHARP_1080 = 1;
    const int SHARP_440 = 2;

class sharpIR
{
    public:

    sharpIR(int pin, int sen);

    sharpIR(int pin, float a, float b, float k);

    float getDist();

    void setVmax(float Vmax);

    private:
    float a;
    float k;
    float b;

    float convert(float Vmax);
    float Vmax;
    int pin;

};

#endif