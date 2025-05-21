#include "sharpIRSensor.h"

sharpIR::sharpIR(int pin, int sen)
{
    switch (sen)
    {
    case 1:
        a = 30.5;
        b = 2.7;
        k = 1;
        break;

    case 2:
        a = 13.5;
        b = 0.7;
        k = 1;
        break;
    default:
        a = 30.5;
        b = 2.7;
        k = 1;
        break;
    }
    this->pin = pin;
    Vmax = 5;
}

sharpIR::sharpIR(int pin, float a, float b, float k)
{
    this->a = a;
    this->b = b;
    this->k = k;

    this->pin = pin;
    Vmax = 5;
}

float sharpIR::getDist(){
    float v = convert(Vmax);
    return a/v - b;
}

float sharpIR::convert(float Vmax){
    int v = analogRead(pin);
    return Vmax/1024 * v;
}

void sharpIR::setVmax(float Vmax){
    this->Vmax = Vmax;
}