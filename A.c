
#include "main.h"

int main(void)
{
    start();
    while(1)
    {
        if (INPUT&IN1)
        {
            if (INPUT&IN2)
            {
                motor1(15);
                motor2(-15);
            }
            else
            {
                motor1(15);
                motor2(02);
            }
        }
        else
        {
            if (INPUT&IN2)
            {
                motor1(-02);
                motor2(-15);
            }
            else
            {
                motor1(00);
                motor2(00);
            }
        }
    }
    return 0;
}
