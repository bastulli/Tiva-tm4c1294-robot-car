#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "inc/hw_gpio.h"

#define PWM_FREQUENCY 100

int get_distance(volatile uint32_t num1);

int main(void)
{
    //Clock and Timing Vars
    volatile uint32_t ui32Load;
    volatile uint32_t ui32PWMClock;
    volatile uint32_t ui32SysClkFreq;

    //ADC sensor Vars
    uint32_t ui32ACCValues[4];
    volatile uint32_t ui32SensorR;
    volatile uint32_t ui32SensorM;
    volatile uint32_t ui32SensorL;

    //Clock frequency
    ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),120000000);

    //GPIO for PWM, ADC, I/O, Button
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    //Delay to make sure peripheral fully started
    SysCtlDelay(10);

    //Assign pins used
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);

    //Write to I/O pins for enable and motor direction
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 1);
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, 0);
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_2, 4);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, 8);

    //Set the PWM clock
    PWMClockSet(PWM0_BASE,PWM_SYSCLK_DIV_64);

    // PWM PF1, motor A (PIN1) & PF2 motor B (PIN2)
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2);
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

    // PWM clk and frequency
    ui32PWMClock = ui32SysClkFreq / 64;  // 120MHz/64 == 1875000
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1; // 1875000/100 == 18750 (18750 is PWM range. Example set PWM to 0-18750 with 18750 being the highest duty cycle)

    //Set first PWM to countdown with no sync
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_RUN);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32Load);

    //Set second PWM to countdown with no sync
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_RUN);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ui32Load);


    // PWM 1 and 2
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, ui32Load/2);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ui32Load/2);

    //Set PWM Bits to true
    PWMOutputState(PWM0_BASE, (PWM_OUT_1_BIT | PWM_OUT_2_BIT), true);

    //PWM Enable Generators
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    //ADC sequence set to ss3.  Enable CH1, CH2, CH3 and Enable ADC0
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH1|ADC_CTL_IE|ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 1);

    //Placeholder int vars for sensor data and statemachine
    int PWM_Sesnor_R;
    int PWM_Sesnor_M;
    int PWM_Sesnor_L;
    int state = 0;

    while(1)
    {
        //Set state to 1 if button press and state == 0
        if (state == 0 && GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == 0)
        {
            state = 1;
            //debounce
            SysCtlDelay(10000000);
        }else if (state == 1 && GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == 0) //Set state to 0 if button press and state == 1
        {
            state = 0;
            //debounce
            SysCtlDelay(10000000);
        }else{
            //Do nothing
            state = state;
        }

        //If state is == 1 run logic on ADC values and drive motors
        if (state == 1){

            //Clear, trigger, and get voltage ADC values
            ADCIntClear(ADC0_BASE, 1);
            ADCProcessorTrigger(ADC0_BASE, 1);
            ADCSequenceDataGet(ADC0_BASE, 1, ui32ACCValues);

            //Enable Hbridge
            GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_2, 4);

            //Set each sensor var from ADC arr
            ui32SensorR = ui32ACCValues[0];
            ui32SensorM = ui32ACCValues[1];
            ui32SensorL = ui32ACCValues[2];

            //Convert voltage to distance in cm and apply function to maximize PWM range
            PWM_Sesnor_R = get_distance(ui32SensorR);
            PWM_Sesnor_M = get_distance(ui32SensorM);
            PWM_Sesnor_L = get_distance(ui32SensorL);

            if (PWM_Sesnor_M <= 4500){

                //Spin around!
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 1);
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, 0);
                GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 4);
                GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, 0);

                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 18750);
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 18750);

                SysCtlDelay(10000000);

            }else if (PWM_Sesnor_R <= 5500 && PWM_Sesnor_L >= PWM_Sesnor_R){
                //Turn Left!
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 1);
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, 0);
                GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0);
                GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, 8);

                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 18750);
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 1000);

                SysCtlDelay(4000000);

            }else if (PWM_Sesnor_L <= 5500 && PWM_Sesnor_R >= PWM_Sesnor_L){
                //Turn Right!
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 1);
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, 0);
                GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0);
                GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, 8);

                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1000);
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 18750);

                SysCtlDelay(4000000);

            }else{
                //Forward!
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 1);
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, 0);
                GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0);
                GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, 8);

                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 18750);
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 18750);

            }

            SysCtlDelay(200000);

        }else{
            // Set PWM to 1 and disable Hbridge
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 1);
            GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_2, 0);
        }

    }
}

//Convert ADC voltage from Sharp GP2Y0A21YK sensor
int get_distance(volatile uint32_t num1)
{

    int result;
    float volts = num1*0.00040283203;  // value from sensor * (1.65/4096)
    int distance = 13*pow(volts, -1); // worked out from datasheet graph
    int var = distance;

    //Set distance proportional to PWM.  Example, if sensor reads 7-80 cm, apply the same linear function to PWM range 0 - 18750 via y = x*255+2000
    int var2 = (225 * var) + 2000;

    if(var2 >= 18750){

        result = 18750;
    }
    else if(var2 <= 0){
        result = 0;
    }
    else{

        result = var2;
    }

    // 0 is stop, 18750 is full speed/power.
    return result;
}
