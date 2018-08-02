
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/* Timer_A Continuous Mode Configuration Parameter */
const Timer_A_UpModeConfig upModeConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,            // ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,       // ACLK/1 = 32Khz
        8192,
        TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer ISR
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE, // Disable CCR0
        TIMER_A_DO_CLEAR                     // Clear Counter
};

/* Timer_A Compare Configuration Parameter */
const Timer_A_CompareModeConfig compareConfig =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_1,          // Use CCR1
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_SET_RESET,               // Toggle output but
        8192                                       // 16000 Period
};

/* Statics */
static volatile uint_fast16_t resultsBuffer[UINT8_MAX];
static volatile uint_fast16_t MovAvgBuffer[UINT8_MAX];
static volatile uint8_t resPos;
static volatile uint8_t Pos;
static volatile uint16_t result;

uint8_t i, ADC_val_true;

int main(void)
{
    /* Halting WDT  */
    MAP_WDT_A_holdTimer();
    MAP_Interrupt_enableSleepOnIsrExit();
    resPos = 0;
    Pos = 0;

    /* Setting up clocks
     * MCLK = MCLK = 3MHzReferenceVoltage
     * ACLK = REFO = 32Khz */
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Initializing ADC (MCLK/1/1) */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1,0);

    /* Configuring GPIOs (5.5 A0) */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5,
    GPIO_TERTIARY_MODULE_FUNCTION);

    /* Configuring ADC Memory */
    MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS,
    ADC_INPUT_A0, false);

    /* Configuring Timer_A in continuous mode and sourced from ACLK */
    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upModeConfig);

    /* Configuring Timer_A0 in CCR1 to trigger at 16000 (0.5s) */
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig);

    /* Configuring the sample trigger to be sourced from Timer_A0  and setting it
     * to automatic iteration after it is triggered*/
    MAP_ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE1, false);

    /* Enabling the interrupt when a conversion on channel 1 is complete and
     * enabling conversions */
    MAP_ADC14_enableInterrupt(ADC_INT0);
    MAP_ADC14_enableConversion();

    /* Enabling Interrupts */
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();

    /* setting-up CS pin */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN4);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4);


    /* Starting the Timer */
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    /* Going to sleep */
    while (1)
    {

    }
}

/* This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM0 */
void ADC14_IRQHandler(void)
{
    uint64_t status;

    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    if (status & ADC_INT0)
    {
        ADC_val_true = 1;
        if(resPos == UINT8_MAX)
        {
           resPos = 0;
        }

        resultsBuffer[resPos++] = MAP_ADC14_getResult(ADC_MEM0);
    }
    else
    {
        ADC_val_true = 0;
    }

}
