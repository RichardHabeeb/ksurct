/****************************************************************************************
* File: ir_sensors.cpp
*
* Description: Implementation of IRSensors class that provides functionality for inferred
*              sensors.
*
* Created: 3/29/2014, by Matthew Olson
****************************************************************************************/

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <cstring>

#include "ir_sensors.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "timer_interrupt_oc.h"

/*---------------------------------------------------------------------------------------
*                                   LITERAL CONSTANTS
*--------------------------------------------------------------------------------------*/

#define ADC_APLHA               1.0f    // APLHA value for rolling average
#define ACD_TO_CM               1.0f    // TEMP const for changing adc values to cm
#define MAX_CALLBACK_SENSORS    1       // Only supporting one instance currently for ease
                                        // Pin and ADC/DMA definitions in header must be
                                        // reworked to have multiple instances

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                   MEMORY CONSTANTS
---------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                      VARIABLES
*--------------------------------------------------------------------------------------*/

// Instantiate output compare timer to step the motors (still need to be initialized)
static TimerInterruptOC timer(TIM4, TIM4_IRQn, RCC_APB1Periph_TIM4, (SystemCoreClock/4), 50000);

// References to all ir classes that have been instantiated for interupt callbacks
static IRSensors* all_irs[MAX_CALLBACK_SENSORS];

 // Count of all instances of ir_sensors
static uint8_t ir_count = 0;

/*---------------------------------------------------------------------------------------
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

static void SetInterruptCallback( IRSensors* );
static void TimerCallback( void );

/*****************************************************************************
* Function: IRSensors - Constructor
*
* Description: Initializes fields for new instance of IRSensors object, sets up
*              ADC with DMA for the sensors.
*****************************************************************************/
IRSensors::IRSensors()
{
    emiter_gpio = IR_EMITER_GPIO;
    emiter_pins = IR_EMITER_PINS;
    reading_dist = false;
    memset( this->raw_readings, 0, number_of_sensors * sizeof( *raw_readings ) );
    memset( this->ambiant_light, 0, number_of_sensors * sizeof( *ambiant_light ) );
    memset( this->rolling_average, 0, number_of_sensors * sizeof( *rolling_average ) );

} // IRSensors()


/*****************************************************************************
* Function: Init
*
* Description: Initializes hardware for ADC, DMA, and interupts
*****************************************************************************/
void IRSensors::Init( void )
{
    // Enable clocks
    RCC_AHB1PeriphClockCmd( IR_EMITER_AHBPERIPH_GPIO, ENABLE );
    RCC_AHB1PeriphClockCmd( IR_COLLECTOR_AHBPERIPH_GPIO, ENABLE );
    RCC_APB2PeriphClockCmd( IR_ADC_APB, ENABLE );
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2, ENABLE );

    InitInterrupts();

    GPIO_InitTypeDef  GPIO_InitStructure;
    // Set up emitter IO
    GPIO_InitStructure.GPIO_Pin     = IR_EMITER_PINS;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_Init(IR_EMITER_GPIO, &GPIO_InitStructure);

    // Set up collector IO
    GPIO_InitStructure.GPIO_Pin     = IR_COLLECTOR_PINS;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_Init(IR_COLLECTOR_GPIO, &GPIO_InitStructure);

    InitDMA();
    InitADC();

    // Wait some time for stuff to get started before starting interupts
    for( int i = 0; i < 10000; i++ )
    {
        asm( "nop" );
    }

    SetInterruptCallback( this );

} // Init()

/*****************************************************************************
* Function: ReadDistance
*
* Description: Reads the distance measured by the sensor indicated in cm
*****************************************************************************/
float IRSensors::ReadDistance( sensor_id_t index )
{
    //MTO - not sure why but array is backwords....
    return( rolling_average[ number_of_sensors - 1 - index ] * ACD_TO_CM );

} // ReadDistance


/*****************************************************************************
* Function: StartRead
*
* Description: Starts the ADC read and turns on the emiter_pins every other
*               read
*****************************************************************************/
void IRSensors::StartRead( void )
{
    // When data is acumulated reading_dist is toggled
    if( reading_dist )
    {
        // Turn on the emitters
        emiter_gpio->ODR |= emiter_pins;
    }

    // Wait some time for emitters to come on
    for( int i = 0; i < 1000; i++ )
    {
        asm( "nop" );
    }

    //Start conversion of regular channel group
    IR_ADC->CR2 |= (uint32_t)ADC_CR2_SWSTART;

} // StartRead

/*****************************************************************************
* Function: AcumulateData
*
* Description: This function acumulates the new data into the rolling average
*              and turns off the emiters if it is trying to read a distance.
*              If it is not, it reads the ambiant light present.
*****************************************************************************/
void IRSensors::AcumulateData( void )
{
    for( int i = 0; i<number_of_sensors; i++ )
    {
        if( reading_dist )
        {
            rolling_average[i] = (raw_readings[i]-ambiant_light[i]) * ADC_APLHA + rolling_average[i] * ( 1 - ADC_APLHA );
            emiter_gpio->ODR &= ~emiter_pins; // Turn off emitters
        }
        else
        {
            ambiant_light[i] = raw_readings[i];
        }
    }
    reading_dist = !reading_dist;

} // AcumulateData()

/*****************************************************************************
* Function: InitDMA
*
* Description: Initializes the DMA for use with the ADC
*****************************************************************************/
void IRSensors::InitDMA()
{
    DMA_InitTypeDef 	DMA_InitStructure;

    DMA_InitStructure.DMA_Channel             = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr  =	(uint32_t)&(IR_ADC->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr     =	(uint32_t)raw_readings;
    DMA_InitStructure.DMA_DIR                 =	DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize          =	number_of_sensors;
    DMA_InitStructure.DMA_PeripheralInc       =	DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc           =	DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize  =	DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize      =	DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode                =	DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority            =	DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode            =	DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold       =	DMA_FIFOThreshold_Full; // This doesn't matter b/c disable
    DMA_InitStructure.DMA_MemoryBurst         =	DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst     =	DMA_PeripheralBurst_Single;

    DMA_Init( DMA2_Stream0, &DMA_InitStructure );
    DMA_Cmd( DMA2_Stream0, ENABLE );

    // Set up transfer error interupt
    DMA_ITConfig( DMA2_Stream0, DMA_IT_TE, ENABLE);

} // InitDMA()


/*****************************************************************************
* Function: InitADC
*
* Description: Initializes the ADC hardware
*****************************************************************************/
void IRSensors::InitADC( void )
{
    ADC_InitTypeDef       ADC_InitStructure;

    ADC_DeInit();

    /* Enable clocks ****************************************/
    ADC_CommonInitTypeDef ADC_Common_InitStructure;
    ADC_Common_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_Common_InitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_Common_InitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_Common_InitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
    ADC_CommonInit( &ADC_Common_InitStructure );

    /* ADC Init **************************************************************/
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = number_of_sensors;

    ADC_Init(IR_ADC, &ADC_InitStructure);

    /* ADC regular channels configuration *************************************/
    uint8_t channels[] = IR_CHANNELS;
    for( uint8_t i = 0; i < number_of_sensors; i++ )
    {
        ADC_RegularChannelConfig(IR_ADC, channels[i], i+1, ADC_SampleTime_480Cycles);
    }

    ADC_EOCOnEachRegularChannelCmd(IR_ADC, DISABLE);
    ADC_ContinuousModeCmd( IR_ADC, DISABLE );
    ADC_DiscModeChannelCountConfig( IR_ADC, number_of_sensors );
    ADC_DiscModeCmd( IR_ADC, ENABLE );

    ADC_ITConfig( IR_ADC, ADC_IT_EOC, ENABLE );

    ADC_DMARequestAfterLastTransferCmd( IR_ADC, ENABLE);

    /* Enable ADC */
    ADC_DMACmd( IR_ADC, ENABLE );
    ADC_Cmd( IR_ADC, ENABLE);

} // InitADC()


/*****************************************************************************
* Function: InitInterrupts
*
* Description: Initializes the interrupts for both ADC and DMA
*****************************************************************************/
void IRSensors::InitInterrupts( void )
{
    NVIC_InitTypeDef NVIC_InitStructure;

    // Enable timer global interrupt
    NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

} // InitInterrupts()


/*****************************************************************************
* Function: SetInterruptCallback
*
* Description: Saves a reference to the instance of the passed in class so
*              that it can be called in the interupt
*             NOTE: This will set up internal interupt callback when
*                   timer_interrupt_oc increases support
*****************************************************************************/
static void SetInterruptCallback( IRSensors* sensor )
{
    if( ir_count >= MAX_CALLBACK_SENSORS ) { return; }

    timer.InitChannel( oc_channel_1, 200, TimerCallback );
    all_irs[ ir_count ] = sensor;
    ir_count++;

} // SetInterruptCallback()


/*****************************************************************************
* Function: TimerCallback
*
* Description: Callback for the timmer interupt, this starts reading distances
*              for each instance of ir sensors
*             NOTE: This will eventually be called internally through
*                   timer_interrupt_oc but currently all channels are taken
*                   up by motors class, need support for more TIM ohannels to work.
*                   For now call this from a timmer interupt
*****************************************************************************/
static void TimerCallback(void)
{
    for( int i = 0; i < ir_count; i++ )
    {
        all_irs[i]->StartRead();
    }

} // TimerCallback()


/*****************************************************************************
* Function: ADC_IRQHandler
*
* Description: IRQ Handler for all ADC, it is set up to be ran on the end of
*              a regular group conversion. (All channels setup on a single ADC)
*              This means new data is available so it is acumulated into the
*              ir instances.
*****************************************************************************/
extern "C" void ADC_IRQHandler( void )
{
    for( int i = 0; i < ir_count; i++ )
    {
        all_irs[i]->AcumulateData();
    }

} // ADC_IRQHandler()


/*****************************************************************************
* Function: DMA2_Stream0_IRQHandler
*
* Description: IRQ Handler for DMA2 Stream 0, it is set up to be ran on
*              failure of a DMA transfer so DMA needs to be reinitialized
*****************************************************************************/
extern "C" void DMA2_Stream0_IRQHandler( void )
{
    for( int i = 0; i < ir_count; i++ )
    {
        all_irs[i]->InitDMA();
    }

} // DMA2_Stream0_IRQHandler()
