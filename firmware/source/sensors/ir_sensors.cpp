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
#include "led_references.h"

#include <cstring>
#include <cmath>

#include "ir_sensors.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "timer_interrupt_oc.h"
#include "system_timer.h"

/*---------------------------------------------------------------------------------------
*                                   LITERAL CONSTANTS
*--------------------------------------------------------------------------------------*/

#define ADC_APLHA               0.9f    // APLHA value for rolling average
#define ADC_TO_MV               ( 1000 * 3.3f / 0x0FFF ) // Scale from 12 bit ADC value to mv
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

// Instantiate output compare timer to read sensors (still need to be initialized)
static TimerInterruptOC timer(TIM4, TIM4_IRQn, RCC_APB1Periph_TIM4, (SystemCoreClock/4), 50000);

// References to all ir classes that have been instantiated for interupt callbacks
static IRSensors* all_irs[MAX_CALLBACK_SENSORS];

 // Count of all instances of ir_sensors
static uint8_t ir_count = 0;

/*---------------------------------------------------------------------------------------
*                                     PROCEDURES
*--------------------------------------------------------------------------------------*/

static void SetInterruptCallback( IRSensors* );
static void StartAdcReadCallback( void );

/*****************************************************************************
* Function: IRSensors - Constructor
*
* Description: Initializes fields for new instance of IRSensors object, sets up
*              ADC with DMA for the sensors.
*****************************************************************************/
IRSensors::IRSensors
    (
        robot_identifier_t current_robot // The robot that is curently being used
    )
{
    this_robot = current_robot;

    memset( calibration_offsets, 0, sizeof calibration_offsets );

    switch( this_robot )
    {
        case BABY_KITTEN: ADCx = ADC1;
                          emiter_gpio = GPIOA;
                          emiter_pins = GPIO_Pin_0
                                      //| GPIO_Pin_1
                                      | GPIO_Pin_2
                                      //| GPIO_Pin_3
                                      | GPIO_Pin_4;
                          break;

        case POWER_LION:  ADCx = ADC1;
                          emiter_gpio = GPIOB;
                          emiter_pins = GPIO_Pin_11
                                      | GPIO_Pin_12
                                      | GPIO_Pin_13
                                      | GPIO_Pin_14
                                      | GPIO_Pin_15;
                          break;

        default: break;
    }
    reading_dist = false;
    memset( this->raw_readings, 0, number_of_sensors * sizeof( *raw_readings ) );
    memset( this->ambiant_light, 0, number_of_sensors * sizeof( *ambiant_light ) );
    memset( this->adc_readings, 0, number_of_sensors * sizeof( *adc_readings ) );
    memset( this->summed_readings, 0, number_of_sensors * sizeof( *summed_readings ) );

    current_summed_count = 0;

    // Temporarily hard code
    number_of_samples_to_average = 50;

} // IRSensors()

/*****************************************************************************
* Function: CalibrateSensor
*
* Description: Calculates difference between measured distance and the specified
*              known distance that sensor should be at.  This error offset will be
*              calculated and added in for every future reading of the sensor.
*****************************************************************************/
void IRSensors::CalibrateSensor( sensor_id_t sensor_id, float known_distance )
{
    float total_reading = 0.f;

    double total_calibration_time = 1.0;
    const uint32_t sampling_count = 250;

    float delta_calibration_time = (float)( total_calibration_time / sampling_count );

    double start_calibration_time = system_timer.get_time();

    double current_time = start_calibration_time;

    double last_reading_time = 0;

    while( current_time < start_calibration_time + total_calibration_time )
    {
        current_time = system_timer.get_time();

        // Calculate time between last sampling.
        float delta_time = (float)(current_time - last_reading_time);

        if( delta_time > delta_calibration_time )
        {
            total_reading += ReadDistance( sensor_id );
            last_reading_time = current_time;
        }
    }

    float average_reading = total_reading / (float)sampling_count;

    calibration_offsets[sensor_id] = known_distance - average_reading;

}

/*****************************************************************************
* Function: Initialize
*
* Description: Initializes hardware for ADC, DMA, and interupts
*****************************************************************************/
void IRSensors::Initialize( void )
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    switch( this_robot )
    {
        case BABY_KITTEN: // Enable clocks
                          RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );
                          RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );
                          RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1, ENABLE );
                          RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2, ENABLE );

                          InitInterrupts();

                          // Set up collector IO
                          GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_5
                                                          | GPIO_Pin_6
                                                          | GPIO_Pin_7;
                          GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AN;
                          GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
                          GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
                          GPIO_Init( GPIOA, &GPIO_InitStructure);

                          // Set up collector IO
                          GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_4
                                                          | GPIO_Pin_5;
                          GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AN;
                          GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
                          GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
                          GPIO_Init(GPIOC, &GPIO_InitStructure);
                          break;

        case POWER_LION:  // Enable clocks
                          RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );
                          RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );
                          RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );
                          RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1, ENABLE );
                          RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2, ENABLE );

                          InitInterrupts();

                          // Set up collector IO
                          GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_4
                                                          | GPIO_Pin_5
                                                          | GPIO_Pin_6
                                                          | GPIO_Pin_7;
                          GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AN;
                          GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
                          GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
                          GPIO_Init( GPIOA, &GPIO_InitStructure);

                          // Set up collector IO
                          GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_4;
                          GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AN;
                          GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
                          GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
                          GPIO_Init(GPIOC, &GPIO_InitStructure);

        default: break;
    }

    // Set up emitter IO
    GPIO_InitStructure.GPIO_Pin     = emiter_pins;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_Init(emiter_gpio, &GPIO_InitStructure);

    InitDMA();
    InitADC();

    // Wait some time for voltages to stabilize before starting interupts
    for( int i = 0; i < 1000; i++ )
    {
        asm( "nop" );
    }

    SetInterruptCallback( this );

} // Initialize()

/*****************************************************************************
* Function: ReadDistance
*
* Description: Reads the distance measured by the sensor indicated in cm
*****************************************************************************/
float IRSensors::ReadDistance( sensor_id_t id )
{
    if( id == sensor_id_front_ne
     || id == sensor_id_front_nw )
    {
        return ( ConvertToDistance( adc_readings[ id ], id ) / 2.0f ) + calibration_offsets[ id ];
    }
    return ConvertToDistance( adc_readings[ id ], id ) + calibration_offsets[ id ];

} // ReadDistance


/*****************************************************************************
* Function: IsSaturated
*
* Description: Returns if the sensor is saturated by checking if the reading
*              is near the logic voltage.
*****************************************************************************/
bool IRSensors::IsSaturated( sensor_id_t sensor_id )
{
    return adc_readings[ sensor_id ] >= 3200.0f;

} // IsSaturated


/*****************************************************************************
* Function: StartRead
*
* Description: Starts the ADC read and turns on the emiter_pins every other
*               read
*****************************************************************************/
void IRSensors::StartRead( void )
{
    // Turn on the emitters
    emiter_gpio->ODR |= emiter_pins;

    // Start ADC conversion.
    ADCx->CR2 |= (uint32_t)ADC_CR2_SWSTART;

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
    ++current_summed_count;

    for( uint32_t i = 0; i < number_of_sensors; i++ )
    {
        summed_readings[i] += raw_readings[i];
    }

    if (current_summed_count == number_of_samples_to_average)
    {
        current_summed_count = 0;

        for( uint32_t i = 0; i < number_of_sensors; i++ )
        {
            adc_readings[i] = summed_readings[i] /  number_of_samples_to_average;
            summed_readings[i] = 0;
        }
    }

} // AcumulateData()

/*****************************************************************************
* Function: InitDMA
*
* Description: Initializes the DMA for use with the ADC
*****************************************************************************/
void IRSensors::InitDMA()
{
    DMA_InitTypeDef 	DMA_InitStructure;

    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)&(ADCx->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr     =	(uint32_t)raw_readings;
    DMA_InitStructure.DMA_DIR                 =	DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize          =	number_of_sensors;
    DMA_InitStructure.DMA_PeripheralInc       =	DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc           =	DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize  =	DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize      =	DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode                =	DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority            =	DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode            =	DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold       =	DMA_FIFOThreshold_Full; // This doesn't matter b/c disable
    DMA_InitStructure.DMA_MemoryBurst         =	DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst     =	DMA_PeripheralBurst_Single;

    switch( this_robot )
    {
        case BABY_KITTEN: // Intentional fallthrough

        case POWER_LION:  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
                          DMA_Init( DMA2_Stream0, &DMA_InitStructure );
                          DMA_Cmd( DMA2_Stream0, ENABLE );
                          // Set up transfer error interupt
                          DMA_ITConfig( DMA2_Stream0, DMA_IT_TE, ENABLE );
                          break;

        default: break;
    }

} // InitDMA()


/*****************************************************************************
* Function: InitADC
*
* Description: Initializes the ADC hardware
*****************************************************************************/
void IRSensors::InitADC( void )
{
    ADC_InitTypeDef       ADC_InitStructure;
    uint8_t channels[number_of_sensors];
    switch( this_robot )
    {
        case BABY_KITTEN: channels[0] = ADC_Channel_15;
                          channels[1] = ADC_Channel_14;
                          channels[2] = ADC_Channel_7;
                          channels[3] = ADC_Channel_6;
                          channels[4] = ADC_Channel_5;
                          break;

        case POWER_LION:  channels[0] = ADC_Channel_14; //Front sensor channels
                          channels[1] = ADC_Channel_7;
                          channels[2] = ADC_Channel_6;
                          channels[3] = ADC_Channel_5;
                          channels[4] = ADC_Channel_4;
                          break;

        default:          break;
    }


    ADC_DeInit();

    /* Enable clocks ****************************************/
    ADC_CommonInitTypeDef ADC_Common_InitStructure;
    ADC_Common_InitStructure.ADC_Mode               = ADC_Mode_Independent;
    ADC_Common_InitStructure.ADC_TwoSamplingDelay   = ADC_TwoSamplingDelay_5Cycles;
    ADC_Common_InitStructure.ADC_DMAAccessMode      = ADC_DMAAccessMode_Disabled;
    ADC_Common_InitStructure.ADC_Prescaler          = ADC_Prescaler_Div8;
    ADC_CommonInit( &ADC_Common_InitStructure );

    /* ADC Init **************************************************************/
    ADC_InitStructure.ADC_Resolution                = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode              = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode        = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge      = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv          = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_DataAlign                 = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion           = number_of_sensors;

    ADC_Init(ADCx, &ADC_InitStructure);

    /* ADC regular channels configuration *************************************/
    for( uint8_t i = 0; i < number_of_sensors; i++ )
    {
        ADC_RegularChannelConfig(ADCx, channels[i], i+1, ADC_SampleTime_84Cycles);
    }

    ADC_EOCOnEachRegularChannelCmd(ADCx, DISABLE);
    ADC_ContinuousModeCmd( ADCx, DISABLE );
    ADC_DiscModeChannelCountConfig( ADCx, number_of_sensors );
    ADC_DiscModeCmd( ADCx, ENABLE );

    ADC_ITConfig( ADCx, ADC_IT_EOC, ENABLE );

    ADC_DMARequestAfterLastTransferCmd( ADCx, ENABLE);

    /* Enable ADC */
    ADC_DMACmd( ADCx, ENABLE );
    ADC_Cmd( ADCx, ENABLE);

} // InitADC()


/*****************************************************************************
* Function: InitInterrupts
*
* Description: Initializes the interrupts for both ADC and DMA
*****************************************************************************/
void IRSensors::InitInterrupts( void )
{
    NVIC_InitTypeDef NVIC_InitStructure;

    switch( this_robot )
    {
        case BABY_KITTEN: // Intentional fallthrough

        case POWER_LION:  // Enable timer global interrupt
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
                          break;

        default: break;
    }


} // InitInterrupts()


/*****************************************************************************
* Function: ConvertToDistance
*
* Description: Returns distance in centimeters corresponding to specified ADC value.
*              No calibration offsets are added in.
*****************************************************************************/
inline float IRSensors::ConvertToDistance( float adc_value, sensor_id_t sensor_id )
{
    // Avoid division by zero.
    if( adc_value <= 0.f )
    {
        return 100.f;
    }

    // First convert reading to millivolts.
    float mv = adc_value * ADC_TO_MV;

    // Then take the inverse to be used in the mapping calculation.
    // 'imv' = Inverse Milli Volts
    float imv = 1.0f / mv;

    switch( this_robot )
    {
        case BABY_KITTEN:

            switch( sensor_id )
            {
                case sensor_id_right:
                    return 4266.4 * powf( mv, -0.825 );
                    break;
                case sensor_id_left:
                    return 4847.4 * powf( mv, -1.235 );
                    break;
                case sensor_id_front:
                    return 3974.4 * powf( mv, -0.842 );
                    break;
                default:
                    return -1.0f; // Not mapped yet.
            }

        case POWER_LION:
            return 361.93f * pow( mv, -0.6135f );

        default:
            return -1.0f;
    }

} // ConvertToDistance()


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

    timer.InitChannel( oc_channel_1, 1000, StartAdcReadCallback );
    all_irs[ ir_count ] = sensor;
    ir_count++;

} // SetInterruptCallback()

/*****************************************************************************
* Function: StartAdcReadCallback
*
* Description:
*****************************************************************************/
static void StartAdcReadCallback(void)
{
    for( int i = 0; i < ir_count; i++ )
    {
        all_irs[i]->StartRead();
    }

} // StartAdcReadCallback()

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
