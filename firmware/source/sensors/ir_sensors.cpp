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
IRSensors::IRSensors
    (
        robot_identifier_t current_robot, // The robot that is curently being used
        IRsensor_offset_t sensor_offsets  // The offset of each sensor to the center of the robot
    )
{
    this_robot = current_robot;

    offsets[sensor_id_left]     = sensor_offsets.side_offset;
    offsets[sensor_id_front_nw] = sensor_offsets.diagnal_offset;
    offsets[sensor_id_front]    = sensor_offsets.front_offset;
    offsets[sensor_id_front_ne] = sensor_offsets.diagnal_offset;
    offsets[sensor_id_right]    = sensor_offsets.side_offset;

    switch( this_robot )
    {
        case BABY_KITTEN: ADCx = ADC1;
                          emiter_gpio = GPIOA;
                          emiter_pins = GPIO_Pin_0
                                      | GPIO_Pin_1
                                      | GPIO_Pin_2
                                      | GPIO_Pin_3
                                      | GPIO_Pin_4;
                          break;

        case POWERLION:   ADCx = ADC1;
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
    memset( this->rolling_average, 0, number_of_sensors * sizeof( *rolling_average ) );

} // IRSensors()

/*****************************************************************************
* Function: isCalibrating
*
* Description: Returns whether or not the sensors are calibrating
*****************************************************************************/
bool IRSensors::isCalibrating( void )
{
    return calibrating;
}

/*****************************************************************************
* Function: Calibrate
*
* Description: Reads many right sensor readings and takes the average of them
*****************************************************************************/
float IRSensors::Calibrate( void )
{
    int i;
    float average_distance;

    // Keep timer from interfering with calibration
    // reading data outside of timers is done to speed up calibration
    calibrating = true;
    {
        ADCx->CR2 |= (uint32_t)ADC_CR2_SWSTART;
        calibration_data_ready = false;
        while( !calibration_data_ready ) {};
        for( i = 0; i<number_of_sensors; i++ )
        {
            ambiant_light[i] = raw_readings[i];
        }
        emiter_gpio->ODR |= emiter_pins;

        // Wait some time for emitters to come on
        for( i = 0; i < 1000; i++ )
        {
            asm( "nop" );
        }

        for( i = 0; i< 250; i++ )
        {
            calibration_data_ready = false;
            ADCx->CR2 |= (uint32_t)ADC_CR2_SWSTART;
            while( !calibration_data_ready ) {};
            average_distance += ( raw_readings[sensor_id_right] - ambiant_light[sensor_id_right] );
        }
        emiter_gpio->ODR &= ~emiter_pins;
    }
    calibrating = false;

    average_distance /= 250.0f;
    average_distance *= ADC_TO_MV;
    average_distance = 1.0f/average_distance;

    return( -269032.0f * average_distance * average_distance + 4556.5 * average_distance + 0.9883 + offsets[sensor_id_right] );
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

        case POWERLION:   // Enable clocks
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
float IRSensors::ReadDistance( sensor_id_t index )
{
    float inv_x = 1.0f /(rolling_average[ index ] * ADC_TO_MV);

    switch( this_robot )
    {
        case BABY_KITTEN: // Temporary fallthrough

        case POWERLION:   return( -269032.0f * inv_x * inv_x + 4556.5 * inv_x + 0.9883 + offsets[index] );

        default: break;
    }

    return -1.0f;

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
    if( !calibrating )
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
    }
    else
    {
        calibration_data_ready = true;
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

        case POWERLION:   DMA_InitStructure.DMA_Channel = DMA_Channel_0;
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

        case POWERLION:   channels[0] = ADC_Channel_14; //Front sensor channels
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
        ADC_RegularChannelConfig(ADCx, channels[i], i+1, ADC_SampleTime_480Cycles);
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

        case POWERLION:   // Enable timer global interrupt
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
        if( !all_irs[i]->isCalibrating() )
        {
            all_irs[i]->StartRead();
        }
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
