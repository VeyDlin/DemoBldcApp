#include <Application.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_dma.h"

#include "stm32f1xx_ll_exti.h"

using namespace BSP;

BLDC Application::bldc;



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc->Instance == ADC1) {

    }
}






void Application::Init() {

	// Board
	BoardSupportPackage::Init();


	auto bldcParam = BLDC::Parameters();
	bldcParam.baseFrequency = 200;
	bldcParam.resolverStepsPerTurn = 4096;
	bldcParam.motorPoles = 96;
	bldcParam.sampingPeriod = 0.001 / 5;
	bldcParam.speedLoopPrescaler = 10;
	bldcParam.alignCountStart = 20000;
	bldcParam.halfPeriodMax = 100;
	bldcParam.motorRunType = BLDC::MotorRunType::Type6;

	bldc = BLDC(bldcParam);

	bldc.enable = {ENABLE_GPIO_Port, ENABLE_Pin};

	bldc.enableChannelA = {EN_CH1_GPIO_Port, EN_CH1_Pin};
	bldc.enableChannelB = {EN_CH2_GPIO_Port, EN_CH2_Pin};
	bldc.enableChannelC = {EN_CH3_GPIO_Port, EN_CH3_Pin};

	bldc.pwmTimer = &Peripheral::htim1;
	bldc.pwmChannelA = TIM_CHANNEL_1;
	bldc.pwmChannelB = TIM_CHANNEL_2;
	bldc.pwmChannelC = TIM_CHANNEL_3;

	bldc.Init();

	bldc.SetState(BLDC::MotorStateType::Run);

	HAL_ADCEx_Calibration_Start(&Peripheral::hadc1);
	HAL_ADC_Start(&Peripheral::hadc1);

    {
    	HAL_ADC_PollForConversion(&Peripheral::hadc1, 100);
    	uint16 adcRawA = HAL_ADC_GetValue(&Peripheral::hadc1);

    	HAL_ADC_PollForConversion(&Peripheral::hadc1, 100);
    	uint16 adcRawB = HAL_ADC_GetValue(&Peripheral::hadc1);

    	HAL_ADC_PollForConversion(&Peripheral::hadc1, 100);
    	uint16 adcRawC = HAL_ADC_GetValue(&Peripheral::hadc1);

    	HAL_ADC_PollForConversion(&Peripheral::hadc1, 100);
    	HAL_ADC_GetValue(&Peripheral::hadc1);

    	bldc.InitOffset(adcRawA, adcRawB, adcRawC);
    }



    while (1) {
    	bldc.UpdateEncoderAngle();

    	HAL_ADC_PollForConversion(&Peripheral::hadc1, 100);
    	uint16 adcRawA = HAL_ADC_GetValue(&Peripheral::hadc1);

    	HAL_ADC_PollForConversion(&Peripheral::hadc1, 100);
    	uint16 adcRawB = HAL_ADC_GetValue(&Peripheral::hadc1);

    	HAL_ADC_PollForConversion(&Peripheral::hadc1, 100);
    	uint16 adcRawC = HAL_ADC_GetValue(&Peripheral::hadc1);

    	HAL_ADC_PollForConversion(&Peripheral::hadc1, 100);
    	HAL_ADC_GetValue(&Peripheral::hadc1);

    	bldc.Tick(adcRawA, adcRawB, adcRawC);
    }
}








