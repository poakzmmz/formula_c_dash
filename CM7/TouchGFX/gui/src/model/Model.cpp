#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#include <cstring>

extern "C"
{
	#include "main.h"
	extern ADC_HandleTypeDef hadc1;
	extern ADC_HandleTypeDef hadc2;
	extern ADC_HandleTypeDef hadc3;
	extern TIM_HandleTypeDef htim4;
	#include "stm32h7xx.h"

	long map(long x, long in_min, long in_max, long out_min, long out_max)
	{
			return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
	}
}


Model::Model() : modelListener(0), ADC_VAL_2(50)
{

}
uint32_t lastDebounceTime = 0;
const uint32_t debounceDelay = 160;

uint32_t lastDebounceTime_2 = 0;
const uint32_t debounceDelay_2 = 160;

uint32_t lastDebounceTimeK6 = 0;
const uint32_t debounceDelayK6 = 50;

//static uint32_t last_tick = 0;
uint32_t interval = 250;

volatile int stopwatch_time = 0;  // stopwatch value
volatile bool running = false;         // stopwatch state
volatile int previous_time;
int Gearstate_num = 1;
bool N_state = false;

void Model::tick()
{

	/********** OIL **********/
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 10);
	int adc_value_2 = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	ADC_VAL_2 = map(adc_value_2, 0, 65535, 0 , 300);
	modelListener->setADC_2(ADC_VAL_2);

	/********** WATER **********/
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	int adc_value_1 = HAL_ADC_GetValue(&hadc1);
	int ADC_1 = map(adc_value_1, 0, 65535, 0 , 300);
	HAL_ADC_Stop(&hadc1);
	modelListener->setADC_3(ADC_1);

	/********** PROCESSOR TEMPERATURE **********/
	HAL_ADC_Start(&hadc3);
	HAL_ADC_PollForConversion(&hadc3, 10);
	uint32_t raw_Ptemperature = HAL_ADC_GetValue(&hadc3);
	HAL_ADC_Stop(&hadc3);
    int vref = 3300;
    int vsense = (raw_Ptemperature * vref) / 65535;  // 16-bit ADC (0~65535)
    int V25 = 760;
    int Avg_Slope = 25;
    int p_temperature = (V25 - vsense) / Avg_Slope + 25;  // 온도 계산
    modelListener->p_temp(p_temperature);


	/********** RPM **********/
	HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_2);
	int tim2_ch2_val = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2);
	HAL_TIM_IC_Stop(&htim4, TIM_CHANNEL_2);
	int rpm; // 나중에 조정
	modelListener->setRPM(rpm);


	/********** GEAR **********/
    if (HAL_GPIO_ReadPin(GPIOK, GPIO_PIN_6) == GPIO_PIN_RESET)
    {
    	if (HAL_GetTick() - lastDebounceTime > debounceDelay)
    	{
    		Gearstate_num++;

    		if(Gearstate_num == 7)
    	    {
    			Gearstate_num = 6;
    	    }

    	    lastDebounceTime = HAL_GetTick();
    	}
    }

	if (HAL_GPIO_ReadPin(GPIOK, GPIO_PIN_3) == GPIO_PIN_RESET)
	{
		if (HAL_GetTick() - lastDebounceTime_2 > debounceDelay_2)
			{
				Gearstate_num--;

				if(Gearstate_num == 0)
			    {
					Gearstate_num = 1;
			    }

				lastDebounceTime_2 = HAL_GetTick();
			}
	}
	modelListener->setGears (Gearstate_num);


	/********** NEUTRAL STATE **********/
	if (HAL_GPIO_ReadPin(GPIOC, button_Pin) == GPIO_PIN_SET)
	{
	    if (HAL_GetTick() - lastDebounceTimeK6 > debounceDelayK6)
	    {
	    	N_state = true;
	        lastDebounceTimeK6 = HAL_GetTick();
	    }
	}
	else N_state = false;
	modelListener->setNstate (N_state);


	/********** LAP TIME **********/
	static int lastTickTime = 0;
	int currentTickTime = HAL_GetTick();
	int deltaTime = currentTickTime - lastTickTime;
	lastTickTime = currentTickTime;
	if (HAL_GPIO_ReadPin(GPIOK, JOY_LEFT_Pin) == GPIO_PIN_RESET)
	{
		stopwatch_time = 0;
		running = 1;
		modelListener->updateprevtime(previous_time);
	}

	else if (HAL_GPIO_ReadPin(GPIOK, JOY_RIGHT_Pin) == GPIO_PIN_RESET)
	{
		running = 0;
		previous_time = stopwatch_time;
	}

	if (running)
	{
		stopwatch_time += deltaTime;
	}

	modelListener->updateStopwatch(stopwatch_time);



}
