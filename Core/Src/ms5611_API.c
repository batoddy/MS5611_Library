/*
 * ms5611_API.c
 *
 *  Created on: May 13, 2023
 *      Author: Batuhan
 */
#include "main.h"
#include "ms5611_API.h"
#include "math.h"
#define MS5611_ADRESS_LOW 0x77 // CSB pini GND'ye bağlıysa
// #define MS5611_ADRESS_HIGH 0x76 // CSB pini VDD'ye bağlıysa

// #define USE_OF_FREERTOS 1 /*
#define USE_OF_FREERTOS 0	  * */

//TIM_HandleTypeDef htim6;
#define TIMER6 &htim6

extern I2C_HandleTypeDef hi2c1;
#define MS5611_I2C &hi2c1

uint16_t timer_val = 0;
uint16_t caliber_value;
Altitude altitude_;

uint8_t barometer_calibration_data_8bit[2];
uint16_t barometer_calibration_data_16bit[6];

uint8_t raw_pressure_data_8[3];
uint8_t raw_temperature_data_8[3];

uint32_t D1, D2, dT, TEMP, P;
uint64_t OFF, SENS;
uint16_t SENS2, OFF2, TCS, TCO, T2, TEMPSENS;

void ms5611_Delay(uint32_t time){	// timer delay ms
	TIM6->CNT=0x00;
	timer_val = TIM6->CNT;
	while(timer_val >= time);
}

void ms5611_i2c_write(uint8_t cmd)
{
	HAL_I2C_Master_Transmit(MS5611_I2C, MS5611_ADRESS_LOW << 1, &cmd, 1, 100);
}

void ms5611_i2c_read(uint8_t cmd, uint8_t *data)
{
//	static uint8_t i;
	HAL_I2C_Master_Transmit(MS5611_I2C, MS5611_ADRESS_LOW << 1, &cmd, 1, 100);
	ms5611_Delay(2);
	HAL_I2C_Master_Receive(MS5611_I2C, MS5611_ADRESS_LOW << 1 | 0x01, data, 1, 100);
}
/*--------------Interrupt Functions---------------*/
void barometer_pressure_IT(){
//	static uint64_t i;
	ms5611_i2c_write(MS5611_D1_4096);

	ms5611_Delay(10); //for 1024 4ms || for 2048 6ms || for 4096 10ms

	HAL_I2C_Master_Transmit(MS5611_I2C, MS5611_ADRESS_LOW << 1, MS5611_ADC, 1, 100);
	ms5611_Delay(2);
	HAL_I2C_Master_Receive_IT(MS5611_I2C, MS5611_ADRESS_LOW << 1 | 0x01, raw_pressure_data_8, 3);
}
void barometer_temperature_IT(){
//	static uint64_t i;
	ms5611_i2c_write(MS5611_D2_4096);

	ms5611_Delay(10); //for 1024 4ms || for 2048 6ms || for 4096 10ms

	HAL_I2C_Master_Transmit(MS5611_I2C, MS5611_ADRESS_LOW << 1, MS5611_ADC, 1, 100);
	ms5611_Delay(2);
	HAL_I2C_Master_Receive_IT(MS5611_I2C, MS5611_ADRESS_LOW << 1 | 0x01, raw_temperature_data_8, 3);
}
/*-------------------------------------------------*/
void ms5611_reset()
{
	ms5611_i2c_write(MS5611_RESET);
	HAL_Delay(10);
}

void read_calibration_data_from_PROM()
{
	static uint8_t counter_1, counter_2;

	for (counter_1 = 0; counter_1 <= 6; counter_1++)
	{
		for (counter_2 = 0; counter_2 < 2; counter_2++)
		{
			ms5611_i2c_read(MS5611_PROM_1 + counter_1, &barometer_calibration_data_8bit[counter_2]);
		}
		barometer_calibration_data_16bit[counter_1] = (barometer_calibration_data_8bit[0] << 8) | barometer_calibration_data_8bit[0];
	}
	SENS2 = barometer_calibration_data_16bit[0];
	OFF2 = barometer_calibration_data_16bit[1];
	TCS = barometer_calibration_data_16bit[2];
	TCO = barometer_calibration_data_16bit[3];
	T2 = barometer_calibration_data_16bit[4];
	TEMPSENS = barometer_calibration_data_16bit[5];
}

void ms5611_config()
{
	ms5611_reset();

	HAL_Delay(20);

	read_calibration_data_from_PROM();

}

void get_base_pressure(){
	static uint8_t counter;
		for(counter = 0; counter <= caliber_value;counter++){
			ms5611_read_pressure_polling(&altitude_);
			altitude_.base_pressure += altitude_.pressure;
			HAL_Delay(2);
		}
		altitude_.base_pressure /= caliber_value;
}

void init_Barometer(){
	ms5611_config();

	get_base_pressure();

	barometer_pressure_IT();
}

void ms5611_read_pressure_IT(Altitude *altitude)
{
	D1 = (raw_pressure_data_8[0] << 8) | (raw_pressure_data_8[0] << 8) | raw_pressure_data_8[0];
	D2 = (raw_temperature_data_8[0] << 8) | (raw_temperature_data_8[0] << 8) | raw_temperature_data_8[0];

	dT = D2 - ((int32_t)barometer_calibration_data_16bit[5] << 8);
	TEMP = (2000 + (((int64_t)dT * (int64_t)barometer_calibration_data_16bit[6]) >> 23)); // temperature before second order compensation

	if (TEMP < 2000) // if temperature of the sensor goes below 20°C, it activates "second order temperature compensation"
	{
		T2 = pow(dT, 2) / 2147483648;
		OFF2 = 5 * pow((TEMP - 2000), 2) / 2;
		SENS2 = 5 * pow((TEMP - 2000), 2) / 4;
		if (TEMP < -1500) // if temperature of the sensor goes even lower, below -15°C, then additional math is utilized
		{
			OFF2 = OFF2 + 7 * pow((TEMP + 1500), 2);
			SENS2 = SENS2 + 11 * pow((TEMP + 1500), 2) / 2;
		}
	}
	else
	{
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}
	TEMP = ((2000 + (((int64_t)dT * (int64_t)barometer_calibration_data_16bit[6]) >> 23)) - T2);							   // second order compensation included
	OFF = (((unsigned int)barometer_calibration_data_16bit[2] << 16) + (((int64_t)barometer_calibration_data_16bit[4] * dT) >> 7) - OFF2);   // second order compensation included
	SENS = (((unsigned int)barometer_calibration_data_16bit[1] << 15) + (((int64_t)barometer_calibration_data_16bit[3] * dT) >> 8) - SENS2); // second order compensation included
	P = (((D1 * SENS) >> 21) - OFF) >> 15;
	altitude->pressure = (float)P/100;
	altitude->temperature = (float)TEMP/100;
}
void ms5611_read_pressure_polling(Altitude *altitude)
{
	static uint8_t counter;
	ms5611_i2c_write(MS5611_D1_4096);
	HAL_Delay(25);
	for(counter=0;counter<3;counter++){
		ms5611_i2c_read(MS5611_ADC,&raw_pressure_data_8[counter]);
	}
	D1 = (raw_pressure_data_8[0] << 8) | (raw_pressure_data_8[0] << 8) | raw_pressure_data_8[0];

	ms5611_i2c_write(MS5611_D2_4096);
	HAL_Delay(25);
	for(counter=0;counter<3;counter++){
		ms5611_i2c_read(MS5611_ADC,&raw_temperature_data_8[counter]);
	}
	D2 = (raw_temperature_data_8[0] << 8) | (raw_temperature_data_8[0] << 8) | raw_temperature_data_8[0];

	dT = D2 - ((int32_t)barometer_calibration_data_16bit[5] << 8);
	TEMP = (2000 + (((int64_t)dT * (int64_t)barometer_calibration_data_16bit[6]) >> 23)); // temperature before second order compensation

	if (TEMP < 2000) // if temperature of the sensor goes below 20°C, it activates "second order temperature compensation"
	{
		T2 = pow(dT, 2) / 2147483648;
		OFF2 = 5 * pow((TEMP - 2000), 2) / 2;
		SENS2 = 5 * pow((TEMP - 2000), 2) / 4;
		if (TEMP < -1500) // if temperature of the sensor goes even lower, below -15°C, then additional math is utilized
		{
			OFF2 = OFF2 + 7 * pow((TEMP + 1500), 2);
			SENS2 = SENS2 + 11 * pow((TEMP + 1500), 2) / 2;
		}
	}
	else
	{
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}
	TEMP = ((2000 + (((int64_t)dT * (int64_t)barometer_calibration_data_16bit[6]) >> 23)) - T2);							   // second order compensation included
	OFF = (((unsigned int)barometer_calibration_data_16bit[2] << 16) + (((int64_t)barometer_calibration_data_16bit[4] * dT) >> 7) - OFF2);   // second order compensation included
	SENS = (((unsigned int)barometer_calibration_data_16bit[1] << 15) + (((int64_t)barometer_calibration_data_16bit[3] * dT) >> 8) - SENS2); // second order compensation included
	P = (((D1 * SENS) >> 21) - OFF) >> 15;
	altitude->pressure = (float)P/100;
	altitude->temperature = (float)TEMP/100;
}

float calculate_Altitude(float pressure, float base_pressure){
    pressure /= 100;
    base_pressure /= 100;
    float alt = (44330 * (1.0 - pow(pressure / base_pressure, 0.1903)));
    return alt;
}

void read_Altitude(Altitude *altitude, Velocity *velocity){
	ms5611_read_pressure_IT(altitude);
	altitude->altitude = /*kalman filter*/calculate_Altitude(altitude->pressure,altitude_.base_pressure);
	// maximumu altitude
	    if (altitude->max_altitude < altitude->temp_altitude)
	    {
	        altitude->max_altitude = altitude->temp_altitude;
	    }
	    altitude->temp_altitude = altitude->altitude;

	    // vertical velocity m/s
	    velocity->time_diff = (HAL_GetTick() - velocity->prev_time) / 1000.00;
	    if (velocity->time_diff > 1.00)
	    {
	        velocity->prev_time = HAL_GetTick();
	        /*  (+) ise cikiyorsun (-) ise iniyorsun  */
	        // 									Bura santimetredir .
	        velocity->velocity = (altitude->altitude - altitude->temp_altitude_for_velocity) / (velocity->time_diff);
	    }
}
