/*
 * ms5611_API.h
 *
 *  Created on: May 13, 2023
 *      Author: Batuhan
 */

#ifndef INC_MS5611_API_H_
#define INC_MS5611_API_H_
#include "main.h"

#define MS5611_RESET   0XE1

#define MS5611_D1_256  0X40
#define MS5611_D1_512  0X42
#define MS5611_D1_1024 0X44
#define MS5611_D1_2048 0X46
#define MS5611_D1_4096 0X48
#define MS5611_D2_256  0X50
#define MS5611_D2_512  0X52
#define MS5611_D2_1024 0X54
#define MS5611_D2_2048 0X56
#define MS5611_D2_4096 0X58

#define MS5611_ADC 0x00
#define MS5611_PROM_1 0xA0

/*------VARIABLES---------*/
typedef struct Altitude
{
	float pressure;
	float base_pressure;
	float temperature;
	float altitude;
	float max_altitude;
	float temp_altitude;
	float temp_altitude_for_velocity;

} Altitude;

typedef struct Velocity
{
	float time_diff;
	float velocity;
	float prev_time;
} Velocity;

enum barometer_state{
	TEMPERATURE_CALLBACK = 0,
	PRESSURE_CALLBACK

};
/*------FUNCTIONS---------*/

void ms5611_config();

void barometer_pressure_IT();
void barometer_temperature_IT();

void init_Barometer();
void ms5611_read_pressure_polling(Altitude *altitude);
void ms5611_read_pressure_IT(Altitude *altitude);
void read_Altitude(Altitude *altitude, Velocity *velocity);
#endif /* INC_MS5611_API_H_ */
