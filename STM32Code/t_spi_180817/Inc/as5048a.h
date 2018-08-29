#ifndef __AS5048A_H__
#define __AS5048A_H__

// If used for FOC 
#define USED_FOR_FOC

#define AS5048A_ANGLE 0x3fff

#include "stm32f0xx_hal.h"

#ifdef USED_FOR_FOC
	#include "encoder_speed_pos_fdbk.h"
	#include "mc_irq_handler.h"

	#include "mc_library_isr_priority_conf.h"
	#include "mc_type.h"
#endif


float read_as5048a_deg(SPI_HandleTypeDef *hspi);

uint16_t read_as5048a_raw(SPI_HandleTypeDef *hspi);

//--------------------------------------------------------------------------------//
//----------------------FOR FOC MOTORS CONTROLLER---------------------------------//
//--------------------------------------------------------------------------------//
#ifdef USED_FOR_FOC

typedef struct
{
	SPI_HandleTypeDef *hspi;
	float enc_deg;
	float last_enc_deg;
	float enc_bias;
	bool enc_inverse;
	float enc_for_foc_f;
	int16_t overflowNb;
	int16_t enc_for_foc_i16;
}AS5048_ENC_handle;

void as5048a_enc_init(SPI_HandleTypeDef *hspi);

void as5048a_enc_calibrate(void);

void as5028a_enc_update(void);

int16_t as5048a_enc_calangle(ENCODER_Handle_t * pHandle);

float as5048_get_enc_deg(void);

bool as5048_CalcAvrgMecSpeed01Hz( ENCODER_Handle_t * pHandle, int16_t * pMecSpeed01Hz );

#endif
//--------------------------------------------------------------------------------//
//----------------------FOR FOC MOTORS CONTROLLER---------------------------------//
//--------------------------------------------------------------------------------//

#endif