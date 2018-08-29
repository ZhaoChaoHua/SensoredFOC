#include "as5048a.h"

//#define ENC_INVERSE

// Used for Jscope
int a_enc_raw;

uint16_t read_as5048a_raw(SPI_HandleTypeDef *hspi)
{
	uint16_t angle_raw;
	uint16_t x = AS5048A_ANGLE;

	HAL_SPI_TransmitReceive(hspi, (uint8_t *)&x, (uint8_t *)&angle_raw, 1, 1);
	
	// Convert
	angle_raw &= 0x3fff;
	
	return angle_raw;
}

float read_as5048a_deg(SPI_HandleTypeDef *hspi)
{
	// Read Write SPI
	uint16_t angle_raw;
	uint16_t x = AS5048A_ANGLE;
	
	float angle_deg;

	HAL_SPI_TransmitReceive(hspi, (uint8_t *)&x, (uint8_t *)&angle_raw, 1, 1);
	
	// Convert
	angle_raw &= 0x3fff;
	angle_deg = 360.0f - ((float)angle_raw * 360.0f) / 16384.0f;
	
	return angle_deg;
}

float angle_limit_360deg(float a)
{
	if(a < 0.0) a += 360.0f;
	else if(a > 360.0f) a -= 360.0f;
	return a;
}

int16_t angle_float2int16(float a)
{
	return (int16_t) (a/360.0f*65536.0f);
}


//--------------------------------------------------------------------------------//
//----------------------FOR FOC MOTORS CONTROLLER---------------------------------//
//--------------------------------------------------------------------------------//
#ifdef USED_FOR_FOC

AS5048_ENC_handle enc;

// FOC ENC initialization
void as5048a_enc_init(SPI_HandleTypeDef *hspi)
{
#ifdef ENC_INVERSE
	enc.enc_inverse = true;
#endif
	enc.hspi = hspi;
	enc.enc_bias = 0.0f;
	enc.enc_deg = 0.0f;
	enc.enc_for_foc_f = 0.0f;
	enc.enc_for_foc_i16 = 0;
	as5028a_enc_update();
	enc.last_enc_deg = enc.enc_deg;
	enc.overflowNb = 0;

}

// FOC ENC Calibration
void as5048a_enc_calibrate(void)
{
	enc.enc_bias = enc.enc_deg;
}

// FOC ENC update
void as5028a_enc_update(void)
{
	enc.enc_deg = read_as5048a_deg(enc.hspi);
	if(enc.enc_inverse) enc.enc_deg = 360.0 - enc.enc_deg;
	
	enc.enc_for_foc_f = enc.enc_deg - enc.enc_bias;
	enc.enc_for_foc_f = angle_limit_360deg(enc.enc_for_foc_f);
	enc.enc_for_foc_i16 = angle_float2int16(enc.enc_for_foc_f);
	
	if(enc.enc_for_foc_f - enc.last_enc_deg > 200.0f) enc.overflowNb -= 1u;
	else if(enc.enc_for_foc_f - enc.last_enc_deg < -200.0f) enc.overflowNb += 1u;
	enc.last_enc_deg = enc.enc_for_foc_f;
}

/**
* @brief  It calculates the rotor electrical and mechanical angle, on the basis
*         of the instantaneous value of the timer counter.
* @param  pHandle: handler of the current instance of the encoder component
* @retval int16_t Measured electrical angle in s16degree format.
*/
int16_t as5048a_enc_calangle(ENCODER_Handle_t * pHandle)
{
  pHandle->_Super.hElAngle = (int16_t)((int32_t)enc.enc_for_foc_i16*(int32_t)pHandle->_Super.bElToMecRatio);
  pHandle->_Super.hMecAngle = enc.enc_for_foc_i16;

  /*Returns rotor electrical angle*/
  return ( pHandle->_Super.hElAngle );
}


float as5048_get_enc_deg(void)
{
	return enc.enc_for_foc_f;
}

bool as5048_CalcAvrgMecSpeed01Hz( ENCODER_Handle_t * pHandle, int16_t * pMecSpeed01Hz )
{
  int32_t wOverallAngleVariation = 0;
  int32_t wtemp1;
  int32_t wtemp2;
  uint8_t bBufferIndex = 0u;
  bool bReliability = true;
  uint8_t bBufferSize = pHandle->SpeedBufferSize;
  uint32_t OverflowCntSample;
  uint32_t CntCapture;

  CntCapture =  enc.enc_for_foc_f * 100;
  OverflowCntSample = enc.overflowNb;
  enc.overflowNb = 0;

  /* If UIFCPY is not present, OverflowCntSample can not be used safely for
  speed computation, but we still use it to check that we do not exceed one overflow
  (sample frequency not less than mechanical motor speed */
  if ( ( OverflowCntSample) > 1 )
  {
    pHandle->TimerOverflowError = true;
  }

  /*Calculation of delta angle*/

  pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex] =
    ( int32_t )( CntCapture ) - ( int32_t )( pHandle->PreviousCapture ) +
    ( ( int32_t )( OverflowCntSample )) * ( int32_t )( 36000);



  /*Computes & returns average mechanical speed [01Hz], var wtemp1*/
  for ( bBufferIndex = 0u; bBufferIndex < bBufferSize; bBufferIndex++ )
  {
    wOverallAngleVariation += pHandle->DeltaCapturesBuffer[bBufferIndex];
  }
  wtemp1 = wOverallAngleVariation * ( int32_t )( pHandle->SpeedSamplingFreq01Hz );
  wtemp2 = ( int32_t )( 36000 ) *
           ( int32_t )( pHandle->SpeedBufferSize );
  wtemp1 /= wtemp2;
  *pMecSpeed01Hz = ( int16_t )( wtemp1 );

  /*Computes & stores average mechanical acceleration [01Hz/SpeedSamplingFreq]*/
  pHandle->_Super.hMecAccel01HzP = ( int16_t )( wtemp1 -
                                   pHandle->_Super.hAvrMecSpeed01Hz );

  /*Stores average mechanical speed [01Hz]*/
  pHandle->_Super.hAvrMecSpeed01Hz = ( int16_t )wtemp1;

  /*Computes & stores the instantaneous electrical speed [dpp], var wtemp1*/
  wtemp1 = pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex] *
           ( int32_t )( pHandle->SpeedSamplingFreqHz ) *
           ( int32_t )pHandle->_Super.bElToMecRatio;
  wtemp1 /= ( int32_t )( 36000 );
  wtemp1 *= ( int32_t )UINT16_MAX;
  wtemp1 /= ( int32_t )( pHandle->_Super.hMeasurementFrequency );

  pHandle->_Super.hElSpeedDpp = ( int16_t )wtemp1;

  /*last captured value update*/
  pHandle->PreviousCapture = CntCapture;
  /*Buffer index update*/
  pHandle->DeltaCapturesIndex++;

  if ( pHandle->DeltaCapturesIndex == pHandle->SpeedBufferSize )
  {
    pHandle->DeltaCapturesIndex = 0u;
  }

  /*Checks the reliability status, then stores and returns it*/
  if ( pHandle->TimerOverflowError )
  {
    bReliability = false;
    pHandle->SensorIsReliable = false;
    pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;

  }
  else
  {
    bReliability = SPD_IsMecSpeedReliable( &pHandle->_Super, pMecSpeed01Hz );
  }

  return ( bReliability );
}


#endif
