#ifndef _HAL_OBJ_H_
#define _HAL_OBJ_H_
/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

//! \file   solutions/instaspin_foc/src/hal_obj.h
//! \brief Defines the structures for the HAL object 
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// drivers
#include "adc.h"
#include "cap.h"
#include "clk.h"
#include "cpu.h"
#include "flash.h"
#include "gpio.h"
#include "i2c.h"
#include "osc.h"
#include "pie.h"
#include "pll.h"
#include "pwm.h"
#include "pwmdac.h"
#include "pwr.h"
#include "sci.h"
//#include "spi.h"
#include "timer.h"
#include "wdog.h"
//#include "drv8301.h"


#ifdef QEP
#include "sw/drivers/qep/src/32b/f28x/f2806x/qep.h"
#endif

// modules
#include "offset.h"
#include "types.h"
#include "usDelay.h"


// platforms
#include "user.h"


//!
//!
//! \defgroup HAL_OBJ HAL_OBJ
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif
//! \brief      Defines the ADC data
//! \details    This data structure contains the voltage and current values that are used when 
//!             performing a HAL_AdcRead and then this structure is passed to the CTRL controller
//!             and the FAST estimator.
//!
typedef struct _HAL_AdcData_t_
{
  MATH_vec3 I;          //!< the current values

  MATH_vec3 V;          //!< the voltage values

  _iq       dcBus;      //!< the dcBus value

} HAL_AdcData_t;


//! \brief      Defines the DAC data
//! \details    This data structure contains the pwm values that are used for the DAC output
//!             on a lot of the hardware kits for debugging.
//!
typedef struct _HAL_DacData_t_
{
  uint16_t	PeriodMax;		//!< the DAC_PWM period
  int32_t  *ptrData[4];     //!< Input: First input pointer
  _iq  		value[4];       //!< the DAC data
  _iq  		offset[4];      //!< the DAC offset value
  _iq  		gain[4];        //!< the DAC gain value
} HAL_DacData_t;


//! \brief      Defines the PWM data
//! \details    This structure contains the pwm voltage values for the three phases.  A
//!             HAL_PwmData_t variable is filled with values from, for example, a space
//!             vector modulator and then sent to functions like HAL_writePwmData() to
//!             write to the PWM peripheral.
//!
typedef struct _HAL_PwmData_t_
{
  MATH_vec3  Tabc;      //!< the PWM time-durations for each motor phase

} HAL_PwmData_t;


//! \brief      Defines the hardware abstraction layer (HAL) data
//! \details    The HAL object contains all handles to peripherals.  When accessing a
//!             peripheral on a processor, use a HAL function along with the HAL handle
//!             for that processor to access its peripherals.
//!
typedef struct _HAL_Obj_
{
  ADC_Handle    adcHandle;        //!< the ADC handle

  CLK_Handle    clkHandle;        //!< the clock handle
 
  CPU_Handle    cpuHandle;        //!< the CPU handle

  FLASH_Handle  flashHandle;      //!< the flash handle

  GPIO_Handle   gpioHandle;       //!< the GPIO handle

  OFFSET_Handle offsetHandle_I[3];  //!< the handles for the current offset estimators
  OFFSET_Obj    offset_I[3];        //!< the current offset objects

  OFFSET_Handle offsetHandle_V[3];  //!< the handles for the voltage offset estimators
  OFFSET_Obj    offset_V[3];        //!< the voltage offset objects

  OSC_Handle    oscHandle;        //!< the oscillator handle

  PIE_Handle    pieHandle;        //<! the PIE handle

  PLL_Handle    pllHandle;        //!< the PLL handle

  PWM_Handle    pwmHandle[3];     //<! the PWM handles

  PWMDAC_Handle pwmDacHandle[3];  //<! the PWMDAC handles

  PWR_Handle    pwrHandle;        //<! the power handle

  TIMER_Handle  timerHandle[3];   //<! the timer handles

  WDOG_Handle   wdogHandle;       //!< the watchdog handle

  HAL_AdcData_t adcBias;          //!< the ADC bias

  _iq           current_sf;       //!< the current scale factor, amps_pu/cnt

  _iq           voltage_sf;       //!< the voltage scale factor, volts_pu/cnt

  uint_least8_t numCurrentSensors; //!< the number of current sensors
  uint_least8_t numVoltageSensors; //!< the number of voltage sensors

  //SPI_Handle    spiAHandle;       //!< the SPI handle
  //SPI_Handle    spiBHandle;       //!< the SPI handle

  //DRV8301_Handle drv8301Handle;   //!< the drv8301 interface handle
  //DRV8301_Obj    drv8301;         //!< the drv8301 interface object

#ifdef QEP
  QEP_Handle    qepHandle[2];     //!< the QEP handles
#endif

  //PWM_Handle    pwmFanHandle[2];  //!< the condenser and evaporator fan handles

  CAP_Handle    capFanHandle[2];  //!< the condenser and evaporator fan feedback

  SCI_Handle    sciAHandle;       //!< used for modbus uart communications

  //I2C_Handle    i2cHandle;        //!< used for i2c communications

} HAL_Obj;


//! \brief      Defines the HAL handle
//! \details    The HAL handle is a pointer to a HAL object.  In all HAL functions
//!             the HAL handle is passed so that the function knows what peripherals
//!             are to be accessed.
//!
typedef struct _HAL_Obj_ *HAL_Handle;


//! \brief      Defines the HAL object
//!
extern HAL_Obj hal;


//! \brief      Runs offset estimation
//! \details    Offsets of the voltage and current feedbacks are required for good low
//!             speed performance of the motor drive.
//! \param[in]  handle    The hardware abstraction layer (HAL) handle
//! \param[in]  pAdcData  The pointer to the ADC data
inline void HAL_runOffsetEst(HAL_Handle handle,const HAL_AdcData_t *pAdcData)
{
  uint_least8_t cnt;
  HAL_Obj *obj = (HAL_Obj *)handle;


  // estimate the current offsets
  for(cnt=0;cnt<obj->numCurrentSensors;cnt++)
    {
      OFFSET_run(obj->offsetHandle_I[cnt],pAdcData->I.value[cnt]);
    }


  // estimate the voltage offsets
  for(cnt=0;cnt<obj->numVoltageSensors;cnt++)
    {
      OFFSET_run(obj->offsetHandle_V[cnt],pAdcData->V.value[cnt]);
    }

  return;
} // end of HAL_runOffsetEst() function


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _HAL_OBJ_H_ definition

