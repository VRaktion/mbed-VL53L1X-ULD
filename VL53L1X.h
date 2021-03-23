/*
 * Copyright (c) 2017, STMicroelectronics - All Rights Reserved
 *
 * This file : part of VL53L1 Core and : dual licensed,
 * either 'STMicroelectronics
 * Proprietary license'
 * or 'BSD 3-clause "New" or "Revised" License' , at your option.
 *
 ********************************************************************************
 *
 * 'STMicroelectronics Proprietary license'
 *
 ********************************************************************************
 *
 * License terms: STMicroelectronics Proprietary in accordance with licensing
 * terms at www.st.com/sla0081
 *
 * STMicroelectronics confidential
 * Reproduction and Communication of this document : strictly prohibited unless
 * specifically authorized in writing by STMicroelectronics.
 *
 *
 ********************************************************************************
 *
 * Alternatively, VL53L1 Core may be distributed under the terms of
 * 'BSD 3-clause "New" or "Revised" License', in which case the following
 * provisions apply instead of the ones mentioned above :
 *
 ********************************************************************************
 *
 * License terms: BSD 3-clause "New" or "Revised" License.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *POSSIBILITY OF SUCH DAMAGE.
 *
 *
 ********************************************************************************
 *
 */

#ifndef __CLASS_H
#define __CLASS_H

#ifdef _MSC_VER
#ifdef API_EXPORTS
#define API __declspec(dllexport)
#else
#define API
#endif
#else
#define API
#endif

/* Includes ------------------------------------------------------------------*/
#include "mbed.h"
// #include "RangeSensor.h"
//#include "vl53l1_int8_t_codes.h"

#define IMPLEMENTATION_VER_MAJOR 3
#define IMPLEMENTATION_VER_MINOR 3
#define IMPLEMENTATION_VER_SUB 0
#define IMPLEMENTATION_VER_REVISION 0000

// typedef signed char int8_t;

#define SOFT_RESET 0x0000
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS 0x0001
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND 0x0008
#define ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS 0x0016
#define ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS 0x0018
#define ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS 0x001A
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM 0x001E
#define MM_CONFIG__INNER_OFFSET_MM 0x0020
#define MM_CONFIG__OUTER_OFFSET_MM 0x0022
#define GPIO_HV_MUX__CTRL 0x0030
#define GPIO__TIO_HV_STATUS 0x0031
#define SYSTEM__INTERRUPT_CONFIG_GPIO 0x0046
#define PHASECAL_CONFIG__TIMEOUT_MACROP 0x004B
#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI 0x005E
#define RANGE_CONFIG__VCSEL_PERIOD_A 0x0060
#define RANGE_CONFIG__VCSEL_PERIOD_B 0x0063
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI 0x0061
#define RANGE_CONFIG__TIMEOUT_MACROP_B_LO 0x0062
#define RANGE_CONFIG__SIGMA_THRESH 0x0064
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS 0x0066
#define RANGE_CONFIG__VALID_PHASE_HIGH 0x0069
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD 0x006C
#define SYSTEM__THRESH_HIGH 0x0072
#define SYSTEM__THRESH_LOW 0x0074
#define SD_CONFIG__WOI_SD0 0x0078
#define SD_CONFIG__INITIAL_PHASE_SD0 0x007A
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD 0x007F
#define ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE 0x0080
#define SYSTEM__SEQUENCE_CONFIG 0x0081
#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD 0x0082
#define SYSTEM__INTERRUPT_CLEAR 0x0086
#define SYSTEM__MODE_START 0x0087
#define VL53L1_RESULT__RANGE_STATUS 0x0089
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0 0x008C
#define RESULT__AMBIENT_COUNT_RATE_MCPS_SD 0x0090
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 0x0096
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0     \
  0x0098
#define VL53L1_RESULT__OSC_CALIBRATE_VAL 0x00DE
#define VL53L1_FIRMWARE__SYSTEM_STATUS 0x00E5
#define VL53L1_IDENTIFICATION__MODEL_ID 0x010F
#define VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD 0x013E

#define DEFAULT_DEVICE_ADDRESS 0x52

/****************************************
 * PRIVATE define do not edit
 ****************************************/

/**
 *  @brief defines SW Version
 */
typedef struct {
  uint8_t major;     /*!< major number */
  uint8_t minor;     /*!< minor number */
  uint8_t build;     /*!< build number */
  uint32_t revision; /*!< revision number */
} VL53L1X_Version_t;

/**
 *  @brief defines packed reading results type
 */
typedef struct {
  uint8_t Status;      /*!< ResultStatus */
  uint16_t Distance;   /*!< ResultDistance */
  uint16_t Ambient;    /*!< ResultAmbient */
  uint16_t SigPerSPAD; /*!< ResultSignalPerSPAD */
  uint16_t NumSPADs;   /*!< ResultNumSPADs */
} VL53L1X_Result_t;

/* Classes -------------------------------------------------------------------*/
/** Class representing a VL53L1 sensor component
 */
class VL53L1X // : public RangeSensor
{
public:
  enum class DistanceModes { Error = 0, Short = 1, Long = 2 };
  enum class TimingBudget {
    Error = 0,
    _15ms = 15,
    _20ms = 20,
    _33ms = 33,
    _50ms = 50,
    _100ms = 100,
    _200ms = 200,
    _500ms = 500
  };

  enum class WindowTypes {
    Below = 0,
    Beyond = 1,
    OutOf = 2,
    Within = 3,
  };

  enum class RangeStatus {
    NoError = 0,
    SigmaFailure = 1,
    SignalFailure = 2,
    RangingError = 4,
    WrapAround = 7,
  };
  /** Constructor
   * @param[in] &i2c device I2C to be used for communication
   * @param[in] &pin_gpio1 pin Mbed InterruptIn PinName to be used as
   * component GPIO_1 INT
   * @param[in] DevAddr device address, 0x52 by default
   */
  VL53L1X(I2C *i2c, PinName pin_xshut = NC, PinName pin_gpio1 = NC);

  /**
   * @brief This function returns the SW driver version
   */
  int8_t GetSWVersion(VL53L1X_Version_t *pVersion);

  /**
   * @brief This function sets the sensor I2C address used in case multiple
   * devices application, default address 0x52
   */
  int8_t SetI2CAddress(uint8_t new_address);

  /**
   * @brief This function loads the 135 bytes default values to initialize the
   * sensor.
   * @param dev Device address
   * @return 0:success, != 0:failed
   */
  int8_t SensorInit();
  int8_t SensorInit0();
  int8_t SensorInit1();

  void EnableInterrupt();
  //void EnableInterruptXshutControlledByHost();

  int8_t SensorReset();

  /**
   * @brief This function clears the interrupt, to be called after a ranging
   * data reading to arm the interrupt for the next data ready event.
   */
  int8_t ClearInterrupt();

  /**
   * @brief This function programs the interrupt polarity\n
   * 1=active high (default), 0=active low
   */
  int8_t SetInterruptPolarity(uint8_t IntPol);

  /**
   * @brief This function returns the current interrupt polarity\n
   * 1=active high (default), 0=active low
   */
  int8_t GetInterruptPolarity(uint8_t *pIntPol);

  /**
   * @brief This function starts the ranging distance operation\n
   * The ranging operation is continuous. The clear interrupt has to be done
   * after each get data to allow the interrupt to raise when the next data is
   * ready\n 1=active high (default), 0=active low, use SetInterruptPolarity()
   * to change the interrupt polarity if required.
   */
  int8_t StartRanging();

  /**
   * @brief This function stops the ranging.
   */
  int8_t StopRanging();

  /**
   * @brief This function checks if the new ranging data is available by polling
   * the dedicated register.
   * @param : isDataReady==0 -> not ready; isDataReady==1 -> ready
   */
  int8_t CheckForDataReady(uint8_t *isDataReady);

  /**
   * @brief This function programs the timing budget in ms.
   * Predefined values = 15, 20, 33, 50, 100(default), 200, 500.
   */
  int8_t SetTimingBudgetInMs(TimingBudget timingBudgetInMs);

  /**
   * @brief This function returns the current timing budget in ms.
   */
  int8_t GetTimingBudgetInMs(TimingBudget *pTimingBudgetInMs);

  /**
   * @brief This function programs the distance mode (1=short, 2=long(default)).
   * Short mode max distance is limited to 1.3 m but better ambient immunity.\n
   * Long mode can range up to 4 m in the dark with 200 ms timing budget.
   */
  int8_t SetDistanceMode(DistanceModes DistanceMode);

  /**
   * @brief This function returns the current distance mode (1=short, 2=long).
   */
  int8_t GetDistanceMode(DistanceModes *pDistanceMode);

  /**
   * @brief This function programs the Intermeasurement period in ms\n
   * Intermeasurement period must be >/= timing budget. This condition is not
   * checked by the API, the customer has the duty to check the condition.
   * Default = 100 ms
   */
  int8_t SetInterMeasurementInMs(uint32_t InterMeasurementInMs);

  /**
   * @brief This function returns the Intermeasurement period in ms.
   */
  int8_t GetInterMeasurementInMs(uint16_t *pIM);

  /**
   * @brief This function returns the boot state of the device (1:booted, 0:not
   * booted)
   */
  int8_t BootState(uint8_t *state);

  /**
   * @brief This function returns the sensor id, sensor Id must be 0xEEAC
   */
  int8_t GetSensorId(uint16_t *id);

  /**
   * @brief This function returns the distance measured by the sensor in mm
   */
  int8_t GetDistance(uint16_t *distance);

  /**
   * @brief This function returns the returned signal per SPAD in kcps/SPAD.
   * With kcps stands for Kilo Count Per Second
   */
  int8_t GetSignalPerSpad(uint16_t *signalPerSp);

  /**
   * @brief This function returns the ambient per SPAD in kcps/SPAD
   */
  int8_t GetAmbientPerSpad(uint16_t *amb);

  /**
   * @brief This function returns the returned signal in kcps.
   */
  int8_t GetSignalRate(uint16_t *signalRate);

  /**
   * @brief This function returns the current number of enabled SPADs
   */
  int8_t GetSpadNb(uint16_t *spNb);

  /**
   * @brief This function returns the ambient rate in kcps
   */
  int8_t GetAmbientRate(uint16_t *ambRate);

  /**
   * @brief This function returns the ranging status int8_t \n
   * (0:no int8_t, 1:sigma failed, 2:signal failed, ..., 7:wrap-around)
   */
  int8_t GetRangeStatus(uint8_t *rangeStatus);

  /**
   * @brief This function returns measurements and the range status in a single
   * read access
   */
  int8_t GetResult(VL53L1X_Result_t *pResult);

  /**
   * @brief This function programs the offset correction in mm
   * @param OffsetValue:the offset correction value to program in mm
   */
  int8_t SetOffset(int16_t OffsetValue);

  /**
   * @brief This function returns the programmed offset correction value in mm
   */
  int8_t GetOffset(int16_t *Offset);

  /**
   * @brief This function programs the xtalk correction value in cps (Count Per
   * Second).\n This is the number of photons reflected back from the cover
   * glass in cps.
   */
  int8_t SetXtalk(uint16_t XtalkValue);

  /**
   * @brief This function returns the current programmed xtalk correction value
   * in cps
   */
  int8_t GetXtalk(uint16_t *Xtalk);

  /**
   * @brief This function programs the threshold detection mode\n
   * Example:\n
   * SetDistanceThreshold(dev,100,300,0,1): Below 100 \n
   * SetDistanceThreshold(dev,100,300,1,1): Above 300 \n
   * SetDistanceThreshold(dev,100,300,2,1): Out of window \n
   * SetDistanceThreshold(dev,100,300,3,1): In window \n
   * @param   dev : device address
   * @param  	ThreshLow(in mm) : the threshold under which one the device
   * raises an interrupt if Window = 0
   * @param 	ThreshHigh(in mm) :  the threshold above which one the device
   * raises an interrupt if Window = 1
   * @param   Window detection mode : 0=below, 1=above, 2=out, 3=in
   * @param   IntOnNoTarget = 1 (No longer used - just use 1)
   */
  int8_t SetDistanceThreshold(uint16_t ThreshLow, uint16_t ThreshHigh,
                              WindowTypes Window, uint8_t IntOnNoTarget = 1);

  /**
   * @brief This function returns the window detection mode (0=below; 1=above;
   * 2=out; 3=in)
   */
  int8_t GetDistanceThresholdWindow(WindowTypes *window);

  /**
   * @brief This function returns the low threshold in mm
   */
  int8_t GetDistanceThresholdLow(uint16_t *low);

  /**
   * @brief This function returns the high threshold in mm
   */
  int8_t GetDistanceThresholdHigh(uint16_t *high);

  /**
   * @brief This function programs the ROI (Region of Interest)\n
   * The ROI position is centered, only the ROI size can be reprogrammed.\n
   * The smallest acceptable ROI size = 4\n
   * @param X:ROI Width; Y=ROI Height
   */
  int8_t SetROI(uint16_t X, uint16_t Y);

  /**
   *@brief This function returns width X and height Y
   */
  int8_t GetROI_XY(uint16_t *ROI_X, uint16_t *ROI_Y);

  /**
   *@brief This function programs the new user ROI center, please to be aware
   *that there is no check in this function. if the ROI center vs ROI size is
   *out of border the ranging function return int8_t #13
   */
  /**Table of Optical Centers**
  *
  * 128,136,144,152,160,168,176,184,  192,200,208,216,224,232,240,248
  * 129,137,145,153,161,169,177,185,  193,201,209,217,225,233,241,249
  * 130,138,146,154,162,170,178,186,  194,202,210,218,226,234,242,250
  * 131,139,147,155,163,171,179,187,  195,203,211,219,227,235,243,251
  * 132,140,148,156,164,172,180,188,  196,204,212,220,228,236,244,252
  * 133,141,149,157,165,173,181,189,  197,205,213,221,229,237,245,253
  * 134,142,150,158,166,174,182,190,  198,206,214,222,230,238,246,254
  * 135,143,151,159,167,175,183,191,  199,207,215,223,231,239,247,255

  * 127,119,111,103, 95, 87, 79, 71,  63, 55, 47, 39, 31, 23, 15, 7
  * 126,118,110,102, 94, 86, 78, 70,  62, 54, 46, 38, 30, 22, 14, 6
  * 125,117,109,101, 93, 85, 77, 69,  61, 53, 45, 37, 29, 21, 13, 5
  * 124,116,108,100, 92, 84, 76, 68,  60, 52, 44, 36, 28, 20, 12, 4
  * 123,115,107, 99, 91, 83, 75, 67,  59, 51, 43, 35, 27, 19, 11, 3
  * 122,114,106, 98, 90, 82, 74, 66,  58, 50, 42, 34, 26, 18, 10, 2
  * 121,113,105, 97, 89, 81, 73, 65,  57, 49, 41, 33, 25, 17, 9, 1
  * 120,112,104, 96, 88, 80, 72, 64,  56, 48, 40, 32, 24, 16, 8, 0 Pin 1
  *
  * To set the center, set the pad that is to the right and above the exact
  center of the region you'd like to measure as your opticalCenter*/
  int8_t SetROICenter(uint8_t ROICenter);

  /**
   *@brief This function returns the current user ROI center
   */
  int8_t GetROICenter(uint8_t *ROICenter);

  /**
   * @brief This function programs a new signal threshold in kcps (default=1024
   * kcps\n
   */
  int8_t SetSignalThreshold(uint16_t signal);

  /**
   * @brief This function returns the current signal threshold in kcps
   */
  int8_t GetSignalThreshold(uint16_t *signal);

  /**
   * @brief This function programs a new sigma threshold in mm (default=15 mm)
   */
  int8_t SetSigmaThreshold(uint16_t sigma);

  /**
   * @brief This function returns the current sigma threshold in mm
   */
  int8_t GetSigmaThreshold(uint16_t *signal);

  /**
   * @brief This function performs the temperature calibration.
   * It is recommended to call this function any time the temperature might have
   * changed by more than 8 deg C without sensor ranging activity for an
   * extended period.
   */
  int8_t StartTemperatureUpdate();

  /**
   * @brief This function performs the offset calibration.\n
   * The function returns the offset value found and programs the offset
   * compensation into the device.
   * @param TargetDistInMm target distance in mm, ST recommended 100 mm
   * Target reflectance = grey17%
   * @return 0:success, !=0: failed
   * @return offset pointer contains the offset found in mm
   */
  int8_t CalibrateOffset(uint16_t TargetDistInMm, int16_t *offset);

  /**
   * @brief This function performs the xtalk calibration.\n
   * The function returns the xtalk value found and programs the xtalk
   * compensation to the device
   * @param TargetDistInMm target distance in mm\n
   * The target distance : the distance where the sensor start to "under
   * range"\n due to the influence of the photons reflected back from the cover
   * glass becoming strong\n It's also called inflection point\n Target
   * reflectance = grey 17%
   * @return 0: success, !=0: failed
   * @return xtalk pointer contains the xtalk value found in cps (number of
   * photons in count per second)
   */
  int8_t CalibrateXtalk(uint16_t TargetDistInMm, uint16_t *xtalk);

  uint16_t GetBufferedDistance(void);
  uint8_t GetBufferedRangeStatus(void);

  void SetInterruptCallback(Callback<void(uint16_t, uint8_t)> cbFct);

  void SetRangeStatusFilter(RangeStatus rangingStatus);

  void WakeUp (void);

  void SetToSleep (void);

private:
  void isr();
  //void isrXshutControlledByHost();
  /* Write and read functions from I2C */

  int8_t WrByte(uint16_t index, uint8_t data);
  int8_t WrWord(uint16_t index, uint16_t data);
  int8_t WrDWord(uint16_t index, uint32_t data);
  int8_t RdByte(uint16_t index, uint8_t *data);
  int8_t RdWord(uint16_t index, uint16_t *data);
  int8_t RdDWord(uint16_t index, uint32_t *data);
  int8_t UpdateByte(uint16_t index, uint8_t AndData, uint8_t OrData);

  int8_t WriteMulti(uint16_t index, uint8_t *pdata, uint32_t count);
  int8_t ReadMulti(uint16_t index, uint8_t *pdata, uint32_t count);

  int8_t I2CWrite(uint8_t dev, uint16_t index, uint8_t *data,
                  uint16_t number_of_bytes);
  int8_t I2CRead(uint8_t dev, uint16_t index, uint8_t *data,
                 uint16_t number_of_bytes);
  int8_t WaitUs(int32_t waitUs);
  int8_t WaitMs(int32_t waitMs);

  /* IO Device */
  I2C *dev_i2c;
  /* Digital out pin */
  DigitalOut *xshut;
  InterruptIn *gpio1Int;

  uint16_t distanceBuffer = 0;
  uint8_t rangeStatus = 0;
  uint8_t i2cAddress;

  RangeStatus rangeStatusFilter = RangeStatus::WrapAround;

  Callback<void(uint16_t, uint8_t)> cbFct = NULL;
};

#endif /* _CLASS_H_ */
