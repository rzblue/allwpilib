// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <jni.h>

#include <cassert>

#include <wpi/jni_util.h>

#include "HALUtil.h"
#include "edu_wpi_first_hal_AnalogJNI.h"
#include "hal/AnalogAccumulator.h"
#include "hal/AnalogInput.h"
#include "hal/AnalogOutput.h"
#include "hal/AnalogTrigger.h"
#include "hal/Ports.h"
#include "hal/handles/HandlesInternal.h"

using namespace hal;

extern "C" {

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    initializeAnalogInputPort
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_hal_AnalogJNI_initializeAnalogInputPort
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  auto stack = wpi::java::GetJavaStackTrace(env, "edu.wpi.first");
  auto analog = HAL_InitializeAnalogInputPort(static_cast<HAL_PortHandle>(id),
                                              stack.c_str(), &status);
  CheckStatusForceThrow(env, status);
  return (jint)analog;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    freeAnalogInputPort
 * Signature: (I)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_AnalogJNI_freeAnalogInputPort
  (JNIEnv* env, jclass, jint id)
{
  HAL_FreeAnalogInputPort(static_cast<HAL_AnalogInputHandle>(id));
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    initializeAnalogOutputPort
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_hal_AnalogJNI_initializeAnalogOutputPort
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  auto stack = wpi::java::GetJavaStackTrace(env, "edu.wpi.first");
  HAL_AnalogOutputHandle analog = HAL_InitializeAnalogOutputPort(
      static_cast<HAL_PortHandle>(id), stack.c_str(), &status);
  CheckStatusForceThrow(env, status);
  return (jlong)analog;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    freeAnalogOutputPort
 * Signature: (I)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_AnalogJNI_freeAnalogOutputPort
  (JNIEnv* env, jclass, jint id)
{
  HAL_FreeAnalogOutputPort(static_cast<HAL_AnalogOutputHandle>(id));
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    checkAnalogModule
 * Signature: (B)Z
 */
JNIEXPORT jboolean JNICALL
Java_edu_wpi_first_hal_AnalogJNI_checkAnalogModule
  (JNIEnv*, jclass, jbyte value)
{
  jboolean returnValue = HAL_CheckAnalogModule(value);
  return returnValue;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    checkAnalogInputChannel
 * Signature: (I)Z
 */
JNIEXPORT jboolean JNICALL
Java_edu_wpi_first_hal_AnalogJNI_checkAnalogInputChannel
  (JNIEnv*, jclass, jint value)
{
  jboolean returnValue = HAL_CheckAnalogInputChannel(value);
  return returnValue;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    checkAnalogOutputChannel
 * Signature: (I)Z
 */
JNIEXPORT jboolean JNICALL
Java_edu_wpi_first_hal_AnalogJNI_checkAnalogOutputChannel
  (JNIEnv*, jclass, jint value)
{
  jboolean returnValue = HAL_CheckAnalogOutputChannel(value);
  return returnValue;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    setAnalogInputSimDevice
 * Signature: (II)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_AnalogJNI_setAnalogInputSimDevice
  (JNIEnv* env, jclass, jint handle, jint device)
{
  HAL_SetAnalogInputSimDevice(static_cast<HAL_AnalogInputHandle>(handle),
                              static_cast<HAL_SimDeviceHandle>(device));
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    setAnalogOutput
 * Signature: (ID)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_AnalogJNI_setAnalogOutput
  (JNIEnv* env, jclass, jint id, jdouble voltage)
{
  int32_t status = 0;
  HAL_SetAnalogOutput(static_cast<HAL_AnalogOutputHandle>(id), voltage,
                      &status);
  CheckStatus(env, status);
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAnalogOutput
 * Signature: (I)D
 */
JNIEXPORT jdouble JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAnalogOutput
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  double val =
      HAL_GetAnalogOutput(static_cast<HAL_AnalogOutputHandle>(id), &status);
  CheckStatus(env, status);
  return val;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    setAnalogSampleRate
 * Signature: (D)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_AnalogJNI_setAnalogSampleRate
  (JNIEnv* env, jclass, jdouble value)
{
  int32_t status = 0;
  HAL_SetAnalogSampleRate(value, &status);
  CheckStatus(env, status);
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAnalogSampleRate
 * Signature: ()D
 */
JNIEXPORT jdouble JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAnalogSampleRate
  (JNIEnv* env, jclass)
{
  int32_t status = 0;
  double returnValue = HAL_GetAnalogSampleRate(&status);
  CheckStatus(env, status);
  return returnValue;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    setAnalogAverageBits
 * Signature: (II)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_AnalogJNI_setAnalogAverageBits
  (JNIEnv* env, jclass, jint id, jint value)
{
  int32_t status = 0;
  HAL_SetAnalogAverageBits(static_cast<HAL_AnalogInputHandle>(id), value,
                           &status);
  CheckStatus(env, status);
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAnalogAverageBits
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAnalogAverageBits
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  jint returnValue =
      HAL_GetAnalogAverageBits(static_cast<HAL_AnalogInputHandle>(id), &status);
  CheckStatus(env, status);
  return returnValue;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    setAnalogOversampleBits
 * Signature: (II)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_AnalogJNI_setAnalogOversampleBits
  (JNIEnv* env, jclass, jint id, jint value)
{
  int32_t status = 0;
  HAL_SetAnalogOversampleBits(static_cast<HAL_AnalogInputHandle>(id), value,
                              &status);
  CheckStatus(env, status);
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAnalogOversampleBits
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAnalogOversampleBits
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  jint returnValue = HAL_GetAnalogOversampleBits(
      static_cast<HAL_AnalogInputHandle>(id), &status);
  CheckStatus(env, status);
  return returnValue;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAnalogValue
 * Signature: (I)S
 */
JNIEXPORT jshort JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAnalogValue
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  jshort returnValue =
      HAL_GetAnalogValue(static_cast<HAL_AnalogInputHandle>(id), &status);
  CheckStatus(env, status);
  return returnValue;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAnalogAverageValue
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAnalogAverageValue
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  jint returnValue = HAL_GetAnalogAverageValue(
      static_cast<HAL_AnalogInputHandle>(id), &status);
  CheckStatus(env, status);
  return returnValue;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAnalogVoltsToValue
 * Signature: (ID)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAnalogVoltsToValue
  (JNIEnv* env, jclass, jint id, jdouble voltageValue)
{
  int32_t status = 0;
  jint returnValue = HAL_GetAnalogVoltsToValue(
      static_cast<HAL_AnalogInputHandle>(id), voltageValue, &status);
  CheckStatus(env, status);
  return returnValue;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAnalogValueToVolts
 * Signature: (II)D
 */
JNIEXPORT jdouble JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAnalogValueToVolts
  (JNIEnv* env, jclass, jint id, jint rawValue)
{
  int32_t status = 0;
  jdouble returnValue = HAL_GetAnalogValueToVolts(
      static_cast<HAL_AnalogInputHandle>(id), rawValue, &status);
  CheckStatus(env, status);
  return returnValue;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAnalogVoltage
 * Signature: (I)D
 */
JNIEXPORT jdouble JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAnalogVoltage
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  jdouble returnValue =
      HAL_GetAnalogVoltage(static_cast<HAL_AnalogInputHandle>(id), &status);
  CheckStatus(env, status);
  return returnValue;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAnalogAverageVoltage
 * Signature: (I)D
 */
JNIEXPORT jdouble JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAnalogAverageVoltage
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  jdouble returnValue = HAL_GetAnalogAverageVoltage(
      static_cast<HAL_AnalogInputHandle>(id), &status);
  CheckStatus(env, status);
  return returnValue;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAnalogLSBWeight
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAnalogLSBWeight
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;

  jint returnValue =
      HAL_GetAnalogLSBWeight(static_cast<HAL_AnalogInputHandle>(id), &status);
  CheckStatus(env, status);
  return returnValue;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAnalogOffset
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAnalogOffset
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;

  jint returnValue =
      HAL_GetAnalogOffset(static_cast<HAL_AnalogInputHandle>(id), &status);
  CheckStatus(env, status);
  return returnValue;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    isAccumulatorChannel
 * Signature: (I)Z
 */
JNIEXPORT jboolean JNICALL
Java_edu_wpi_first_hal_AnalogJNI_isAccumulatorChannel
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;

  jboolean returnValue =
      HAL_IsAccumulatorChannel(static_cast<HAL_AnalogInputHandle>(id), &status);
  CheckStatus(env, status);
  return returnValue;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    initAccumulator
 * Signature: (I)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_AnalogJNI_initAccumulator
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  HAL_InitAccumulator(static_cast<HAL_AnalogInputHandle>(id), &status);
  CheckStatus(env, status);
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    resetAccumulator
 * Signature: (I)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_AnalogJNI_resetAccumulator
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  HAL_ResetAccumulator(static_cast<HAL_AnalogInputHandle>(id), &status);
  CheckStatus(env, status);
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    setAccumulatorCenter
 * Signature: (II)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_AnalogJNI_setAccumulatorCenter
  (JNIEnv* env, jclass, jint id, jint center)
{
  int32_t status = 0;
  HAL_SetAccumulatorCenter(static_cast<HAL_AnalogInputHandle>(id), center,
                           &status);
  CheckStatus(env, status);
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    setAccumulatorDeadband
 * Signature: (II)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_AnalogJNI_setAccumulatorDeadband
  (JNIEnv* env, jclass, jint id, jint deadband)
{
  int32_t status = 0;
  HAL_SetAccumulatorDeadband(static_cast<HAL_AnalogInputHandle>(id), deadband,
                             &status);
  CheckStatus(env, status);
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAccumulatorValue
 * Signature: (I)J
 */
JNIEXPORT jlong JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAccumulatorValue
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  jlong returnValue =
      HAL_GetAccumulatorValue(static_cast<HAL_AnalogInputHandle>(id), &status);
  CheckStatus(env, status);

  return returnValue;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAccumulatorCount
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAccumulatorCount
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  jint returnValue =
      HAL_GetAccumulatorCount(static_cast<HAL_AnalogInputHandle>(id), &status);
  CheckStatus(env, status);
  return returnValue;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAccumulatorOutput
 * Signature: (ILjava/lang/Object;)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAccumulatorOutput
  (JNIEnv* env, jclass, jint id, jobject accumulatorResult)
{
  int32_t status = 0;
  int64_t value = 0;
  int64_t count = 0;
  HAL_GetAccumulatorOutput(static_cast<HAL_AnalogInputHandle>(id), &value,
                           &count, &status);
  SetAccumulatorResultObject(env, accumulatorResult, value, count);
  CheckStatus(env, status);
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    initializeAnalogTrigger
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_hal_AnalogJNI_initializeAnalogTrigger
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  HAL_AnalogTriggerHandle analogTrigger = HAL_InitializeAnalogTrigger(
      static_cast<HAL_AnalogInputHandle>(id), &status);
  CheckStatus(env, status);
  return (jint)analogTrigger;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    initializeAnalogTriggerDutyCycle
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_hal_AnalogJNI_initializeAnalogTriggerDutyCycle
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  HAL_AnalogTriggerHandle analogTrigger = HAL_InitializeAnalogTriggerDutyCycle(
      static_cast<HAL_DutyCycleHandle>(id), &status);
  CheckStatus(env, status);
  return (jint)analogTrigger;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    cleanAnalogTrigger
 * Signature: (I)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_AnalogJNI_cleanAnalogTrigger
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  HAL_CleanAnalogTrigger(static_cast<HAL_AnalogTriggerHandle>(id), &status);
  CheckStatus(env, status);
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    setAnalogTriggerLimitsRaw
 * Signature: (III)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_AnalogJNI_setAnalogTriggerLimitsRaw
  (JNIEnv* env, jclass, jint id, jint lower, jint upper)
{
  int32_t status = 0;
  HAL_SetAnalogTriggerLimitsRaw(static_cast<HAL_AnalogTriggerHandle>(id), lower,
                                upper, &status);
  CheckStatus(env, status);
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    setAnalogTriggerLimitsDutyCycle
 * Signature: (IDD)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_AnalogJNI_setAnalogTriggerLimitsDutyCycle
  (JNIEnv* env, jclass, jint id, jdouble lower, jdouble upper)
{
  int32_t status = 0;
  HAL_SetAnalogTriggerLimitsDutyCycle(static_cast<HAL_AnalogTriggerHandle>(id),
                                      lower, upper, &status);
  CheckStatus(env, status);
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    setAnalogTriggerLimitsVoltage
 * Signature: (IDD)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_AnalogJNI_setAnalogTriggerLimitsVoltage
  (JNIEnv* env, jclass, jint id, jdouble lower, jdouble upper)
{
  int32_t status = 0;
  HAL_SetAnalogTriggerLimitsVoltage(static_cast<HAL_AnalogTriggerHandle>(id),
                                    lower, upper, &status);
  CheckStatus(env, status);
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    setAnalogTriggerAveraged
 * Signature: (IZ)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_AnalogJNI_setAnalogTriggerAveraged
  (JNIEnv* env, jclass, jint id, jboolean averaged)
{
  int32_t status = 0;
  HAL_SetAnalogTriggerAveraged(static_cast<HAL_AnalogTriggerHandle>(id),
                               averaged, &status);
  CheckStatus(env, status);
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    setAnalogTriggerFiltered
 * Signature: (IZ)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_AnalogJNI_setAnalogTriggerFiltered
  (JNIEnv* env, jclass, jint id, jboolean filtered)
{
  int32_t status = 0;
  HAL_SetAnalogTriggerFiltered(static_cast<HAL_AnalogTriggerHandle>(id),
                               filtered, &status);
  CheckStatus(env, status);
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAnalogTriggerInWindow
 * Signature: (I)Z
 */
JNIEXPORT jboolean JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAnalogTriggerInWindow
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  jboolean val = HAL_GetAnalogTriggerInWindow(
      static_cast<HAL_AnalogTriggerHandle>(id), &status);
  CheckStatus(env, status);
  return val;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAnalogTriggerTriggerState
 * Signature: (I)Z
 */
JNIEXPORT jboolean JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAnalogTriggerTriggerState
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  jboolean val = HAL_GetAnalogTriggerTriggerState(
      static_cast<HAL_AnalogTriggerHandle>(id), &status);
  CheckStatus(env, status);
  return val;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAnalogTriggerOutput
 * Signature: (II)Z
 */
JNIEXPORT jboolean JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAnalogTriggerOutput
  (JNIEnv* env, jclass, jint id, jint type)
{
  int32_t status = 0;
  jboolean val =
      HAL_GetAnalogTriggerOutput(static_cast<HAL_AnalogTriggerHandle>(id),
                                 (HAL_AnalogTriggerType)type, &status);
  CheckStatus(env, status);
  return val;
}

/*
 * Class:     edu_wpi_first_hal_AnalogJNI
 * Method:    getAnalogTriggerFPGAIndex
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_hal_AnalogJNI_getAnalogTriggerFPGAIndex
  (JNIEnv* env, jclass, jint id)
{
  int32_t status = 0;
  auto val = HAL_GetAnalogTriggerFPGAIndex(
      static_cast<HAL_AnalogTriggerHandle>(id), &status);
  CheckStatus(env, status);
  return val;
}

}  // extern "C"
