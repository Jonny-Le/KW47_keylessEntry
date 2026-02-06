/*! *********************************************************************************
* \file rssi_integration.h
*
* Integration of RSSI filtering into Digital Key Car Anchor CS example
*
* Copyright 2025
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#ifndef RSSI_INTEGRATION_H
#define RSSI_INTEGRATION_H

#include "EmbeddedTypes.h"
#include "proximity_state_machine.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! *********************************************************************************
* \brief     Initialize RSSI integration module
********************************************************************************** */
void RssiIntegration_Init(void);

/*! *********************************************************************************
* \brief     Handle device connected event
********************************************************************************** */
void RssiIntegration_DeviceConnected(uint8_t deviceId);

/*! *********************************************************************************
* \brief     Handle device disconnected event
********************************************************************************** */
void RssiIntegration_DeviceDisconnected(uint8_t deviceId);

/*! *********************************************************************************
* \brief     Update RSSI value from connected device
********************************************************************************** */
void RssiIntegration_UpdateRssi(uint8_t deviceId, int8_t rssi);

/*! *********************************************************************************
* \brief     Get current proximity state
********************************************************************************** */
proximityState_t RssiIntegration_GetState(void);

/*! *********************************************************************************
* \brief     Check if unlock should occur
********************************************************************************** */
bool_t RssiIntegration_ShouldUnlock(void);

/*! *********************************************************************************
* \brief     Print current status
********************************************************************************** */
void RssiIntegration_PrintStatus(void);

/*! *********************************************************************************
* \brief     Start continuous RSSI monitoring
********************************************************************************** */
void RssiIntegration_StartMonitoring(void);

/*! *********************************************************************************
* \brief     Stop continuous RSSI monitoring
********************************************************************************** */
void RssiIntegration_StopMonitoring(void);

#ifdef __cplusplus
}
#endif

#endif /* RSSI_INTEGRATION_H */
