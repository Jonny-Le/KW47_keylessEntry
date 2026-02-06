/*! *********************************************************************************
* \file rssi_integration.h
*
* Integration of ProxRssi filtering into Digital Key Car Anchor CS example.
* Provides the same public API that digital_key_car_anchor_cs.c and
* shell_digital_key_car_anchor_cs.c expect, but delegates to ProxRssi internally.
*
* Copyright 2025
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#ifndef RSSI_INTEGRATION_H
#define RSSI_INTEGRATION_H

#include "EmbeddedTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Re-export proximity types expected by the NXP app layer */
typedef enum
{
    ProximityState_Disconnected_c = 0u,
    ProximityState_Monitoring_c   = 1u,
    ProximityState_Approach_c     = 2u,
    ProximityState_Proximity_c    = 3u,
    ProximityState_Unlock_c       = 4u
} proximityState_t;

typedef enum
{
    ProximityEvent_None_c         = 0u,
    ProximityEvent_Connected_c    = 1u,
    ProximityEvent_Disconnected_c = 2u,
    ProximityEvent_EnterNear_c    = 3u,
    ProximityEvent_ExitNear_c     = 4u,
    ProximityEvent_Unlock_c       = 5u,
    ProximityEvent_Lockout_c      = 6u
} proximityEvent_t;

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
