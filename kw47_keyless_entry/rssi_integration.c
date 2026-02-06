/*! *********************************************************************************
* \file rssi_integration.c
*
* Integration layer: bridges the NXP Digital Key Car Anchor CS application to
* the ProxRssi module.  Implements the same public API that
* digital_key_car_anchor_cs.c and shell_digital_key_car_anchor_cs.c expect.
*
* Copyright 2025
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

/************************************************************************************
* Include
************************************************************************************/

#include "EmbeddedTypes.h"
#include "fsl_component_timer_manager.h"
#include "rssi_integration.h"
#include "ProxRssi.h"
#include "gap_interface.h"
#include "fsl_format.h"

/* Shell output - defined in shell file */
#if defined(gAppUseShellInApplication_d) && (gAppUseShellInApplication_d == 1)
#include "fsl_shell.h"
extern SHELL_HANDLE_DEFINE(g_shellHandle);
#define RSSI_PRINT(a) (void)SHELL_PrintfSynchronization((shell_handle_t)g_shellHandle, a)
#define RSSI_DBG(a)   (void)0
#else
#define RSSI_PRINT(a) (void)0
#define RSSI_DBG(a)   (void)0
#endif

/************************************************************************************
* Private macros
************************************************************************************/

#define RSSI_MONITOR_INTERVAL_MS      (100u)
#define RSSI_ALPHA_LUT_LEN            (1001u)
#define RSSI_PRINT_INTERVAL           (5u)

/************************************************************************************
* Private variables
************************************************************************************/

static ProxRssi_CtxType  gProxCtx;
static bool_t            gRssiIntegrationInitialized = FALSE;
static bool_t            gRssiMonitoringActive       = FALSE;
static uint8_t           gConnectedDeviceId          = 0xFFu;
static bool_t            gUnlockPending              = FALSE;
static uint32_t          gSampleCount                = 0u;

/* Timer for continuous RSSI monitoring */
static TIMER_MANAGER_HANDLE_DEFINE(gRssiTimerHandle);
static bool_t gRssiTimerInitialized = FALSE;

/* Alpha LUT (pre-computed at init): linear ramp 0.10..0.80 over 0..1000 ms */
static uint16 gAlphaLutQ15[RSSI_ALPHA_LUT_LEN];

/************************************************************************************
* Private function prototypes
************************************************************************************/

static void RssiIntegration_TimerCallback(void *pParam);
static void RssiIntegration_BuildAlphaLut(void);
static uint32_t RssiIntegration_GetTimestampMs(void);

/************************************************************************************
* Public functions
************************************************************************************/

/*! *********************************************************************************
* \brief     Initialize RSSI integration module
********************************************************************************** */
void RssiIntegration_Init(void)
{
    ProxRssi_ParamsType params;
    uint32 i;

    if (gRssiIntegrationInitialized == TRUE)
    {
        return;
    }

    /* Build alpha LUT: linear ramp 0.10 .. 0.80 over 0..1000 ms */
    RssiIntegration_BuildAlphaLut();

    /* Zero-init params */
    for (i = 0u; i < (uint32)(sizeof(params)); i++)
    {
        ((uint8 *)&params)[i] = 0u;
    }

    params.wRawMs    = 2000u;
    params.wSpikeMs  = 800u;
    params.wFeatMs   = 2000u;

    params.hampelKQ4 = 40u;   /* K = 2.5 (tighter spike rejection) */
    params.madEpsQ4  = 8u;    /* 0.5 dB floor */

    params.enterNearQ4 = ProxRssi_DbmToQ4((sint8)-50);
    params.exitNearQ4  = ProxRssi_DbmToQ4((sint8)-60);
    params.hystQ4      = (uint16)ProxRssi_DbToQ4((sint16)10);

    params.pctThQ15       = 13107u;  /* ~40% of smoothed samples above enter */
    params.stdThQ4        = 128u;    /* 8 dB — realistic for BLE RSSI noise */
    params.stableMs       = 2000u;
    params.minFeatSamples = 6u;

    params.exitConfirmMs    = 1500u;
    params.lockoutMs        = 5000u;
    params.maxReasonableDtMs = 2000u;

    (void)ProxRssi_Init(&gProxCtx, &params, gAlphaLutQ15, RSSI_ALPHA_LUT_LEN);

    gUnlockPending = FALSE;
    gSampleCount   = 0u;
    gRssiIntegrationInitialized = TRUE;

    RSSI_DBG("ProxRssi initialized");
}

/*! *********************************************************************************
* \brief     Handle device connected event
********************************************************************************** */
void RssiIntegration_DeviceConnected(uint8_t deviceId)
{
    if (gRssiIntegrationInitialized != TRUE)
    {
        RssiIntegration_Init();
    }

    gConnectedDeviceId = deviceId;
    gUnlockPending     = FALSE;
    gSampleCount       = 0u;

    /* Reset filter state for new connection */
    (void)ProxRssi_ForceFar(&gProxCtx);

    RSSI_DBG("Device connected");
}

/*! *********************************************************************************
* \brief     Handle device disconnected event
********************************************************************************** */
void RssiIntegration_DeviceDisconnected(uint8_t deviceId)
{
    (void)deviceId;

    gConnectedDeviceId = 0xFFu;
    gUnlockPending     = FALSE;
    gRssiMonitoringActive = FALSE;

    /* Reset filter */
    (void)ProxRssi_ForceFar(&gProxCtx);

    if (gRssiTimerInitialized == TRUE)
    {
        (void)TM_Stop((timer_handle_t)gRssiTimerHandle);
    }

    RSSI_DBG("Device disconnected");
}

/*! *********************************************************************************
* \brief     Update RSSI value from connected device
********************************************************************************** */
void RssiIntegration_UpdateRssi(uint8_t deviceId, int8_t rssi)
{
    ProxRssi_EventType ev   = PROX_RSSI_EVT_NONE;
    ProxRssi_FeaturesType feat;
    uint32_t nowMs;

    (void)deviceId;

    if (gRssiIntegrationInitialized != TRUE)
    {
        return;
    }

    /* BLE Core Spec: RSSI 127 (0x7F) = "not available". Also reject >= 0
     * since real BLE RSSI is always negative. Feeding these into the
     * pipeline poisons the Hampel median and corrupts the state machine. */
    if ((rssi == (int8_t)127) || (rssi >= (int8_t)0))
    {
        return;
    }

    nowMs = RssiIntegration_GetTimestampMs();

    (void)ProxRssi_PushRaw(&gProxCtx, (uint32)nowMs, (sint8)rssi);
    (void)ProxRssi_MainFunction(&gProxCtx, (uint32)nowMs, &ev, &feat);

    /* Track unlock */
    if (ev == PROX_RSSI_EVT_UNLOCK_TRIGGERED)
    {
        gUnlockPending = TRUE;
    }

    /* Periodic diagnostic output */
    gSampleCount++;
    if ((gSampleCount % RSSI_PRINT_INTERVAL) == 0u)
    {
        RSSI_PRINT("[RSSI] R:");
        RSSI_PRINT((const char*)FORMAT_Dec2Str((uint32_t)(rssi < 0 ? (int32_t)(-rssi) : (int32_t)rssi)));
        RSSI_PRINT(" EMA:");
        {
            sint16 ema = gProxCtx.emaQ4;
            int32_t emaAbs = (ema < 0) ? (int32_t)(-ema) : (int32_t)ema;
            RSSI_PRINT((const char*)FORMAT_Dec2Str((uint32_t)(emaAbs / 16)));
            RSSI_PRINT(".");
            RSSI_PRINT((const char*)FORMAT_Dec2Str((uint32_t)((emaAbs % 16) * 10 / 16)));
        }
        RSSI_PRINT(" SD:");
        RSSI_PRINT((const char*)FORMAT_Dec2Str((uint32_t)(feat.stdQ4 / 16u)));
        RSSI_PRINT(".");
        RSSI_PRINT((const char*)FORMAT_Dec2Str((uint32_t)((feat.stdQ4 % 16u) * 10u / 16u)));
        RSSI_PRINT(" P:");
        RSSI_PRINT((const char*)FORMAT_Dec2Str((uint32_t)((uint32_t)feat.pctAboveEnterQ15 * 100u / 32767u)));
        RSSI_PRINT("% ST:");

        switch (gProxCtx.st)
        {
            case PROX_RSSI_ST_FAR:       RSSI_PRINT("FAR");       break;
            case PROX_RSSI_ST_CANDIDATE: RSSI_PRINT("CANDIDATE"); break;
            case PROX_RSSI_ST_LOCKOUT:   RSSI_PRINT("LOCKOUT");   break;
            default:                     RSSI_PRINT("?");          break;
        }
        RSSI_PRINT("\r\n");
    }

    /* Print state-change events immediately */
    if (ev != PROX_RSSI_EVT_NONE)
    {
        RSSI_PRINT("*** ");
        switch (ev)
        {
            case PROX_RSSI_EVT_CANDIDATE_STARTED:
                RSSI_PRINT("CANDIDATE (checking stability)");
                break;
            case PROX_RSSI_EVT_UNLOCK_TRIGGERED:
                RSSI_PRINT(">>> UNLOCK TRIGGERED <<< (lockout 5s)");
                break;
            case PROX_RSSI_EVT_EXIT_TO_FAR:
                RSSI_PRINT("EXIT -> FAR/LOCKED (confirmed)");
                break;
            default:
                RSSI_PRINT("EVENT");
                break;
        }
        RSSI_PRINT(" ***\r\n");
    }
}

/*! *********************************************************************************
* \brief     Get current proximity state (mapped to legacy enum)
********************************************************************************** */
proximityState_t RssiIntegration_GetState(void)
{
    proximityState_t result;

    switch (gProxCtx.st)
    {
        case PROX_RSSI_ST_CANDIDATE:
            result = ProximityState_Approach_c;
            break;
        case PROX_RSSI_ST_LOCKOUT:
            result = ProximityState_Unlock_c;
            break;
        case PROX_RSSI_ST_FAR:
        default:
            result = ProximityState_Monitoring_c;
            break;
    }
    return result;
}

/*! *********************************************************************************
* \brief     Check if unlock should occur
********************************************************************************** */
bool_t RssiIntegration_ShouldUnlock(void)
{
    bool_t result = gUnlockPending;
    gUnlockPending = FALSE;  /* read-once semantics */
    return result;
}

/*! *********************************************************************************
* \brief     Print current status
********************************************************************************** */
void RssiIntegration_PrintStatus(void)
{
    if (gRssiIntegrationInitialized != TRUE)
    {
        RSSI_DBG("Not initialized");
        return;
    }
    RSSI_DBG("Status printed");
}

/*! *********************************************************************************
* \brief     Start continuous RSSI monitoring
********************************************************************************** */
void RssiIntegration_StartMonitoring(void)
{
    if (gConnectedDeviceId == 0xFFu)
    {
        RSSI_PRINT("\r\n[RSSI] No device connected\r\n");
        return;
    }

    if (gRssiMonitoringActive == TRUE)
    {
        RSSI_PRINT("\r\n[RSSI] Already monitoring\r\n");
        return;
    }

    /* Initialize timer if not done */
    if (gRssiTimerInitialized != TRUE)
    {
        timer_status_t status = TM_Open((timer_handle_t)gRssiTimerHandle);
        if (status != kStatus_TimerSuccess)
        {
            RSSI_PRINT("\r\n[RSSI] Timer init failed\r\n");
            return;
        }
        gRssiTimerInitialized = TRUE;
    }

    /* Install callback and start timer */
    (void)TM_InstallCallback((timer_handle_t)gRssiTimerHandle,
                             RssiIntegration_TimerCallback, NULL);
    (void)TM_Start((timer_handle_t)gRssiTimerHandle,
                   (uint8_t)kTimerModeIntervalTimer,
                   RSSI_MONITOR_INTERVAL_MS);

    gRssiMonitoringActive = TRUE;
    RSSI_PRINT("\r\n[RSSI] Monitoring STARTED (100ms)\r\n");
    RSSI_PRINT("[RSSI] Pipeline: Hampel->EMA->Features->StateMachine\r\n");

    /* Trigger first read immediately */
    (void)Gap_ReadRssi(gConnectedDeviceId);
}

/*! *********************************************************************************
* \brief     Stop continuous RSSI monitoring
********************************************************************************** */
void RssiIntegration_StopMonitoring(void)
{
    if (gRssiMonitoringActive != TRUE)
    {
        RSSI_PRINT("\r\n[RSSI] Not monitoring\r\n");
        return;
    }

    gRssiMonitoringActive = FALSE;

    if (gRssiTimerInitialized == TRUE)
    {
        (void)TM_Stop((timer_handle_t)gRssiTimerHandle);
    }

    RSSI_PRINT("\r\n[RSSI] Monitoring STOPPED\r\n");
}

/************************************************************************************
* Private functions
************************************************************************************/

static void RssiIntegration_BuildAlphaLut(void)
{
    /* Linear ramp: alpha = 0.05 at dt=0ms, alpha = 0.30 at dt=1000ms
     * Gives ~1.3s time constant at 100ms sampling — smooth enough to
     * suppress BLE RSSI jitter while still tracking movement.
     * Q15: 0.05 = 1638, 0.30 = 9830 */
    uint32 i;
    for (i = 0u; i < RSSI_ALPHA_LUT_LEN; i++)
    {
        /* alpha_q15 = 1638 + i * (9830 - 1638) / 1000 */
        uint32 alpha = 1638u + ((i * 8192u) / 1000u);
        if (alpha > 32767u) { alpha = 32767u; }
        gAlphaLutQ15[i] = (uint16)alpha;
    }
}

static uint32_t RssiIntegration_GetTimestampMs(void)
{
    /* TM_GetTimestamp() returns MICROSECONDS (HAL uses COUNT_TO_USEC).
     * ProxRssi expects milliseconds for all window/timer parameters. */
    return (uint32_t)(TM_GetTimestamp() / 1000u);
}

static void RssiIntegration_TimerCallback(void *pParam)
{
    (void)pParam;

    if ((gRssiMonitoringActive == TRUE) && (gConnectedDeviceId != 0xFFu))
    {
        (void)Gap_ReadRssi(gConnectedDeviceId);
    }
}
