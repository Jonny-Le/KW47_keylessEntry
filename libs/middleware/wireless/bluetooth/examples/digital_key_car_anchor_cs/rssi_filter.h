/*! *********************************************************************************
* \file rssi_filter.h
*
* \brief  RSSI filtering for KW47 keyless entry system
*         MISRA C:2012 / AUTOSAR coding guidelines compliant.
*         Pipeline: Hampel -> Adaptive EMA -> Feature Extraction -> State Machine
*         Fixed-point Q4 throughout (1/16 dB resolution). No float. No dynamic memory.
*
* Copyright 2025
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#ifndef RSSI_FILTER_H
#define RSSI_FILTER_H

#include "EmbeddedTypes.h"
#include "fsl_component_timer_manager.h"
#include <stdint.h>

/************************************************************************************
*************************************************************************************
* Fixed-point format constants
*************************************************************************************
************************************************************************************/

#define RSSI_Q4_SCALE           ((int16_t)16)
#define RSSI_Q15_ONE            ((uint16_t)32767U)

/************************************************************************************
*************************************************************************************
* Ring buffer capacities
*************************************************************************************
************************************************************************************/

#define RSSI_RAW_CAP            (32U)
#define RSSI_SMOOTH_CAP         (40U)

/************************************************************************************
*************************************************************************************
* Stage 1: Hampel filter configuration
*************************************************************************************
************************************************************************************/

#define RSSI_HAMPEL_WIN_MS      (800U)    /* Time window for spike detection (ms) */
#define RSSI_HAMPEL_K_Q4        (48U)     /* K=3.0 in Q4. Threshold = K * 1.5 * MAD */
#define RSSI_MAD_EPS_Q4         (8U)      /* MAD floor = 0.5 dB in Q4 */

/************************************************************************************
*************************************************************************************
* Stage 2: Adaptive EMA configuration
*************************************************************************************
************************************************************************************/

#define RSSI_EMA_ALPHA_MIN_Q15  (3277U)   /* ~0.10 in Q15 (dt <= 50ms)  */
#define RSSI_EMA_ALPHA_MAX_Q15  (26214U)  /* ~0.80 in Q15 (dt >= 500ms) */
#define RSSI_EMA_DT_MIN_MS      (50U)
#define RSSI_EMA_DT_MAX_MS      (500U)
#define RSSI_EMA_ANOMALY_DT_MS  (2000U)   /* dt > this => full EMA reset */

/************************************************************************************
*************************************************************************************
* Stage 3: Feature extraction configuration
*************************************************************************************
************************************************************************************/

#define RSSI_FEAT_WIN_MS        (2000U)   /* 2-second statistics window */
#define RSSI_MIN_FEAT_SAMPLES   (6U)      /* Minimum samples for valid features */

/************************************************************************************
*************************************************************************************
* Stage 4: State machine thresholds
*************************************************************************************
************************************************************************************/

/* Enter / exit thresholds (Q4 dB) */
#define RSSI_ENTER_NEAR_DBM     (-50)
#define RSSI_EXIT_NEAR_DBM      (-60)
#define RSSI_ENTER_NEAR_Q4      ((int16_t)(RSSI_ENTER_NEAR_DBM * 16))
#define RSSI_EXIT_NEAR_Q4       ((int16_t)(RSSI_EXIT_NEAR_DBM * 16))

/* Stability gate for CANDIDATE -> UNLOCK */
#define RSSI_STD_TH_Q4          ((uint16_t)40U)       /* Max StdDev: 2.5 dB in Q4 */
#define RSSI_PCT_TH_Q15         ((uint16_t)16384U)    /* Min pctAbove: ~0.50 */
#define RSSI_STABLE_MS          (2000U)               /* Hold time in CANDIDATE (ms) */

/* Exit confirmation and lockout */
#define RSSI_EXIT_CONFIRM_MS    (1500U)   /* Signal below exit for 1.5s => lock */
#define RSSI_LOCKOUT_MS         (5000U)   /* After unlock, lockout for 5s */

/* Legacy thresholds (backward compat with proximity_state_machine.c) */
#define RSSI_UNLOCK_THRESHOLD   (RSSI_ENTER_NEAR_DBM)
#define RSSI_LOCK_THRESHOLD     (RSSI_EXIT_NEAR_DBM)
#define RSSI_UNLOCK_THR         (RSSI_ENTER_NEAR_DBM)
#define RSSI_LOCK_THR           (RSSI_EXIT_NEAR_DBM)

/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/

/* RSSI proximity state */
typedef enum
{
    RssiState_Idle_c     = 0,   /* Not initialized / no signal       */
    RssiState_Locked_c   = 1,   /* FAR  - signal weak or absent      */
    RssiState_Approach_c = 2,   /* CANDIDATE - checking stability    */
    RssiState_Unlocked_c = 3    /* LOCKOUT - unlock fired, cooldown  */
} rssiState_t;

/* Events emitted by state machine */
typedef enum
{
    RssiEvent_None_c             = 0,
    RssiEvent_CandidateStarted_c = 1,
    RssiEvent_UnlockTriggered_c  = 2,
    RssiEvent_ExitToFar_c        = 3
} rssiEvent_t;

/* Feature extraction output */
typedef struct
{
    uint16_t n;               /* Samples in window                          */
    uint16_t pctAboveQ15;     /* Fraction above enter threshold (Q15)       */
    uint16_t stdQ4;           /* Standard deviation (Q4 dB)                 */
    int16_t  lastQ4;          /* Last smoothed value (Q4)                   */
    int16_t  minQ4;           /* Min in window (Q4)                         */
    int16_t  maxQ4;           /* Max in window (Q4)                         */
} rssiFeatures_t;

/* Raw RSSI ring buffer */
typedef struct
{
    uint32_t tMs[RSSI_RAW_CAP];
    int8_t   rssiDbm[RSSI_RAW_CAP];
    uint16_t head;
    uint16_t count;
} rssiRawRing_t;

/* Smoothed RSSI ring buffer (Q4) */
typedef struct
{
    uint32_t tMs[RSSI_SMOOTH_CAP];
    int16_t  rssiQ4[RSSI_SMOOTH_CAP];
    uint16_t head;
    uint16_t count;
} rssiSmoothRing_t;

/* Main filter context - deterministic memory, no malloc */
typedef struct
{
    /* Ring buffers */
    rssiRawRing_t    raw;
    rssiSmoothRing_t smooth;

    /* EMA state */
    uint8_t  emaValid;        /* 0U = not initialized, 1U = valid */
    int16_t  emaQ4;           /* Current EMA value (Q4)           */
    uint32_t emaPrevMs;       /* Timestamp of last EMA update     */

    /* State machine */
    rssiState_t  currentState;
    rssiState_t  previousState;
    uint32_t     tCandidateStartMs;   /* CANDIDATE entry time       */
    uint32_t     tBelowExitStartMs;   /* Exit-confirm timer start   */
    uint32_t     tLockoutUntilMs;     /* Lockout expiry timestamp   */
    uint8_t      stateChanged;        /* 1U if state changed        */
    rssiEvent_t  lastEvent;           /* Last event emitted         */

    /* Cached features */
    rssiFeatures_t features;
    uint32_t lastUpdateTime;

    /* Scratch arrays (pre-allocated, no heap) */
    int16_t tmpA[RSSI_RAW_CAP];
    int16_t tmpB[RSSI_RAW_CAP];
    int16_t tmpS[RSSI_SMOOTH_CAP];
} rssiFilter_t;

/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*! *********************************************************************************
* \brief     Initialize RSSI filter (all stages reset)
*
* \param[in,out] pFilter  Pointer to filter context
********************************************************************************** */
void RssiFilter_Init(rssiFilter_t *pFilter);

/*! *********************************************************************************
* \brief     Push new raw RSSI sample and run full pipeline
*
* \param[in,out] pFilter  Pointer to filter context
* \param[in]     rssi     Raw RSSI measurement in dBm
********************************************************************************** */
void RssiFilter_AddMeasurement(rssiFilter_t *pFilter, int8_t rssi);

/*! *********************************************************************************
* \brief     Get current filtered RSSI (EMA output, integer dBm)
*
* \param[in] pFilter  Pointer to filter context
*
* \return    Filtered RSSI in dBm
********************************************************************************** */
int8_t RssiFilter_GetFilteredRssi(const rssiFilter_t *pFilter);

/*! *********************************************************************************
* \brief     Get current proximity state
*
* \param[in] pFilter  Pointer to filter context
*
* \return    Current state (RssiState_Idle_c .. RssiState_Unlocked_c)
********************************************************************************** */
rssiState_t RssiFilter_GetState(const rssiFilter_t *pFilter);

/*! *********************************************************************************
* \brief     Check and clear the state-changed flag
*
* \param[in,out] pFilter  Pointer to filter context
*
* \return    TRUE if state changed since last call
********************************************************************************** */
bool_t RssiFilter_HasStateChanged(rssiFilter_t *pFilter);

/*! *********************************************************************************
* \brief     Reset filter to initial state
*
* \param[in,out] pFilter  Pointer to filter context
********************************************************************************** */
void RssiFilter_Reset(rssiFilter_t *pFilter);

/*! *********************************************************************************
* \brief     Get feature extraction results for diagnostics
*
* \param[in]  pFilter    Pointer to filter context
* \param[out] pStdQ4     StdDev in Q4 (divide by 16 for dB). May be NULL.
* \param[out] pPctAbove  Percentage of samples above threshold (0-100). May be NULL.
* \param[out] pMean      Mean RSSI in dBm. May be NULL.
********************************************************************************** */
void RssiFilter_GetFeatures(const rssiFilter_t *pFilter, uint16_t *pStdQ4,
                            uint8_t *pPctAbove, int8_t *pMean);

/*! *********************************************************************************
* \brief     Get last event emitted by state machine
*
* \param[in] pFilter  Pointer to filter context
*
* \return    Last event (RssiEvent_None_c if no event since init/reset)
********************************************************************************** */
rssiEvent_t RssiFilter_GetLastEvent(const rssiFilter_t *pFilter);

#ifdef __cplusplus
}
#endif

#endif /* RSSI_FILTER_H */
