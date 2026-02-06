/*! *********************************************************************************
* \file rssi_filter.c
*
* \brief  RSSI filtering for KW47 keyless entry system
*         MISRA C:2012 / AUTOSAR coding guidelines compliant.
*         Pipeline: Hampel -> Adaptive EMA -> Feature Extraction -> State Machine
*         Fixed-point Q4 throughout. No float. No dynamic memory.
*
* Copyright 2025
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/

#include "rssi_filter.h"
#include "FunctionLib.h"
#include "fsl_component_timer_manager.h"

/************************************************************************************
*************************************************************************************
* Private helper declarations
*************************************************************************************
************************************************************************************/

/* Wrap-safe time difference: returns (a - b) as uint32_t, OK across 32-bit wrap */
static uint32_t RssiFilter_TimeDiff(uint32_t a, uint32_t b);

/* Ring buffer helpers */
static uint16_t RssiFilter_RingNext(uint16_t idx, uint16_t cap);
static uint16_t RssiFilter_RingTail(uint16_t head, uint16_t count, uint16_t cap);

/* Q4 conversion */
static int16_t  RssiFilter_DbmToQ4(int8_t dbm);
static int8_t   RssiFilter_Q4ToDbm(int16_t q4);

/* Q15 multiply: alpha(Q15) * delta(Q4) -> Q4 */
static int16_t  RssiFilter_MulQ15Q4(uint16_t alphaQ15, int16_t deltaQ4);

/* Deterministic insertion sort for int16_t arrays (N bounded by buffer cap) */
static void     RssiFilter_SortS16(int16_t *pArr, uint16_t n);

/* Median of sorted array */
static int16_t  RssiFilter_MedianS16(const int16_t *pSorted, uint16_t n);

/* Integer sqrt (deterministic, no float) */
static uint16_t RssiFilter_IsqrtU32(uint32_t x);

/* Ring buffer operations */
static void     RssiFilter_RawPush(rssiFilter_t *pCtx, uint32_t tMs, int8_t rssiDbm);
static void     RssiFilter_SmoothPush(rssiFilter_t *pCtx, uint32_t tMs, int16_t rssiQ4);
static void     RssiFilter_RawPrune(rssiFilter_t *pCtx, uint32_t nowMs, uint32_t winMs);
static void     RssiFilter_SmoothPrune(rssiFilter_t *pCtx, uint32_t nowMs, uint32_t winMs);

/* Copy time-windowed samples into scratch array (Q4) */
static uint16_t RssiFilter_CopyRawWindowQ4(const rssiFilter_t *pCtx,
                                           uint32_t nowMs, uint32_t winMs,
                                           int16_t *pOut, uint16_t cap);
static uint16_t RssiFilter_CopySmoothWindowQ4(const rssiFilter_t *pCtx,
                                              uint32_t nowMs, uint32_t winMs,
                                              int16_t *pOut, uint16_t cap,
                                              int16_t *pLastQ4);

/* Pipeline stages */
static uint8_t  RssiFilter_HampelReject(rssiFilter_t *pCtx, uint32_t nowMs, int16_t *pOutQ4);
static void     RssiFilter_EmaUpdate(rssiFilter_t *pCtx, uint32_t nowMs, int16_t xQ4);
static uint8_t  RssiFilter_ComputeFeatures(rssiFilter_t *pCtx, uint32_t nowMs);
static rssiEvent_t RssiFilter_StateStep(rssiFilter_t *pCtx, uint32_t nowMs);

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief     Initialize RSSI filter (all stages reset)
********************************************************************************** */
void RssiFilter_Init(rssiFilter_t *pFilter)
{
    if (pFilter == NULL)
    {
        return;
    }

    /* Raw ring */
    FLib_MemSet(&pFilter->raw, 0, sizeof(rssiRawRing_t));

    /* Smooth ring */
    FLib_MemSet(&pFilter->smooth, 0, sizeof(rssiSmoothRing_t));

    /* EMA */
    pFilter->emaValid  = 0U;
    pFilter->emaQ4     = (int16_t)(-100 * 16);
    pFilter->emaPrevMs = 0U;

    /* State machine */
    pFilter->currentState      = RssiState_Idle_c;
    pFilter->previousState     = RssiState_Idle_c;
    pFilter->tCandidateStartMs = 0U;
    pFilter->tBelowExitStartMs = 0U;
    pFilter->tLockoutUntilMs   = 0U;
    pFilter->stateChanged      = 0U;
    pFilter->lastEvent         = RssiEvent_None_c;

    /* Features */
    FLib_MemSet(&pFilter->features, 0, sizeof(rssiFeatures_t));
    pFilter->lastUpdateTime = 0U;

    /* Scratch arrays zeroed implicitly by memset if struct is static,
     * but we do not rely on that — they are written before read. */
}

/*! *********************************************************************************
* \brief     Push raw RSSI sample and run full pipeline
********************************************************************************** */
void RssiFilter_AddMeasurement(rssiFilter_t *pFilter, int8_t rssi)
{
    int16_t xQ4;
    int16_t emaQ4Out;
    uint8_t hampelOk;
    rssiEvent_t ev;
    uint32_t nowMs;

    if (pFilter == NULL)
    {
        return;
    }

    nowMs = TM_GetTimestamp();

    /* Clamp input (MISRA: defensive) */
    if (rssi < (int8_t)-127)
    {
        rssi = (int8_t)-127;
    }
    else if (rssi > (int8_t)20)
    {
        rssi = (int8_t)20;
    }
    else
    {
        /* No clamp needed */
    }

    /* Push raw sample */
    RssiFilter_RawPush(pFilter, nowMs, rssi);

    /* Prune old samples */
    RssiFilter_RawPrune(pFilter, nowMs, (uint32_t)RSSI_HAMPEL_WIN_MS * 2U);
    RssiFilter_SmoothPrune(pFilter, nowMs, (uint32_t)RSSI_FEAT_WIN_MS);

    /* Stage 1: Hampel spike rejection */
    hampelOk = RssiFilter_HampelReject(pFilter, nowMs, &xQ4);
    if (hampelOk == 0U)
    {
        pFilter->lastUpdateTime = nowMs;
        return;  /* Not enough data yet */
    }

    /* Stage 2: Adaptive EMA */
    RssiFilter_EmaUpdate(pFilter, nowMs, xQ4);
    emaQ4Out = pFilter->emaQ4;

    /* Push smoothed to smooth ring */
    RssiFilter_SmoothPush(pFilter, nowMs, emaQ4Out);
    RssiFilter_SmoothPrune(pFilter, nowMs, (uint32_t)RSSI_FEAT_WIN_MS);

    /* Stage 3: Feature extraction */
    (void)RssiFilter_ComputeFeatures(pFilter, nowMs);

    /* Stage 4: State machine */
    {
        rssiState_t prevSt = pFilter->currentState;
        ev = RssiFilter_StateStep(pFilter, nowMs);

        /* Only overwrite lastEvent when an actual event fires */
        if (ev != RssiEvent_None_c)
        {
            pFilter->lastEvent = ev;
        }

        if (pFilter->currentState != prevSt)
        {
            pFilter->stateChanged = 1U;
        }
    }

    pFilter->lastUpdateTime = nowMs;
}

/*! *********************************************************************************
* \brief     Get current filtered RSSI (EMA output, integer dBm)
********************************************************************************** */
int8_t RssiFilter_GetFilteredRssi(const rssiFilter_t *pFilter)
{
    if (pFilter == NULL)
    {
        return (int8_t)-100;
    }

    if (pFilter->emaValid != 0U)
    {
        return RssiFilter_Q4ToDbm(pFilter->emaQ4);
    }

    return (int8_t)-100;
}

/*! *********************************************************************************
* \brief     Get current proximity state
********************************************************************************** */
rssiState_t RssiFilter_GetState(const rssiFilter_t *pFilter)
{
    if (pFilter == NULL)
    {
        return RssiState_Idle_c;
    }

    return pFilter->currentState;
}

/*! *********************************************************************************
* \brief     Check and clear the state-changed flag
********************************************************************************** */
bool_t RssiFilter_HasStateChanged(rssiFilter_t *pFilter)
{
    bool_t changed;

    if (pFilter == NULL)
    {
        return (bool_t)FALSE;
    }

    changed = (pFilter->stateChanged != 0U) ? (bool_t)TRUE : (bool_t)FALSE;
    pFilter->stateChanged = 0U;
    return changed;
}

/*! *********************************************************************************
* \brief     Reset filter to initial state
********************************************************************************** */
void RssiFilter_Reset(rssiFilter_t *pFilter)
{
    if (pFilter == NULL)
    {
        return;
    }

    RssiFilter_Init(pFilter);
}

/*! *********************************************************************************
* \brief     Get feature extraction results for diagnostics
*            Converts internal Q4/Q15 to user-friendly units at the API boundary.
********************************************************************************** */
void RssiFilter_GetFeatures(const rssiFilter_t *pFilter, uint16_t *pStdQ4,
                            uint8_t *pPctAbove, int8_t *pMean)
{
    if (pFilter == NULL)
    {
        return;
    }

    if (pStdQ4 != NULL)
    {
        *pStdQ4 = pFilter->features.stdQ4;
    }

    if (pPctAbove != NULL)
    {
        /* Convert Q15 fraction to percentage (0..100) */
        *pPctAbove = (uint8_t)(((uint32_t)pFilter->features.pctAboveQ15 * 100U) / (uint32_t)RSSI_Q15_ONE);
    }

    if (pMean != NULL)
    {
        if (pFilter->features.n > 0U)
        {
            *pMean = RssiFilter_Q4ToDbm(pFilter->features.lastQ4);
        }
        else
        {
            *pMean = (int8_t)-100;
        }
    }
}

/*! *********************************************************************************
* \brief     Get last event emitted by state machine
********************************************************************************** */
rssiEvent_t RssiFilter_GetLastEvent(const rssiFilter_t *pFilter)
{
    if (pFilter == NULL)
    {
        return RssiEvent_None_c;
    }

    return pFilter->lastEvent;
}

/************************************************************************************
*************************************************************************************
* Private utility functions
*************************************************************************************
************************************************************************************/

static uint32_t RssiFilter_TimeDiff(uint32_t a, uint32_t b)
{
    return (uint32_t)(a - b);
}

static uint16_t RssiFilter_RingNext(uint16_t idx, uint16_t cap)
{
    idx++;
    if (idx >= cap)
    {
        idx = 0U;
    }
    return idx;
}

static uint16_t RssiFilter_RingTail(uint16_t head, uint16_t count, uint16_t cap)
{
    uint32_t h = (uint32_t)head;
    uint32_t c = (uint32_t)count;
    uint32_t t = (h + (uint32_t)cap - (c % (uint32_t)cap)) % (uint32_t)cap;
    return (uint16_t)t;
}

static int16_t RssiFilter_DbmToQ4(int8_t dbm)
{
    return (int16_t)((int16_t)dbm * RSSI_Q4_SCALE);
}

static int8_t RssiFilter_Q4ToDbm(int16_t q4)
{
    /* Round toward zero */
    int16_t rounded;
    if (q4 >= (int16_t)0)
    {
        rounded = (int16_t)((q4 + (int16_t)8) / RSSI_Q4_SCALE);
    }
    else
    {
        rounded = (int16_t)((q4 - (int16_t)8) / RSSI_Q4_SCALE);
    }
    return (int8_t)rounded;
}

static int16_t RssiFilter_MulQ15Q4(uint16_t alphaQ15, int16_t deltaQ4)
{
    int32_t prod = (int32_t)alphaQ15 * (int32_t)deltaQ4;
    return (int16_t)(prod >> 15);
}

static void RssiFilter_SortS16(int16_t *pArr, uint16_t n)
{
    uint16_t i;
    for (i = 1U; i < n; i++)
    {
        int16_t key = pArr[i];
        uint16_t j = i;
        while ((j > 0U) && (pArr[j - 1U] > key))
        {
            pArr[j] = pArr[j - 1U];
            j--;
        }
        pArr[j] = key;
    }
}

static int16_t RssiFilter_MedianS16(const int16_t *pSorted, uint16_t n)
{
    return pSorted[n >> 1U];
}

static uint16_t RssiFilter_IsqrtU32(uint32_t x)
{
    uint32_t op  = x;
    uint32_t res = 0U;
    uint32_t one = 1UL << 30;

    while (one > op)
    {
        one >>= 2;
    }

    while (one != 0U)
    {
        if (op >= (res + one))
        {
            op  -= (res + one);
            res  = res + (2U * one);
        }
        res >>= 1;
        one >>= 2;
    }

    return (uint16_t)res;
}

/************************************************************************************
*************************************************************************************
* Ring buffer operations
*************************************************************************************
************************************************************************************/

static void RssiFilter_RawPush(rssiFilter_t *pCtx, uint32_t tMs, int8_t rssiDbm)
{
    pCtx->raw.tMs[pCtx->raw.head]     = tMs;
    pCtx->raw.rssiDbm[pCtx->raw.head] = rssiDbm;
    pCtx->raw.head = RssiFilter_RingNext(pCtx->raw.head, (uint16_t)RSSI_RAW_CAP);

    if (pCtx->raw.count < (uint16_t)RSSI_RAW_CAP)
    {
        pCtx->raw.count++;
    }
}

static void RssiFilter_SmoothPush(rssiFilter_t *pCtx, uint32_t tMs, int16_t rssiQ4)
{
    pCtx->smooth.tMs[pCtx->smooth.head]     = tMs;
    pCtx->smooth.rssiQ4[pCtx->smooth.head]  = rssiQ4;
    pCtx->smooth.head = RssiFilter_RingNext(pCtx->smooth.head, (uint16_t)RSSI_SMOOTH_CAP);

    if (pCtx->smooth.count < (uint16_t)RSSI_SMOOTH_CAP)
    {
        pCtx->smooth.count++;
    }
}

static void RssiFilter_RawPrune(rssiFilter_t *pCtx, uint32_t nowMs, uint32_t winMs)
{
    uint16_t tail;
    uint16_t remaining;
    uint32_t minT;

    if (pCtx->raw.count == 0U)
    {
        return;
    }

    tail = RssiFilter_RingTail(pCtx->raw.head, pCtx->raw.count, (uint16_t)RSSI_RAW_CAP);
    remaining = pCtx->raw.count;
    minT = (RssiFilter_TimeDiff(nowMs, 0U) >= winMs) ? (nowMs - winMs) : 0U;

    while (remaining > 0U)
    {
        if (pCtx->raw.tMs[tail] >= minT)
        {
            break;
        }
        tail = RssiFilter_RingNext(tail, (uint16_t)RSSI_RAW_CAP);
        remaining--;
    }

    pCtx->raw.count = remaining;
}

static void RssiFilter_SmoothPrune(rssiFilter_t *pCtx, uint32_t nowMs, uint32_t winMs)
{
    uint16_t tail;
    uint16_t remaining;
    uint32_t minT;

    if (pCtx->smooth.count == 0U)
    {
        return;
    }

    tail = RssiFilter_RingTail(pCtx->smooth.head, pCtx->smooth.count, (uint16_t)RSSI_SMOOTH_CAP);
    remaining = pCtx->smooth.count;
    minT = (RssiFilter_TimeDiff(nowMs, 0U) >= winMs) ? (nowMs - winMs) : 0U;

    while (remaining > 0U)
    {
        if (pCtx->smooth.tMs[tail] >= minT)
        {
            break;
        }
        tail = RssiFilter_RingNext(tail, (uint16_t)RSSI_SMOOTH_CAP);
        remaining--;
    }

    pCtx->smooth.count = remaining;
}

static uint16_t RssiFilter_CopyRawWindowQ4(const rssiFilter_t *pCtx,
                                           uint32_t nowMs, uint32_t winMs,
                                           int16_t *pOut, uint16_t cap)
{
    uint16_t n = 0U;
    uint16_t idx;
    uint16_t i;
    uint32_t minT;

    if (pCtx->raw.count == 0U)
    {
        return 0U;
    }

    minT = (RssiFilter_TimeDiff(nowMs, 0U) >= winMs) ? (nowMs - winMs) : 0U;
    idx = RssiFilter_RingTail(pCtx->raw.head, pCtx->raw.count, (uint16_t)RSSI_RAW_CAP);

    for (i = 0U; i < pCtx->raw.count; i++)
    {
        if ((pCtx->raw.tMs[idx] >= minT) && (n < cap))
        {
            pOut[n] = RssiFilter_DbmToQ4(pCtx->raw.rssiDbm[idx]);
            n++;
        }
        idx = RssiFilter_RingNext(idx, (uint16_t)RSSI_RAW_CAP);
    }

    return n;
}

static uint16_t RssiFilter_CopySmoothWindowQ4(const rssiFilter_t *pCtx,
                                              uint32_t nowMs, uint32_t winMs,
                                              int16_t *pOut, uint16_t cap,
                                              int16_t *pLastQ4)
{
    uint16_t n = 0U;
    uint16_t idx;
    uint16_t i;
    uint32_t minT;
    int16_t last = 0;

    if (pCtx->smooth.count == 0U)
    {
        *pLastQ4 = 0;
        return 0U;
    }

    minT = (RssiFilter_TimeDiff(nowMs, 0U) >= winMs) ? (nowMs - winMs) : 0U;
    idx = RssiFilter_RingTail(pCtx->smooth.head, pCtx->smooth.count, (uint16_t)RSSI_SMOOTH_CAP);

    for (i = 0U; i < pCtx->smooth.count; i++)
    {
        if ((pCtx->smooth.tMs[idx] >= minT) && (n < cap))
        {
            pOut[n] = pCtx->smooth.rssiQ4[idx];
            last = pCtx->smooth.rssiQ4[idx];
            n++;
        }
        idx = RssiFilter_RingNext(idx, (uint16_t)RSSI_SMOOTH_CAP);
    }

    *pLastQ4 = last;
    return n;
}

/************************************************************************************
*************************************************************************************
* Stage 1: Hampel spike rejection (time-windowed, Q4, with MAD floor)
*************************************************************************************
************************************************************************************/

static uint8_t RssiFilter_HampelReject(rssiFilter_t *pCtx, uint32_t nowMs, int16_t *pOutQ4)
{
    uint16_t n;
    int16_t medQ4;
    int16_t madQ4;
    int32_t prodQ8;
    int32_t thrQ8;
    int16_t thrQ4;
    uint16_t lastIdx;
    int16_t xLatestQ4;
    int16_t diff;
    int16_t absDiff;
    uint16_t i;

    n = RssiFilter_CopyRawWindowQ4(pCtx, nowMs, (uint32_t)RSSI_HAMPEL_WIN_MS,
                                   pCtx->tmpA, (uint16_t)RSSI_RAW_CAP);
    if (n < 3U)
    {
        return 0U;  /* Not enough data */
    }

    /* Sort and find median */
    RssiFilter_SortS16(pCtx->tmpA, n);
    medQ4 = RssiFilter_MedianS16(pCtx->tmpA, n);

    /* Compute MAD = median(|Xi - median|) */
    for (i = 0U; i < n; i++)
    {
        int16_t d = (int16_t)(pCtx->tmpA[i] - medQ4);
        pCtx->tmpB[i] = (d < (int16_t)0) ? (int16_t)(-d) : d;
    }
    RssiFilter_SortS16(pCtx->tmpB, n);
    madQ4 = RssiFilter_MedianS16(pCtx->tmpB, n);

    if (madQ4 < (int16_t)RSSI_MAD_EPS_Q4)
    {
        madQ4 = (int16_t)RSSI_MAD_EPS_Q4;
    }

    /* threshold = K * 1.5 * MAD  (K is Q4, MAD is Q4 => product is Q8) */
    prodQ8 = (int32_t)RSSI_HAMPEL_K_Q4 * (int32_t)madQ4;
    thrQ8  = (prodQ8 * (int32_t)3) / (int32_t)2;
    thrQ4  = (int16_t)(thrQ8 / (int32_t)RSSI_Q4_SCALE);

    /* Get latest raw sample in Q4 */
    lastIdx = (pCtx->raw.head == 0U)
              ? ((uint16_t)RSSI_RAW_CAP - 1U)
              : (uint16_t)(pCtx->raw.head - 1U);
    xLatestQ4 = RssiFilter_DbmToQ4(pCtx->raw.rssiDbm[lastIdx]);

    diff    = (int16_t)(xLatestQ4 - medQ4);
    absDiff = (diff < (int16_t)0) ? (int16_t)(-diff) : diff;

    *pOutQ4 = (absDiff > thrQ4) ? medQ4 : xLatestQ4;
    return 1U;  /* Success */
}

/************************************************************************************
*************************************************************************************
* Stage 2: Adaptive EMA (Q4, dt-based alpha, time anomaly handling)
*************************************************************************************
************************************************************************************/

static void RssiFilter_EmaUpdate(rssiFilter_t *pCtx, uint32_t nowMs, int16_t xQ4)
{
    uint32_t dtMs;
    uint16_t alphaQ15;
    int16_t deltaQ4;
    int16_t stepQ4;

    if (pCtx->emaValid == 0U)
    {
        pCtx->emaValid  = 1U;
        pCtx->emaQ4     = xQ4;
        pCtx->emaPrevMs = nowMs;
        return;
    }

    dtMs = RssiFilter_TimeDiff(nowMs, pCtx->emaPrevMs);

    /* Time anomaly: full reset (safety-first) */
    if ((dtMs == 0U) || (dtMs > (uint32_t)RSSI_EMA_ANOMALY_DT_MS))
    {
        pCtx->emaQ4     = xQ4;
        pCtx->emaPrevMs = nowMs;
        return;
    }

    /* Interpolate alpha in Q15 from dt */
    if (dtMs <= (uint32_t)RSSI_EMA_DT_MIN_MS)
    {
        alphaQ15 = (uint16_t)RSSI_EMA_ALPHA_MIN_Q15;
    }
    else if (dtMs >= (uint32_t)RSSI_EMA_DT_MAX_MS)
    {
        alphaQ15 = (uint16_t)RSSI_EMA_ALPHA_MAX_Q15;
    }
    else
    {
        /* Linear interpolation */
        uint32_t range  = (uint32_t)RSSI_EMA_ALPHA_MAX_Q15 - (uint32_t)RSSI_EMA_ALPHA_MIN_Q15;
        uint32_t dtOff  = dtMs - (uint32_t)RSSI_EMA_DT_MIN_MS;
        uint32_t dtSpan = (uint32_t)RSSI_EMA_DT_MAX_MS - (uint32_t)RSSI_EMA_DT_MIN_MS;
        alphaQ15 = (uint16_t)((uint32_t)RSSI_EMA_ALPHA_MIN_Q15 + ((range * dtOff) / dtSpan));
    }

    /* EMA step: ema += alpha * (x - ema) */
    deltaQ4 = (int16_t)(xQ4 - pCtx->emaQ4);
    stepQ4  = RssiFilter_MulQ15Q4(alphaQ15, deltaQ4);
    pCtx->emaQ4     = (int16_t)(pCtx->emaQ4 + stepQ4);
    pCtx->emaPrevMs = nowMs;
}

/************************************************************************************
*************************************************************************************
* Stage 3: Feature extraction (pctAbove + stdDev, deterministic, Q4)
*************************************************************************************
************************************************************************************/

static uint8_t RssiFilter_ComputeFeatures(rssiFilter_t *pCtx, uint32_t nowMs)
{
    uint16_t n;
    int16_t lastQ4 = 0;
    int64_t sumQ4  = 0;
    int64_t sumSqQ8 = 0;
    uint32_t cntAbove = 0U;
    int16_t mn;
    int16_t mx;
    uint16_t i;
    uint16_t stdQ4 = 0U;

    n = RssiFilter_CopySmoothWindowQ4(pCtx, nowMs, (uint32_t)RSSI_FEAT_WIN_MS,
                                      pCtx->tmpS, (uint16_t)RSSI_SMOOTH_CAP, &lastQ4);

    if (n < (uint16_t)RSSI_MIN_FEAT_SAMPLES)
    {
        pCtx->features.n            = n;
        pCtx->features.pctAboveQ15  = 0U;
        pCtx->features.stdQ4        = 0xFFFFU;
        pCtx->features.lastQ4       = lastQ4;
        pCtx->features.minQ4        = lastQ4;
        pCtx->features.maxQ4        = lastQ4;
        return 0U;  /* Not enough data */
    }

    mn = pCtx->tmpS[0];
    mx = pCtx->tmpS[0];

    for (i = 0U; i < n; i++)
    {
        int16_t xQ4 = pCtx->tmpS[i];
        sumQ4  += (int64_t)xQ4;
        sumSqQ8 += (int64_t)xQ4 * (int64_t)xQ4;

        if (xQ4 >= (int16_t)RSSI_ENTER_NEAR_Q4)
        {
            cntAbove++;
        }
        if (xQ4 < mn) { mn = xQ4; }
        if (xQ4 > mx) { mx = xQ4; }
    }

    /* Variance in Q8: (sumSq - sum^2/n) / (n-1) */
    if (n > 1U)
    {
        int64_t meanSqTerm = (sumQ4 * sumQ4) / (int64_t)n;
        int64_t diffV = sumSqQ8 - meanSqTerm;
        if (diffV < (int64_t)0) { diffV = 0; }
        uint32_t varQ8 = (uint32_t)(diffV / (int64_t)(n - 1U));
        stdQ4 = RssiFilter_IsqrtU32(varQ8);
    }
    else
    {
        stdQ4 = 0xFFFFU;
    }

    pCtx->features.n            = n;
    pCtx->features.pctAboveQ15  = (uint16_t)((cntAbove * (uint32_t)RSSI_Q15_ONE) / (uint32_t)n);
    pCtx->features.stdQ4        = stdQ4;
    pCtx->features.lastQ4       = lastQ4;
    pCtx->features.minQ4        = mn;
    pCtx->features.maxQ4        = mx;
    return 1U;  /* Success */
}

/************************************************************************************
*************************************************************************************
* Stage 4: State machine — FAR / CANDIDATE / LOCKOUT
*
* FAR (Locked):
*   lastQ4 >= enterNearQ4 -> CANDIDATE (event: CandidateStarted)
*
* CANDIDATE (Approach):
*   Stable for stableMs AND features pass -> LOCKOUT (event: UnlockTriggered)
*   lastQ4 < exitNearQ4 for exitConfirmMs -> FAR (event: ExitToFar)
*
* LOCKOUT (Unlocked):
*   Wait lockoutMs, then:
*     lastQ4 < exitNearQ4 for exitConfirmMs -> FAR (event: ExitToFar)
*************************************************************************************
************************************************************************************/

static rssiEvent_t RssiFilter_StateStep(rssiFilter_t *pCtx, uint32_t nowMs)
{
    int16_t lastQ4  = pCtx->features.lastQ4;
    uint8_t isStable;

    /* ---- LOCKOUT state ---- */
    if (pCtx->currentState == RssiState_Unlocked_c)
    {
        /* Still in lockout period? */
        if (nowMs < pCtx->tLockoutUntilMs)
        {
            return RssiEvent_None_c;
        }

        /* Lockout expired: check if signal dropped */
        if (lastQ4 < (int16_t)RSSI_EXIT_NEAR_Q4)
        {
            if (pCtx->tBelowExitStartMs == 0U)
            {
                pCtx->tBelowExitStartMs = nowMs;
            }

            if (RssiFilter_TimeDiff(nowMs, pCtx->tBelowExitStartMs) >= (uint32_t)RSSI_EXIT_CONFIRM_MS)
            {
                pCtx->previousState     = pCtx->currentState;
                pCtx->currentState      = RssiState_Locked_c;
                pCtx->tBelowExitStartMs = 0U;
                return RssiEvent_ExitToFar_c;
            }
        }
        else
        {
            pCtx->tBelowExitStartMs = 0U;
        }

        return RssiEvent_None_c;
    }

    /* ---- IDLE state ---- */
    if (pCtx->currentState == RssiState_Idle_c)
    {
        if (pCtx->emaValid != 0U)
        {
            pCtx->previousState = pCtx->currentState;
            pCtx->currentState  = RssiState_Locked_c;
        }
        return RssiEvent_None_c;
    }

    /* ---- FAR state ---- */
    if (pCtx->currentState == RssiState_Locked_c)
    {
        if (lastQ4 >= (int16_t)RSSI_ENTER_NEAR_Q4)
        {
            pCtx->previousState      = pCtx->currentState;
            pCtx->currentState       = RssiState_Approach_c;
            pCtx->tCandidateStartMs  = nowMs;
            pCtx->tBelowExitStartMs  = 0U;
            return RssiEvent_CandidateStarted_c;
        }

        return RssiEvent_None_c;
    }

    /* ---- CANDIDATE state ---- */
    /* Check exit condition: signal below exit for exitConfirmMs */
    if (lastQ4 < (int16_t)RSSI_EXIT_NEAR_Q4)
    {
        if (pCtx->tBelowExitStartMs == 0U)
        {
            pCtx->tBelowExitStartMs = nowMs;
        }

        if (RssiFilter_TimeDiff(nowMs, pCtx->tBelowExitStartMs) >= (uint32_t)RSSI_EXIT_CONFIRM_MS)
        {
            pCtx->previousState      = pCtx->currentState;
            pCtx->currentState       = RssiState_Locked_c;
            pCtx->tBelowExitStartMs  = 0U;
            pCtx->tCandidateStartMs  = 0U;
            return RssiEvent_ExitToFar_c;
        }
    }
    else
    {
        pCtx->tBelowExitStartMs = 0U;
    }

    /* Check unlock condition: stable for stableMs */
    isStable = ((pCtx->features.pctAboveQ15 >= (uint16_t)RSSI_PCT_TH_Q15) &&
                (pCtx->features.stdQ4 <= (uint16_t)RSSI_STD_TH_Q4)) ? 1U : 0U;

    if (isStable != 0U)
    {
        if (RssiFilter_TimeDiff(nowMs, pCtx->tCandidateStartMs) >= (uint32_t)RSSI_STABLE_MS)
        {
            pCtx->previousState      = pCtx->currentState;
            pCtx->currentState       = RssiState_Unlocked_c;
            pCtx->tLockoutUntilMs    = nowMs + (uint32_t)RSSI_LOCKOUT_MS;
            pCtx->tBelowExitStartMs  = 0U;
            return RssiEvent_UnlockTriggered_c;
        }
    }
    else
    {
        /* Reset candidate timer on instability */
        pCtx->tCandidateStartMs = nowMs;
    }

    return RssiEvent_None_c;
}
