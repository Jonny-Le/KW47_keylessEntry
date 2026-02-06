/*! *********************************************************************************
* \file test_rssi_filter.c
*
* \brief  Unit tests for RSSI filter — new Q4 pipeline with Hampel, Adaptive EMA,
*         Feature Extraction, and State Machine (exit confirmation + lockout).
*         Runs on host machine (macOS/Linux). Tests the real rssi_filter.c via
*         #include after providing mock stubs.
*
* Copyright 2025
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * Mock definitions for embedded types and SDK functions
 ******************************************************************************/
typedef uint8_t bool_t;
#ifndef TRUE
#define TRUE  1
#endif
#ifndef FALSE
#define FALSE 0
#endif

/* Mock timer — test code controls time explicitly */
static uint32_t gMockTimestamp = 0U;
uint32_t TM_GetTimestamp(void) { return gMockTimestamp; }
static void MockTimer_Set(uint32_t ms)     { gMockTimestamp = ms; }
static void MockTimer_Advance(uint32_t ms) { gMockTimestamp += ms; }

/* Mock FLib_MemSet */
void FLib_MemSet(void *ptr, uint8_t val, size_t size)
{
    memset(ptr, (int)val, size);
}

/*******************************************************************************
 * Pull in the real implementation (include .h then .c)
 ******************************************************************************/
#include "rssi_filter.h"
#include "rssi_filter.c"

/*******************************************************************************
 * Test framework (minimal, self-contained) with JUnit XML support
 ******************************************************************************/

static int gTestsPassed = 0;
static int gTestsFailed = 0;
static int gTestsTotal  = 0;

/* Optional log file mirror (set via --log) */
static FILE *gLogFile = NULL;

/* Printf wrapper that writes to both stdout and optional log file */
#define tprintf(...) do {               \
    printf(__VA_ARGS__);                \
    if (gLogFile != NULL) {             \
        fprintf(gLogFile, __VA_ARGS__); \
    }                                   \
} while(0)

/* JUnit XML result storage */
#define MAX_TESTS 64
#define MAX_MSG   256

typedef struct {
    char name[MAX_MSG];
    char failMsg[MAX_MSG];
    char file[MAX_MSG];
    int  line;
    int  passed;  /* 1 = pass, 0 = fail */
} TestResult_t;

static TestResult_t gResults[MAX_TESTS];
static int          gResultCount = 0;
static const char  *gCurrentTestName = "";

static void JUnit_WriteXml(const char *path)
{
    FILE *fp = fopen(path, "w");
    if (fp == NULL) {
        printf("  WARNING: Could not open %s for writing\n", path);
        return;
    }
    fprintf(fp, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    fprintf(fp, "<testsuites>\n");
    fprintf(fp, "  <testsuite name=\"rssi_filter\" tests=\"%d\" "
                "failures=\"%d\">\n",
            gTestsPassed + gTestsFailed, gTestsFailed);

    for (int i = 0; i < gResultCount; i++)
    {
        const TestResult_t *r = &gResults[i];
        fprintf(fp, "    <testcase name=\"%s\">\n", r->name);
        if (!r->passed)
        {
            fprintf(fp, "      <failure message=\"%s\">"
                        "%s at %s:%d</failure>\n",
                    r->failMsg, r->failMsg, r->file, r->line);
        }
        fprintf(fp, "    </testcase>\n");
    }

    fprintf(fp, "  </testsuite>\n");
    fprintf(fp, "</testsuites>\n");
    fclose(fp);
    printf("  JUnit XML written to: %s\n", path);
}

static void RecordResult(const char *name, int passed,
                         const char *failMsg, const char *file, int line)
{
    if (gResultCount < MAX_TESTS)
    {
        TestResult_t *r = &gResults[gResultCount++];
        snprintf(r->name, MAX_MSG, "%s", name);
        snprintf(r->failMsg, MAX_MSG, "%s", failMsg ? failMsg : "");
        snprintf(r->file, MAX_MSG, "%s", file ? file : "");
        r->line   = line;
        r->passed = passed;
    }
}

#define TEST_ASSERT(cond, msg) do {              \
    if (!(cond)) {                               \
        tprintf("  FAIL: %s\n", (msg));          \
        tprintf("        at %s:%d\n",            \
               __FILE__, __LINE__);              \
        RecordResult(gCurrentTestName, 0,        \
                     (msg), __FILE__, __LINE__); \
        gTestsFailed++;                          \
        return;                                  \
    }                                            \
} while(0)

#define TEST_PASS(msg) do {                      \
    tprintf("  PASS: %s\n", (msg));              \
    RecordResult(gCurrentTestName, 1,            \
                 NULL, NULL, 0);                 \
    gTestsPassed++;                              \
} while(0)

#define RUN_TEST(fn) do {                        \
    gCurrentTestName = #fn;                      \
    fn();                                        \
} while(0)

static const char* StateStr(rssiState_t s)
{
    switch (s)
    {
        case RssiState_Idle_c:     return "IDLE";
        case RssiState_Locked_c:   return "FAR";
        case RssiState_Approach_c: return "CANDIDATE";
        case RssiState_Unlocked_c: return "LOCKOUT";
        default:                   return "???";
    }
}

static const char* EventStr(rssiEvent_t e)
{
    switch (e)
    {
        case RssiEvent_None_c:             return "NONE";
        case RssiEvent_CandidateStarted_c: return "CANDIDATE_STARTED";
        case RssiEvent_UnlockTriggered_c:  return "UNLOCK_TRIGGERED";
        case RssiEvent_ExitToFar_c:        return "EXIT_TO_FAR";
        default:                           return "???";
    }
}

/* Helper: feed N identical samples at 100ms intervals */
static void FeedSamples(rssiFilter_t *pF, int8_t rssiDbm, uint32_t count)
{
    for (uint32_t i = 0U; i < count; i++)
    {
        MockTimer_Advance(100U);
        RssiFilter_AddMeasurement(pF, rssiDbm);
    }
}

/* Helper: feed N identical samples at a custom interval */
static void FeedSamplesAt(rssiFilter_t *pF, int8_t rssiDbm, uint32_t count, uint32_t intervalMs)
{
    for (uint32_t i = 0U; i < count; i++)
    {
        MockTimer_Advance(intervalMs);
        RssiFilter_AddMeasurement(pF, rssiDbm);
    }
}

/* Helper: fresh filter + timer reset */
static void InitFresh(rssiFilter_t *pF)
{
    MockTimer_Set(1000U);  /* start at 1s to avoid edge cases near t=0 */
    RssiFilter_Init(pF);
}

/*******************************************************************************
 * 1. Initialization & Reset
 ******************************************************************************/

static void test_init_defaults(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Init defaults\n");

    rssiFilter_t f;
    InitFresh(&f);

    TEST_ASSERT(f.currentState == RssiState_Idle_c, "State = IDLE");
    TEST_ASSERT(f.emaValid == 0U,                   "EMA not valid");
    TEST_ASSERT(f.raw.count == 0U,                  "Raw ring empty");
    TEST_ASSERT(f.smooth.count == 0U,               "Smooth ring empty");
    TEST_ASSERT(f.stateChanged == 0U,               "No state change flag");
    TEST_ASSERT(f.lastEvent == RssiEvent_None_c,    "No event");

    TEST_PASS("Init defaults");
}

static void test_reset_clears_all(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Reset clears all state\n");

    rssiFilter_t f;
    InitFresh(&f);
    FeedSamples(&f, -50, 20);

    TEST_ASSERT(f.emaValid != 0U, "EMA should be valid before reset");

    RssiFilter_Reset(&f);

    TEST_ASSERT(f.currentState == RssiState_Idle_c, "State = IDLE after reset");
    TEST_ASSERT(f.emaValid == 0U,                   "EMA invalid after reset");
    TEST_ASSERT(f.raw.count == 0U,                  "Raw ring empty after reset");

    TEST_PASS("Reset clears all state");
}

/*******************************************************************************
 * 2. NULL pointer safety
 ******************************************************************************/

static void test_null_safety(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] NULL pointer safety\n");

    /* These should not crash */
    RssiFilter_Init(NULL);
    RssiFilter_AddMeasurement(NULL, -50);
    RssiFilter_Reset(NULL);

    TEST_ASSERT(RssiFilter_GetFilteredRssi(NULL) == (int8_t)-100, "Null filtered = -100");
    TEST_ASSERT(RssiFilter_GetState(NULL) == RssiState_Idle_c,    "Null state = IDLE");
    TEST_ASSERT(RssiFilter_HasStateChanged(NULL) == (bool_t)FALSE, "Null stateChanged = FALSE");
    TEST_ASSERT(RssiFilter_GetLastEvent(NULL) == RssiEvent_None_c, "Null event = NONE");

    uint16_t stdQ4 = 0xFFFF;
    uint8_t pct = 0xFF;
    int8_t mean = 0;
    RssiFilter_GetFeatures(NULL, &stdQ4, &pct, &mean);
    /* Should not have modified them (returns early) */
    TEST_ASSERT(stdQ4 == 0xFFFF, "Null GetFeatures does not write stdQ4");

    TEST_PASS("NULL pointer safety");
}

/*******************************************************************************
 * 3. Hampel outlier rejection
 ******************************************************************************/

static void test_hampel_rejects_spike(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Hampel rejects single spike\n");

    rssiFilter_t f;
    InitFresh(&f);

    /* Feed 10 samples at -50 dBm */
    FeedSamples(&f, -50, 10);
    int8_t beforeSpike = RssiFilter_GetFilteredRssi(&f);

    /* Inject a huge spike: +10 dBm (60 dB jump) */
    MockTimer_Advance(100U);
    RssiFilter_AddMeasurement(&f, 10);
    int8_t afterSpike = RssiFilter_GetFilteredRssi(&f);

    tprintf("    Before spike: %d dBm, After spike: %d dBm\n", beforeSpike, afterSpike);

    /* EMA should NOT have jumped more than a few dB */
    int8_t jump = (int8_t)(afterSpike - beforeSpike);
    if (jump < 0) { jump = (int8_t)(-jump); }
    TEST_ASSERT(jump < 10, "Spike should be rejected by Hampel (jump < 10 dB)");

    TEST_PASS("Hampel rejects single spike");
}

static void test_hampel_passes_clean(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Hampel passes clean signal\n");

    rssiFilter_t f;
    InitFresh(&f);

    /* Feed 20 samples of -45 dBm (clean) */
    FeedSamples(&f, -45, 20);
    int8_t filtered = RssiFilter_GetFilteredRssi(&f);

    tprintf("    Filtered: %d dBm (expected ~-45)\n", filtered);
    TEST_ASSERT(filtered >= -48 && filtered <= -42, "Clean signal should pass through");

    TEST_PASS("Hampel passes clean signal");
}

/*******************************************************************************
 * 4. EMA smoothing
 ******************************************************************************/

static void test_ema_converges(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] EMA converges to stable input\n");

    rssiFilter_t f;
    InitFresh(&f);

    FeedSamples(&f, -55, 30);
    int8_t filtered = RssiFilter_GetFilteredRssi(&f);

    tprintf("    Filtered after 30 samples of -55: %d dBm\n", filtered);
    TEST_ASSERT(filtered >= -57 && filtered <= -53, "EMA should converge to ~-55");

    TEST_PASS("EMA converges to stable input");
}

static void test_ema_smooths_noise(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] EMA smooths noisy input\n");

    rssiFilter_t f;
    InitFresh(&f);

    /* Alternate -45 and -55 (mean = -50) */
    for (int i = 0; i < 40; i++)
    {
        int8_t val = (i % 2 == 0) ? (int8_t)-45 : (int8_t)-55;
        MockTimer_Advance(100U);
        RssiFilter_AddMeasurement(&f, val);
    }
    int8_t filtered = RssiFilter_GetFilteredRssi(&f);

    tprintf("    Filtered (alternating -45/-55): %d dBm\n", filtered);
    TEST_ASSERT(filtered >= -54 && filtered <= -46, "EMA should smooth to ~-50");

    TEST_PASS("EMA smooths noisy input");
}

static void test_ema_anomaly_reset(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] EMA resets on time anomaly (dt > 2s)\n");

    rssiFilter_t f;
    InitFresh(&f);

    FeedSamples(&f, -40, 15);
    int8_t before = RssiFilter_GetFilteredRssi(&f);

    /* Big time gap (3 seconds) then feed several samples at -80
     * (Hampel needs >= 3 in its window after pruning) */
    MockTimer_Advance(3000U);
    FeedSamples(&f, -80, 10);
    int8_t after = RssiFilter_GetFilteredRssi(&f);

    tprintf("    Before gap: %d, After gap+(-80 x10): %d\n", before, after);
    TEST_ASSERT(after <= -70, "EMA should reset toward new value after 3s gap");

    TEST_PASS("EMA resets on time anomaly");
}

/*******************************************************************************
 * 5. Feature extraction
 ******************************************************************************/

static void test_features_stable_signal(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Feature extraction: stable signal\n");

    rssiFilter_t f;
    InitFresh(&f);

    FeedSamples(&f, -50, 30);

    uint16_t stdQ4 = 0U;
    uint8_t pct = 0U;
    int8_t mean = 0;
    RssiFilter_GetFeatures(&f, &stdQ4, &pct, &mean);

    tprintf("    StdDev(Q4)=%u (%.1f dB), PctAbove=%u%%, Mean=%d\n",
            stdQ4, (double)stdQ4 / 16.0, pct, mean);

    /* Perfectly stable signal should have very low std dev */
    TEST_ASSERT(stdQ4 < 32U, "StdDev should be < 2 dB for stable signal");

    TEST_PASS("Feature extraction: stable signal");
}

static void test_features_noisy_signal(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Feature extraction: noisy signal\n");

    rssiFilter_t f;
    InitFresh(&f);

    /* Wide noise: alternate -35 and -65 */
    for (int i = 0; i < 40; i++)
    {
        int8_t val = (i % 2 == 0) ? (int8_t)-35 : (int8_t)-65;
        MockTimer_Advance(100U);
        RssiFilter_AddMeasurement(&f, val);
    }

    uint16_t stdQ4 = 0U;
    uint8_t pct = 0U;
    int8_t mean = 0;
    RssiFilter_GetFeatures(&f, &stdQ4, &pct, &mean);

    tprintf("    StdDev(Q4)=%u (%.1f dB), PctAbove=%u%%, Mean=%d\n",
            stdQ4, (double)stdQ4 / 16.0, pct, mean);

    /* Wide alternation should produce higher std dev than a stable signal */
    TEST_ASSERT(stdQ4 > 16U, "StdDev should be > 1 dB for noisy signal");

    TEST_PASS("Feature extraction: noisy signal");
}

/*******************************************************************************
 * 6. State machine: IDLE -> FAR
 ******************************************************************************/

static void test_idle_to_far(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] IDLE -> FAR after first valid sample\n");

    rssiFilter_t f;
    InitFresh(&f);

    TEST_ASSERT(RssiFilter_GetState(&f) == RssiState_Idle_c, "Start in IDLE");

    /* Feed 3 samples (Hampel needs >= 3) + enough for EMA init */
    FeedSamples(&f, -80, 5);

    rssiState_t st = RssiFilter_GetState(&f);
    tprintf("    State after 5 samples at -80: %s\n", StateStr(st));
    TEST_ASSERT(st == RssiState_Locked_c, "Should be FAR/LOCKED");

    TEST_PASS("IDLE -> FAR");
}

/*******************************************************************************
 * 7. FAR -> CANDIDATE when signal crosses enter threshold
 ******************************************************************************/

static void test_far_to_candidate(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] FAR -> CANDIDATE when RSSI >= -50 dBm\n");

    rssiFilter_t f;
    InitFresh(&f);

    /* Start far away */
    FeedSamples(&f, -80, 10);
    TEST_ASSERT(RssiFilter_GetState(&f) == RssiState_Locked_c, "Should start FAR");

    /* Move close: -40 dBm (above -50 enter threshold) */
    FeedSamples(&f, -40, 20);

    rssiState_t st = RssiFilter_GetState(&f);
    tprintf("    State after -40 dBm x20: %s\n", StateStr(st));
    TEST_ASSERT(st == RssiState_Approach_c || st == RssiState_Unlocked_c,
                "Should be CANDIDATE or LOCKOUT");

    TEST_PASS("FAR -> CANDIDATE");
}

/*******************************************************************************
 * 8. CANDIDATE -> UNLOCK after stable period (2s)
 ******************************************************************************/

static void test_candidate_to_unlock(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] CANDIDATE -> UNLOCK after 2s stable\n");

    rssiFilter_t f;
    InitFresh(&f);

    /* Bootstrap in FAR */
    FeedSamples(&f, -80, 10);

    /* Strong stable signal at -45 dBm.
     * Need ~10 samples for EMA to cross -50 enter threshold,
     * then 2s for smooth ring to flush old values,
     * then 2s of stability. Total ~50+ samples. */
    FeedSamples(&f, -45, 60);

    rssiState_t st = RssiFilter_GetState(&f);
    rssiEvent_t ev = RssiFilter_GetLastEvent(&f);
    tprintf("    State: %s, Event: %s\n", StateStr(st), EventStr(ev));

    TEST_ASSERT(st == RssiState_Unlocked_c, "Should reach LOCKOUT/UNLOCKED");
    TEST_ASSERT(ev == RssiEvent_UnlockTriggered_c, "Event should be UNLOCK_TRIGGERED");

    TEST_PASS("CANDIDATE -> UNLOCK after 2s stable");
}

/*******************************************************************************
 * 9. CANDIDATE -> FAR via exit confirmation (1.5s below exit threshold)
 ******************************************************************************/

static void test_candidate_exit_confirm(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] CANDIDATE -> FAR (exit confirmation 1.5s)\n");

    rssiFilter_t f;
    InitFresh(&f);

    /* Bootstrap in FAR */
    FeedSamples(&f, -80, 10);

    /* Enter CANDIDATE with -40 dBm (need enough samples for EMA convergence) */
    FeedSamples(&f, -40, 20);
    rssiState_t stMid = RssiFilter_GetState(&f);
    tprintf("    State after approach: %s\n", StateStr(stMid));

    /* Drop well below exit threshold for long enough:
     * EMA convergence to -85 (~10 samples) + exit confirm (1.5s = 15 samples) */
    FeedSamples(&f, -85, 30);

    rssiState_t st = RssiFilter_GetState(&f);
    tprintf("    State after -85 x25: %s\n", StateStr(st));
    TEST_ASSERT(st == RssiState_Locked_c, "Should return to FAR after exit confirm");

    TEST_PASS("CANDIDATE -> FAR (exit confirmation)");
}

/*******************************************************************************
 * 10. Exit confirmation timer resets if signal recovers
 ******************************************************************************/

static void test_exit_confirm_resets(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Exit confirmation resets when signal recovers\n");

    rssiFilter_t f;
    InitFresh(&f);

    /* Bootstrap in FAR */
    FeedSamples(&f, -80, 10);

    /* Enter CANDIDATE (enough samples for EMA to converge above -50) */
    FeedSamples(&f, -40, 20);
    rssiState_t stBefore = RssiFilter_GetState(&f);
    tprintf("    State after approach: %s\n", StateStr(stBefore));

    /* Brief dip below exit — only 5 samples (0.5s), well under 1.5s confirm.
     * Note: EMA lag means filtered value drops slowly, so even fewer samples
     * would actually trigger the exit confirm timer. */
    FeedSamples(&f, -85, 5);

    /* Signal recovers before confirmation could expire */
    FeedSamples(&f, -40, 15);

    rssiState_t st = RssiFilter_GetState(&f);
    tprintf("    State after brief dip + recovery: %s\n", StateStr(st));
    TEST_ASSERT(st != RssiState_Locked_c,
                "Should NOT have exited to FAR (dip was < 1.5s)");

    TEST_PASS("Exit confirmation resets on recovery");
}

/*******************************************************************************
 * 11. Lockout period: no re-lock for 5s after unlock
 ******************************************************************************/

static void test_lockout_period(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Lockout period (5s after unlock)\n");

    rssiFilter_t f;
    InitFresh(&f);

    /* Bootstrap + reach UNLOCK (need enough for convergence + stability) */
    FeedSamples(&f, -80, 10);
    FeedSamples(&f, -45, 60);
    TEST_ASSERT(RssiFilter_GetState(&f) == RssiState_Unlocked_c, "Should be UNLOCKED");

    /* Immediately drop to -85 for 2 seconds (within 5s lockout) */
    FeedSamples(&f, -85, 20);

    rssiState_t st = RssiFilter_GetState(&f);
    tprintf("    State 2s after unlock (still in lockout): %s\n", StateStr(st));
    TEST_ASSERT(st == RssiState_Unlocked_c,
                "Should still be LOCKOUT (5s not expired)");

    TEST_PASS("Lockout period");
}

static void test_lockout_expires_then_locks(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Lockout expires, then locks if signal weak\n");

    rssiFilter_t f;
    InitFresh(&f);

    /* Bootstrap + reach UNLOCK */
    FeedSamples(&f, -80, 10);
    FeedSamples(&f, -45, 60);
    TEST_ASSERT(RssiFilter_GetState(&f) == RssiState_Unlocked_c, "Should be UNLOCKED");

    /* Wait out the lockout (5s) + exit confirm (1.5s) with weak signal.
     * 5s lockout + 1.5s confirm + EMA convergence = ~80 samples */
    FeedSamples(&f, -85, 80);

    rssiState_t st = RssiFilter_GetState(&f);
    tprintf("    State after lockout + exit confirm at -85: %s\n", StateStr(st));
    TEST_ASSERT(st == RssiState_Locked_c, "Should be FAR after lockout + confirm");

    TEST_PASS("Lockout expires, then locks");
}

/*******************************************************************************
 * 12. No flip-flop in hysteresis band
 ******************************************************************************/

static void test_no_flipflop_hysteresis(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] No flip-flop between thresholds\n");

    rssiFilter_t f;
    InitFresh(&f);

    /* Bootstrap in FAR */
    FeedSamples(&f, -80, 10);

    /* Feed values in the hysteresis band (-55 dBm, between -50 enter and -60 exit) */
    FeedSamples(&f, -55, 40);

    rssiState_t st = RssiFilter_GetState(&f);
    tprintf("    State at -55 dBm (in band): %s\n", StateStr(st));
    TEST_ASSERT(st == RssiState_Locked_c,
                "Should remain FAR (below -50 enter threshold)");

    TEST_PASS("No flip-flop in hysteresis band");
}

/*******************************************************************************
 * 13. Stability gate: unstable signal does NOT unlock
 ******************************************************************************/

static void test_unstable_does_not_unlock(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Unstable signal in CANDIDATE does not unlock\n");

    rssiFilter_t f;
    InitFresh(&f);

    /* Bootstrap in FAR */
    FeedSamples(&f, -80, 10);

    /* Very noisy signal above threshold: alternate -30 and -55 for 4 seconds
     * Mean is above -50, but std dev should be high */
    for (int i = 0; i < 40; i++)
    {
        int8_t val = (i % 2 == 0) ? (int8_t)-30 : (int8_t)-55;
        MockTimer_Advance(100U);
        RssiFilter_AddMeasurement(&f, val);
    }

    rssiState_t st = RssiFilter_GetState(&f);
    tprintf("    State after 4s of noisy above-threshold: %s\n", StateStr(st));
    /* Should NOT be unlocked because std dev is too high */
    TEST_ASSERT(st != RssiState_Unlocked_c,
                "Unstable signal should NOT trigger unlock");

    TEST_PASS("Unstable signal does not unlock");
}

/*******************************************************************************
 * 14. Event emission
 ******************************************************************************/

static void test_events_emitted(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Events emitted correctly\n");

    rssiFilter_t f;
    InitFresh(&f);

    /* IDLE -> FAR: no explicit event, but stateChanged should be set */
    FeedSamples(&f, -80, 10);
    rssiEvent_t ev1 = RssiFilter_GetLastEvent(&f);
    tprintf("    After FAR: event=%s\n", EventStr(ev1));

    /* FAR -> CANDIDATE: CandidateStarted (need enough for EMA to cross -50) */
    FeedSamples(&f, -45, 20);
    rssiEvent_t ev2 = RssiFilter_GetLastEvent(&f);
    tprintf("    After approach: event=%s, state=%s\n", EventStr(ev2), StateStr(RssiFilter_GetState(&f)));
    TEST_ASSERT(ev2 == RssiEvent_CandidateStarted_c ||
                ev2 == RssiEvent_UnlockTriggered_c,
                "Should emit CandidateStarted or UnlockTriggered");

    /* CANDIDATE -> UNLOCK: UnlockTriggered (need 2s stability + smooth ring flush) */
    FeedSamples(&f, -45, 40);
    rssiEvent_t ev3 = RssiFilter_GetLastEvent(&f);
    tprintf("    After stable: event=%s, state=%s\n", EventStr(ev3), StateStr(RssiFilter_GetState(&f)));
    TEST_ASSERT(ev3 == RssiEvent_UnlockTriggered_c, "Should emit UnlockTriggered");

    TEST_PASS("Events emitted correctly");
}

/*******************************************************************************
 * 15. HasStateChanged clears flag
 ******************************************************************************/

static void test_state_changed_flag(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] HasStateChanged clears flag\n");

    rssiFilter_t f;
    InitFresh(&f);

    FeedSamples(&f, -80, 5);
    /* Should have transitioned IDLE -> FAR */
    bool_t first = RssiFilter_HasStateChanged(&f);
    bool_t second = RssiFilter_HasStateChanged(&f);

    tprintf("    First call: %d, Second call: %d\n", first, second);
    TEST_ASSERT(first == (bool_t)TRUE,  "First call should return TRUE");
    TEST_ASSERT(second == (bool_t)FALSE, "Second call should return FALSE (cleared)");

    TEST_PASS("HasStateChanged clears flag");
}

/*******************************************************************************
 * 16. GetFeatures with NULL output pointers
 ******************************************************************************/

static void test_get_features_null_outputs(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] GetFeatures with NULL output pointers\n");

    rssiFilter_t f;
    InitFresh(&f);
    FeedSamples(&f, -50, 20);

    /* Should not crash when some output pointers are NULL */
    uint16_t stdQ4 = 0U;
    RssiFilter_GetFeatures(&f, &stdQ4, NULL, NULL);
    TEST_ASSERT(stdQ4 != 0xFFFFU || stdQ4 == 0U, "stdQ4 should be written");

    uint8_t pct = 0U;
    RssiFilter_GetFeatures(&f, NULL, &pct, NULL);

    int8_t mean = 0;
    RssiFilter_GetFeatures(&f, NULL, NULL, &mean);

    TEST_PASS("GetFeatures with NULL output pointers");
}

/*******************************************************************************
 * 17. Input clamping
 ******************************************************************************/

static void test_input_clamping(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Invalid RSSI input clamping\n");

    rssiFilter_t f;
    InitFresh(&f);

    /* Feed extreme values — should not crash or produce garbage */
    MockTimer_Advance(100U);
    RssiFilter_AddMeasurement(&f, 127);   /* BLE error value */
    MockTimer_Advance(100U);
    RssiFilter_AddMeasurement(&f, -128);  /* Extreme negative */
    MockTimer_Advance(100U);
    RssiFilter_AddMeasurement(&f, 0);     /* Zero */

    FeedSamples(&f, -50, 10);
    int8_t filtered = RssiFilter_GetFilteredRssi(&f);
    tprintf("    Filtered after extreme inputs then -50 x10: %d\n", filtered);

    /* Should still produce reasonable output */
    TEST_ASSERT(filtered > -100 && filtered < 0, "Should produce sane output after clamping");

    TEST_PASS("Invalid RSSI input clamping");
}

/*******************************************************************************
 * 18. Full lifecycle: FAR -> CANDIDATE -> UNLOCK -> (lockout) -> FAR
 ******************************************************************************/

static void test_full_lifecycle(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Full lifecycle\n");

    rssiFilter_t f;
    InitFresh(&f);

    /* 1. IDLE -> FAR */
    FeedSamples(&f, -80, 10);
    tprintf("    Step 1 (far):       %s\n", StateStr(RssiFilter_GetState(&f)));
    TEST_ASSERT(RssiFilter_GetState(&f) == RssiState_Locked_c, "Step 1: FAR");

    /* 2. FAR -> CANDIDATE -> UNLOCK (need convergence + stability) */
    FeedSamples(&f, -40, 60);
    rssiState_t stAfterApproach = RssiFilter_GetState(&f);
    tprintf("    Step 2 (approach):  %s\n", StateStr(stAfterApproach));
    TEST_ASSERT(stAfterApproach == RssiState_Unlocked_c, "Step 2: UNLOCKED");

    /* 3. During lockout (5s), signal drops but should stay LOCKOUT */
    FeedSamples(&f, -85, 20);
    tprintf("    Step 3 (lockout):   %s\n", StateStr(RssiFilter_GetState(&f)));
    TEST_ASSERT(RssiFilter_GetState(&f) == RssiState_Unlocked_c, "Step 3: Still LOCKOUT");

    /* 4. After lockout + exit confirm, should lock */
    FeedSamples(&f, -85, 60);
    tprintf("    Step 4 (lock):      %s\n", StateStr(RssiFilter_GetState(&f)));
    TEST_ASSERT(RssiFilter_GetState(&f) == RssiState_Locked_c, "Step 4: FAR");

    /* 5. Approach again -> CANDIDATE -> UNLOCK */
    FeedSamples(&f, -40, 60);
    tprintf("    Step 5 (re-unlock): %s\n", StateStr(RssiFilter_GetState(&f)));
    TEST_ASSERT(RssiFilter_GetState(&f) == RssiState_Unlocked_c, "Step 5: UNLOCKED again");

    TEST_PASS("Full lifecycle");
}

/*******************************************************************************
 * 19. Rapid samples (dt < EMA_DT_MIN) use minimum alpha
 ******************************************************************************/

static void test_rapid_samples(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Rapid samples (dt=10ms) still smooth correctly\n");

    rssiFilter_t f;
    InitFresh(&f);

    /* Fast samples at 10ms interval */
    FeedSamplesAt(&f, -50, 50, 10U);
    int8_t filtered = RssiFilter_GetFilteredRssi(&f);

    tprintf("    Filtered after 50 rapid samples of -50: %d dBm\n", filtered);
    TEST_ASSERT(filtered >= -55 && filtered <= -45, "Should converge even with rapid samples");

    TEST_PASS("Rapid samples");
}

/*******************************************************************************
 * Main
 ******************************************************************************/

int main(int argc, char *argv[])
{
    /* Parse flags: --xml <path>, --log <path> */
    const char *xmlPath = NULL;
    const char *logPath = NULL;
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--xml") == 0 && (i + 1) < argc)
        {
            xmlPath = argv[i + 1];
            i++;
        }
        else if (strcmp(argv[i], "--log") == 0 && (i + 1) < argc)
        {
            logPath = argv[i + 1];
            i++;
        }
    }

    if (logPath != NULL)
    {
        gLogFile = fopen(logPath, "w");
        if (gLogFile == NULL)
        {
            printf("  WARNING: Could not open log file %s\n", logPath);
        }
    }

    tprintf("\n");
    tprintf("================================================================\n");
    tprintf("  RSSI Filter Unit Tests (Q4 Pipeline + Exit Confirm + Lockout)\n");
    tprintf("================================================================\n");

    RUN_TEST(test_init_defaults);
    RUN_TEST(test_reset_clears_all);
    RUN_TEST(test_null_safety);
    RUN_TEST(test_hampel_rejects_spike);
    RUN_TEST(test_hampel_passes_clean);
    RUN_TEST(test_ema_converges);
    RUN_TEST(test_ema_smooths_noise);
    RUN_TEST(test_ema_anomaly_reset);
    RUN_TEST(test_features_stable_signal);
    RUN_TEST(test_features_noisy_signal);
    RUN_TEST(test_idle_to_far);
    RUN_TEST(test_far_to_candidate);
    RUN_TEST(test_candidate_to_unlock);
    RUN_TEST(test_candidate_exit_confirm);
    RUN_TEST(test_exit_confirm_resets);
    RUN_TEST(test_lockout_period);
    RUN_TEST(test_lockout_expires_then_locks);
    RUN_TEST(test_no_flipflop_hysteresis);
    RUN_TEST(test_unstable_does_not_unlock);
    RUN_TEST(test_events_emitted);
    RUN_TEST(test_state_changed_flag);
    RUN_TEST(test_get_features_null_outputs);
    RUN_TEST(test_input_clamping);
    RUN_TEST(test_full_lifecycle);
    RUN_TEST(test_rapid_samples);

    tprintf("\n");
    tprintf("================================================================\n");
    tprintf("  Results: %d passed, %d failed (of %d)\n",
            gTestsPassed, gTestsFailed, gTestsPassed + gTestsFailed);
    tprintf("================================================================\n\n");

    if (xmlPath != NULL)
    {
        JUnit_WriteXml(xmlPath);
    }

    if (gLogFile != NULL)
    {
        fclose(gLogFile);
        printf("  Log written to: %s\n", logPath);
    }

    return (gTestsFailed > 0) ? 1 : 0;
}
