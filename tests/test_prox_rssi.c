/*! *********************************************************************************
* \file test_prox_rssi.c
*
* \brief  Unit tests for ProxRssi — fixed-point BLE RSSI proximity filter with
*         Hampel spike rejection, adaptive EMA, feature extraction, and
*         state machine (FAR / CANDIDATE / LOCKOUT).
*         Runs on host machine (macOS/Linux). Tests the real ProxRssi.c via
*         #include after providing a default alpha LUT.
*
* Copyright 2025
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*******************************************************************************
 * Pull in the real implementation
 ******************************************************************************/
#include "ProxRssi.h"
#include "ProxRssi.c"

/*******************************************************************************
 * Test framework (minimal, self-contained) with JUnit XML support
 ******************************************************************************/

static int gTestsPassed = 0;
static int gTestsFailed = 0;
static int gTestsTotal  = 0;

static FILE *gLogFile = NULL;

#define tprintf(...) do {               \
    printf(__VA_ARGS__);                \
    if (gLogFile != NULL) {             \
        fprintf(gLogFile, __VA_ARGS__); \
    }                                   \
} while(0)

#define MAX_TESTS 64
#define MAX_MSG   256

typedef struct {
    char name[MAX_MSG];
    char failMsg[MAX_MSG];
    char file[MAX_MSG];
    int  line;
    int  passed;
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
    fprintf(fp, "  <testsuite name=\"prox_rssi\" tests=\"%d\" "
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

/*******************************************************************************
 * Default alpha LUT: linear ramp from alpha=0.10 (dt=0) to alpha=0.80 (dt=1000ms)
 * Values in Q15: 0.10 = 3277, 0.80 = 26214
 ******************************************************************************/
#define TEST_ALPHA_LUT_LEN  1001u

static uint16 gAlphaLut[TEST_ALPHA_LUT_LEN];

static void BuildAlphaLut(void)
{
    for (uint32 i = 0u; i < TEST_ALPHA_LUT_LEN; i++)
    {
        double frac = (double)i / 1000.0;
        if (frac > 1.0) frac = 1.0;
        double alpha = 0.10 + frac * (0.80 - 0.10);
        gAlphaLut[i] = (uint16)(alpha * 32767.0);
    }
}

/*******************************************************************************
 * Default params for testing
 ******************************************************************************/
static ProxRssi_ParamsType DefaultParams(void)
{
    ProxRssi_ParamsType p;
    memset(&p, 0, sizeof(p));

    p.wRawMs    = 2000u;
    p.wSpikeMs  = 800u;
    p.wFeatMs   = 2000u;

    p.hampelKQ4 = 48u;   /* K = 3.0 */
    p.madEpsQ4  = 8u;    /* 0.5 dB */

    p.enterNearQ4 = ProxRssi_DbmToQ4(-50);
    p.exitNearQ4  = ProxRssi_DbmToQ4(-60);
    p.hystQ4      = (uint16)ProxRssi_DbToQ4(10);

    p.pctThQ15       = 16384u;  /* ~50% */
    p.stdThQ4        = 40u;     /* 2.5 dB */
    p.stableMs       = 2000u;
    p.minFeatSamples = 6u;

    p.exitConfirmMs    = 1500u;
    p.lockoutMs        = 5000u;
    p.maxReasonableDtMs = 2000u;

    return p;
}

/*******************************************************************************
 * Helpers
 ******************************************************************************/
static const char* StateStr(ProxRssi_StateType s)
{
    switch (s) {
        case PROX_RSSI_ST_FAR:       return "FAR";
        case PROX_RSSI_ST_CANDIDATE: return "CANDIDATE";
        case PROX_RSSI_ST_LOCKOUT:   return "LOCKOUT";
        default:                     return "???";
    }
}

static const char* EventStr(ProxRssi_EventType e)
{
    switch (e) {
        case PROX_RSSI_EVT_NONE:              return "NONE";
        case PROX_RSSI_EVT_CANDIDATE_STARTED: return "CANDIDATE_STARTED";
        case PROX_RSSI_EVT_UNLOCK_TRIGGERED:  return "UNLOCK_TRIGGERED";
        case PROX_RSSI_EVT_EXIT_TO_FAR:       return "EXIT_TO_FAR";
        default:                              return "???";
    }
}

/* Feed N identical samples at a given interval, running MainFunction each time */
static void FeedSamples(ProxRssi_CtxType *ctx, sint8 rssiDbm,
                        uint32 count, uint32 intervalMs, uint32 *tMs)
{
    ProxRssi_EventType ev;
    for (uint32 i = 0u; i < count; i++)
    {
        *tMs += intervalMs;
        ProxRssi_PushRaw(ctx, *tMs, rssiDbm);
        ProxRssi_MainFunction(ctx, *tMs, &ev, NULL);
    }
}

/* Feed and return the last event */
static ProxRssi_EventType FeedSamplesGetEvent(ProxRssi_CtxType *ctx, sint8 rssiDbm,
                                               uint32 count, uint32 intervalMs, uint32 *tMs)
{
    ProxRssi_EventType ev = PROX_RSSI_EVT_NONE;
    ProxRssi_EventType lastSignificant = PROX_RSSI_EVT_NONE;
    for (uint32 i = 0u; i < count; i++)
    {
        *tMs += intervalMs;
        ProxRssi_PushRaw(ctx, *tMs, rssiDbm);
        ProxRssi_MainFunction(ctx, *tMs, &ev, NULL);
        if (ev != PROX_RSSI_EVT_NONE) { lastSignificant = ev; }
    }
    return lastSignificant;
}

static void InitFresh(ProxRssi_CtxType *ctx)
{
    ProxRssi_ParamsType p = DefaultParams();
    ProxRssi_Init(ctx, &p, gAlphaLut, TEST_ALPHA_LUT_LEN);
}

/*******************************************************************************
 * 1. Initialization
 ******************************************************************************/

static void test_init_defaults(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Init defaults\n");

    ProxRssi_CtxType ctx;
    InitFresh(&ctx);

    TEST_ASSERT(ctx.st == PROX_RSSI_ST_FAR, "State = FAR after init");
    TEST_ASSERT(ctx.emaValid == FALSE, "EMA not valid");
    TEST_ASSERT(ctx.raw.count == 0u, "Raw ring empty");
    TEST_ASSERT(ctx.smooth.count == 0u, "Smooth ring empty");

    TEST_PASS("Init defaults");
}

static void test_init_null_safety(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Init NULL safety\n");

    ProxRssi_ParamsType p = DefaultParams();
    Std_ReturnType r;

    r = ProxRssi_Init(NULL, &p, gAlphaLut, TEST_ALPHA_LUT_LEN);
    TEST_ASSERT(r == E_NOT_OK, "NULL ctx returns E_NOT_OK");

    ProxRssi_CtxType ctx;
    r = ProxRssi_Init(&ctx, NULL, gAlphaLut, TEST_ALPHA_LUT_LEN);
    TEST_ASSERT(r == E_NOT_OK, "NULL params returns E_NOT_OK");

    r = ProxRssi_Init(&ctx, &p, NULL, TEST_ALPHA_LUT_LEN);
    TEST_ASSERT(r == E_NOT_OK, "NULL alphaLut returns E_NOT_OK");

    r = ProxRssi_Init(&ctx, &p, gAlphaLut, 0u);
    TEST_ASSERT(r == E_NOT_OK, "Zero alphaLen returns E_NOT_OK");

    TEST_PASS("Init NULL safety");
}

/*******************************************************************************
 * 2. PushRaw clamping
 ******************************************************************************/

static void test_push_raw_clamping(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] PushRaw clamping\n");

    ProxRssi_CtxType ctx;
    InitFresh(&ctx);

    /* Should not crash with extreme values */
    Std_ReturnType r;
    r = ProxRssi_PushRaw(&ctx, 100u, (sint8)-50);
    TEST_ASSERT(r == E_OK, "Push -50 succeeds");
    r = ProxRssi_PushRaw(&ctx, 200u, (sint8)-127);
    TEST_ASSERT(r == E_OK, "Push -127 succeeds");
    r = ProxRssi_PushRaw(NULL, 300u, (sint8)-50);
    TEST_ASSERT(r == E_NOT_OK, "Push NULL ctx returns E_NOT_OK");

    /* BLE Core Spec: RSSI 127 = "not available", reject non-negative */
    r = ProxRssi_PushRaw(&ctx, 400u, (sint8)127);
    TEST_ASSERT(r == E_NOT_OK, "Push 127 (not available) rejected");
    r = ProxRssi_PushRaw(&ctx, 500u, (sint8)0);
    TEST_ASSERT(r == E_NOT_OK, "Push 0 (non-negative) rejected");
    r = ProxRssi_PushRaw(&ctx, 600u, (sint8)20);
    TEST_ASSERT(r == E_NOT_OK, "Push +20 (non-negative) rejected");

    /* Only the two valid pushes should be in the buffer */
    TEST_ASSERT(ctx.raw.count == 2u, "Only valid samples in buffer");

    TEST_PASS("PushRaw clamping + validation");
}

/*******************************************************************************
 * 3. Hampel spike rejection
 ******************************************************************************/

static void test_hampel_rejects_spike(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Hampel rejects single spike\n");

    ProxRssi_CtxType ctx;
    InitFresh(&ctx);
    uint32 t = 1000u;

    /* Feed 10 samples at -50 dBm */
    FeedSamples(&ctx, (sint8)-50, 10u, 100u, &t);

    sint16 emaBefore = ctx.emaQ4;

    /* Inject a huge spike: +10 dBm (60 dB jump) */
    t += 100u;
    ProxRssi_PushRaw(&ctx, t, (sint8)10);
    ProxRssi_EventType ev;
    ProxRssi_MainFunction(&ctx, t, &ev, NULL);

    sint16 emaAfter = ctx.emaQ4;
    sint16 jumpQ4 = (sint16)(emaAfter - emaBefore);
    if (jumpQ4 < 0) jumpQ4 = (sint16)(-jumpQ4);

    tprintf("    EMA before: %d Q4, after: %d Q4, jump: %d Q4 (%.1f dB)\n",
            emaBefore, emaAfter, jumpQ4, (double)jumpQ4 / 16.0);

    /* Spike should be rejected — EMA jump should be small */
    TEST_ASSERT(jumpQ4 < (sint16)(10 * PROX_RSSI_Q4_SCALE),
                "Spike should be rejected by Hampel (jump < 10 dB)");

    TEST_PASS("Hampel rejects single spike");
}

static void test_hampel_passes_clean(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Hampel passes clean signal\n");

    ProxRssi_CtxType ctx;
    InitFresh(&ctx);
    uint32 t = 1000u;

    FeedSamples(&ctx, (sint8)-45, 20u, 100u, &t);

    sint16 emaQ4 = ctx.emaQ4;
    sint16 expectedQ4 = ProxRssi_DbmToQ4((sint8)-45);
    sint16 diff = (sint16)(emaQ4 - expectedQ4);
    if (diff < 0) diff = (sint16)(-diff);

    tprintf("    EMA: %d Q4 (%.1f dBm), expected: %d Q4, diff: %d Q4\n",
            emaQ4, (double)emaQ4 / 16.0, expectedQ4, diff);

    TEST_ASSERT(diff < (sint16)(3 * PROX_RSSI_Q4_SCALE),
                "Clean signal should pass through (~3 dB tolerance)");

    TEST_PASS("Hampel passes clean signal");
}

/*******************************************************************************
 * 4. EMA smoothing
 ******************************************************************************/

static void test_ema_converges(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] EMA converges to stable input\n");

    ProxRssi_CtxType ctx;
    InitFresh(&ctx);
    uint32 t = 1000u;

    FeedSamples(&ctx, (sint8)-55, 30u, 100u, &t);

    sint16 emaQ4 = ctx.emaQ4;
    sint16 expectedQ4 = ProxRssi_DbmToQ4((sint8)-55);
    sint16 diff = (sint16)(emaQ4 - expectedQ4);
    if (diff < 0) diff = (sint16)(-diff);

    tprintf("    EMA after 30 samples of -55: %.1f dBm (diff: %.1f dB)\n",
            (double)emaQ4 / 16.0, (double)diff / 16.0);

    TEST_ASSERT(diff < (sint16)(2 * PROX_RSSI_Q4_SCALE),
                "EMA should converge to ~-55 dBm");

    TEST_PASS("EMA converges to stable input");
}

static void test_ema_anomaly_reset(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] EMA resets on time anomaly (dt > 2s)\n");

    ProxRssi_CtxType ctx;
    InitFresh(&ctx);
    uint32 t = 1000u;

    FeedSamples(&ctx, (sint8)-40, 15u, 100u, &t);
    sint16 emaBefore = ctx.emaQ4;

    /* Big time gap (3 seconds) then feed samples at -80 */
    t += 3000u;
    FeedSamples(&ctx, (sint8)-80, 10u, 100u, &t);
    sint16 emaAfter = ctx.emaQ4;

    tprintf("    Before gap: %.1f dBm, After gap+(-80 x10): %.1f dBm\n",
            (double)emaBefore / 16.0, (double)emaAfter / 16.0);

    sint16 expectedQ4 = ProxRssi_DbmToQ4((sint8)-80);
    sint16 diff = (sint16)(emaAfter - expectedQ4);
    if (diff < 0) diff = (sint16)(-diff);

    TEST_ASSERT(diff < (sint16)(5 * PROX_RSSI_Q4_SCALE),
                "EMA should reset toward -80 after 3s gap");

    TEST_PASS("EMA resets on time anomaly");
}

/*******************************************************************************
 * 5. Feature extraction
 ******************************************************************************/

static void test_features_stable_signal(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Feature extraction: stable signal\n");

    ProxRssi_CtxType ctx;
    InitFresh(&ctx);
    uint32 t = 1000u;

    FeedSamples(&ctx, (sint8)-50, 30u, 100u, &t);

    ProxRssi_FeaturesType feat;
    ProxRssi_EventType ev;
    t += 100u;
    ProxRssi_PushRaw(&ctx, t, (sint8)-50);
    ProxRssi_MainFunction(&ctx, t, &ev, &feat);

    tprintf("    StdDev(Q4)=%u (%.1f dB), PctAbove(Q15)=%u, n=%u\n",
            feat.stdQ4, (double)feat.stdQ4 / 16.0, feat.pctAboveEnterQ15, feat.n);

    TEST_ASSERT(feat.stdQ4 < 32u, "StdDev should be < 2 dB for stable signal");

    TEST_PASS("Feature extraction: stable signal");
}

/*******************************************************************************
 * 6. State machine: FAR -> CANDIDATE
 ******************************************************************************/

static void test_far_to_candidate(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] FAR -> CANDIDATE when RSSI >= enterNear\n");

    ProxRssi_CtxType ctx;
    InitFresh(&ctx);
    uint32 t = 1000u;

    /* Start far away */
    FeedSamples(&ctx, (sint8)-80, 10u, 100u, &t);
    TEST_ASSERT(ctx.st == PROX_RSSI_ST_FAR, "Should start FAR");

    /* Move close: -40 dBm (well above -50 enter threshold) */
    ProxRssi_EventType ev = FeedSamplesGetEvent(&ctx, (sint8)-40, 25u, 100u, &t);

    tprintf("    State: %s, Last event: %s\n", StateStr(ctx.st), EventStr(ev));
    TEST_ASSERT(ctx.st == PROX_RSSI_ST_CANDIDATE || ctx.st == PROX_RSSI_ST_LOCKOUT,
                "Should be CANDIDATE or LOCKOUT");

    TEST_PASS("FAR -> CANDIDATE");
}

/*******************************************************************************
 * 7. CANDIDATE -> LOCKOUT (unlock) after stable 2s
 ******************************************************************************/

static void test_candidate_to_unlock(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] CANDIDATE -> LOCKOUT after 2s stable\n");

    ProxRssi_CtxType ctx;
    InitFresh(&ctx);
    uint32 t = 1000u;

    /* Bootstrap in FAR */
    FeedSamples(&ctx, (sint8)-80, 10u, 100u, &t);

    /* Strong stable signal at -40 dBm for 6s (convergence + 2s stability) */
    ProxRssi_EventType ev = FeedSamplesGetEvent(&ctx, (sint8)-40, 60u, 100u, &t);

    tprintf("    State: %s, Event: %s\n", StateStr(ctx.st), EventStr(ev));
    TEST_ASSERT(ctx.st == PROX_RSSI_ST_LOCKOUT, "Should reach LOCKOUT");
    TEST_ASSERT(ev == PROX_RSSI_EVT_UNLOCK_TRIGGERED, "Should emit UNLOCK_TRIGGERED");

    TEST_PASS("CANDIDATE -> LOCKOUT after 2s stable");
}

/*******************************************************************************
 * 8. Exit confirmation: CANDIDATE -> FAR
 ******************************************************************************/

static void test_candidate_exit_confirm(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] CANDIDATE -> FAR (exit confirmation 1.5s)\n");

    ProxRssi_CtxType ctx;
    InitFresh(&ctx);
    uint32 t = 1000u;

    /* Bootstrap in FAR */
    FeedSamples(&ctx, (sint8)-80, 10u, 100u, &t);

    /* Enter CANDIDATE */
    FeedSamples(&ctx, (sint8)-40, 20u, 100u, &t);
    tprintf("    State after approach: %s\n", StateStr(ctx.st));

    /* Drop well below exit threshold for long enough */
    ProxRssi_EventType ev = FeedSamplesGetEvent(&ctx, (sint8)-85, 30u, 100u, &t);

    tprintf("    State after -85 x30: %s, event: %s\n", StateStr(ctx.st), EventStr(ev));
    TEST_ASSERT(ctx.st == PROX_RSSI_ST_FAR, "Should return to FAR");
    TEST_ASSERT(ev == PROX_RSSI_EVT_EXIT_TO_FAR, "Should emit EXIT_TO_FAR");

    TEST_PASS("CANDIDATE -> FAR (exit confirmation)");
}

/*******************************************************************************
 * 9. Exit confirmation timer resets on recovery
 ******************************************************************************/

static void test_exit_confirm_resets(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Exit confirmation resets when signal recovers\n");

    ProxRssi_CtxType ctx;
    InitFresh(&ctx);
    uint32 t = 1000u;

    /* Bootstrap FAR then enter CANDIDATE */
    FeedSamples(&ctx, (sint8)-80, 10u, 100u, &t);
    FeedSamples(&ctx, (sint8)-40, 20u, 100u, &t);
    tprintf("    State after approach: %s\n", StateStr(ctx.st));

    /* Brief dip below exit — only 0.5s, well under 1.5s confirm */
    FeedSamples(&ctx, (sint8)-85, 5u, 100u, &t);

    /* Signal recovers */
    FeedSamples(&ctx, (sint8)-40, 15u, 100u, &t);

    tprintf("    State after brief dip + recovery: %s\n", StateStr(ctx.st));
    TEST_ASSERT(ctx.st != PROX_RSSI_ST_FAR,
                "Should NOT have exited to FAR (dip was < 1.5s)");

    TEST_PASS("Exit confirmation resets on recovery");
}

/*******************************************************************************
 * 10. Lockout period
 ******************************************************************************/

static void test_lockout_period(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Lockout period (5s after unlock)\n");

    ProxRssi_CtxType ctx;
    InitFresh(&ctx);
    uint32 t = 1000u;

    /* Bootstrap + reach LOCKOUT */
    FeedSamples(&ctx, (sint8)-80, 10u, 100u, &t);
    FeedSamples(&ctx, (sint8)-40, 60u, 100u, &t);
    TEST_ASSERT(ctx.st == PROX_RSSI_ST_LOCKOUT, "Should be LOCKOUT");

    /* Drop signal for 2s (within 5s lockout) */
    FeedSamples(&ctx, (sint8)-85, 20u, 100u, &t);

    tprintf("    State 2s after unlock: %s\n", StateStr(ctx.st));
    TEST_ASSERT(ctx.st == PROX_RSSI_ST_LOCKOUT,
                "Should still be LOCKOUT (5s not expired)");

    TEST_PASS("Lockout period");
}

static void test_lockout_expires_then_locks(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Lockout expires, then locks if signal weak\n");

    ProxRssi_CtxType ctx;
    InitFresh(&ctx);
    uint32 t = 1000u;

    /* Bootstrap + reach LOCKOUT */
    FeedSamples(&ctx, (sint8)-80, 10u, 100u, &t);
    FeedSamples(&ctx, (sint8)-40, 60u, 100u, &t);
    TEST_ASSERT(ctx.st == PROX_RSSI_ST_LOCKOUT, "Should be LOCKOUT");

    /* Wait out lockout (5s) + exit confirm (1.5s) + EMA convergence */
    ProxRssi_EventType ev = FeedSamplesGetEvent(&ctx, (sint8)-85, 80u, 100u, &t);

    tprintf("    State after lockout + exit confirm: %s, event: %s\n",
            StateStr(ctx.st), EventStr(ev));
    TEST_ASSERT(ctx.st == PROX_RSSI_ST_FAR, "Should be FAR after lockout + confirm");

    TEST_PASS("Lockout expires, then locks");
}

/*******************************************************************************
 * 11. No flip-flop in hysteresis band
 ******************************************************************************/

static void test_no_flipflop_hysteresis(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] No flip-flop between thresholds\n");

    ProxRssi_CtxType ctx;
    InitFresh(&ctx);
    uint32 t = 1000u;

    /* Bootstrap in FAR */
    FeedSamples(&ctx, (sint8)-80, 10u, 100u, &t);

    /* Feed values in the hysteresis band (-55 dBm, between -50 enter and -60 exit) */
    FeedSamples(&ctx, (sint8)-55, 40u, 100u, &t);

    tprintf("    State at -55 dBm (in band): %s\n", StateStr(ctx.st));
    TEST_ASSERT(ctx.st == PROX_RSSI_ST_FAR,
                "Should remain FAR (below -50 enter threshold)");

    TEST_PASS("No flip-flop in hysteresis band");
}

/*******************************************************************************
 * 12. Unstable signal does NOT unlock
 ******************************************************************************/

static void test_unstable_does_not_unlock(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Unstable signal does not unlock\n");

    ProxRssi_CtxType ctx;
    InitFresh(&ctx);
    uint32 t = 1000u;

    /* Bootstrap in FAR */
    FeedSamples(&ctx, (sint8)-80, 10u, 100u, &t);

    /* Very noisy signal: alternate -30 and -55 for 4 seconds */
    ProxRssi_EventType ev;
    for (uint32 i = 0u; i < 40u; i++)
    {
        sint8 val = (i % 2u == 0u) ? (sint8)-30 : (sint8)-55;
        t += 100u;
        ProxRssi_PushRaw(&ctx, t, val);
        ProxRssi_MainFunction(&ctx, t, &ev, NULL);
    }

    tprintf("    State after 4s noisy: %s\n", StateStr(ctx.st));
    TEST_ASSERT(ctx.st != PROX_RSSI_ST_LOCKOUT,
                "Unstable signal should NOT trigger unlock");

    TEST_PASS("Unstable signal does not unlock");
}

/*******************************************************************************
 * 13. ForceFar resets everything
 ******************************************************************************/

static void test_force_far(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] ForceFar resets to FAR\n");

    ProxRssi_CtxType ctx;
    InitFresh(&ctx);
    uint32 t = 1000u;

    /* Get to LOCKOUT */
    FeedSamples(&ctx, (sint8)-80, 10u, 100u, &t);
    FeedSamples(&ctx, (sint8)-40, 60u, 100u, &t);
    TEST_ASSERT(ctx.st == PROX_RSSI_ST_LOCKOUT, "Should be LOCKOUT");

    Std_ReturnType r = ProxRssi_ForceFar(&ctx);
    TEST_ASSERT(r == E_OK, "ForceFar returns E_OK");
    TEST_ASSERT(ctx.st == PROX_RSSI_ST_FAR, "State = FAR after ForceFar");
    TEST_ASSERT(ctx.emaValid == FALSE, "EMA reset");
    TEST_ASSERT(ctx.raw.count == 0u, "Raw ring cleared");
    TEST_ASSERT(ctx.smooth.count == 0u, "Smooth ring cleared");

    TEST_PASS("ForceFar resets to FAR");
}

/*******************************************************************************
 * 14. Full lifecycle: FAR -> CANDIDATE -> LOCKOUT -> FAR -> (repeat)
 ******************************************************************************/

static void test_full_lifecycle(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Full lifecycle\n");

    ProxRssi_CtxType ctx;
    InitFresh(&ctx);
    uint32 t = 1000u;

    /* 1. FAR */
    FeedSamples(&ctx, (sint8)-80, 10u, 100u, &t);
    tprintf("    Step 1 (far):       %s\n", StateStr(ctx.st));
    TEST_ASSERT(ctx.st == PROX_RSSI_ST_FAR, "Step 1: FAR");

    /* 2. FAR -> CANDIDATE -> LOCKOUT */
    FeedSamples(&ctx, (sint8)-40, 60u, 100u, &t);
    tprintf("    Step 2 (unlock):    %s\n", StateStr(ctx.st));
    TEST_ASSERT(ctx.st == PROX_RSSI_ST_LOCKOUT, "Step 2: LOCKOUT");

    /* 3. During lockout, signal drops but stays LOCKOUT */
    FeedSamples(&ctx, (sint8)-85, 20u, 100u, &t);
    tprintf("    Step 3 (lockout):   %s\n", StateStr(ctx.st));
    TEST_ASSERT(ctx.st == PROX_RSSI_ST_LOCKOUT, "Step 3: Still LOCKOUT");

    /* 4. After lockout + exit confirm -> FAR */
    FeedSamples(&ctx, (sint8)-85, 60u, 100u, &t);
    tprintf("    Step 4 (lock):      %s\n", StateStr(ctx.st));
    TEST_ASSERT(ctx.st == PROX_RSSI_ST_FAR, "Step 4: FAR");

    /* 5. Approach again -> LOCKOUT */
    FeedSamples(&ctx, (sint8)-40, 60u, 100u, &t);
    tprintf("    Step 5 (re-unlock): %s\n", StateStr(ctx.st));
    TEST_ASSERT(ctx.st == PROX_RSSI_ST_LOCKOUT, "Step 5: LOCKOUT again");

    TEST_PASS("Full lifecycle");
}

/*******************************************************************************
 * 15. Q4 conversion helpers
 ******************************************************************************/

static void test_q4_conversions(void)
{
    gTestsTotal++;
    tprintf("\n[TEST] Q4 conversion helpers\n");

    sint16 q4 = ProxRssi_DbmToQ4((sint8)-50);
    TEST_ASSERT(q4 == (sint16)(-50 * 16), "DbmToQ4(-50) = -800");

    sint16 q4b = ProxRssi_DbToQ4((sint16)10);
    TEST_ASSERT(q4b == (sint16)(10 * 16), "DbToQ4(10) = 160");

    TEST_PASS("Q4 conversion helpers");
}

/*******************************************************************************
 * Main
 ******************************************************************************/

int main(int argc, char *argv[])
{
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

    BuildAlphaLut();

    tprintf("\n");
    tprintf("================================================================\n");
    tprintf("  ProxRssi Unit Tests (Hampel + EMA + Features + State Machine)\n");
    tprintf("================================================================\n");

    RUN_TEST(test_init_defaults);
    RUN_TEST(test_init_null_safety);
    RUN_TEST(test_push_raw_clamping);
    RUN_TEST(test_hampel_rejects_spike);
    RUN_TEST(test_hampel_passes_clean);
    RUN_TEST(test_ema_converges);
    RUN_TEST(test_ema_anomaly_reset);
    RUN_TEST(test_features_stable_signal);
    RUN_TEST(test_far_to_candidate);
    RUN_TEST(test_candidate_to_unlock);
    RUN_TEST(test_candidate_exit_confirm);
    RUN_TEST(test_exit_confirm_resets);
    RUN_TEST(test_lockout_period);
    RUN_TEST(test_lockout_expires_then_locks);
    RUN_TEST(test_no_flipflop_hysteresis);
    RUN_TEST(test_unstable_does_not_unlock);
    RUN_TEST(test_force_far);
    RUN_TEST(test_full_lifecycle);
    RUN_TEST(test_q4_conversions);

    tprintf("\n================================================================\n");
    tprintf("  Results: %d passed, %d failed, %d total\n",
            gTestsPassed, gTestsFailed, gTestsPassed + gTestsFailed);
    tprintf("================================================================\n\n");

    if (xmlPath != NULL) { JUnit_WriteXml(xmlPath); }

    if (gLogFile != NULL) { fclose(gLogFile); gLogFile = NULL; }

    return (gTestsFailed > 0) ? 1 : 0;
}
