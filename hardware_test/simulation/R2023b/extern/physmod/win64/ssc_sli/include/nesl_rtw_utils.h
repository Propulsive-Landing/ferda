/* Copyright 2013-2021 The MathWorks, Inc. */
/*!
 * @file
 * Utility functions to be called in generated code.
 * This header must be included after _nesl_rtw.h
 */

#ifndef nesl_rtw_utils_h
#define nesl_rtw_utils_h

#include "nesl_diag.h"

/*
 * UNUSED_PARAMETER(x)
 *   Used to specify that a function parameter (argument) is required but not
 *   accessed by the function body.
 */
#ifndef UNUSED_PARAMETER
#if defined(__LCC__)
#define UNUSED_PARAMETER(x) /* do nothing */
#else
/*
 * This is the semi-ANSI standard way of indicating that a
 * unused function parameter is required.
 */
#define UNUSED_PARAMETER(x) (void)(x)
#endif
#endif

/*
 * NeslSimulator
 */

PMF_DEPLOY_STATIC NeslSimulator* nesl_lease_simulator(const char* key, int category, int index) {
    const NeslSimulatorGroupRegistry* registry = nesl_get_registry();

    /* Const cast required to pass through RTWCG-generated pointer type */
    union {
        const NeslSimulator* constSimulator;
        NeslSimulator*       simulator;
    } u;

    u.constSimulator =
        registry->mLeaseSimulator(registry, key, (NeslSimulatorCategory)category, (size_t)index);

    return u.simulator;
}

PMF_DEPLOY_STATIC void nesl_erase_simulator(const char* key) {
    const NeslSimulatorGroupRegistry* registry = nesl_get_registry();
    registry->mErase(registry, key);
}

PMF_DEPLOY_STATIC void nesl_destroy_registry(void) {
    nesl_destroy_current_registry();
}

PMF_DEPLOY_STATIC void nesl_simulator_noop_status(void* bd) {
    UNUSED_PARAMETER(bd);
}

PMF_DEPLOY_STATIC int32_T nesl_initialize_simulator(NeslSimulator*           simulator,
                                                    const NeModelParameters* mp,
                                                    NeuDiagnosticManager*    mgr) {
    return simulator->mInitialize(simulator, mp, mgr, nesl_simulator_noop_status,
                                  nesl_simulator_noop_status, NULL);
}

PMF_DEPLOY_STATIC void* nesl_save_simdata(void* simulator, unsigned int* sizeInBytes) {
    NeslSimulator* sim = (NeslSimulator*)simulator;
    NeCustomData*  ncd = sim->mGetCustomData(sim);
    char*          store;
    *sizeInBytes = (unsigned int)ne_measure_packed_custom_data(ncd);
    store        = (char*)malloc(*sizeInBytes);
    ne_pack_custom_data(ncd, store);
    ne_free_custom_data(ncd);
    return (void*)store;
}

PMF_DEPLOY_STATIC void nesl_restore_simdata(void*        simulator,
                                            const void*  data,
                                            unsigned int sizeInBytes) {
    NeslSimulator* sim = (NeslSimulator*)simulator;
    NeCustomData*  ncd = ne_recreate_custom_data((const char*)data, sizeInBytes);
    sim->mSetCustomData(ncd, sim, rtw_create_diagnostics());
    ne_free_custom_data(ncd);
}

/*
 * NeslSimulationData
 */

PMF_DEPLOY_STATIC void nesl_destroy_simulation_data(NeslSimulationData* sd) {
    if (sd != NULL) {
        sd->mDestroy(sd);
    }
}

/*
 * Pointer-arithmetic (not supported by RTWCG)
 */

PMF_DEPLOY_STATIC boolean_T pointer_is_null(void* ptr) {
    return (ptr == NULL);
}

PMF_DEPLOY_STATIC double* double_pointer_shift(double* base, int offset) {
    return &(base[offset]);
}

#endif /* __nesl_rtw_utils_h__ */
/* [EOF] nesl_rtw_utils.h */
