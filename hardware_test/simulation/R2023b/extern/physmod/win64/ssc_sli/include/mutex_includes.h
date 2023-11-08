/* Copyright 2020 The MathWorks, Inc. */
#ifndef mutex_includes_h
#define mutex_includes_h

#if !defined(NO_MULTITHREADED_SUPPORT) &&                                            \
    (defined(__QNX__) || defined(__linux__) || defined(__linux) || defined(linux) || \
     (defined(__APPLE_CC__) && (__clang_major__ > 4)))
#include <pthread.h>
typedef pthread_t       threadnum_t_ssc;
typedef pthread_mutex_t mutex_type_ssc;

threadnum_t_ssc getThisThreadId_ssc(void) {
    return pthread_self();
}

void lock_mutex_ssc(mutex_type_ssc* x) {
    int mutex_result = pthread_mutex_lock(x);
}
mutex_type_ssc init_mutex_ssc(void) {
    mutex_type_ssc x = PTHREAD_MUTEX_INITIALIZER;
    return x;
}
void release_mutex_ssc(mutex_type_ssc* x) {
    int mutex_result = pthread_mutex_unlock(x);
}

#elif !defined(NO_MULTITHREADED_SUPPORT) && !defined(__clang__) && \
    (defined(_MSC_VER) || defined(__WIN32__) || defined(WIN32) || defined(_WIN32))
/* include windows.h first */
#include <windows.h>

/* then the rest of these */
#include <processthreadsapi.h>
#include <synchapi.h>
typedef DWORD   threadnum_t_ssc;
typedef HANDLE  mutex_type_ssc;
threadnum_t_ssc getThisThreadId_ssc(void) {
    return GetCurrentThreadId();
}
void lock_mutex_ssc(mutex_type_ssc* x) {
    DWORD mutex_result = WaitForSingleObject(*x, INFINITE);
}
mutex_type_ssc init_mutex_ssc(void) {
    mutex_type_ssc ret = CreateMutex(NULL, FALSE, NULL);
    return ret;
}
void release_mutex_ssc(mutex_type_ssc* x) {
    BOOL mutex_result = ReleaseMutex(*x);
}

#else
typedef size_t  threadnum_t_ssc;
typedef size_t  mutex_type_ssc;
threadnum_t_ssc getThisThreadId_ssc(void) {
    return 0;
}
void lock_mutex_ssc(mutex_type_ssc* x) {
    return;
}

mutex_type_ssc init_mutex_ssc() {
    return 0;
}
void release_mutex_ssc(mutex_type_ssc* x) {
    return;
}

#endif

#endif
