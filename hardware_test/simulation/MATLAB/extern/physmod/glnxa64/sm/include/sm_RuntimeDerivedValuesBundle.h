#ifndef __sm_RuntimeDerivedValuesBundle_h__
#define __sm_RuntimeDerivedValuesBundle_h__
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
struct SmRealVectorTag{size_t mSize;size_t mCap;real_T*mValues;};typedef struct
SmRealVectorTag SmRealVector;void sm_core_SmRealVector_create(SmRealVector*
vector,size_t n,real_T val);void sm_core_SmRealVector_copy(SmRealVector*copy,
const SmRealVector*orig);void sm_core_SmRealVector_destroy(SmRealVector*vector
);int sm_core_SmRealVector_isEmpty(const SmRealVector*vector);size_t
sm_core_SmRealVector_size(const SmRealVector*vector);size_t
sm_core_SmRealVector_capacity(const SmRealVector*vector);real_T
sm_core_SmRealVector_value(const SmRealVector*vector,size_t index);void
sm_core_SmRealVector_setValue(SmRealVector*vector,size_t index,real_T value);
const real_T*sm_core_SmRealVector_values(const SmRealVector*vector);real_T*
sm_core_SmRealVector_nonConstValues(SmRealVector*vector);void
sm_core_SmRealVector_reserve(SmRealVector*vector,size_t n);void
sm_core_SmRealVector_clear(SmRealVector*vector);void
sm_core_SmRealVector_pushBack(SmRealVector*vector,real_T val);void
sm_core_SmRealVector_popBack(SmRealVector*vector);struct SmIntVectorTag{size_t
mSize;size_t mCap;int*mValues;};typedef struct SmIntVectorTag SmIntVector;void
sm_core_SmIntVector_create(SmIntVector*vector,size_t n,int val);void
sm_core_SmIntVector_copy(SmIntVector*copy,const SmIntVector*orig);void
sm_core_SmIntVector_destroy(SmIntVector*vector);int sm_core_SmIntVector_isEmpty
(const SmIntVector*vector);size_t sm_core_SmIntVector_size(const SmIntVector*
vector);size_t sm_core_SmIntVector_capacity(const SmIntVector*vector);int
sm_core_SmIntVector_value(const SmIntVector*vector,size_t index);void
sm_core_SmIntVector_setValue(SmIntVector*vector,size_t index,int value);const
int*sm_core_SmIntVector_values(const SmIntVector*vector);int*
sm_core_SmIntVector_nonConstValues(SmIntVector*vector);void
sm_core_SmIntVector_reserve(SmIntVector*vector,size_t n);void
sm_core_SmIntVector_clear(SmIntVector*vector);void sm_core_SmIntVector_pushBack
(SmIntVector*vector,int val);void sm_core_SmIntVector_popBack(SmIntVector*
vector);struct RuntimeDerivedValuesBundleTag{SmRealVector mDoubles;SmIntVector
mInts;};typedef struct RuntimeDerivedValuesBundleTag RuntimeDerivedValuesBundle
;void sm_core_RuntimeDerivedValuesBundle_create(RuntimeDerivedValuesBundle*
bundle,size_t nDoubles,size_t nInts);void
sm_core_RuntimeDerivedValuesBundle_copy(const RuntimeDerivedValuesBundle*orig,
RuntimeDerivedValuesBundle*copy);void
sm_core_RuntimeDerivedValuesBundle_destroy(RuntimeDerivedValuesBundle*bundle);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __sm_RuntimeDerivedValuesBundle_h__ */
