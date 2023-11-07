#ifndef __sm_ssci_run_time_errors_h__
#define __sm_ssci_run_time_errors_h__
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
PmfMessageId sm_ssci_recordRunTimeError(const char*errorId,const char*errorMsg
,NeuDiagnosticManager*mgr);PmfMessageId sm_ssci_recordRunTimeErrorSegmented(
const char*errorId,size_t numSegments,const char*errorMsgs[],char*
concatenationBuffer,NeuDiagnosticManager*mgr);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __sm_ssci_run_time_errors_h__ */
