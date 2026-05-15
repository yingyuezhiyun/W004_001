#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

    int thread_init();
    void *gnss_thread_func(void *arg);  

#ifdef __cplusplus
}
#endif
