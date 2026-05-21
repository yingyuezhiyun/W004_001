#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "src_thread.h"

pthread_t gnss_thread;
pthread_t lora_thread;
pthread_t eth_4g_thread;
pthread_t dbb_thread;

int thread_init()
{

    // pthread_mutex_init(&g_ctrl_args.tx_modul._mutex, NULL);
    // pthread_mutex_init(&g_ctrl_args.rx_demodul._mutex, NULL);
    // sem_init(&g_ctrl_args.tx_modul._sem, 0, 0);
    // sem_init(&g_ctrl_args.rx_demodul._sem, 0, 0);

    // pthread_create(&gnss_thread, NULL, gnss_thread_func, NULL);
    // pthread_create(&lora_thread, NULL, lora_thread_func, NULL);
    // pthread_create(&eth_4g_thread, NULL, eth_4g_thread_func, NULL);
    // pthread_create(&dbb_thread, NULL, dbb_thread_func, NULL);

    return 0;
}
