#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "src_thread.h"

pthread_t gnss_thread;

int thread_init()
{

    // pthread_mutex_init(&g_ctrl_args.tx_modul._mutex, NULL);
    // pthread_mutex_init(&g_ctrl_args.rx_demodul._mutex, NULL);
    // sem_init(&g_ctrl_args.tx_modul._sem, 0, 0);
    // sem_init(&g_ctrl_args.rx_demodul._sem, 0, 0);

    pthread_create(&gnss_thread, NULL, gnss_thread_func, NULL);

    return 0;
}
