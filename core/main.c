#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include "src_thread.h"

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IOLBF, 0);

    const char *label = (argc > 1) ? argv[1] : "ok3506";
    const char *delay_text = (argc > 2) ? argv[2] : "0";
    int delay_seconds = atoi(delay_text);

    printf("ok3506 demo started\n");
    printf("label: %s\n", label);
    printf("pid: %d\n", getpid());
    printf("sleep: %d seconds\n", delay_seconds);

    thread_init();

   
    while (1)
    {
        usleep(1000);
    }

    printf("done\n");
    return 0;
}
