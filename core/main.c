#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include "src_thread.h"
#include "console/cli.h"

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IOLBF, 0);

    // cli_pre_init("config.conf", "/var/run/zebra_vty.pid", 2612);

    printf("ok3506 demo started\n");
  
    printf("pid: %d\n", getpid());

    cli_init(argc, argv);
    thread_init();
    cli_run();

    while (1)
    {
        usleep(1000);
    }

    printf("done\n");
    return 0;
}
