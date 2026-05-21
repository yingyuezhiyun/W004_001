
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "quagga/zebra.h"
#include "quagga/if.h"
#include "quagga/command.h"
#include "quagga/prefix.h"
#include "quagga/table.h"
#include "quagga/thread.h"
#include "memory.h"
#include "quagga/log.h"
#include "quagga/stream.h"
#include "quagga/filter.h"
#include "quagga/sockunion.h"
#include "quagga/sockopt.h"
#include "quagga/routemap.h"
#include "quagga/if_rmap.h"
#include "quagga/plist.h"
#include "quagga/distribute.h"
#include "quagga/md5.h"
#include "quagga/keychain.h"
#include "quagga/privs.h"
#include "quagga/vty.h"
#include "quagga/memory.h"


#include "console/tty.h"






#define RADIO_NODE RIP_NODE

/*  node structure. */
static struct cmd_node vty_node =
    {
        RIP_NODE,
        "%s(config-radio)# ",
        1};

struct vty_cfg_s
{
    int sock;
};

struct vty_cfg_s *vty_cfg = NULL;

DEFUN(gnss_type_on_cfg,
      gnss_type_on_cfg_cmd,
      "gnss (gpsephb|rmc|gga|gll|gsa|gst|gsv|vtg|zda) (onchange|on <1-10>)",
      "gnss ctrl\n"
      "Set gnss type on or off\n"
      "on or off\n")
{
    if (strcmp(argv[1], "onchange") == 0)
    {
        vty_out(vty, "set gnss type %s to onchange", argv[0]);
    }
    else
    {
        int per_second = atoi(argv[2]);
        vty_out(vty, "set gnss type %s to on %d/s", argv[0], per_second);
    }
    return CMD_SUCCESS;
}

DEFUN(gnss_type_off_cfg,
      gnss_type_off_cfg_cmd,
      "gnss (gpsephb|rmc|gga|gll|gsa|gst|gsv|vtg|zda|all) off",
      "gnss ctrl\n"
      "Set gnss type on or off\n"
      "on or off\n")
{
    if (strcmp(argv[0], "all") == 0)
    {
        vty_out(vty, "set gnss all type to off");
    }
    else
    {
        vty_out(vty, "set gnss type %s to off", argv[0]);
    }
    return CMD_SUCCESS;
}

DEFUN(config_radio,
      config_radio_cmd,
      "configure radio",
      "Enabale a configure process\n"
      "Enabale a radio configure process.\n")
{
    if (!vty_cfg)
    {
        vty_cfg = XCALLOC(MTYPE_RIP, sizeof(struct vty_cfg_s));
    }
    vty->node = RADIO_NODE;
    vty->index = vty_cfg;
    return CMD_SUCCESS;
}





/* RADIO configuration write function. */
static int config_write_vty(struct vty *vty)
{
    int write = 0;

    if (vty_cfg)
    {
        /* configure radio */
        vty_out(vty, "configure radio%s", VTY_NEWLINE);
        write++;
     
    }

    return write;
}

void tty_init(void)
{
    /* Randomize for triggered update random(). */
    srand(time(NULL));

    /* Install top nodes. */
    install_node(&vty_node, config_write_vty);
    install_element(CONFIG_NODE, &config_radio_cmd);
   
    install_element(ENABLE_NODE, &gnss_type_off_cfg_cmd);
    install_element(RADIO_NODE, &gnss_type_off_cfg_cmd);

    install_element(ENABLE_NODE, &gnss_type_on_cfg_cmd);
    install_element(RADIO_NODE, &gnss_type_on_cfg_cmd);

}
