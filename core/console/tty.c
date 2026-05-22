
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

#include "glob_cfg.h"
#include "gnss_func.h"

#define RADIO_NODE RIP_NODE

#define GNSS_TYPE "gpsephb|rmc|gga|gll|gsa|gst|gsv|vtg|zda"

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

DEFUN(gnss_type_on_change_cfg,
      gnss_type_on_change_cfg_cmd,
      "gnss (" GNSS_TYPE ") onchange",
      "gnss ctrl\n"
      "Set gnss type on or off\n"
      "on or off\n")
{
    gnss_cfg_eable_onchange(gnss_ctrl.fd, argv[0]);
    vty_out(vty, "set gnss type %s to onchange%s", argv[0], VTY_NEWLINE);
    return CMD_SUCCESS;
}

DEFUN(gnss_type_on_cfg,
      gnss_type_on_cfg_cmd,
      "gnss (" GNSS_TYPE ") on <1-10>",
      "gnss ctrl\n"
      "Set gnss type on or off\n"
      "on or off\n")
{

    int per_second = atoi(argv[1]);
    gnss_cfg(gnss_ctrl.fd, argv[0], 1, per_second);
    vty_out(vty, "set gnss type %s to on %d/s%s", argv[0], per_second, VTY_NEWLINE);

    return CMD_SUCCESS;
}

DEFUN(gnss_type_off_cfg,
      gnss_type_off_cfg_cmd,
      "gnss (" GNSS_TYPE "|all) off",
      "gnss ctrl\n"
      "Set gnss type on or off\n"
      "on or off\n")
{
    if (strcmp(argv[0], "all") == 0)
    {
        gnss_cfg_close_all(gnss_ctrl.fd);
        vty_out(vty, "set gnss all type to off%s", VTY_NEWLINE);
    }
    else
    {
        gnss_cfg(gnss_ctrl.fd, argv[0], 0, 0);
        vty_out(vty, "set gnss type %s to off%s", argv[0], VTY_NEWLINE);
    }
    return CMD_SUCCESS;
}

DEFUN(gnss_gps_test,
      gnss_gps_test_cmd,
      "gnss gps test  (on|off)",
      "gnss gps test\n"
      "Set gnss gps test on or off\n"
      "on or off\n")
{
    if (strcmp(argv[0], "on") == 0)
    {
        gnss_cfg_close_all(gnss_ctrl.fd);
        usleep(100000);
        gnss_cfg(gnss_ctrl.fd, "GPSEPHB", 1, 1);
        gnss_ctrl.data_type = GNSS_DATA_RAW;
        vty_out(vty, "set gnss gps test to on%s", VTY_NEWLINE);
    }
    else
    {
        gnss_cfg_close_all(gnss_ctrl.fd);
        usleep(100000);
        gnss_cfg(gnss_ctrl.fd, "RMC", 1, 1);
        gnss_ctrl.data_type = GNSS_DATA_NMEA;

        vty_out(vty, "set gnss gps test to off%s", VTY_NEWLINE);
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

    install_element(ENABLE_NODE, &gnss_type_on_change_cfg_cmd);
    install_element(RADIO_NODE, &gnss_type_on_change_cfg_cmd);

    install_element(ENABLE_NODE, &gnss_gps_test_cmd);
}
