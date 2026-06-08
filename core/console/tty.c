
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

#define GNSS_TYPE "bdxwephb|gpsephb|bd2ephb|bd3ephb|bd3cnav2ephb|bd3cnav3ephb|gloephb|galephb|prangeb|posdatab|rmc|gga|gll|gsa|gst|gsv|vtg|zda"

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


DEFUN(gnss_data_type_cfg,
      gnss_data_type_cfg_cmd,
      "gnss data type (auto|nmea|raw)",
      "gnss ctrl\n"
      "Set gnss data type\n"
      "auto, nmea or raw\n")
{
    if (strcmp(argv[0], "auto") == 0)
    {
        gnss_ctrl.data_type = GNSS_DATA_AUTO;
        vty_out(vty, "set gnss data type to auto%s", VTY_NEWLINE);
    }
    else if (strcmp(argv[0], "nmea") == 0)
    {
        gnss_ctrl.data_type = GNSS_DATA_NMEA;
        vty_out(vty, "set gnss data type to nmea%s", VTY_NEWLINE);
    }
    else if (strcmp(argv[0], "raw") == 0)
    {
        gnss_ctrl.data_type = GNSS_DATA_RAW;
        vty_out(vty, "set gnss data type to raw%s", VTY_NEWLINE);
    }   
    return CMD_SUCCESS;
}

DEFUN(gnss_type_on_change_cfg,
      gnss_type_on_change_cfg_cmd,
      "gnss (" GNSS_TYPE ") onchange",
      "gnss ctrl\n"
      "Set gnss <type> on or off\n"
      "on or off\n")
{
    gnss_cfg_enable_onchange(gnss_ctrl.fd, argv[0]);
    vty_out(vty, "set gnss type %s to onchange%s", argv[0], VTY_NEWLINE);
    return CMD_SUCCESS;
}

DEFUN(gnss_type_on_cfg,
      gnss_type_on_cfg_cmd,
      "gnss (" GNSS_TYPE ") on <1-10>",
      "gnss ctrl\n"
      "Set gnss <type> on or off\n"
      "on or off\n")
{

    int per_second = atoi(argv[1]);
    gnss_cfg_dis_enable(gnss_ctrl.fd, argv[0], 1, per_second);
    vty_out(vty, "set gnss type %s to on %d/s%s", argv[0], per_second, VTY_NEWLINE);

    return CMD_SUCCESS;
}

DEFUN(gnss_type_off_cfg,
      gnss_type_off_cfg_cmd,
      "gnss (" GNSS_TYPE "|all) off",
      "gnss ctrl\n"
      "Set gnss <type> on or off\n"
      "on or off\n")
{
    if (strcmp(argv[0], "all") == 0)
    {
        gnss_cfg_disable_all(gnss_ctrl.fd);
        vty_out(vty, "set gnss all type to off%s", VTY_NEWLINE);
    }
    else
    {
        gnss_cfg_dis_enable(gnss_ctrl.fd, argv[0], 0, 0);
        vty_out(vty, "set gnss type %s to off%s", argv[0], VTY_NEWLINE);
    }
    return CMD_SUCCESS;
}

DEFUN(gnss_mode_cfg,
      gnss_mode_cfg_cmd,
      "gnss mode (base|rover) (rtd|rtk|ppp|dppp|fppp) (1|2|10|13)",
      "gnss ctrl\n"
      "Set gnss mode\n"
      "Work mode\n"
      "Calculation type\n"
      "Frequency code\n")
{
    char *workMode = argv[0];
    char *calcType = argv[1];
    uint8_t freqCode = atoi(argv[2]);
    gnss_cfg_mode(gnss_ctrl.fd, workMode, calcType, freqCode);
    vty_out(vty, "set gnss mode to %s %s freq code %d%s", workMode, calcType, freqCode, VTY_NEWLINE);
    vty_out(vty, "Note: the module will restart after setting mode, please re-enable the desired data output%s", VTY_NEWLINE);
    return CMD_SUCCESS;
}



DEFUN(gnss_file_cfg,
      gnss_file_cfg_cmd,
      "gnss file (bdxwephb|gpsephb|bd2ephb|bd3ephb|bd3cnav2ephb|bd3cnav3ephb|gloephb|galephb) (on|off)",
      "gnss file <type> \n"
      "Set gnss file <type>  on or off\n"
      "on or off\n")
{
    gnss_ctrl.data_type = GNSS_DATA_AUTO;
    uint8_t sw = strcmp(argv[1], "on") == 0;
    // gnss_cfg_disable_all(gnss_ctrl.fd);
    usleep(100000);
    char *file_path = gnss_raw_info_file_header(argv[0], sw);
    if (sw)
    {
        vty_out(vty, "set gnss file %s to on, file path: %s %s", argv[0], file_path, VTY_NEWLINE);
    }
    else
    {
        vty_out(vty, "set gnss file %s to off%s", argv[0], VTY_NEWLINE);
    }
    return CMD_SUCCESS;
}

DEFUN(gnss_print_cfg,
gnss_print_cfg_cmd,
"gnss print (nmea|raw) (summary|full|none)",
"gnss print <type> \n"
"Set gnss print <type> on summary or detail\n"
"summary | full | none\n"
)
{
    uint8_t print_type = 0;
    if (strcmp(argv[1], "summary") == 0)
    {
        print_type = GNSS_PRINT_SUMMARY;
    }
    else if (strcmp(argv[1], "full") == 0)
    {
        print_type = GNSS_PRINT_FULL;
    }
    else if (strcmp(argv[1], "none") == 0)
    {
        print_type = GNSS_PRINT_NONE;
    }
   

    if (strcmp(argv[0], "nmea") == 0)
    {
        gnss_ctrl.print.nmea = print_type;
        vty_out(vty, "set gnss print nmea to %s%s", argv[1], VTY_NEWLINE);
    }
    else if (strcmp(argv[0], "raw") == 0)
    {
        gnss_ctrl.print.raw = print_type;
        vty_out(vty, "set gnss print raw to %s%s", argv[1], VTY_NEWLINE);
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


    install_element(ENABLE_NODE, &gnss_data_type_cfg_cmd);
    install_element(RADIO_NODE, &gnss_data_type_cfg_cmd);


    install_element(ENABLE_NODE, &gnss_mode_cfg_cmd);
    install_element(RADIO_NODE, &gnss_mode_cfg_cmd);


    install_element(ENABLE_NODE, &gnss_file_cfg_cmd);
    install_element(RADIO_NODE, &gnss_file_cfg_cmd);

    install_element(ENABLE_NODE, &gnss_print_cfg_cmd);
    install_element(RADIO_NODE, &gnss_print_cfg_cmd);

}
