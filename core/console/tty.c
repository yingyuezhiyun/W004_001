
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
#include "dbb_func.h"
#include "glob_value.h"
#include "eth_4g.h"

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

DEFUN(gnss_parse_data_type_cfg,
      gnss_parse_data_type_cfg_cmd,
      "gnss data type (auto|nmea|ephb)",
      "gnss ctrl\n"
      "Set gnss data type\n"
      "auto, nmea or ephb\n")
{
    if (strcmp(argv[0], "auto") == 0)
    {
        gnss_ctrl.parseDataType = GNSS_DATA_AUTO;
        vty_out(vty, "set gnss data type to auto%s", VTY_NEWLINE);
    }
    else if (strcmp(argv[0], "nmea") == 0)
    {
        gnss_ctrl.parseDataType = GNSS_DATA_NMEA;
        vty_out(vty, "set gnss data type to nmea%s", VTY_NEWLINE);
    }
    else if (strcmp(argv[0], "ephb") == 0)
    {
        gnss_ctrl.parseDataType = GNSS_DATA_EPHB;
        vty_out(vty, "set gnss data type to ephb%s", VTY_NEWLINE);
    }
    return CMD_SUCCESS;
}

DEFUN(gnss_datatype_onchange_cfg,
      gnss_datatype_onchange_cfg_cmd,
      "gnss (" GNSS_TYPE ") onchange",
      "gnss ctrl\n"
      "Set gnss <type> on or off\n"
      "on or off\n")
{
    gnss_cfg_dataType_onchange(argv[0]);
    vty_out(vty, "set gnss type %s to onchange%s", argv[0], VTY_NEWLINE);
    return CMD_SUCCESS;
}

DEFUN(gnss_datatype_cfg,
      gnss_datatype_cfg_cmd,
      "gnss (" GNSS_TYPE ") on <1-10>",
      "gnss ctrl\n"
      "Set gnss <type> on or off\n"
      "on or off\n")
{

    int per_second = atoi(argv[1]);
    gnss_cfg_dataType(argv[0], 1, per_second);
    vty_out(vty, "set gnss type %s to on %d/s%s", argv[0], per_second, VTY_NEWLINE);

    return CMD_SUCCESS;
}


DEFUN(gnss_datatype_off_cfg,
      gnss_datatype_off_cfg_cmd,
      "gnss (" GNSS_TYPE "|all) off",
      "gnss ctrl\n"
      "Set gnss <type> on or off\n"
      "on or off\n")
{
    if (strcmp(argv[0], "all") == 0)
    {
        gnss_cfg_disable_out();
        vty_out(vty, "set gnss all type to off%s", VTY_NEWLINE);
    }
    else
    {
        gnss_cfg_dataType(argv[0], 0, 0);
        vty_out(vty, "set gnss type %s to off%s", argv[0], VTY_NEWLINE);
    }
    return CMD_SUCCESS;
}

DEFUN(show_gnss_datatype,
      show_gnss_datatype_cmd,
      "show gnss data type",
      SHOW_STR
      "gnss ctrl\n"
      "Show gnss data type\n")
{
    vty_out(vty, "gnss data type: %s", VTY_NEWLINE);
    vty_out(vty,"\tgbs: \t\t%s%s", gnss_ctrl.dataType.nmea.gbs ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\tgga: \t\t%s%s", gnss_ctrl.dataType.nmea.gga ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\tgll: \t\t%s%s", gnss_ctrl.dataType.nmea.gll ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\tgsa: \t\t%s%s", gnss_ctrl.dataType.nmea.gsa ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\tgst: \t\t%s%s", gnss_ctrl.dataType.nmea.gst ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\tgsv: \t\t%s%s", gnss_ctrl.dataType.nmea.gsv ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\trmc: \t\t%s%s", gnss_ctrl.dataType.nmea.rmc ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\tvtg: \t\t%s%s", gnss_ctrl.dataType.nmea.vtg ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\tzda: \t\t%s%s", gnss_ctrl.dataType.nmea.zda ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\tbdxwephb: \t%s%s", gnss_ctrl.dataType.ephb.bdxw ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\tgpsephb: \t%s%s", gnss_ctrl.dataType.ephb.gps ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\tbd2ephb: \t%s%s", gnss_ctrl.dataType.ephb.bd2 ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\tbd3ephb: \t%s%s", gnss_ctrl.dataType.ephb.bd3 ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\tbd3cnav2ephb: \t%s%s", gnss_ctrl.dataType.ephb.bd3cnav2 ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\tbd3cnav3ephb: \t%s%s", gnss_ctrl.dataType.ephb.bd3cnav3 ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\tgloephb: \t%s%s", gnss_ctrl.dataType.ephb.glo ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\tgalephb: \t%s%s", gnss_ctrl.dataType.ephb.gal ? "on" : "off", VTY_NEWLINE);
    return CMD_SUCCESS;
}


DEFUN(gnss_sys_cfg,
      gnss_sys_cfg_cmd,
      "gnss sys (on|off) {all|bds|gsp|glo|gal}",
      "gnss sys ctrl\n"
      "Set gnss <sys> on or off\n"
      "on or off\n")
{

    uint8_t enable = strcmp(argv[0], "on") == 0;
    char sys[50]={0};
    for (size_t i = 1; i < argc; i++)
    {
        // 多个系统可组合
        if (argv[i] != NULL)
        {
            strcat(sys, " ");
            strcat(sys, argv[i]);
        }
    }
    gnss_cfg_sys(sys, enable);

    vty_out(vty, "set gnss sys %s to %s%s", sys, enable ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\033[1;33mNote: the module will restart after setting mode, please re-enable the desired data output%s\033[0m", VTY_NEWLINE);
    return CMD_SUCCESS;
}

DEFUN(show_gnss_sys_cfg,
      show_gnss_sys_cfg_cmd,
      "show gnss sys",
      SHOW_STR
      "gnss sys ctrl\n"
      "Show gnss sys\n")
{
    vty_out(vty, "gnss sys: %s", VTY_NEWLINE);
    vty_out(vty, "\tbds: \t\t%s%s", gnss_ctrl.sysType.content.bds ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\tgps: \t\t%s%s", gnss_ctrl.sysType.content.gps ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\tglo: \t\t%s%s", gnss_ctrl.sysType.content.glo ? "on" : "off", VTY_NEWLINE);
    vty_out(vty, "\tgal: \t\t%s%s", gnss_ctrl.sysType.content.gal ? "on" : "off", VTY_NEWLINE);
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
    gnss_cfg_mode(workMode, calcType, freqCode);
    vty_out(vty, "set gnss mode to %s %s freq code %d%s", workMode, calcType, freqCode, VTY_NEWLINE);
    vty_out(vty, "\033[1;33mNote: the module will restart after setting mode, please re-enable the desired data output%s\033[0m", VTY_NEWLINE);
    return CMD_SUCCESS;
}

DEFUN(gnss_file_cfg,
      gnss_file_cfg_cmd,
      "gnss file (gbs|gga|gll|gsa|gst|gsv|rmc|vtg|zda|bdxwephb|gpsephb|bd2ephb|bd3ephb|bd3cnav2ephb|bd3cnav3ephb|gloephb|galephb) (on|off)",
      "gnss file <type> \n"
      "Set gnss file <type>  on or off\n"
      "on or off\n")
{
    gnss_ctrl.parseDataType = GNSS_DATA_AUTO;
    uint8_t sw = strcmp(argv[1], "on") == 0;
    // gnss_cfg_disable_all();
    usleep(100000);
    char *file_path;
    if (strcmp(argv[0], "gbs") == 0 || strcmp(argv[0], "gga") == 0 ||
        strcmp(argv[0], "gll") == 0 || strcmp(argv[0], "gsa") == 0 ||
        strcmp(argv[0], "gst") == 0 || strcmp(argv[0], "gsv") == 0 ||
        strcmp(argv[0], "rmc") == 0 || strcmp(argv[0], "vtg") == 0 ||
        strcmp(argv[0], "zda") == 0)
    {
        file_path = gnss_nmea_file_header(argv[0], sw);
    }
    else
    {
        file_path = gnss_ephb_info_file_header(argv[0], sw);
    }
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
      "gnss print (nmea|nmea_raw|ephb) (summary|full|none)",
      "gnss print <type> \n"
      "Set gnss print <type> on summary or detail\n"
      "summary | full | none\n")
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
    else if (strcmp(argv[0], "ephb") == 0)
    {
        gnss_ctrl.print.ephb = print_type;
        vty_out(vty, "set gnss print ephb to %s%s", argv[1], VTY_NEWLINE);
    }
    else if (strcmp(argv[0], "nmea_raw") == 0)
    {
        gnss_ctrl.print.nmea_raw = print_type;
        vty_out(vty, "set gnss print nmea_raw to %s%s", argv[1], VTY_NEWLINE);
    }   
    return CMD_SUCCESS;
}

DEFUN(dbb_print_cfg,
      dbb_print_cfg_cmd,
      "dbb print (info|err) (on|off)",
      "dbb print <type> \n"
      "Set dbb print <type> on or off\n"
      "on or off\n")
{
    uint8_t enable = strcmp(argv[1], "on") == 0;
    if (strcmp(argv[0], "info") == 0)
    {
        dbb_ctrl.print.info = enable ? DBB_PRINT_ON : DBB_PRINT_OFF;
        vty_out(vty, "set dbb print info to %s%s", argv[1], VTY_NEWLINE);
    }
    else if (strcmp(argv[0], "err") == 0)
    {
        dbb_ctrl.print.err = enable ? DBB_PRINT_ON : DBB_PRINT_OFF;
        vty_out(vty, "set dbb print err to %s%s", argv[1], VTY_NEWLINE);
    }
    return CMD_SUCCESS;
}

DEFUN(eth_4g_print_cfg,
      eth_4g_print_cfg_cmd,
      "eth_4g print (info|err) (on|off)",
      "eth_4g print <type> \n"
      "Set eth_4g print <type> on or off\n"
      "on or off\n")
{
    uint8_t enable = strcmp(argv[1], "on") == 0;
    if (strcmp(argv[0], "info") == 0)
    {
        eth_4g_ctrl.print.info = enable ? ETH_4G_PRINT_ON : ETH_4G_PRINT_OFF;
        vty_out(vty, "set eth_4g print info to %s%s", argv[1], VTY_NEWLINE);
    }
    else if (strcmp(argv[0], "err") == 0)
    {
        eth_4g_ctrl.print.err = enable ? ETH_4G_PRINT_ON : ETH_4G_PRINT_OFF;
        vty_out(vty, "set eth_4g print err to %s%s", argv[1], VTY_NEWLINE);
    }
    return CMD_SUCCESS;
}

DEFUN(gnss_freset,
      gnss_freset_cmd,
      "gnss reset",
      "gnss reset\n"
      "Reset gnss module\n")
{
    gnss_reset();
    vty_out(vty, "gnss module reset%s", VTY_NEWLINE);
    return CMD_SUCCESS;
}

DEFUN(gnss_cfgSave,
      gnss_cfgSave_cmd,
      "gnss cfg save",
      "gnss cfg save\n"
      "Save gnss module config\n")
{
    gnss_cfg_save();
    vty_out(vty, "gnss module config saved%s", VTY_NEWLINE);
    return CMD_SUCCESS;
}

DEFUN(test_opt,
      test_opt_cmd,
      "test {opt1 <1-9>|opt2 (on|off)| opt3 | opt4 [IFNAME]}",
      "Test <type> \n"
      "Set test <type> on or off\n"
      "on or off\n")
{
    
    return CMD_SUCCESS;
}

DEFUN(gnss_net_up_cfg,
      gnss_net_up_cfg_cmd,
      "gnss net up (bdxwephb|gpsephb|bd2ephb|bd3ephb|bd3cnav2ephb|bd3cnav3ephb|gloephb|galephb) (on|off)",
      "gnss net up <type> (on|off)\n"
      "Set gnss net up <type> on or off\n"
      "on or off\n")
{
    uint8_t sw = strcmp(argv[1], "on") == 0;
    if (strcmp(argv[0], "bdxwephb") == 0)
    {
        glob_comm_config.eph_sw.content.bdxw = sw;
    }
    else if (strcmp(argv[0], "gpsephb") == 0)
    {
        glob_comm_config.eph_sw.content.gps = sw;
    }
    else if (strcmp(argv[0], "bd2ephb") == 0)
    {
        glob_comm_config.eph_sw.content.bd2 = sw;
    }
    else if (strcmp(argv[0], "bd3ephb") == 0)
    {
        glob_comm_config.eph_sw.content.bd3 = sw;
    }
    else if (strcmp(argv[0], "bd3cnav2ephb") == 0)
    {
        glob_comm_config.eph_sw.content.bd3cnav2 = sw;
    }
    else if (strcmp(argv[0], "bd3cnav3ephb") == 0)
    {
        glob_comm_config.eph_sw.content.bd3cnav3 = sw;
    }
    else if (strcmp(argv[0], "gloephb") == 0)
    {
        glob_comm_config.eph_sw.content.glo = sw;
    }
    else if (strcmp(argv[0], "galephb") == 0)
    {
        glob_comm_config.eph_sw.content.gal = sw;
    }

    vty_out(vty, "set gnss net up %s to %s%s", argv[0], argv[1], VTY_NEWLINE);

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

    install_element(ENABLE_NODE, &gnss_datatype_off_cfg_cmd);
    install_element(RADIO_NODE, &gnss_datatype_off_cfg_cmd);

    install_element(ENABLE_NODE, &gnss_datatype_cfg_cmd);
    install_element(RADIO_NODE, &gnss_datatype_cfg_cmd);

    install_element(ENABLE_NODE, &gnss_datatype_onchange_cfg_cmd);
    install_element(RADIO_NODE, &gnss_datatype_onchange_cfg_cmd);

    install_element(ENABLE_NODE, &show_gnss_datatype_cmd);

    install_element(ENABLE_NODE, &gnss_parse_data_type_cfg_cmd);
    install_element(RADIO_NODE, &gnss_parse_data_type_cfg_cmd);

    install_element(ENABLE_NODE, &gnss_mode_cfg_cmd);
    install_element(RADIO_NODE, &gnss_mode_cfg_cmd);

    install_element(ENABLE_NODE, &gnss_file_cfg_cmd);
    install_element(RADIO_NODE, &gnss_file_cfg_cmd);

    install_element(ENABLE_NODE, &gnss_print_cfg_cmd);
    install_element(RADIO_NODE, &gnss_print_cfg_cmd);

    install_element(ENABLE_NODE, &gnss_net_up_cfg_cmd);
    install_element(RADIO_NODE, &gnss_net_up_cfg_cmd);

    install_element(ENABLE_NODE, &gnss_sys_cfg_cmd);
    install_element(RADIO_NODE, &gnss_sys_cfg_cmd);

    install_element(ENABLE_NODE, &show_gnss_sys_cfg_cmd);

    install_element(ENABLE_NODE, &gnss_freset_cmd);
    install_element(ENABLE_NODE, &gnss_cfgSave_cmd);

    install_element(ENABLE_NODE, &dbb_print_cfg_cmd);
    install_element(RADIO_NODE, &dbb_print_cfg_cmd);

    install_element(ENABLE_NODE, &eth_4g_print_cfg_cmd);
    install_element(RADIO_NODE, &eth_4g_print_cfg_cmd);

    install_element(ENABLE_NODE, &test_opt_cmd);
}
