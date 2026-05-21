
#include "quagga/zebra.h"
#include "quagga/version.h"
#include "quagga/getopt.h"
#include "quagga/thread.h"
#include "quagga/command.h"
#include "quagga/memory.h"
#include "quagga/prefix.h"
#include "quagga/filter.h"
#include "quagga/keychain.h"
#include "quagga/log.h"
#include "quagga/privs.h"
#include "quagga/sigevent.h"
#include "quagga/vty.h"
#include "tty.h"
#include <errno.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <stdio.h>

static const char default_config_template[] =
"!\n"
"! Zebra configuration saved from vty\n"
"!   2025/11/25 14:15:32\n"
"!\n"
"hostname Zebra\n"
"password root\n"
"enable password root\n"
"log stdout\n"
"log syslog\n"
"log record-priority\n"
"log timestamp precision 3\n"
"!\n"
"configure radio\n"
" \n"
"!\n"
"line vty\n"
" exec-timeout 0 0\n"
"!\n";

#ifndef CLI_DEFAULT_CONFIG
#define CLI_DEFAULT_CONFIG "/root/config.conf"
#endif

#ifndef CLI_DEFAULT_PID
#define CLI_DEFAULT_PID "/var/run/zebra_vty.pid"
#endif

#ifndef CLI_VTYSH_PATH
#ifdef VTYSH_PATH
#define CLI_VTYSH_PATH VTYSH_PATH "cli"
#else
#define CLI_VTYSH_PATH "/var/run/quagga/cli_vty"
#endif
#endif

#define VTY_PORT                  2612

static void start_vty_server(void);
static const char *resolved_vty_addr(void);
static const char *resolved_vty_socket(void);
static void cli_reload_config(void);
static int file_exists_and_regular(const char *path);
static int ensure_default_config_file(void);

static struct option longopts[] =
	{
		{"daemon", no_argument, NULL, 'd'},
		{"config_file", required_argument, NULL, 'f'},
		{"pid_file", required_argument, NULL, 'i'},
		{"socket", required_argument, NULL, 'z'},
		{"help", no_argument, NULL, 'h'},
		{"dryrun", no_argument, NULL, 'C'},
		{"vty_addr", required_argument, NULL, 'A'},
		{"vty_port", required_argument, NULL, 'P'},
		{"retain", no_argument, NULL, 'r'},
		{"user", required_argument, NULL, 'u'},
		{"group", required_argument, NULL, 'g'},
		{"version", no_argument, NULL, 'v'},
		{0}};

/* ripd privileges */
zebra_capabilities_t _caps_p[] = {ZCAP_NET_RAW, ZCAP_BIND};

static const char *default_config_name = CLI_DEFAULT_CONFIG;
static const char *config_default = CLI_DEFAULT_CONFIG;

struct zebra_privs_t main_vty_privs = {
#if defined(QUAGGA_USER)
	.user = QUAGGA_USER,
#endif
#if defined QUAGGA_GROUP
	.group = QUAGGA_GROUP,
#endif
#ifdef VTY_GROUP
	.vty_group = VTY_GROUP,
#endif
	.caps_p = _caps_p,
	.cap_num_p = 2,
	.cap_num_i = 0};
/* Configuration file and directory. */
char *config_file = NULL;

/* Route retain mode flag. */
static int retain_mode = 0;

/* RIP VTY bind address. */
static char *vty_addr = NULL;

/* RIP VTY connection port. */
static int vty_port = VTY_PORT;

/* Optional unix socket path for vtysh */
static const char *vty_path = CLI_VTYSH_PATH;

/* Master of threads. */
static struct thread_master *master;

/* Process ID saved for use by init system */
static const char *pid_file = CLI_DEFAULT_PID;

/* Generated fallback config file in the current working directory. */
static char generated_config_file[128];
static int generated_config_file_ready = 0;

/* Help information display. */
static void usage(char *progname, int status)
{
	if (status != 0)
		fprintf(stderr, "Try `%s --help' for more information.\n", progname);
	else
	{
		printf("Usage :  [OPTION...]\n\
	Daemon which manages user_cli.\n\n\
		-d, --daemon       Runs in daemon mode\n\
		-f, --config_file  Set configuration file name\n\
		-i, --pid_file     Set process identifier file name\n\
		-z, --socket       Set unix socket path used by vtysh\n\
		-C, --dryrun       Check configuration for validity and exit\n\
		-r, --retain       When program terminates, retain added route by ripd.\n\
		-A, --vty_addr     Set vty's bind address\n\
		-P, --vty_port     Set vty's port number\n\
		-u, --user         User to run as\n\
		-g, --group        Group to run as\n\
		-v, --version      Print program version\n\
		-h, --help         Display this help and exit\n\
");
	}

	exit(status);
}

/* SIGHUP handler. */
static void sighup(void)
{
	zlog_info("SIGHUP received");
	zlog_info("vty restarting!");
	cli_reload_config();
	start_vty_server();

	/* Try to return to normal operation. */
}



/* SIGINT handler. */
static void sigint(void)
{
	zlog_notice("Terminating on signal");
	exit(0);
}

/* SIGUSR1 handler. */
static void sigusr1(void)
{
	zlog_rotate(NULL);
}

static inline void out_stack(void)
{
	void *array[32];
	size_t size;
	char **strings;
	int i;

	size = backtrace(array, 32);
	if(size == 0)
	{
		printf("backtrace.\n");
		return;
	}
	strings = backtrace_symbols(array, size);
	if(strings == NULL)
	{
		printf("backtrace symbols\n");
		return;
	}

	for(i=0;i<size;i++)
	{
		printf("%s->", strings[i]);
	}
	free(strings);
}

static void signal_segv_func(int sig_num)
{
	printf("Terminating on signal %d.\n", sig_num);
	out_stack();
	exit(-1);
}

static struct quagga_signal_t ripd_signals[] = {
	{
		.signal = SIGHUP,
		.handler = &sighup,
	},
	{
		.signal = SIGUSR1,
		.handler = &sigusr1,
	},
	{
		.signal = SIGINT,
		.handler = &sigint,
	},
	{
		.signal = SIGTERM,
		.handler = &sigint,
	},
};

void cli_pre_init(const char *default_cfg, const char *default_pid, int default_vty_port)
{
	if (default_cfg && *default_cfg)
		default_config_name = default_cfg;
	pid_file = default_pid;
	vty_port = default_vty_port;
}




static const char *resolved_vty_addr(void)
{
	return (vty_addr && *vty_addr) ? vty_addr : "0.0.0.0";
}

static const char *resolved_vty_socket(void)
{
	if (vty_path && *vty_path)
		return vty_path;
	return CLI_VTYSH_PATH;
}

static void cli_reload_config(void)
{
	vty_read_config(config_file, (char *)config_default);
}

static int file_exists_and_regular(const char *path)
{
	struct stat st;

	return path && *path && stat(path, &st) == 0 && S_ISREG(st.st_mode);
}

static int ensure_default_config_file(void)
{
	char cwd[PATH_MAX];
	const char *base_name = strrchr(default_config_name, '/');

	if (base_name && base_name[1] != '\0')
		base_name++;
	else
		base_name = default_config_name;

	if (!generated_config_file_ready)
	{
		if (!getcwd(cwd, sizeof(cwd)))
		{
			zlog_err("failed to resolve current working directory: %s", strerror(errno));
			return -1;
		}

		if (snprintf(generated_config_file, sizeof(generated_config_file), "%s/%s", cwd, base_name) >= (int)sizeof(generated_config_file))
		{
			zlog_err("default config path is too long");
			return -1;
		}

		generated_config_file_ready = 1;
	}

	if (!file_exists_and_regular(generated_config_file))
	{
		FILE *fp = fopen(generated_config_file, "w");
		if (!fp)
		{
			zlog_err("failed to create default config '%s': %s", generated_config_file, strerror(errno));
			return -1;
		}
		fputs(default_config_template, fp);
		fclose(fp);
	}

	config_default = generated_config_file;
	return 0;
}

static void start_vty_server(void)
{
	const char *bind_addr = resolved_vty_addr();
	const char *socket_path = resolved_vty_socket();

	if (socket_path)
		zlog_info("vty socket: bind='%s' port=%d unix='%s'", bind_addr, vty_port, socket_path);
	else
		zlog_info("vty socket: bind='%s' port=%d", bind_addr, vty_port);

	vty_serv_sock(bind_addr, vty_port, socket_path);
}

int cli_init(int argc, char **argv)
{
	char *p;
	int daemon_mode = 0;
	int dryrun = 0;
	char *progname;
	const char *requested_config = NULL;

	/* Set umask before anything for security */
	umask(0027);
	

	/* Get program name. */
	progname = ((p = strrchr(argv[0], '/')) ? ++p : argv[0]);

	/* First of all we need logging init. */
	zlog_default = openzlog(progname, ZLOG_RIP,
							LOG_CONS | LOG_NDELAY | LOG_PID, LOG_DAEMON);
	/* Command line option parse. */
	while (1)
	{
		int opt;

		opt = getopt_long(argc, argv, "df:i:z:hA:P:u:g:rvC", longopts, 0);

		if (opt == EOF)
			break;

		switch (opt)
		{
		case 0:
			break;
		case 'd':
			daemon_mode = 1;
			break;
		case 'f':
			requested_config = optarg;
			break;
		case 'i':
			pid_file = optarg;
			break;
		case 'z':
			vty_path = optarg;
			break;
		case 'A':
			vty_addr = optarg;
			break;
		case 'r':
			retain_mode = 1;
			break;
		case 'C':
			dryrun = 1;
			break;
		case 'u':
			main_vty_privs.user = optarg;
			break;
		case 'P':
			/* Deal with atoi() returning 0 on failure, and ripd not
                listening on rip port... */
			if (strcmp(optarg, "0") == 0)
			{
				vty_port = 0;
				break;
			}
			vty_port = atoi(optarg);
			if (vty_port <= 0 || vty_port > 0xffff)
			{
				vty_port = VTY_PORT;
			}
			break;
		case 'g':
			main_vty_privs.group = optarg;
			break;
		case 'v':
			print_version(progname);
			exit(0);
			break;
		case 'h':
			usage(progname, 0);
			break;
		default:
			usage(progname, 1);
			break;
		}
	}

	if (ensure_default_config_file() != 0)
		exit(1);

	if (requested_config && file_exists_and_regular(requested_config))
		config_file = (char *)requested_config;
	else
		config_file = generated_config_file;

	if (requested_config && !file_exists_and_regular(requested_config))
		zlog_warn("config file '%s' is missing or invalid, using default '%s'", requested_config, config_file);

	/* Prepare master thread. */
	master = thread_master_create();
	if (!master)
	{
		zlog_err("Failed to create thread master");
		exit(1);
	}

	/* Library initialization. */
	zprivs_init(&main_vty_privs);
	signal_init(master, array_size(ripd_signals), ripd_signals);
	cmd_init(1);
	vty_init(master);
	memory_init();
	tty_init();
	
	/* Get configuration file. */
	cli_reload_config();

	/* Start execution only if not in dry-run mode */
	if (dryrun)
		return (0);

	/* Change to the daemon program. */
	if (daemon_mode && daemon(0, 0) < 0)
	{
		zlog_err("CLId daemon failed: %s", strerror(errno));
		exit(1);
	}

	/* Pid file create. */
	pid_output(pid_file);
	signal(SIGSEGV, signal_segv_func);
	signal(SIGABRT, signal_segv_func);

	/* Create VTY's socket */
	start_vty_server();

	/* Print banner. */
	zlog_notice("CLId starting: vty@%d", vty_port);

	return 0;
}

int cli_run()
{
	if (!master)
	{
		zlog_err("thread master is not initialized");
		return -1;
	}
	/* Run thread main loop. */
	thread_main(master);
	return (0);
}
