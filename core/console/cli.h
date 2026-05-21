
#ifndef _CLI_H_
#define _CLI_H_

/**
 * cli_pre_init
 * 初始化控制台的默认配置文件路径、PID文件路径及监听端口
 * @default_cfg: 默认的配置文件路径
 * @default_pid: 默认的PID文件路径
 * @default_vty_port: 默认端口号
 */
extern void cli_pre_init(const char *default_cfg, const char *default_pid, int default_vty_port);

/**
 * cli_run
 * 初始化完成后，开始运行控制台线程
 */
extern int cli_run(void);

/**
 * cli_init
 * 控制台初始化。
 * 启动参数中可通过 -f/--config_file 或第三个位置参数传入配置文件；
 * 如果没有传入、参数无效或文件不存在，会在当前工作目录生成默认配置文件。
 */
extern int cli_init(int argc, char **argv);

#endif
