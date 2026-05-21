
#ifndef __TTY_H_
#define __TTY_H_

#define VAR_PN(val) (val >= 0 ? "p" : "n")
#define VAR_ABS(val) (val >= 0 ? val : 0 - val)



/* 用户自己定义的控制台指令初始化 */
extern void tty_init(void);

#endif
