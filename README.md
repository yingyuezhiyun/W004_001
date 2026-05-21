# ok3506_demo

这是一个面向 OK3506 的最小可执行文件工程，默认使用 `/home/forlinx/work/toolchain` 下的 Buildroot 交叉工具链进行编译，并保留了网络远程调试入口。

## 目录

- `src/main.c`：示例程序，便于先验证编译和运行链路。
- `cmake/ok3506-toolchain.cmake`：交叉编译工具链文件。
- `scripts/build.sh`：一键交叉编译。
- `scripts/run_remote_debug.sh`：宿主机通过网口连接目标板的 gdb 调试入口。

## 构建

在宿主机上执行：

```sh
chmod +x scripts/build.sh scripts/run_remote_debug.sh
./scripts/build.sh
```

产物会生成在 `build/ok3506_demo`。

## 部署到目标板

将 `build/ok3506_demo` 拷贝到 OK3506 目标板上，例如 `/tmp/ok3506_demo`。

如果目标板上有 `gdbserver`，可以这样启动：

```sh
chmod +x /tmp/ok3506_demo
# 目标板上执行
# gdbserver 0.0.0.0:1234 /tmp/ok3506_demo demo 5
```

如果要在 VS Code 调试时固定传参，可以在 `.vscode/settings.json` 里设置 `ok3506.targetArgs`，例如 `demo 5`。调试任务会把它原样传给目标板上的 `gdbserver`。

## 宿主机远程调试

宿主机上连接目标板的网口：

```sh
./scripts/run_remote_debug.sh <target-ip> 1234
```

如果你的宿主机装的是 `gdb-multiarch`，脚本会优先使用它；否则回退到 `gdb`。

## 调试参数

如果你想修改调试时传给程序的参数，直接改 [`.vscode/settings.json`](.vscode/settings.json) 里的这几个键：

- `ok3506.deployLabel`
- `ok3506.deployDelay`
- `ok3506.deployConfigSource`

然后继续使用现有的 `OK3506 Debug with Deploy` 即可。默认会复制工作区根目录下的 [config.conf](config.conf) 到目标板，并作为 `cli_init` 的配置文件。
OK3506 Debug with Deploy
```

这个配置会先执行 `scripts/deploy_and_run_gdbserver.sh`：

- 使用 `scp` 把 `build/ok3506_demo` 拷贝到目标板 `/root/ok3506_demo`
- 通过 `ssh` 在目标板前台启动 `gdbserver 0.0.0.0:1234 /root/ok3506_demo`
- 程序的 `printf` 输出会出现在 VS Code 的任务终端里，而不是单独的 SSH 窗口

如果你想改目标板地址、SSH 用户或端口，可以通过环境变量覆盖：

```sh
TARGET_HOST=192.168.0.232 TARGET_USER=root TARGET_PORT=1234 ./scripts/deploy_and_run_gdbserver.sh
```

## 调试参数

如果你想在 VS Code 里顺手传入程序参数和配置文件，直接选择：

```text
OK3506 Debug with Deploy Args
```

它会在启动前提示你填写：

- `label`，传给 `main()` 的第一个参数
- `delay`，传给 `main()` 的第二个参数
- `config`，要复制到目标板并传给 `cli_init` 的配置文件源路径

默认会使用工作区根目录下的 [config.conf](config.conf)。如果这个文件不存在，程序仍会在目标板当前目录生成一份默认配置。

## 说明

- 该工程默认以 `Debug` 构建，保留符号信息，便于断点和单步调试。
- 如果目标板上没有 `gdbserver`，也可以先把程序正常跑起来，再根据系统镜像补装调试组件。
