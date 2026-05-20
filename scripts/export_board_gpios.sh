#!/bin/sh
# export_board_gpios.sh
# 导出并初始化板级 GPIO（sysfs 方法）

# 端口字母到索引（A=0,B=1,C=2,D=3）
port_index() {
  case "$1" in
    A|a) echo 0 ;;
    B|b) echo 1 ;;
    C|c) echo 2 ;;
    D|d) echo 3 ;;
    *) echo 0 ;;
  esac
}

# 计算全局 GPIO 编号：bank * 32 + (port_index*8 + pin)
# bank: 0..n, port: A/B/C/D, pin: 0..7
pin_to_gpio() {
  bank=$1; port=$2; pin=$3
  pidx=$(port_index "$port")
  echo $(( bank * 32 + (pidx * 8) + pin ))
}

export_gpio() {
  gpio=$1
  if [ ! -d /sys/class/gpio/gpio$gpio ]; then
    echo "$gpio" > /sys/class/gpio/export || true
  fi
}

unexport_gpio() {
  gpio=$1
  if [ -d /sys/class/gpio/gpio$gpio ]; then
    echo "$gpio" > /sys/class/gpio/unexport || true
  fi
}

set_dir() {
  gpio=$1; dir=$2
  echo "$dir" > /sys/class/gpio/gpio$gpio/direction
}

set_val() {
  gpio=$1; val=$2
  echo "$val" > /sys/class/gpio/gpio$gpio/value
}

get_val() {
  gpio=$1
  cat /sys/class/gpio/gpio$gpio/value
}

# --- 定义板上用到的 GPIO（按外设.md 与 DTS 映射推导） ---
# EC800M
EC_PWRKEY=$(pin_to_gpio 0 B 0)   # GPIO0_B0 -> e.g. 8
EC_RESET=$(pin_to_gpio 0 B 1)    # GPIO0_B1 -> e.g. 9

# NM890
NM_EN_POWER=$(pin_to_gpio 0 A 4)  # GPIO0_A4 -> e.g. 4
NM_RESET=$(pin_to_gpio 0 A 5)     # GPIO0_A5 -> e.g. 5
NM_DBB_TX_FLAG=$(pin_to_gpio 0 A 6) # GPIO0_A6 (输入) -> e.g. 6
NM_A2B_SLEEP=$(pin_to_gpio 0 B 2) # GPIO0_B2 -> e.g. 10
NM_A2B_WAKE=$(pin_to_gpio 0 B 3)  # GPIO0_B3 -> e.g. 11

# E22
E22_RESET=$(pin_to_gpio 1 D 0)    # GPIO1_D0 -> e.g. 56
E22_AUX=$(pin_to_gpio 1 D 1)      # GPIO1_D1 -> e.g. 57

# USB_SWITCH
USB_SW0=$(pin_to_gpio 3 B 6)      # GPIO3_B6 -> e.g. 110
USB_SW1=$(pin_to_gpio 3 B 5)      # GPIO3_B5 -> e.g. 109

# 列表（用于导出/卸载循环）
OUTS="$EC_PWRKEY $EC_RESET $NM_EN_POWER $NM_RESET $NM_A2B_SLEEP $NM_A2B_WAKE $E22_RESET $USB_SW0 $USB_SW1"
INS="$NM_DBB_TX_FLAG $E22_AUX"

echo "Exporting GPIOs..."
for g in $OUTS $INS; do
  export_gpio "$g"
done

echo "Setting directions and initial values..."
# 输出口：默认置 0 (off)
for g in $OUTS; do
  set_dir "$g" out
  set_val "$g" 0
done

# 输入口
for g in $INS; do
  set_dir "$g" in
done

echo "Initialization done."
echo "Current values (0 or 1):"
for g in $OUTS $INS; do
  printf "gpio%-4s: %s\n" "$g" "$(get_val $g)"
done
