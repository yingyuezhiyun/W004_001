#!/bin/sh

set -eu

APN="${1:-${APN:-internet}}"
NET_IF="${NET_IF:-usb0}"
AT_PORT="${AT_PORT:-}"
PING_HOST="${PING_HOST:-1.1.1.1}"
AT_TIMEOUT="${AT_TIMEOUT:-2}"
AT_POST_WAIT="${AT_POST_WAIT:-1}"

log() {
  printf '%s\n' "$*"
}

warn() {
  printf '%s\n' "$*" >&2
}

fail() {
  printf '%s\n' "$*" >&2
  exit 1
}

pick_at_port() {
  if [ -n "$AT_PORT" ] && [ -e "$AT_PORT" ]; then
    return 0
  fi

  #for port in /dev/ttyS4 /dev/ttyUSB2; do
  for port in /dev/ttyUSB2; do
    if [ -e "$port" ]; then
      AT_PORT="$port"
      return 0
    fi
  done

  return 1
}

capture_at_response() {
  cmd="$1"
  tmp_file="/tmp/ec800m_at.$$"
  rm -f "$tmp_file"

  cat "$AT_PORT" > "$tmp_file" &
  reader_pid=$!
  sleep 0.2
  printf '%s\r' "$cmd" > "$AT_PORT"
  sleep "$AT_POST_WAIT"
  kill "$reader_pid" >/dev/null 2>&1 || true
  wait "$reader_pid" >/dev/null 2>&1 || true
  response=""
  if [ -s "$tmp_file" ]; then
    response=$(cat "$tmp_file")
  fi
  rm -f "$tmp_file"
  printf '%s' "$response"
}

configure_at_port() {
  stty -F "$AT_PORT" 115200 raw -echo -ixon -ixoff -crtscts
}

at_expect() {
  name="$1"
  cmd="$2"
  expect="$3"

  log "AT check: $name"
  response=$(capture_at_response "$cmd")
  log "$response"

  case "$response" in
    *OK*) ;;
    *) fail "Error: $name did not return OK." ;;
  esac

  if [ -n "$expect" ]; then
    case "$response" in
      *"$expect"*) ;;
      *) fail "Error: $name did not return expected state: $expect" ;;
    esac
  fi
}

at_warn_if_slow_signal() {
  response="$1"
  case "$response" in
    *"+CSQ: 99,"*)
      warn "Warning: signal quality is unknown or unavailable."
      ;;
  esac
}

at_check_signal() {
  log "AT check: signal"
  response=$(capture_at_response "AT+CSQ")
  log "$response"
  case "$response" in
    *OK*) ;;
    *) fail "Error: AT+CSQ did not return OK." ;;
  esac
  at_warn_if_slow_signal "$response"
}

wait_for_ip() {
  count=0
  while [ $count -lt 30 ]; do
    if command -v ip >/dev/null 2>&1; then
      if ip addr show dev "$NET_IF" | grep -q 'inet '; then
        return 0
      fi
    else
      if ifconfig "$NET_IF" 2>/dev/null | grep -q 'inet '; then
        return 0
      fi
    fi
    sleep 1
    count=$((count + 1))
  done

  return 1
}

check_registration() {
  log "AT check: network registration"
  response=$(capture_at_response "AT+CEREG?")
  log "$response"
  case "$response" in
    *OK*) ;;
    *) fail "Error: AT+CEREG? did not return OK." ;;
  esac

  case "$response" in
    *"+CEREG: 0,1"*|*"+CEREG: 0,5"*|*"+CEREG: 1,1"*|*"+CEREG: 1,5"*)
      ;;
    *)
      fail "Error: module is not registered on the network." ;;
  esac
}

check_packet_attach() {
  log "AT check: packet attach"
  response=$(capture_at_response "AT+CGATT?")
  log "$response"
  case "$response" in
    *OK*) ;;
    *) fail "Error: AT+CGATT? did not return OK." ;;
  esac

  case "$response" in
    *"+CGATT: 1"*) ;;
    *) fail "Error: packet service is not attached." ;;
  esac
}

set_apn() {
  if [ -n "$APN" ]; then
    at_expect "set APN" "AT+CGDCONT=1,\"IP\",\"$APN\"" "OK"
  fi
}

test_internet() {
  if ! command -v ping >/dev/null 2>&1; then
    warn "Warning: ping is not available, skipping connectivity test."
    return 0
  fi

  log "Connectivity check: ping $PING_HOST"
  if ping -c 3 -W 2 "$PING_HOST" >/dev/null 2>&1; then
    log "Connectivity check passed."
    return 0
  fi

  fail "Error: connectivity test failed."
}

show_status() {
  if command -v ip >/dev/null 2>&1; then
    ip addr show dev "$NET_IF" || true
    ip route show default || true
  else
    ifconfig "$NET_IF" || true
    route -n || true
  fi
}

bring_iface_up() {
  if command -v ip >/dev/null 2>&1; then
    ip link set "$NET_IF" up >/dev/null 2>&1 || true
  else
    ifconfig "$NET_IF" up >/dev/null 2>&1 || true
  fi
}

start_dhcp() {
  if command -v udhcpc >/dev/null 2>&1; then
    udhcpc -i "$NET_IF" -n -q -t 10 -T 3
    return 0
  fi

  if command -v dhclient >/dev/null 2>&1; then
    dhclient "$NET_IF"
    return 0
  fi

  fail "Error: no DHCP client found on the system."
}

wait_for_iface() {
  count=0
  while [ $count -lt 30 ]; do
    if [ -d "/sys/class/net/$NET_IF" ]; then
      return 0
    fi
    sleep 1
    count=$((count + 1))
  done

  return 1
}



echo 0 > /sys/class/gpio/gpio9/value

echo 1 > /sys/class/gpio/gpio8/value

echo 1 > /sys/class/gpio/gpio109/value

sleep 2

if pick_at_port; then
  configure_at_port
  at_expect "AT" "AT" "OK"
  at_expect "echo off" "ATE0" "OK"
  at_expect "SIM status" "AT+CPIN?" "+CPIN: READY"
  at_check_signal
  at_expect "full functionality" "AT+CFUN=1" "OK"
  at_expect "confirm functionality" "AT+CFUN?" "+CFUN: 1"
  check_registration
  set_apn
  at_expect "attach packet service" "AT+CGATT=1" "OK"
  check_packet_attach
else
  echo "Warning: no AT port found at /dev/ttyS4 or /dev/ttyUSB2." >&2
fi

bring_iface_up

if ! wait_for_iface; then
  echo "Error: network interface $NET_IF was not found." >&2
  exit 1
fi

start_dhcp
if ! wait_for_ip; then
  fail "Error: no IPv4 address was obtained on $NET_IF."
fi

show_status
test_internet
