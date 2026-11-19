#!/bin/sh
# Trace the wl driver joining a network: register accesses, PCI config writes
# and the head of every transmitted frame. Run as root in the Ubuntu guest.
IF=wlp0s6
T=/sys/kernel/tracing
cd /root
m() { echo "MARK $1" > $T/trace_marker; echo "MARK $1" > $T/instances/cfg/trace_marker; }

wpa_cli -i $IF terminate >/dev/null 2>&1
ip link set $IF down 2>/dev/null
rmmod wl 2>/dev/null
mkdir -p $T/instances/cfg
if ! grep -q write_dword $T/kprobe_events; then
	for w in dword word byte; do
		echo "p:write_$w pci_write_config_$w dev=%di:x64 where=%si:x32 val=%dx:x64" >> $T/kprobe_events
	done
fi
echo 1 > $T/instances/cfg/events/kprobes/enable
echo 262144 > $T/buffer_size_kb
echo mmiotrace > $T/current_tracer
setsid sh -c "cat $T/trace_pipe > /root/join-mmio.txt" </dev/null >/dev/null 2>&1 &
setsid sh -c "cat $T/instances/cfg/trace_pipe > /root/join-cfg.txt" </dev/null >/dev/null 2>&1 &
sleep 1
dmesg -C
# 128-byte header plus 48 bytes of frame: enough for the 802.11 and LLC
# headers, and short enough to leave out handshake key material.
insmod /root/ringdump/txsnoop.ko dumplen=176

m "modprobe start"; modprobe wl; m "modprobe returned"; sleep 2
m "ifup start"; ip link set $IF up; m "ifup returned"; sleep 2
m "wpa start"
wpa_supplicant -B -i $IF -c /root/wpa.conf -D nl80211,wext -f /root/wpa.log
i=0
while [ $i -lt 30 ]; do
	wpa_cli -i $IF status 2>/dev/null | grep -q wpa_state=COMPLETED && break
	sleep 1; i=$((i + 1))
done
m "wpa done"
wpa_cli -i $IF status | grep -E "^(bssid|freq|wpa_state|key_mgmt|pairwise_cipher|group_cipher)="
sleep 2
m "dhcp start"
dhcpcd -4 -w -t 20 -G -C resolv.conf -C hostname $IF 2>&1 | tail -3
m "dhcp done"
ip -4 -br addr show $IF
GW=$(dhcpcd -U $IF 2>/dev/null | sed -n "s/^routers='\{0,1\}\([0-9.]*\).*/\1/p" | head -1)
[ -n "$GW" ] || GW=$(ip -4 -br addr show $IF | awk '{print $3}' | sed 's/\.[0-9]*\/.*/.1/')
m "ping start"
ping -I $IF -c 10 -i 0.3 -W 2 "$GW" | tail -3
m "ping done"
iw dev $IF link | grep -E "freq|signal|tx bitrate"
sleep 1
m "teardown start"
dhcpcd -k $IF >/dev/null 2>&1
wpa_cli -i $IF terminate >/dev/null 2>&1
sleep 1
ip link set $IF down
rmmod wl
m "teardown done"
sleep 1
rmmod txsnoop
dmesg | grep txsnoop > /root/join-txsnoop.txt
echo nop > $T/current_tracer 2>/dev/null
pkill -f "trace_pipe" 2>/dev/null
sync
wc -l /root/join-mmio.txt /root/join-cfg.txt /root/join-txsnoop.txt
grep -c "map len" /root/join-txsnoop.txt
