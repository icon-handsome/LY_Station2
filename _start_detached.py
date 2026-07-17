# -*- coding: utf-8 -*-
import sys
import time
import paramiko

sys.stdout.reconfigure(encoding="utf-8")
c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(
    "192.168.8.15",
    username="Administrator",
    password="123456",
    timeout=20,
    allow_agent=False,
    look_for_keys=False,
)

def run(cmd):
    i, o, e = c.exec_command(cmd)
    code = o.channel.recv_exit_status()
    return (
        code,
        o.read().decode("utf-8", errors="replace"),
        e.read().decode("utf-8", errors="replace"),
    )

run(
    "powershell -NoProfile -Command "
    "\"Get-Process scan-tracking -ErrorAction SilentlyContinue | Stop-Process -Force\""
)
time.sleep(0.5)

# Detach from SSH session via WMI Create
code, out, err = run(
    "powershell -NoProfile -Command "
    "\"$r = Invoke-CimMethod -ClassName Win32_Process -MethodName Create -Arguments @{"
    "CommandLine='D:\\work\\IPC_Station2\\scan-tracking.exe'; "
    "CurrentDirectory='D:\\work\\IPC_Station2'}; "
    "'CreateReturn=' + $r.ReturnValue + ' pid=' + $r.ProcessId\""
)
print(out or err)

# Give it time, then check in a NEW short connection simulation (same session but after wait)
time.sleep(6)
code, out, err = run(
    "powershell -NoProfile -Command "
    "\"$p=Get-Process scan-tracking -ErrorAction SilentlyContinue; "
    "if($p){'RUNNING pid='+$p.Id+' Start='+$p.StartTime}else{'NOT_RUNNING'}\""
)
print(out or err)
c.close()

# Reconnect fresh to confirm survives disconnect
time.sleep(1)
c2 = paramiko.SSHClient()
c2.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c2.connect(
    "192.168.8.15",
    username="Administrator",
    password="123456",
    timeout=20,
    allow_agent=False,
    look_for_keys=False,
)
i, o, e = c2.exec_command(
    "powershell -NoProfile -Command "
    "\"$p=Get-Process scan-tracking -ErrorAction SilentlyContinue; "
    "if($p){'AFTER_RECONNECT_OK pid='+$p.Id}else{'AFTER_RECONNECT_DEAD'}\""
)
print(o.read().decode("utf-8", errors="replace"))
c2.close()
