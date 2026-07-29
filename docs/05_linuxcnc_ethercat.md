# 05. LinuxCNC & EtherCAT Real-Time Driver Stack

## 1. Host Network Configuration

Run these configuration steps to bind your real-time network adapter to the IgH EtherCAT driver stack.

1. Edit the core master system configuration file:

```bash
   sudo nano /etc/ethercat.conf
```

Set your dedicated port's MAC address and specify the generic network driver module:

```bash 
MASTER0_DEVICE="00:11:22:33:44:55"
DEVICE_MODULES="generic"
```
Prevent standard Linux network managers from interacting with the dedicated interface:

```bash
sudo nano /etc/NetworkManager/NetworkManager.conf
```
Append the following device exclusion keyfile entry:

```bash
[keyfile]
unmanaged-devices=mac:00:11:22:33:44:55
```

Restart the low-level communication service daemon:
```bash
sudo systemctl restart ethercat
```

Verify that the network hardware path can view all three connected axes down the wire:

```bash
ethercat slaves
```


## Servo Drive Firmware Configuration

Configure these baseline parameters using the physical hardware keypad on each individual A6-EC drive:

- C00.00 = 10 : Sets primary operation control mode to EtherCAT bus control.

- C13.01 = 1 (Axis 2 = 2, Axis 3 = 3) : Assigns sequential Node Station Aliases down the physical daisy chain line.

- C13.05 = 1 : Standardizes Distributed Clocks (DC) Mode 1 for sub-microsecond synchronization.

Important: To commit these settings to the non-volatile memory, navigate to parameter 31.01 within the F (Function) menu, set the value to 1, and hold down the Enter key for 3–5 seconds until the screen flashes StArt followed by FinisH. Cycle the main power afterwards.

## Configuration Setup Files

ethercat-conf.xml

```xml


<master idx="0" appTimePeriod="1000000" refClockSyncCycles="1">
  <slave alias="0" position="0" vendor="0x00000000" product="0x00000000" id="axis1">
    <syncManager idx="2" dir="out" watch="1">
      <pdo idx="0x1702"/>
    </syncManager>
    <syncManager idx="3" dir="in" watch="0">
      <pdo idx="0x1b02"/>
    </syncManager>
  </slave>
  </master>

```

Rotary_Delta.hal

```text
# Load the LinuxCNC Real-Time Motion Module
loadrt motmod servo_period=1000000 joints=3

# Load the Low-Level EtherCAT Driver Wrapper Component
loadusr -W lcec_conf ethercat-conf.xml
loadrt lcec

# Link Driver Real-Time Execution Threads to Servo Interrupt Clocks
addf lcec.read-all servo-thread
addf motion-command-handler servo-thread
addf motion-controller servo-thread
addf lcec.write-all servo-thread

# Establish Simple Hardware Loop Testing Interlinks (Axis 1 Example)
net ax1-pos-cmd  joint.0.motor-pos-cmd  => lcec.0.axis1.pos-cmd
net ax1-pos-fb   lcec.0.axis1.pos-fb    => joint.0.motor-pos-fb
```
