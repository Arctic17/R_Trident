# 04. Electrical Cabinet Engineering & Infrastructure

## 4-Row Physical Component Layout

The enclosure layout separates high-power electrical noise from low-voltage control signals using a vertical layout approach:

```text
+-------------------------------------------------------------+
| [Row 1] LOW-VOLTAGE CONTROL                                 |
|  - Raspberry Pi CM5 Controller, WAGO Bus Coupler Train      |
+-------------------------------------------------------------+
| [Row 2] DISTRIBUTION & SAFETY                               |
|  - Rotary Disconnect, Breakers, Moeller Safety Contactor     |
+-------------------------------------------------------------+
| [Row 3] HIGH-POWER SERVO DRIVES                             |
|  - 3x StepperOnline A6-EC Drives (50mm Top/Bottom Clearance)|
+-------------------------------------------------------------+
| [Row 4] FIELD WIRING TERMINAL RAIL                          |
|  - Left: Clean Low-Voltage Field Signals | Right: High-Volt |
+-------------------------------------------------------------+
```


## Power Distribution & Breaker Bill of Materials

To prevent high-current inductive inductive spikes (e.g., from mechanical brake coils) from causing logic voltage drops on the host controller, use a dedicated star-wiring configuration back to the main power supply output terminals.

### Circuit Protection Ratings

- Main AC Line Supply Input: 1x 13A Curve C Miniature Circuit Breaker (MCB)

- Servo Drives AC Input Lines: 3x 6A Curve C Miniature Circuit Breakers

- 24V Logic Power Supply Unit: 1x 6A Curve C Miniature Circuit Breaker

## Emergency Interlock Loop & Safety Design

Because the StepperOnline A6-EC drives lack built-in Safe Torque Off (STO) inputs, a hardware-level safety layer must be wired into the panel supply:

- The Circuit: Wire a dual-channel Normally Closed (NC) latching Emergency Stop button in series directly with the 24V DC coil of the 3-pole master AC contactor (Moeller DILMC7-10).

- The Function: Pressing the E-stop cuts the coil power, opening the main contacts and removing AC power from the three servo drives. This stops motor motion while maintaining the separate 24V DC logic rail for controller feedback.

## Grounding and Noise Suppression

- Star Grounding Tree: Connect the motor power cable's green/yellow ground wire and the braided shield layer directly to the PE terminal block on the drive chassis. Run an independent ground wire from each drive's ground stud to the main copper busbar in the cabinet. Do not daisy-chain ground wires between drives.

- Inductive Brake Suppression: Connect a high-voltage flyback diode (e.g., 1N4007) in reverse-bias directly across the terminals of each mechanical brake relay contact to clip high-voltage spikes during inductive switching.