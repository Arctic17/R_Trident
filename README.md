
<h1 align="left">
  <br>
  <img src=".\docs\img\PersonalLogoNema.svg" width="150">
  <br>
  R Trident: 
  
  High-Speed Rotary Delta Robot
</h1>

Every successful project starts with a goal, and this one is no different. My objective is to design and build a Rotary Delta kinematic motion system from the ground up covering feasibility research, groundwork, development, manufacturing, and programming, through to final assembly.
But why?
The purpose is twofold: to deepen my understanding of systems integration across multiple disciplines and to
refine my ability to manage every stage of a complex engineering project from start to finish.

So it can be summeriesed like this

R_Trident is a high-performance, direct-drive Rotary Delta Robot built using three **AC Servo Motors (A6M60-400H2B1-M17)** paired with real-time **EtherCAT** communications. The control system runs on a custom **Raspberry Pi** executing a hard **real-time** LinuxCNC kernel.

## System Architecture Overview

* **Host Controller:** Raspberry Pi CM5 (Real-Time Preempt Kernel)
* **Fieldbus:** EtherCAT (IgH EtherCAT Master Stack) @ 1ms loop time
* **Actuation:** 3x Direct-Drive 400W AC Servos (No gearboxes, peak torque 4.45 Nm)
* **Feedback Loops:** 17-bit absolute magnetic encoder feedback over CoE (CANopen over EtherCAT)

---

## Documentation Index

To read or modify specific engineering subsystems, navigate to the sub-files listed below:

1. [Mathematical Kinematics & Control Math](docs/01_mathematical_kinematics.md)
   * Spatial loops, 3D vector intersections, Simulink models, PID tuning data, and Z-axis trajectory bowing fixes.
2. [Mechanical Design & Topology Optimization](docs/02_mechanical_design.md)
   * Inventor 2025 System assembly and machined parts plans. Fusion 360 generative setup, weight metrics, load cases, filament selection.
3. [Electronics & Breakout Architecture](docs/03_electronics_breakout.md)
   * Dual Ethernet paths, isolated logic rails, buck converters, and headless hardware diagnostics interfaces.
4. [Electrical Cabinet Engineering](docs/04_electrical_cabinet.md)
   * 4-row layout, star distribution wiring, emergency interlocks without STO, and grounding trees.
5. [LinuxCNC & EtherCAT Drive Stack Setup](docs/05_linuxcnc_ethercat.md)
   * Host configuration, drive parameters, `ethercat-conf.xml` mapping, and `.hal` files.

## File structure

All custom files and entries in the BOM follow the **RT_XXX_YY_000_Name** naming system.

### Schema 
**`RT`** (Project) _ **`XXX`** (System) _ **`YY`** (Category) _ **`000`** (Index) _ **`Name`**

| System Code (XXX) | Description | Category Code (YY) | Description |
| :--- | :--- | :--- | :--- |
| **FRM** | Frame & Structure | **PR** | 3D Printed Part |
| **DRV** | Drive Train (Shoulder) | **MC** | Machined / Laser Cut |
| **ARM** | Biceps & Forearms | **CO** | Commercial (Off-the-shelf) |
| **EFF** | Effector & Hotend | **SK** | Skeleton / Reference |
| **BED** | Bed & Z-Axis | **AS** | Sub-Assembly |
| **ENC** | Enclosure | **EL** | Electronics Component |

---

## Project Navigation

* **`01_Management`**: Costs, Sourcing (Mädler/StepperOnline), and CDC.
* **`02_CAD`**: 
    * `00_Skeleton`: Master geometry and trig parameters.
    * `03_Parts_Custom`: All `RT_XXX_PR` and `RT_XXX_MC` files.
    * `05_Top_Assembly`: The "Digital Twin" (The complete machine).
* **`04_Electronics`**: Wiring schematics and pinout maps for the BIQU Manta.
* **`05_Firmware`**: Klipper `printer.cfg` and kinematic math.
* **`06_Outputs`**: Ready-to-print STLs and production drawings.

---
