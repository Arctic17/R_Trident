# R_Trident
Development of a Delta Robot using Rotary kinematics


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
| **ENC** | Enclosure (PA12) | **EL** | Electronics Component |

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
