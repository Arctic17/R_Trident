# 02. Mechanical Design & Topology Optimization

## Direct-Drive Mass Optimization Strategy
The robot is compriesed of a direct drive motion system. this is deu to sevral factors. The first of which is the cost of a gear box with axeptibal backlash is far out of buget. this eliminates any lower cost planetary or other gearbox system. Secondly a belt driven approche was considered but due to the added cost and complexity it was scrached. this also leaves room for future development. Due to the choice of gearbox-less direct drive, the motor rotor directly experiences the unbuffered load inertia of the entire arm assembly ($J_{\text{reflected}} = J_{\text{load}}$). The main mechanical goal is to prioritize **mass minimization**. A particulare attentions was given to the elbow joint to avoid high-frequency servo oscillations and inertia mismatch errors.

---

## Fusion 360 Generative Design Criteria

The upper bicep arm structure is optimized using automated generative algorithms inside Fusion 360 based on the following targets:

| Boundary Constraint Parameter | Value Limit Set |
| :--- | :--- |
| **Target Envelope Mass Limit** | 60g – 75g (Plastic body only) |
| **Structural Factor of Safety** | 2.0 to 2.5 (Anisotropic scaling adjustment) |
| **First Fundamental Modal Frequency** | $\ge 130\text{ Hz}$ (To bypass encoder noise) |
| **Maximum Displacement / Deflection** | 0.5 mm under peak motor acceleration torque |

> **Note:** Explicitly disable *Elastic Buckling* and *Thermal Stress Constraints* within the study parameters to prevent unnecessary solver loops.

---

## Load Case Models

1. **Load Case A (Peak Dynamic Torsion):** 45 N applied tangentially to the elbow joint axis (derived from motor peak acceleration torque of 4.45 Nm).
2. **Load Case B (Forearm Tension/Compression):** 45 N acting axially through the linkage mounting interface points.
3. **Load case C (Complex Torsion):** The actual torque value of 4.45 Nm was applyed round the motor shaft using the key way and constrained on the linkage mounting interface.

---

## Materials Selection & Joinery Architecture

* **Filament Selection:** **Nylon-GF30** (30% Glass Fiber Reinforced Nylon). Nylon-GF provides excellent cyclic fatigue resistance and structural damping. Standard PLA-CF must be rejected due to severe long-term material creep under mechanical fastener torque, which introduces joint backlash.

## Study outcomes
<div align="center">
<figure>
    <img src="./img/Study_2_-_Structural_Component_-_Outcome_6_Transparent2.png "
         alt=""
         width="100%">
    <figcaption> MatLab Simscape model of the phsical linkages</figcaption>
</figure>
</div>


