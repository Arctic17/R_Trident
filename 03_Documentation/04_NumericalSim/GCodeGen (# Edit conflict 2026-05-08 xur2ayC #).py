import math

# ==========================================
# 3-AXIS OSCILLATION PARAMETERS
# ==========================================
DISTANCE = 100  # Total travel distance in mm for each axis
CYCLES = 5  # How many full waves to complete
TIME_PER_CYCLE = 5.0  # Seconds per wave. (Lower = faster, more aggressive)
STEPS_PER_CYCLE = 200  # Resolution
FILENAME = "3_Axis_Oscillation.gcode"


def generate_3d_oscillation():
    print(f"Generating {FILENAME}...")

    with open(FILENAME, 'w') as f:
        # 1. Startup Sequence
        f.write("; === 3-AXIS 120-DEGREE PHASE SHIFT TEST ===\n")
        f.write("G21 ; Set units to millimeters\n")
        f.write("G90 ; Absolute positioning\n")
        f.write("G28 X Y Z ; Home ALL axes to start\n")
        f.write("M400 ; Wait for homing to finish\n\n")

        # Time-step setup
        dt = TIME_PER_CYCLE / STEPS_PER_CYCLE
        prev_x, prev_y, prev_z = 0.0, 0.0, 0.0

        # 120 degrees converted to Radians (2*PI/3)
        PHASE_SHIFT = (2 * math.pi) / 3

        # 2. Generate the continuous 3D motion
        for i in range(CYCLES * STEPS_PER_CYCLE):
            t = (i + 1) * dt

            # Base phase for the X axis
            phase = (t / TIME_PER_CYCLE) * 2 * math.pi

            # Calculate positions with 120-degree offsets
            # X gets base phase (0 deg offset)
            # Y gets base phase + 120 deg
            # Z gets base phase + 240 deg
            x = (DISTANCE / 2) * (1 - math.cos(phase))
            y = (DISTANCE / 2) * (1 - math.cos(phase + PHASE_SHIFT))
            z = (DISTANCE / 2) * (1 - math.cos(phase + (2 * PHASE_SHIFT)))

            # Calculate distance moved on each axis since the last step
            dx = abs(x - prev_x)
            dy = abs(y - prev_y)
            dz = abs(z - prev_z)

            # TRUE 3D VECTOR DISTANCE (Pythagorean theorem in 3D)
            dist_3d = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

            # Calculate global feedrate in mm/minute
            feedrate = (dist_3d / dt) * 60

            # Safety minimum feedrate
            if feedrate < 1.0:
                feedrate = 1.0

            # Write the synchronized 3-axis movement line
            f.write(f"G1 X{x:.3f} Y{y:.3f} Z{z:.3f} F{feedrate:.0f}\n")

            # Update previous positions
            prev_x, prev_y, prev_z = x, y, z 

        # 3. Finish Sequence
        f.write("\nM400 ; Wait for moves to finish\n")
        f.write("G1 X0 Y0 Z0 F1500 ; Return all axes to zero safely\n")

    print(f"Success! Upload '{FILENAME}' to Mainsail.")


if __name__ == "__main__":
    generate_3d_oscillation()