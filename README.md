# Runtime Order

1. Run `main.c` from within Eclipse ModusToolbox using the XMC Link Debug probe.
2. After firmware is running, start `GUI.py`.
3. Connect the PC to the board through a UART-to-USB probe to view terminal output.

# What Each File Does

- `main.c`:
  - Embedded firmware for the Infineon REF_MTR_48V_30A_GaN reference board which must be built and run in Eclipse ModusToolbox.
- `GUI.py`:
  - Desktop interface that runs after firmware is active. Connects over UART to receive onboard sensor telemetry and terminal output.
- `test_script_MotorDrive.py`:
  - Test utility script for communication checks. Sends ping messages back and forth and validates fault telemetry reporting.
