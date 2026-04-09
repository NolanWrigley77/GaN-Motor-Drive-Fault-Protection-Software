import time

# ---------------- Helper read function ----------------
# Read until expected line
# ------------------------------------------------------
def _read_until(ser, expected: str, timeout_s=2.0, lock=None):
    t_end = time.time() + timeout_s

    while time.time() < t_end:
        if lock:
            with lock:
                line = ser.readline().decode(errors="ignore").strip()
        else:
            line = ser.readline().decode(errors="ignore").strip()

        if not line:
            continue

        if line == expected:
            return True, line

    return False, ""
# ----------------------------------------------------

# ---------------- Communication test ----------------
# Host <-> MCU two sided communication check.
# Sends PING, expects OK PING.
# ----------------------------------------------------
def run_comm_test(ser, timeout_s=2.0):
    # Clearl buffer Data
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Send ping 
    ser.write(b"PING\n")
    ser.flush()

    t_end = time.time() + timeout_s

    #Conditions for failure/success 
    while time.time() < t_end:
        line = ser.readline().decode(errors="ignore").strip()

        if not line:
            continue

        if line == "OK PING":
            return True, "Communication OK (PING acknowledged)"

        # Received something, but not what we expected
        return False, f"Unexpected response: '{line}'"

    # Timeout hit
    return False, f"Timeout waiting for OK PING ({timeout_s:.1f}s)"

# ----------------------------------------------------