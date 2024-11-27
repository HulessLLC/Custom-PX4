uint64 timestamp                 # Time since system start (microseconds)

# Failsafe Injection Command
bool inject_failsafe             # Flag indicating whether to inject a failsafe
bool inject_prearm_failure
bool inject_parachute
uint8 failure_type               # Type of failure to inject (0: None, 1: Roll, 2: Pitch, 3: Altitude, 4: External, etc.)
uint8 severity                   # Severity level of the failure (0-255)
