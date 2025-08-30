import subprocess
import os
import time

try:
    while True:
        print("\n=== Starting drone client ===")
        result = subprocess.run(
            ["uv", "run", "python", "py-src/drone/drone.py"],
            env={"PYTHONPATH": "./py-src", **os.environ}
        )

        print("--- Drone client ended. Restarting in 2s ---")
        time.sleep(2)

except KeyboardInterrupt:
    print("\nStopped by user ( ͡° ͜ʖ ͡° )")
