import json
import matplotlib.pyplot as plt

# Load JSON data
with open("data.json", "r") as f:
    data = json.load(f)

rpm_data = data["rpm_data"]
time_data = data["time_data"]

# Optional sanity check
if len(rpm_data) != len(time_data):
    raise ValueError(
        f"Length mismatch: rpm_data={len(rpm_data)}, time_data={len(time_data)}"
    )

# Plot
plt.figure(figsize=(10, 5))
plt.plot(time_data, rpm_data)
plt.xlabel("Time [ms]")
plt.ylabel("RPM")
plt.title("Fan RPM Response")
plt.grid(True)

plt.tight_layout()
plt.show()