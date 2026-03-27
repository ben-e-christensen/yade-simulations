import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

print("Loading CSV data (this might take a few seconds)...")
# 1. Load the data
df = pd.read_csv('repose_data.csv')

# 2. Grab the very last snapshot of the simulation
# We want to measure the angle after the drum has been spinning for a bit!
last_time = df['sim_time'].max()
snapshot = df[df['sim_time'] == last_time]

x = snapshot['x_pos'].values
y = snapshot['y_pos'].values

# 3. Find the "surface" beads
# We divide the drum into 30 vertical columns (bins) and find the highest bead in each column.
num_bins = 30
bins = np.linspace(x.min(), x.max(), num_bins)
digitized = np.digitize(x, bins)

surface_x = []
surface_y = []

for i in range(1, num_bins):
    in_bin = (digitized == i)
    if np.any(in_bin):
        # Find the bead with the highest Y value in this column
        max_y_idx = np.argmax(y[in_bin])
        surface_x.append(x[in_bin][max_y_idx])
        surface_y.append(y[in_bin][max_y_idx])

surface_x = np.array(surface_x)
surface_y = np.array(surface_y)

# 4. Crop the edges
# The beads resting against the far left and right walls of the drum curve upwards.
# We only want to measure the flat "flowing" zone in the middle.
# This grabs just the middle 60% of the surface points.
x_center = np.mean(surface_x)
x_range = surface_x.max() - surface_x.min()
valid_mask = (surface_x > x_center - 0.3 * x_range) & (surface_x < x_center + 0.3 * x_range)

clean_x = surface_x[valid_mask]
clean_y = surface_y[valid_mask]

# 5. Calculate the Angle of Repose (Linear Regression)
# Fit a line (degree 1 polynomial) to the clean surface points
slope, intercept = np.polyfit(clean_x, clean_y, 1)

# The angle in radians is the arctangent of the slope!
angle_rad = np.arctan(abs(slope))
angle_deg = np.degrees(angle_rad)

print(f"Success! Calculated Dynamic Angle of Repose: {angle_deg:.2f}°")

# 6. Plot the results
plt.figure(figsize=(8, 8))

# Draw all the beads in light gray
plt.scatter(x, y, color='lightgray', label='All Beads', alpha=0.5)

# Draw the top surface beads in red
plt.scatter(clean_x, clean_y, color='red', label='Detected Surface')

# Draw the line of best fit in blue
fit_line_x = np.linspace(clean_x.min(), clean_x.max(), 100)
fit_line_y = slope * fit_line_x + intercept
plt.plot(fit_line_x, fit_line_y, color='blue', linewidth=2, label=f'Fit Line ({angle_deg:.1f}°)')

# Make the graph look pretty
plt.title(f'Dynamic Angle of Repose at t={last_time}s\nCalculated Angle: {angle_deg:.2f}°', fontsize=14)
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.axis('equal') # Forces the X and Y axes to scale equally so the drum isn't squished
plt.legend()
plt.grid(True, linestyle='--', alpha=0.6)

# Save the plot as an image file so you can open it easily on the Pi
plt.savefig('repose_plot.png', dpi=150, bbox_inches='tight')
print("Saved visualization to 'repose_plot.png'")