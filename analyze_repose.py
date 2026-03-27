import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

print("Loading data...")
df = pd.read_csv('repose_data.csv')

# --- NEW: CHOP OFF THE STARTUP TRANSIENT ---
# Only keep the rows where the simulation time is 30 seconds or greater
df = df[df['sim_time'] >= 30.0]

times = []
angles = []

print("Calculating angle for every frame...")
# ... (the rest of the script continues exactly the same from here)

times = []
angles = []

print("Calculating angle for every frame... (This might take a minute or two on the Pi)")
# Group the data by every unique time step
for t, snapshot in df.groupby('sim_time'):
    x = snapshot['x_pos'].values
    y = snapshot['y_pos'].values

    # 1. Surface Detection (same as before)
    num_bins = 30
    bins = np.linspace(x.min(), x.max(), num_bins)
    digitized = np.digitize(x, bins)

    surface_x, surface_y = [], []
    for i in range(1, num_bins):
        in_bin = (digitized == i)
        if np.any(in_bin):
            max_y_idx = np.argmax(y[in_bin])
            surface_x.append(x[in_bin][max_y_idx])
            surface_y.append(y[in_bin][max_y_idx])

    surface_x = np.array(surface_x)
    surface_y = np.array(surface_y)

    # 2. Crop middle 60%
    if len(surface_x) > 0:
        x_center = np.mean(surface_x)
        x_range = surface_x.max() - surface_x.min()
        valid_mask = (surface_x > x_center - 0.3 * x_range) & (surface_x < x_center + 0.3 * x_range)
        clean_x = surface_x[valid_mask]
        clean_y = surface_y[valid_mask]

        # 3. Calculate Angle
        if len(clean_x) > 1:
            slope, _ = np.polyfit(clean_x, clean_y, 1)
            angle = np.degrees(np.arctan(abs(slope)))
            times.append(t)
            angles.append(angle)

# Put the results into a new DataFrame for easy math
timeline = pd.DataFrame({'time': times, 'angle': angles})


# 1. APPLY THE FILTER: Smooth the data using a 25-frame (0.25s) rolling average
# This erases the microscopic bead bounces that are tricking the logic
timeline['smooth_angle'] = timeline['angle'].rolling(window=5, center=True, min_periods=1).mean()

# 2. FIND PEAKS ON THE SMOOTH CURVE
# We ask SciPy to find peaks that stand out by at least 1.5 degrees from the surrounding valleys,
# and enforce a minimum distance of 0.5 seconds (50 frames) between peaks.
peak_indices, properties = find_peaks(timeline['smooth_angle'], prominence=1.5, distance=25)

# Extract the exact times and raw angles at those indices
peak_times = timeline['time'].iloc[peak_indices].values
peak_angles = timeline['angle'].iloc[peak_indices].values

peaks_df = pd.DataFrame({'time': peak_times, 'angle': peak_angles})

print(f"\nAnalysis Complete!")
print(f"Total frames analyzed: {len(timeline)}")
print(f"Total distinct avalanches detected: {len(peaks_df)}")
if len(peaks_df) > 0:
    print(f"Average Peak Angle of Repose: {peaks_df['angle'].mean():.2f}°")

# --- PLOTTING ---
plt.figure(figsize=(12, 6))

# Plot the raw, noisy data in a faint, light blue
plt.plot(timeline['time'], timeline['angle'], color='lightblue', label='Raw Noisy Angle', alpha=0.5)

# Plot the new, smoothed curve in a bold, dark blue
plt.plot(timeline['time'], timeline['smooth_angle'], color='darkblue', linewidth=2, label='Smoothed Angle (0.25s avg)')

# Put the red dots strictly on the peaks of the smoothed curve
plt.scatter(peaks_df['time'], peaks_df['angle'], color='red', zorder=5, s=70, label='True Peak Angle of Repose')

plt.title('Maximum Angle of Stability (Filtered for Avalanches)')
plt.xlabel('Simulation Time (s)')
plt.ylabel('Angle (Degrees)')
plt.legend()
plt.grid(True, linestyle='--', alpha=0.5)

plt.savefig('slip_timeline.png', dpi=150, bbox_inches='tight')
print("Saved timeline graph to 'slip_timeline.png'")