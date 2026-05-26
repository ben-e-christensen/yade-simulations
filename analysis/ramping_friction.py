import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
import sys
import os
import re
import gc # Garbage collector to free up RAM

# --- 1. COMMAND LINE ARGUMENT SETUP ---
if len(sys.argv) < 2:
    print("Error: No input files provided.")
    print("Usage: python analyze_multi.py <path/to/files/*.csv>")
    sys.exit(1)

input_files = sys.argv[1:]

# Dictionaries to store the processed, lightweight data for plotting
all_timelines = {}
all_summaries = {}

# --- 2. SEQUENTIAL DATA PROCESSING ---
print(f"Found {len(input_files)} files. Processing sequentially to save RAM...")

for filepath in input_files:
    filename = os.path.basename(filepath)
    
    # Try to extract the RPM value from the filename (e.g., 'repose_data_16RPM.csv' -> '16')
    match = re.search(r'(\d+)RPM', filename)
    if match:
        label = f"{match.group(1)} RPM"
        sort_key = int(match.group(1)) # For sorting the legend later
    else:
        label = filename
        sort_key = 0
        
    print(f"\n--- Loading {filename} ({label}) ---")
    
    # Load the massive CSV
    df = pd.read_csv(filepath)
    
    times, angles, frictions = [], [], []

    # Calculate angle for every frame
    for t, snapshot in df.groupby('sim_time'):
        fric = snapshot['fric_deg'].iloc[0]
        x, y = snapshot['x_pos'].values, snapshot['y_pos'].values

        # Surface Detection
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

        # Crop middle 60%
        if len(surface_x) > 0:
            x_center = np.mean(surface_x)
            x_range = surface_x.max() - surface_x.min()
            valid_mask = (surface_x > x_center - 0.3 * x_range) & (surface_x < x_center + 0.3 * x_range)
            clean_x, clean_y = surface_x[valid_mask], surface_y[valid_mask]

            if len(clean_x) > 1:
                slope, _ = np.polyfit(clean_x, clean_y, 1)
                angle = np.degrees(np.arctan(abs(slope)))
                times.append(t)
                angles.append(angle)
                frictions.append(fric)

    # Put the results into a lightweight DataFrame
    timeline = pd.DataFrame({'time': times, 'angle': angles, 'fric_deg': frictions})
    
    # Smooth data within each phase
    timeline['smooth_angle'] = timeline.groupby('fric_deg')['angle'].transform(
        lambda x: x.rolling(window=25, center=True, min_periods=1).mean()
    )
    
    summary_frics, summary_mean_peaks = [], []
    
    # Peak finding per friction phase
    for fric_val, group in timeline.groupby('fric_deg'):
        group = group.reset_index()
        peak_indices, _ = find_peaks(group['smooth_angle'], prominence=1.5, distance=25)
        peak_angles = group['angle'].iloc[peak_indices].values
        
        if len(peak_angles) > 0:
            summary_frics.append(fric_val)
            summary_mean_peaks.append(np.mean(peak_angles))

    # Store the lightweight results using the label as the key
    all_timelines[label] = {'data': timeline, 'sort': sort_key}
    all_summaries[label] = {'fric': summary_frics, 'peaks': summary_mean_peaks, 'sort': sort_key}
    
    # CRITICAL: Delete the 4.3GB raw dataframe and force garbage collection before next loop
    del df
    gc.collect()
    print(f"Finished {label}. Cleared memory.")

# Sort the keys so our legends go in order (1 RPM, 6 RPM, etc.)
sorted_labels = sorted(all_timelines.keys(), key=lambda x: all_timelines[x]['sort'])

# --- 3. PLOTTING OVERLAYS ---
print("\nGenerating Consolidated Plots...")

# Using a colormap so the 6 lines look distinct and professional
colors = plt.cm.viridis(np.linspace(0, 1, len(sorted_labels)))

# Plot A: Master Timeline Overlays
plt.figure(figsize=(15, 6))
for idx, label in enumerate(sorted_labels):
    tl = all_timelines[label]['data']
    # Plotting ONLY the smoothed lines. Plotting the raw noise for 6 files would be illegible.
    plt.plot(tl['time'], tl['smooth_angle'], label=label, color=colors[idx], linewidth=1.5)

# Try to draw the vertical phase lines using the first timeline's timestamps
first_tl = all_timelines[sorted_labels[0]]['data']
unique_times = first_tl.groupby('fric_deg')['time'].min().values
for ut in unique_times:
    if ut > 0:
        plt.axvline(x=ut, color='gray', linestyle='--', alpha=0.5)

plt.title('Avalanche Timelines Across Stepped Friction (Smoothed)')
plt.xlabel('Simulation Time (s)')
plt.ylabel('Angle of Repose (Degrees)')
plt.legend(loc='upper left', bbox_to_anchor=(1, 1))
plt.grid(True, linestyle='--', alpha=0.3)
plt.savefig('multi_avalanche_timeline.png', dpi=150, bbox_inches='tight')

# Plot B: Summary Overlay (Friction vs Repose for each RPM)
plt.figure(figsize=(10, 6))
for idx, label in enumerate(sorted_labels):
    frics = all_summaries[label]['fric']
    peaks = all_summaries[label]['peaks']
    plt.plot(frics, peaks, marker='o', label=label, color=colors[idx], linewidth=2)

plt.title('Effect of Drum Speed & Particle Friction on Macroscopic Angle of Repose')
plt.xlabel('Microscopic Friction Angle (Degrees)')
plt.ylabel('Average Maximum Angle of Stability (Degrees)')
plt.legend(loc='upper left', bbox_to_anchor=(1, 1))
plt.grid(True, linestyle='--', alpha=0.5)
plt.savefig('multi_friction_vs_repose.png', dpi=150, bbox_inches='tight')

print("Success! Saved 'multi_avalanche_timeline.png' and 'multi_friction_vs_repose.png'")