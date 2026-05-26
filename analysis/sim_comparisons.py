import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from matplotlib import rcParams
import sys
import os
import glob
import re
import gc

# --- ACADEMIC PLOT SETTINGS ---
rcParams['font.sans-serif'] = "Arial"
rcParams['font.family'] = "sans-serif"
FONT_TICK = 12
FONT_LABEL = 14
FONT_TITLE = 16

def main():
    if len(sys.argv) < 2:
        print("Error: No input directory provided.")
        print("Usage: python plot_binned_dashboard.py <path/to/sim_comparisons>")
        sys.exit(1)

    root_dir = sys.argv[1]
    search_pattern = os.path.join(root_dir, '*rpm*', 'experiment_log.csv')
    file_list = glob.glob(search_pattern)

    if not file_list:
        print(f"Error: No files found matching {search_pattern}")
        sys.exit(1)

    all_processed_data = {}

    print(f"Found {len(file_list)} files. Processing and binning data...")

    # Define standard columns
    cols = ["index", "timestamp", "seq", "ms", "motor_angle_deg", "motor_speed", 
            "CH0_volts", "CH2_volts", "CH3_volts", "ellipse_angle_deg", 
            "ellipse_area_px2", "frame_name", "ch2_dv/dt", "ch3_dv/dt", "ch2_flag", "ch3_flag"]

    for filepath in file_list:
        # Extract RPM for labeling
        match = re.search(r'(\d+)rpm', filepath, re.IGNORECASE)
        if not match:
            continue
            
        rpm = int(match.group(1))
        label = f"{rpm} RPM"
        
        print(f"Loading {label}...")
        
        # Load and clean CSV
        df = pd.read_csv(filepath, names=cols, header=0, on_bad_lines="skip", engine="python")
        df = df.dropna(subset=["ms", "ellipse_angle_deg", "CH2_volts"])
        
        start_ms = df["ms"].min()
        df["Time (min)"] = (df["ms"] - start_ms) / 60000.0
        
        # Format electrical and mechanical columns
        df["Voltage (V)"] = pd.to_numeric(df["CH2_volts"], errors='coerce') * -1.0 
        df["Angle (deg)"] = df["ellipse_angle_deg"]
        df = df.dropna(subset=["Voltage (V)", "Angle (deg)"])

        # --- 1. Fast Binning: Angle of Repose (30s Average) ---
        # Smooth the entire track once
        df['smooth_angle'] = df['Angle (deg)'].rolling(window=25, center=True, min_periods=1).mean()
        
        # Find all peaks in the run
        peak_indices, _ = find_peaks(df['smooth_angle'].dropna(), prominence=1.5, distance=25)
        
        # Isolate just the peak moments into a new dataframe
        peaks_df = df.iloc[peak_indices].copy()
        
        # Floor times to nearest 0.5 min (30s) and calculate the average angle per bin
        peaks_df['Bin_30s'] = (peaks_df['Time (min)'] // 0.5) * 0.5
        binned_angle = peaks_df.groupby('Bin_30s')['Angle (deg)'].mean().reset_index()


        # --- 2. Fast Binning: Charge (15s Std Dev) ---
        # Floor times to nearest 0.25 min (15s) and calculate the standard dev per bin
        df['Bin_15s'] = (df['Time (min)'] // 0.25) * 0.25
        binned_charge = df.groupby('Bin_15s')['Voltage (V)'].std().reset_index()


        # Store processed data to be plotted
        all_processed_data[label] = {
            'sort': rpm,
            'angle_time': binned_angle['Bin_30s'],
            'angle_avg': binned_angle['Angle (deg)'],
            'charge_time': binned_charge['Bin_15s'],
            'charge_std': binned_charge['Voltage (V)']
        }
        
        # Free memory before loading the next giant CSV
        del df
        del peaks_df
        gc.collect()

    # --- 3. PLOTTING ---
    print("\nGenerating Stacked Dashboard Plot...")

    # Sort labels so the legend matches up cleanly (1 RPM, 6 RPM, etc.)
    sorted_labels = sorted(all_processed_data.keys(), key=lambda x: all_processed_data[x]['sort'])
    
    # Generate a distinct color for each RPM
    colors = plt.cm.viridis(np.linspace(0, 1, len(sorted_labels)))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10), sharex=True)
    fig.subplots_adjust(hspace=0.08) # Pack them closely

    # Plot lines for every RPM
    for idx, label in enumerate(sorted_labels):
        data = all_processed_data[label]
        c = colors[idx]
        
        # Angle of Repose Track
        ax1.plot(data['angle_time'], data['angle_avg'], color=c, marker='o', 
                 linewidth=2, markersize=5, label=label, alpha=0.9)
                 
        # Charge Std Dev Track
        ax2.plot(data['charge_time'], data['charge_std'], color=c, marker='s', 
                 linewidth=2, markersize=5, label=label, alpha=0.9)

    # Format Top Plot (Angle)
    ax1.set_ylabel("Avg Angle of Repose (deg)", fontsize=FONT_LABEL)
    ax1.grid(True, linestyle='--', alpha=0.5)
    ax1.legend(loc='upper left', bbox_to_anchor=(1.01, 1), borderaxespad=0., title="Speed")

    # Format Bottom Plot (Charge)
    ax2.set_ylabel("Voltage Std Dev (V)", fontsize=FONT_LABEL)
    ax2.set_xlabel("Experiment Time (Minutes)", fontsize=FONT_LABEL)
    ax2.grid(True, linestyle='--', alpha=0.5)
    # Don't duplicate the legend since the colors match, but adjust right margin to make room
    fig.subplots_adjust(right=0.85)

    # Clean axes formatting
    for ax in [ax1, ax2]:
        ax.xaxis.set_tick_params(labelsize=FONT_TICK)
        ax.yaxis.set_tick_params(labelsize=FONT_TICK)

    plt.suptitle("Mechanical vs. Electrical Dynamics Over Time", fontsize=FONT_TITLE, fontweight='bold', y=0.93)
    
    output_filename = 'master_binned_dashboard.png'
    plt.savefig(output_filename, dpi=150, bbox_inches='tight')
    print(f"Success! Saved to {output_filename}")


if __name__ == "__main__":
    main()