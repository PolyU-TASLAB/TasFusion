# /**
# * This file is part of tasfusion.
# *
# * Copyright (C) 2025 Trustworthy AI and Autonomous Systems Laboratory, The Hong Kong Polytechnic University (PolyU)
# * Author: ZHAO Jiaqi (jiaqi.zhao@connect.polyu.hk) Wang Xiangru (xiangru.wang@polyu.edu.hk) Gao Yixin (yixin.gao@connect.polyu.hk)
# *
# * tasfusion is free software: you can redistribute it and/or modify
# * it under the terms of the GNU General Public License as published by
# * the Free Software Foundation, either version 3 of the License, or
# * (at your option) any later version.
# *
# * tasfusion is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied warranty of
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# * GNU General Public License for more details.
# *
# * You should have received a copy of the GNU General Public License
# * along with tasfusion. If not, see <http://www.gnu.org/licenses/>.
# */
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import sys
from collections import defaultdict

# --- User Settings ---
ROSBAG_FILEPATH = '~/code/tasfusion_ws/src/data/phone_sensors_20250707_160418.bag'

# Define the ROS topics you are interested in
TOPICS_TO_ANALYZE = ['/cam0/image_raw', '/imu/data', '/phone_raw_measurements']

# --- Main Script ---

def analyze_rosbag_topics(bag_path, topics):
    """
    Performs a comprehensive analysis on specified topics in a rosbag file,
    including interval, instantaneous frequency, and binned frequency.

    Args:
        bag_path (str): The path to the rosbag file.
        topics (list): A list of ROS topic names to analyze.
    """
    print(f"Opening rosbag file: {bag_path}")
    
    try:
        bag = rosbag.Bag(bag_path, 'r')
    except Exception as e:
        print(f"Error: Could not open or read file '{bag_path}'.", file=sys.stderr)
        print(f"Details: {e}", file=sys.stderr)
        sys.exit(1)

    # This 'with' block ensures the bag is properly closed
    with bag:
        # 1. Extract all timestamps
        timestamps = defaultdict(list)
        print("Reading messages and extracting timestamps...")
        for topic, msg, t in bag.read_messages(topics=topics):
            timestamps[topic].append(t.to_sec())
        
        print("Timestamp extraction complete.")
        
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()

        # --- Part A: Interval and Instantaneous Frequency Analysis ---
        print("\n--- Interval and Instantaneous Frequency Analysis ---")
        interval_analysis_data = defaultdict(dict)
        for topic, ts_list in timestamps.items():
            if len(ts_list) < 2:
                print(f"Warning: Topic '{topic}' has fewer than 2 messages, cannot calculate intervals.")
                continue
            
            ts_array = np.array(ts_list)
            intervals = np.diff(ts_array)
            intervals[intervals == 0] = np.finfo(float).eps # Avoid division by zero

            interval_analysis_data[topic]['intervals'] = intervals
            interval_analysis_data[topic]['timestamps'] = ts_array[1:]
            interval_analysis_data[topic]['instantaneous_freq'] = 1.0 / intervals

            print(f"Topic '{topic}':")
            print(f"  - Message Count: {len(ts_list)}")
            print(f"  - Average Interval: {np.mean(intervals):.4f} s (~{1/np.mean(intervals):.2f} Hz)")
            print(f"  - Interval Std Dev (Jitter): {np.std(intervals):.6f} s")

        # --- Part B: Binned Frequency Analysis ---
        print("\n--- Binned Frequency Analysis (1s Bins) ---")
        time_bins = np.arange(start_time, end_time, 1.0)
        binned_frequency_data = {}
        for topic, ts_list in timestamps.items():
            if not ts_list:
                counts = np.zeros(len(time_bins) - 1)
            else:
                counts, _ = np.histogram(ts_list, bins=time_bins)
            binned_frequency_data[topic] = counts
            print(f"Topic '{topic}': Processed for binned frequency analysis.")

    # --- Part C: Plotting ---
    if not timestamps:
        print("\nNo messages found for any topic. Cannot generate plots.")
        return
        
    print("\nGenerating plots... (Each plot will appear in a separate window)")

    # Plot 1: Histogram of message interval distribution
    if interval_analysis_data:
        num_topics_interval = len(interval_analysis_data)
        fig1, axes1 = plt.subplots(num_topics_interval, 1, figsize=(10, 4 * num_topics_interval), squeeze=False)
        fig1.canvas.manager.set_window_title('Plot 1: Interval Distribution')
        fig1.suptitle('Message Interval Distribution (Histogram)', fontsize=16)
        for i, (topic, data) in enumerate(interval_analysis_data.items()):
            ax = axes1[i, 0]
            ax.hist(data['intervals'], bins=50, alpha=0.75)
            mean_val = np.mean(data['intervals'])
            ax.axvline(mean_val, color='r', linestyle='--', linewidth=2, label=f'Mean: {mean_val:.4f}s')
            ax.set_title(f"Topic: '{topic}'")
            ax.set_xlabel("Message Interval (s)")
            ax.set_ylabel("Count")
            ax.legend()
            ax.grid(True, linestyle=':', linewidth=0.6)
        plt.tight_layout(rect=[0, 0, 1, 0.96])

    # Plot 2: Message intervals over time
    if interval_analysis_data:
        num_topics_interval = len(interval_analysis_data)
        fig2, axes2 = plt.subplots(num_topics_interval, 1, figsize=(15, 4 * num_topics_interval), sharex=True, squeeze=False)
        fig2.canvas.manager.set_window_title('Plot 2: Interval Over Time')
        fig2.suptitle('Message Interval Over Time', fontsize=16)
        for i, (topic, data) in enumerate(interval_analysis_data.items()):
            ax = axes2[i, 0]
            ax.plot(data['timestamps'], data['intervals'], marker='.', linestyle='None', alpha=0.6)
            mean_val = np.mean(data['intervals'])
            ax.axhline(mean_val, color='r', linestyle='--', linewidth=1.5, label=f'Mean: {mean_val:.4f}s')
            ax.set_title(f"Topic: '{topic}'")
            ax.set_ylabel("Message Interval (s)")
            ax.legend()
            ax.grid(True, linestyle=':', linewidth=0.6)
        axes2[-1, 0].set_xlabel("Time (s)")
        plt.tight_layout(rect=[0, 0, 1, 0.96])

    # Plot 3: Instantaneous Message Frequency over Time
    if interval_analysis_data:
        num_topics_interval = len(interval_analysis_data)
        fig3, axes3 = plt.subplots(num_topics_interval, 1, figsize=(15, 4 * num_topics_interval), sharex=True, squeeze=False)
        fig3.canvas.manager.set_window_title('Plot 3: Instantaneous Frequency')
        fig3.suptitle('Instantaneous Message Frequency Over Time (1/Interval)', fontsize=16)
        for i, (topic, data) in enumerate(interval_analysis_data.items()):
            ax = axes3[i, 0]
            ax.plot(data['timestamps'], data['instantaneous_freq'], marker='.', linestyle='-', alpha=0.7)
            mean_freq = 1.0 / np.mean(data['intervals'])
            ax.axhline(mean_freq, color='r', linestyle='--', linewidth=1.5, label=f'Avg. Freq: {mean_freq:.2f} Hz')
            ax.set_title(f"Topic: '{topic}'")
            ax.set_ylabel("Instantaneous Frequency (Hz)")
            ax.legend()
            ax.grid(True, linestyle=':', linewidth=0.6)
        axes3[-1, 0].set_xlabel("Time (s)")
        plt.tight_layout(rect=[0, 0, 1, 0.96])

    # Plot 4: Binned Message Frequency over Time (1s Bins)
    fig4, ax4 = plt.subplots(figsize=(15, 7))
    fig4.canvas.manager.set_window_title('Plot 4: Binned Frequency')
    fig4.suptitle('Binned Message Frequency Over Time (1s Bins)', fontsize=16)
    x_axis_time = time_bins[:-1]
    for topic, freqs in binned_frequency_data.items():
        ax4.step(x_axis_time, freqs, where='post', label=topic)
    
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Frequency (Hz)")
    ax4.legend()
    ax4.grid(True, which='both', linestyle='--', linewidth=0.5)
    ax4.set_xlim(start_time, end_time)
    ax4.set_ylim(bottom=0)
    plt.tight_layout(rect=[0, 0, 1, 0.96])

    # Show all generated plots
    plt.show()


if __name__ == '__main__':
    analyze_rosbag_topics(ROSBAG_FILEPATH, TOPICS_TO_ANALYZE)