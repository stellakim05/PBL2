import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from scipy.signal import savgol_filter
import matplotlib.patches as mpatches

def load_and_process_data(filepath):
    """Load and process the system performance CSV data."""
    df = pd.read_csv(filepath)
    
    numeric_columns = ['Timestamp', 'AverageLatency', 'MessageRate']
    for col in numeric_columns:
        df[col] = pd.to_numeric(df[col], errors='coerce')
    
    df['Timestamp'] = df['Timestamp'] - df['Timestamp'].min()
    return df

def smooth_data(data, window=11, poly=3):
    """Apply Savitzky-Golay filter to smooth the data."""
    try:
        return savgol_filter(data, window, poly)
    except:
        return data

def detect_robot_movements(df, threshold=0.1):
    """Detect robot movement events based on message rate."""
    events = []
    message_rate = df['MessageRate'].values
    timestamps = df['Timestamp'].values
    
    in_movement = False
    movement_start = 0
    min_duration = 0.1
    
    for i in range(1, len(message_rate)):
        if not in_movement and message_rate[i] > threshold:
            movement_start = timestamps[i]
            in_movement = True
        elif in_movement and message_rate[i] <= threshold:
            if timestamps[i] - movement_start >= min_duration:
                event_type = "Initialize\n Robots\nMoving" if len(events) % 2 == 0 else "Robots\n Stopped"
                events.append((movement_start, timestamps[i], event_type))
                in_movement = False
    
    return events

def create_latency_plot(df, output_dir='experiment_plots'):
    """Create compact plot showing latency and robot movements."""
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    
    plt.style.use('seaborn-v0_8-darkgrid')
    fig, ax = plt.subplots(figsize=(10, 5))
    
    # Detect events
    events = detect_robot_movements(df)
    
    # # Smooth Plot latency
    latency_smooth = smooth_data(df['AverageLatency'])
    l1 = ax.plot(df['Timestamp'], latency_smooth, 
                 color='#e74c3c', label='Response Time', 
                 linewidth=3, alpha=0.8)
    
    # Plot latency
    # l1 = ax.plot(df['Timestamp'], df['AverageLatency'], 
    #              color='#e74c3c', label='Response Time', 
    #              linewidth=3, alpha=0.8)
    
    # Calculate maximum height for text positioning
    max_height = max(latency_smooth)
    text_height = max_height * 1.2
    
    # Add event markers
    first_initialize = True
    first_stop = True
    
    for start, end, event in events:
        color = '#3498db' if 'Initialize' in event else '#e74c3c'
        ax.axvspan(start, end, color=color, alpha=0.2)
        
        if ('Initialize' in event and first_initialize) or ('Stopped' in event and first_stop):
            mid_point = (start + end) / 2
            
            if 'Initialize' in event:
                label_height = text_height
                first_initialize = False
            else:
                label_height = text_height * 1.1
                first_stop = False
            
            # Add connecting line
            ax.vlines(mid_point, max_height * 1.1, label_height - max_height * 0.1,
                     colors=color, linestyles=':', alpha=0.5, linewidth=1)
            
            # Add text with background
            bbox_props = dict(
                boxstyle='round,pad=0.5',
                facecolor='white',
                edgecolor=color,
                alpha=0.9,
                linewidth=1
            )
            
            ax.text(mid_point - 0.5, label_height, event,
                   rotation=0, ha='right', va='bottom',
                   fontsize=10, color=color,
                   bbox=bbox_props)
    
    # Create custom legend elements
    blue_patch = mpatches.Patch(color='#3498db', alpha=0.2, label='Robot Movement Command')
    red_patch = mpatches.Patch(color='#e74c3c', alpha=0.2, label='Robot Stop Command')
    
    # Combine legend elements
    lines = l1
    labels = [l.get_label() for l in lines]
    all_handles = lines + [blue_patch, red_patch]
    all_labels = labels + ['Robot Movement Command', 'Robot Stop Command']
    
    # Add legend
    ax.legend(all_handles, all_labels, 
             bbox_to_anchor=(1.15, 1), loc='upper left',
             fontsize=10, facecolor='white', edgecolor='none')
    
    # Customize plot
    ax.set_xlabel('Time (seconds)', fontsize=10)
    ax.set_ylabel('Response Time (seconds)', color='#e74c3c', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # Set y-axis limits to fit text boxes
    ax.set_ylim(0, text_height * 1.3)
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/latency_analysis.png',
                dpi=300, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    plt.close()

def main():
    """Main function to run the analysis."""
    filepath = r"ExperimentData\system_performance__20241109_225117.csv"
    
    df = load_and_process_data(filepath)
    create_latency_plot(df)

if __name__ == "__main__":
    main()