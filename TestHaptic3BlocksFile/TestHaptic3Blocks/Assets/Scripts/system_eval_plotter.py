import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from scipy.signal import savgol_filter
import matplotlib.patches as mpatches

def load_and_process_data(filepath):
    """Load and process the system performance CSV data."""
    df = pd.read_csv(filepath)
    
    numeric_columns = ['Timestamp', 'Connected', 'TimeSinceLastReceived', 
                      'TimeSinceLastSent', 'Port', 'MessageRate', 
                      'AverageLatency', 'MessagesSent', 'MessagesReceived', 
                      'PacketLoss', 'Stability']
    
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

def detect_activity_bursts(df, threshold=0.1):
    """Detect robot movement events based on message rate bursts."""
    events = []
    message_rate = df['MessageRate'].values
    timestamps = df['Timestamp'].values
    
    # Find activity bursts
    in_burst = False
    burst_start = 0
    min_burst_duration = 0.1  # Minimum duration of a burst in seconds
    
    for i in range(1, len(message_rate)):
        if not in_burst and message_rate[i] > threshold:
            # Start of burst
            burst_start = timestamps[i]
            in_burst = True
        elif in_burst and message_rate[i] <= threshold:
            # End of burst
            if timestamps[i] - burst_start >= min_burst_duration:
                # Alternate between "Initialize Moving" and "Robots Stopped"
                event_type = "Initialize Robots\n Moving" if len(events) % 2 == 0 else "Robots Stopped"
                events.append((burst_start, timestamps[i], event_type))
                in_burst = False
    
    return events

def create_user_friendly_dashboard(df, output_dir='plots'):
    """Create dashboard with simplified label placement."""
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    
    plt.style.use('seaborn-v0_8-darkgrid')
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 12))
    # fig.suptitle('Robot Movement Monitor', fontsize=16, y=0.95)
    
    # Detect events
    events = detect_activity_bursts(df)
    
    # Plot 1: Communication Performance
    message_rate_smooth = smooth_data(df['MessageRate'])
    latency_smooth = smooth_data(df['AverageLatency'])
    
    l1 = ax1.plot(df['Timestamp'], message_rate_smooth, 
                  color='#2ecc71', label='Speed of Messages', 
                  linewidth=3, alpha=0.8)
    
    ax1_twin = ax1.twinx()
    l2 = ax1_twin.plot(df['Timestamp'], latency_smooth, 
                       color='#e74c3c', label='Response Time', 
                       linewidth=3, alpha=0.8)
    
    # Calculate the maximum height for scaling
    max_height = max(max(message_rate_smooth), max(latency_smooth))
    text_height = max_height * 1.4  # Height for text
    
    # Add event markers and regions
    first_initialize = True
    first_stop = True
    
    for start, end, event in events:
        # Add colored region for the event duration
        color = '#3498db' if 'Initialize' in event else '#e74c3c'
        alpha = 0.2
        ax1.axvspan(start, end, color=color, alpha=alpha)
        
        # Add labels only for first initialize and first stop
        if ('Initialize' in event and first_initialize) or ('Stopped' in event and first_stop):
            mid_point = (start + end) / 2
            
            # Adjust heights to prevent overlap
            if 'Initialize' in event:
                label_height = text_height
                first_initialize = False
            else:
                label_height = text_height * 1.2  # Place "Robots Stopped" slightly higher
                first_stop = False
            
            # Add connecting line
            ax1.vlines(mid_point, max_height * 1.1, label_height - max_height * 0.1,
                      colors=color, linestyles=':', alpha=0.5, linewidth=1)
            
            # Add text with background
            bbox_props = dict(
                boxstyle='round,pad=0.5',
                facecolor='white',
                edgecolor=color,
                alpha=0.9,
                linewidth=1
            )
            
            ax1.text(mid_point - 0.5, label_height + 1, event,
                    rotation=0, ha='right', va='bottom',
                    fontsize=10, color=color,
                    bbox=bbox_props)
    
    # Make the grid less prominent
    ax1.grid(True, alpha=0.3)
    
    # Set labels
    ax1.set_xlabel('Time (seconds)', fontsize=10)
    ax1.set_ylabel('Messages per Second', color='#2ecc71', fontsize=10)
    ax1_twin.set_ylabel('Response Time (seconds)', color='#e74c3c', fontsize=10)
    
    # Create custom legend elements for the colored regions
    blue_patch = mpatches.Patch(color='#3498db', alpha=0.2, label='Robot Movement Command')
    red_patch = mpatches.Patch(color='#e74c3c', alpha=0.2, label='Robot Stop Command')
    
    # Combine all legend elements
    lines = l1 + l2
    labels = [l.get_label() for l in lines]
    all_handles = lines + [blue_patch, red_patch]
    all_labels = labels + ['Robot Movement Command', 'Robot Stop Command']
    
    # Create legend with all elements
    ax1.legend(all_handles, all_labels, 
              bbox_to_anchor=(1.15, 1), loc='upper left',
              fontsize=10, facecolor='white', edgecolor='none', 
              framealpha=0.9)
    
    # Adjust y-axis limits to accommodate text
    ax1.set_ylim(0, text_height * 1.4)
    
    ax1.set_title('Communication Performance', fontsize=12, pad=20)
    
    # Plot 2: System Health Status
    stability_smooth = smooth_data(df['Stability'])
    connection_status = df['Connected']
    
    l3 = ax2.plot(df['Timestamp'], stability_smooth,
                  color='#f1c40f', label='System Stability',
                  linewidth=3, alpha=0.8)
    
    ax2_twin = ax2.twinx()
    l4 = ax2_twin.plot(df['Timestamp'], connection_status,
                       color='#3498db', label='Connection Status',
                       linewidth=3, alpha=0.8)
    
    # Add event regions to second plot
    for start, end, event in events:
        color = '#3498db' if 'Initialize' in event else '#e74c3c'
        ax2.axvspan(start, end, color=color, alpha=0.1)
    
    ax2.grid(True, alpha=0.3)
    
    # Set labels
    ax2.set_xlabel('Time (seconds)', fontsize=10)
    ax2.set_ylabel('Stability Score', color='#f1c40f', fontsize=10)
    ax2_twin.set_ylabel('Connection Status', color='#3498db', fontsize=10)
    
    ax2.set_ylim(0, 1.1)
    ax2_twin.set_ylim(0, 1.1)
    
    # Add explanatory text
    ax2.text(1.15, 0,
             'Connection Status: 1 = Connected, 0 = Disconnected\n' +
             'Stability Score: Higher is better (0-1 scale)',
             transform=ax2.transAxes, fontsize=9, alpha=0.7)
    
    # Move legend outside the plot
    lines = l3 + l4
    labels = [l.get_label() for l in lines]
    ax2.legend(lines, labels, bbox_to_anchor=(1.15, 1), loc='upper left',
               fontsize=10, facecolor='white', edgecolor='none', framealpha=0.9)
    
    ax2.set_title('System Health Status', fontsize=12, pad=20)
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/user_friendly_dashboard.png',
                dpi=300, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    plt.close()

def main():
    """Main function to run the analysis."""
    filepath = r"ExperimentData\system_performance__20241127_140247.csv"
    
    df = load_and_process_data(filepath)
    create_user_friendly_dashboard(df)

if __name__ == "__main__":
    main()