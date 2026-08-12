import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

class ExperimentPlotter:
    def __init__(self, csv_file):
        self.data = self.read_data_file(csv_file)
        self.setup_plot_style()
        self.detect_events()

    def read_data_file(self, csv_file):
        """Read CSV file and exclude summary section"""
        try:
            with open(csv_file, 'r') as f:
                lines = []
                for line in f:
                    if line.strip() == '' or 'Session Summary' in line:
                        break
                    lines.append(line)
            
            from io import StringIO
            data = pd.read_csv(StringIO(''.join(lines)))
            
            # Convert numeric columns
            numeric_columns = ['TaskTime', 'BoxRotation', 'ForceMagnitude', 'IsInContact']
            for col in numeric_columns:
                if col in data.columns:
                    data[col] = pd.to_numeric(data[col], errors='coerce')
            
            return data.fillna(0)
        except Exception as e:
            print(f"Error reading file: {e}")
            return pd.DataFrame()

    def setup_plot_style(self):
        """Set up global plot style with larger, more visible elements"""
        plt.style.use('default')
        plt.rcParams['figure.figsize'] = [16, 12]
        plt.rcParams['font.size'] = 14          # Base font size
        plt.rcParams['axes.labelsize'] = 16     # Axis labels
        plt.rcParams['axes.titlesize'] = 18     # Subplot titles
        plt.rcParams['figure.titlesize'] = 20   # Figure title
        plt.rcParams['xtick.labelsize'] = 14    # X-axis tick labels
        plt.rcParams['ytick.labelsize'] = 14    # Y-axis tick labels
        plt.rcParams['legend.fontsize'] = 14    # Legend text
        plt.rcParams['axes.grid'] = True        # Show grid
        plt.rcParams['grid.alpha'] = 0.3        # Grid transparency
        plt.rcParams['lines.linewidth'] = 2.5   # Line thickness
        plt.rcParams['axes.linewidth'] = 2      # Axis line thickness
        plt.rcParams['grid.linewidth'] = 1.5    # Grid line thickness

    def detect_events(self):
        """Detect important events in the experiment"""
        self.events = {}
        
        # Detect first contact
        contact_mask = self.data['IsInContact'] == 1
        if contact_mask.any():
            first_contact = self.data[contact_mask].iloc[0]
            self.events['First Contact'] = first_contact['TaskTime']

        # Detect significant rotation (> 5 degrees)
        rotation_mask = abs(self.data['BoxRotation']) > 5
        if rotation_mask.any():
            sig_rotation = self.data[rotation_mask].iloc[0]
            self.events['Significant Rotation'] = sig_rotation['TaskTime']

        print("\nDetected Events:")
        for event, time in self.events.items():
            print(f"{event}: {time:.2f}s")

    def plot_experiment_timeline(self):
        """Plot main experiment timeline with improved layout and spacing"""
        # Increase overall figure size
        fig = plt.figure(figsize=(24, 20))
        
        # Increase spacing between subplots
        gs = plt.GridSpec(4, 1, height_ratios=[1.5, 1.5, 1.5, 0.8], hspace=0.8)
        
        # # Move coordinate system explanation to top-right instead of top-left
        # # and improve its formatting
        # coord_text = (
        #     "Coordinate System:\n"
        #     "• Position: X (left/right), Y (up/down), Z (forward/back)\n"
        #     "• Rotation: 0° is neutral, -ve is clockwise, +ve is counter-clockwise\n"
        #     "• Forces: Measured in Newtons (N), normalized to max force capacity"
        # )
        # fig.text(0.02, 1.02, coord_text, fontsize=14, 
        #         bbox=dict(facecolor='white', alpha=0.9, edgecolor='gray', 
        #                 pad=10, boxstyle='round'))

        # Box Rotation plot
        ax1 = fig.add_subplot(gs[0])
        rotation_data = self.data['BoxRotation']
        time_data = self.data['TaskTime']
        
        window = 5
        rotation_std = rotation_data.rolling(window=window).std().fillna(0.5)
        
        ax1.plot(time_data, rotation_data, 'b-', linewidth=3, label='Actual Rotation')
        ax1.fill_between(time_data, 
                        rotation_data - 2*rotation_std,
                        rotation_data + 2*rotation_std,
                        color='blue', alpha=0.2, label='95% Confidence')
        
        ax1.axhline(y=0, color='g', linestyle='--', linewidth=2, label='Target (0°)')
        ax1.axhspan(-2, 2, color='g', alpha=0.1, label='Acceptable Range')
        
        ax1.set_ylabel('Box Rotation (°)', fontsize=18, labelpad=15)
        ax1.tick_params(axis='both', which='major', labelsize=14)
        ax1.grid(True, alpha=0.3)
        # Move legend down slightly
        ax1.legend(loc='center left', fontsize=14, framealpha=0.9, 
                bbox_to_anchor=(1.05, 0.3))

        # Robot Speeds plot
        ax2 = fig.add_subplot(gs[1])
        window = 5
        robot1_speed_smooth = self.data['Robot1Speed'].rolling(window=window, center=True).mean()
        robot2_speed_smooth = self.data['Robot2Speed'].rolling(window=window, center=True).mean()
        
        speed_std1 = self.data['Robot1Speed'].rolling(window=window).std().fillna(0)
        speed_std2 = self.data['Robot2Speed'].rolling(window=window).std().fillna(0)
        
        ax2.plot(time_data, robot1_speed_smooth, 'b-', linewidth=3, label='Robot 1')
        ax2.plot(time_data, robot2_speed_smooth, 'g-', linewidth=3, label='Robot 2')
        ax2.fill_between(time_data, 
                        robot1_speed_smooth - speed_std1,
                        robot1_speed_smooth + speed_std1,
                        color='blue', alpha=0.2)
        ax2.fill_between(time_data, 
                        robot2_speed_smooth - speed_std2,
                        robot2_speed_smooth + speed_std2,
                        color='green', alpha=0.2)
        
        ax2.set_ylabel('Speed (m/s)', fontsize=18, labelpad=15)
        ax2.tick_params(axis='both', which='major', labelsize=14)
        ax2.grid(True, alpha=0.3)
        # Move legend down slightly
        ax2.legend(loc='center left', fontsize=14, framealpha=0.9, 
                bbox_to_anchor=(1.05, 0.3))

        # Force plot
        ax3 = fig.add_subplot(gs[2])
        max_force = self.data['ForceMagnitude'].max()
        if max_force > 0:
            normalized_force = self.data['ForceMagnitude'] / max_force
        else:
            normalized_force = self.data['ForceMagnitude']
        
        ax3.plot(time_data, self.data['ForceMagnitude'], 'purple', 
                linewidth=3, label='Force (N)')
        
        contact_mask = self.data['IsInContact'] == 1
        if contact_mask.any():
            # Remove contact points scatter plot and colorbar
            pass
        
        ax3.set_ylabel('Force (N)', fontsize=18, labelpad=15, color='purple')
        ax3.tick_params(axis='both', which='major', labelsize=14)
        ax3.grid(True, alpha=0.3)
        
        lines1, labels1 = ax3.get_legend_handles_labels()
        # Move legend down slightly
        ax3.legend(lines1, labels1, 
                loc='center left', fontsize=14, framealpha=0.9, 
                bbox_to_anchor=(1.05, 0.3))

        # Redesigned contact timeline
        ax4 = fig.add_subplot(gs[3])
        contact_periods = self.data['IsInContact'].astype(bool)
        
        # Create custom colormap for contact states with better contrast
        colors = {'contact': '#27ae60', 'no_contact': '#e74c3c'}
        
        # Add background grid for better time reference
        ax4.grid(True, axis='x', alpha=0.3)
        
        # Plot contact timeline as a gantt-chart style visualization
        ax4.fill_between(time_data, 0.2, 0.8, where=contact_periods, 
                        color=colors['contact'], alpha=0.8, 
                        label='Contact Active')
        ax4.fill_between(time_data, 0.2, 0.8, where=~contact_periods, 
                        color=colors['no_contact'], alpha=0.4, 
                        label='No Contact')
        
        # Add time markers and improve visibility
        major_ticks = np.linspace(time_data.min(), time_data.max(), 10)
        ax4.set_xticks(major_ticks)
        ax4.set_xticklabels([f'{t:.1f}s' for t in major_ticks])
        
        # Improve timeline appearance
        ax4.set_ylim(0, 1)
        ax4.set_yticks([])
        ax4.set_xlabel('Time (seconds)', fontsize=18, labelpad=15)
        ax4.tick_params(axis='x', which='major', labelsize=14)
        
        # Add contact state labels on the y-axis
        ax4.text(-0.05, 0.5, 'Contact\nState', fontsize=14, 
                verticalalignment='center', horizontalalignment='right',
                transform=ax4.transAxes)
        
        # Move legend down slightly
        ax4.legend(loc='center left', bbox_to_anchor=(1.05, 0.3), 
                fontsize=14, framealpha=0.9)

        # Add event markers to the left of the dotted line
        def add_event_markers(axes):
            for ax in axes:
                # Get the y-axis limits
                ymin, ymax = ax.get_ylim()
                
                # Calculate a position slightly above the bottom of the graph
                # Using 5% of the total y-axis range
                text_y_pos = ymin + (ymax - ymin) * 0.05
                
                # Add vertical lines and annotations slightly to the left
                ax.axvline(x=2.5, color='red', linestyle=':', alpha=0.5)
                ax.text(2.3, text_y_pos, 'Robots begin\n moving',
                        horizontalalignment='right', verticalalignment='bottom')
                
                ax.axvline(x=7.5, color='red', linestyle=':', alpha=0.5)
                ax.text(7.3, text_y_pos, 'Initial box contact',
                        horizontalalignment='right', verticalalignment='bottom')
                
                ax.axvline(x=32.5, color='red', linestyle=':', alpha=0.5)
                ax.text(32.3, text_y_pos, 'Robots stop\n moving',
                        horizontalalignment='right', verticalalignment='bottom')

        # Add event markers
        add_event_markers([ax1, ax2, ax3])

        # Adjust layout to prevent overlapping
        plt.tight_layout()
        
        # Add more padding on the right for legends and adjust top margin
        plt.subplots_adjust(right=0.85, top=0.92)

        # Add overall title with more space
        fig.suptitle('Experiment Timeline', fontsize=24, y=0.95)
        
        return fig
    
    def plot_interaction_phases(self, ax):
        """Plot interaction phase timeline"""
        phases = []
        times = []
        current_phase = "No Contact"
        
        for idx, row in self.data.iterrows():
            if row['IsInContact'] == 0:
                phase = "No Contact"
            elif row['ContactCount'] == 1:
                phase = "Box Contact"
            else:
                phase = "Robot Contact"
                
            if phase != current_phase:
                phases.append(phase)
                times.append(row['TaskTime'])
                current_phase = phase

        # Plot phases as colored regions
        colors = {'No Contact': 'white', 'Box Contact': 'lightblue', 'Robot Contact': 'lightcoral'}
        
        for i in range(len(times)-1):
            ax.axvspan(times[i], times[i+1], 
                    color=colors[phases[i]], 
                    alpha=0.5, 
                    label=phases[i] if phases[i] not in ax.get_legend_handles_labels()[1] else "")

        ax.set_yticks([])
        ax.set_xlabel('Time (s)', fontsize=16)
        ax.legend(loc='center right', bbox_to_anchor=(1.15, 0.5))

    def add_event_markers(self, axes):
        """Add event markers with improved descriptions"""
        event_descriptions = {
            'Initial box contact',
            'Robot Contact',
            'Max Force',
            'Robots Movement Start',
            'Robots Movement Stop'
        }

        # Improved movement detection
        speed_threshold = 0.05  # Increased threshold to filter out noise (was too low)
        window_size = 5  # For smoothing
        
        # Smooth the speed data to reduce noise
        robot1_speed_smooth = self.data['Robot1Speed'].rolling(window=window_size, center=True).mean()
        robot2_speed_smooth = self.data['Robot2Speed'].rolling(window=window_size, center=True).mean()
        
        # Combined movement mask with higher threshold
        movement_mask = (robot1_speed_smooth > speed_threshold) | \
                    (robot2_speed_smooth > speed_threshold)
        
        # Find transitions (0 to 1 for start, 1 to 0 for stop)
        movement_transitions = np.diff(movement_mask.astype(int))
        
        # Get timestamps of transitions
        movement_starts = self.data['TaskTime'][1:][movement_transitions > 0]
        movement_stops = self.data['TaskTime'][1:][movement_transitions < 0]

        # Only add movement events if actual movement detected
        if len(movement_starts) > 0:
            self.events['Movement Start'] = movement_starts.iloc[0]
        if len(movement_stops) > 0:
            self.events['Movement Stop'] = movement_stops.iloc[-1]  # Use last stop if multiple

        # Add event markers with staggered heights and detailed descriptions
        for i, (event, time) in enumerate(self.events.items()):
            for ax in axes:
                ymin, ymax = ax.get_ylim()
                text_y = ymax - (i + 1) * (ymax - ymin) * 0.15
                
                ax.axvline(x=time, color='black', linestyle='--', 
                        alpha=0.5, linewidth=2)
                ax.text(time + 0.5, text_y, 
                    f"{event}\n{event_descriptions.get(event, '')}",
                    fontsize=12,
                    bbox=dict(facecolor='white',
                                edgecolor='black',
                                alpha=0.8,
                                pad=5))

        # Add debug log for movement detection
        if self.data['Robot1Speed'].max() < speed_threshold and \
        self.data['Robot2Speed'].max() < speed_threshold:
            print(f"No significant movement detected. Max speeds: " \
                f"Robot1={self.data['Robot1Speed'].max():.3f}, " \
                f"Robot2={self.data['Robot2Speed'].max():.3f}")

    def plot_robot_interaction(self):
            """Plot robot positions and distances with improved visibility"""
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 12))
            plt.subplots_adjust(hspace=0.3)

            # Robot positions
            ax1.plot(self.data['TaskTime'], self.data['Robot1PosX'], 
                    color='red', linewidth=2.5, label='Robot 1')
            ax1.plot(self.data['TaskTime'], self.data['Robot2PosX'], 
                    color='blue', linewidth=2.5, label='Robot 2')
            ax1.set_ylabel('X Position (m)', fontsize=16)
            ax1.grid(True, alpha=0.3)
            ax1.legend(loc='center right', fontsize=14, 
                    bbox_to_anchor=(1.15, 0.5),
                    framealpha=0.9)
            ax1.set_title('Robot Positions', pad=20, fontsize=18)

            # Robot distance
            distance = self.data['RobotDistanceDiff']
            ax2.plot(self.data['TaskTime'], distance, 
                    color='green', linewidth=2.5)
            ax2.set_xlabel('Time (s)', fontsize=16)
            ax2.set_ylabel('Distance (m)', fontsize=16)
            ax2.grid(True, alpha=0.3)
            ax2.set_title('Distance Between Robots', pad=20, fontsize=18)

            # Add event markers with improved positioning
            for i, (event, time) in enumerate(self.events.items()):
                for ax in [ax1, ax2]:
                    ax.axvline(x=time, color='black', linestyle='--', 
                            alpha=0.5, linewidth=2)
                    ymin, ymax = ax.get_ylim()
                    # Stagger text positions
                    text_y = ymax - (i + 1) * (ymax - ymin) * 0.15
                    ax.text(time + 0.5, text_y, event,
                        fontsize=14,
                        bbox=dict(facecolor='white',
                                    edgecolor='black',
                                    alpha=0.8,
                                    pad=5))

            fig.suptitle('Robot Interaction Analysis', fontsize=20, y=0.95)
            return fig

    def plot_haptic_analysis(self):
        """Plot haptic interaction analysis with improved visibility"""
        fig = plt.figure(figsize=(16, 12))
        gs = plt.GridSpec(2, 2, figure=fig)
        ax1 = fig.add_subplot(gs[0, 0])
        ax2 = fig.add_subplot(gs[0, 1])
        ax3 = fig.add_subplot(gs[1, 0])
        ax4 = fig.add_subplot(gs[1, 1])

        try:
            # Force components during contact
            contact_mask = self.data['IsInContact'] == 1
            if contact_mask.any():
                contact_data = self.data[contact_mask]
                ax1.plot(contact_data['TaskTime'], contact_data['HapticForceX'], 
                        color='red', linewidth=2.5, label='X')
                ax1.plot(contact_data['TaskTime'], contact_data['HapticForceY'], 
                        color='green', linewidth=2.5, label='Y')
                ax1.plot(contact_data['TaskTime'], contact_data['HapticForceZ'], 
                        color='blue', linewidth=2.5, label='Z')
            ax1.set_ylabel('Force (N)', fontsize=16)
            ax1.grid(True, alpha=0.3)
            ax1.legend(loc='upper right', fontsize=14, framealpha=0.9)
            ax1.set_title('Force Components During Contact', pad=20, fontsize=18)

            # Force distribution histogram
            force_data = self.data[contact_mask]['ForceMagnitude']
            if not force_data.empty:
                bins = np.linspace(0, force_data.max(), 30)
                ax2.hist(force_data, bins=bins, color='blue', alpha=0.7,
                        edgecolor='black', linewidth=1.5)
                ax2.set_xlabel('Force Magnitude (N)', fontsize=16)
                ax2.set_ylabel('Count', fontsize=16)
                ax2.grid(True, alpha=0.3)
                ax2.set_title('Force Distribution', pad=20, fontsize=18)

            # Force vs Rotation with time-based coloring
            if contact_mask.any():
                times = pd.to_numeric(contact_data['TaskTime'])
                scatter = ax3.scatter(contact_data['ForceMagnitude'],
                                   contact_data['BoxRotation'],
                                   c=times,
                                   cmap='viridis',
                                   s=100,  # Increased marker size
                                   alpha=0.6)
                cbar = plt.colorbar(scatter, ax=ax3)
                cbar.set_label('Time (s)', fontsize=14, labelpad=15)
                cbar.ax.tick_params(labelsize=12)
            ax3.set_xlabel('Force Magnitude (N)', fontsize=16)
            ax3.set_ylabel('Box Rotation (°)', fontsize=16)
            ax3.grid(True, alpha=0.3)
            ax3.set_title('Force-Rotation Relationship', pad=20, fontsize=18)

            # Cumulative contact duration
            times = pd.to_numeric(self.data['TaskTime'])
            cumulative_contact = np.cumsum(self.data['IsInContact']) * \
                               np.mean(np.diff(times))
            ax4.plot(times, cumulative_contact, 
                    color='black', linewidth=2.5)
            ax4.set_xlabel('Time (s)', fontsize=16)
            ax4.set_ylabel('Contact Duration (s)', fontsize=16)
            ax4.grid(True, alpha=0.3)
            ax4.set_title('Cumulative Contact Time', pad=20, fontsize=18)

        except Exception as e:
            print(f"Error in haptic analysis plot: {e}")

        plt.tight_layout()
        fig.suptitle('Haptic Interaction Analysis', fontsize=20, y=1.02)
        return fig
    
    def plot_system_latency(self):
        """Plot system communication and response latencies"""
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 12))
        
        # ROS-Unity Communication Time
        time_diff = np.diff(self.data['TaskTime'])
        ax1.plot(self.data['TaskTime'][1:], time_diff * 1000, 'b-', label='Update Rate')
        ax1.set_ylabel('Update Interval (ms)', fontsize=16)
        ax1.set_title('ROS-Unity Communication Performance', fontsize=18)
        ax1.grid(True, alpha=0.3)
        ax1.axhline(y=20, color='r', linestyle='--', label='Target (20ms)')
        ax1.legend()

        # Force Feedback Response Time
        force_changes = np.diff(self.data['ForceMagnitude']) != 0
        contact_changes = np.diff(self.data['IsInContact']) != 0
        response_events = force_changes | contact_changes
        
        if any(response_events):
            ax2.scatter(self.data['TaskTime'][1:][response_events], 
                    self.data['ForceMagnitude'][1:][response_events],
                    c='g', label='Force Response')
        ax2.set_ylabel('Force Response (N)', fontsize=16)
        ax2.set_xlabel('Time (s)', fontsize=16)
        ax2.set_title('Haptic Response Performance', fontsize=18)
        ax2.grid(True)
        
        return fig
    
    def plot_haptic_quality_metrics(self):
        """Plot metrics showing haptic interaction quality"""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        
        # Force Resolution and Stability
        contact_mask = self.data['IsInContact'] == 1
        if contact_mask.any():
            contact_forces = self.data['ForceMagnitude'][contact_mask]
            
            # Force resolution histogram
            ax1.hist(contact_forces, bins=50, alpha=0.7)
            ax1.set_xlabel('Force Magnitude (N)', fontsize=14)
            ax1.set_ylabel('Frequency', fontsize=14)
            ax1.set_title('Force Resolution Distribution', fontsize=16)
            
            # Force stability over time
            window = 10
            force_stability = contact_forces.rolling(window=window).std()
            ax2.plot(self.data['TaskTime'][contact_mask], force_stability)
            ax2.set_xlabel('Time (s)', fontsize=14)
            ax2.set_ylabel('Force Stability (N)', fontsize=14)
            ax2.set_title('Force Feedback Stability', fontsize=16)
        
        # Position Tracking Accuracy
        ax3.plot(self.data['TaskTime'], self.data['HapticPosX'], label='X')
        ax3.plot(self.data['TaskTime'], self.data['HapticPosY'], label='Y')
        ax3.plot(self.data['TaskTime'], self.data['HapticPosZ'], label='Z')
        ax3.set_xlabel('Time (s)', fontsize=14)
        ax3.set_ylabel('Position (m)', fontsize=14)
        ax3.set_title('Haptic Position Tracking', fontsize=16)
        ax3.legend()
        
        # Contact Detection Performance
        if contact_mask.any():
            contact_durations = self.data['ContactDuration'][contact_mask]
            ax4.hist(contact_durations, bins=30, alpha=0.7)
            ax4.set_xlabel('Contact Duration (s)', fontsize=14)
            ax4.set_ylabel('Frequency', fontsize=14)
            ax4.set_title('Contact Detection Performance', fontsize=16)
        
        fig.suptitle('Haptic Interaction Quality Metrics', fontsize=20)
        return fig
    
    def plot_system_stability(self):
        """Plot overall system stability metrics"""
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(16, 12))
        
        # Box Control Stability
        ax1.plot(self.data['TaskTime'], self.data['BoxRotation'], 'b-')
        ax1.fill_between(self.data['TaskTime'], 
                        self.data['BoxRotation'] - 2,
                        self.data['BoxRotation'] + 2,
                        color='blue', alpha=0.2)
        ax1.set_ylabel('Box Rotation (°)', fontsize=14)
        ax1.set_title('Object Control Stability', fontsize=16)
        ax1.grid(True)

        # Robot Movement Smoothness
        # Calculate jerk (rate of acceleration change)
        robot1_jerk = np.diff(self.data['Robot1Speed'], 2) / np.diff(self.data['TaskTime'][:-1]) ** 2
        robot2_jerk = np.diff(self.data['Robot2Speed'], 2) / np.diff(self.data['TaskTime'][:-1]) ** 2
        
        ax2.plot(self.data['TaskTime'][2:], abs(robot1_jerk), 'r-', label='Robot 1', alpha=0.7)
        ax2.plot(self.data['TaskTime'][2:], abs(robot2_jerk), 'g-', label='Robot 2', alpha=0.7)
        ax2.set_ylabel('Movement Smoothness\n(Jerk)', fontsize=14)
        ax2.legend()
        ax2.grid(True)

        # System State Coherence
        # Compare intended vs. actual states
        ax3.scatter(self.data['TaskTime'][self.data['IsInContact'] == 1],
                    np.ones(sum(self.data['IsInContact'] == 1)),
                    c='g', label='Contact States', alpha=0.5)
        ax3.set_ylabel('System State\nCoherence', fontsize=14)
        ax3.set_xlabel('Time (s)', fontsize=14)
        ax3.grid(True)
        
        fig.suptitle('System Integration Stability Analysis', fontsize=20)
        return fig

    def save_all_plots(self, output_dir):
        """Save all plots to files with high resolution"""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        plots = {
            'system_latency': self.plot_system_latency(),
            'haptic_quality': self.plot_haptic_quality_metrics(),
            'system_stability': self.plot_system_stability(),
            'timeline': self.plot_experiment_timeline(),
            'robot_interaction': self.plot_robot_interaction(),
            'haptic_analysis': self.plot_haptic_analysis()
        }
        
        for name, fig in plots.items():
            filepath = output_path / f'{name}.png'
            fig.savefig(filepath, 
                       dpi=300,
                       bbox_inches='tight',
                       pad_inches=0.5)
            plt.close(fig)
            print(f'Saved {filepath}')

if __name__ == "__main__":
    try:
        plotter = ExperimentPlotter("ExperimentData\experiment_session_20241109_224136.csv")
        plotter.save_all_plots("experiment_plots")
        print("All plots generated successfully!")
    except Exception as e:
        print(f"Error occurred: {str(e)}")
        import traceback
        traceback.print_exc()