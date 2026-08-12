import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

class VerticalLineHandler:
    def __init__(self, color='red', linestyle=':', linewidth=2):
        self.color = color
        self.linestyle = linestyle
        self.linewidth = linewidth
        
    def legend_artist(self, legend, orig_handle, fontsize, handlebox):
        x0, y0 = handlebox.xdescent, handlebox.ydescent
        width = handlebox.width
        height = handlebox.height
        
        # Center the line horizontally and make it longer
        x_center = x0 + width * 0.5
        height_reduction = height   # Reduce height by only 15%
        y_center = y0   # Find vertical center
        line_height = height * 0.7  # Make line 70% of box height
        
        # Position line centered vertically
        y_start = y_center - line_height/2
        y_end = y_center + line_height/2
        
        line = plt.Line2D([x_center, x_center],  # Centered x position
                         [y_start, y_end],  # Centered and longer
                         color=self.color,
                         linestyle=self.linestyle,
                         linewidth=self.linewidth)
        handlebox.add_artist(line)
        return line

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
        plt.rcParams['font.size'] = 14
        plt.rcParams['axes.labelsize'] = 16
        plt.rcParams['axes.titlesize'] = 20
        plt.rcParams['figure.titlesize'] = 24
        plt.rcParams['xtick.labelsize'] = 14
        plt.rcParams['ytick.labelsize'] = 14
        plt.rcParams['legend.fontsize'] = 16
        plt.rcParams['axes.grid'] = True
        plt.rcParams['grid.alpha'] = 0.3
        plt.rcParams['lines.linewidth'] = 2.5
        plt.rcParams['axes.linewidth'] = 2
        plt.rcParams['grid.linewidth'] = 1.5

    def detect_events(self):
        """Detect only robot movement events"""
        self.events = {}

        # Only detect robot movement (using speed threshold)
        if 'Robot1Speed' in self.data.columns and 'Robot2Speed' in self.data.columns:
            speed_threshold = 0.05
            movement_mask = (self.data['Robot1Speed'] > speed_threshold) | (self.data['Robot2Speed'] > speed_threshold)
            if movement_mask.any():
                first_movement = self.data[movement_mask].iloc[0]
                last_movement = self.data[movement_mask].iloc[-1]
                self.events['Robots begin moving'] = first_movement['TaskTime']
                self.events['Robots stop moving'] = last_movement['TaskTime']

        print("\nDetected Events:")
        for event, time in self.events.items():
            print(f"{event}: {time:.2f}s")

    # Add event markers
    def add_event_markers(self, axes):
        """Add event markers with improved text placement"""
        # Define vertical positions for staggered text
        text_positions = {
            'Robots begin moving': -0.05,  # Lower position
            'Robots stop moving': -0.05    # Lower position
        }

        # Define text alignments and offsets for each event
        text_configs = {
            'Robots begin moving': {
                'align': 'right',
                'offset': -0.1  # Shift left of the line
            },
            'Robots stop moving': {
                'align': 'left',
                'offset': 0.1   # Shift right of the line
            }
        }
        
        for ax in axes:
            ymin, ymax = ax.get_ylim()
            y_range = ymax - ymin
            
            for event, time in self.events.items():
                # Add vertical line
                ax.axvline(x=time, color='red', linestyle=':', alpha=0.5)
                
                # Add text with staggered positioning
                relative_pos = text_positions[event]
                text_y = ymin + y_range * relative_pos

                # Get configuration for this event
                config = text_configs[event]
                text_x = time + config['offset']  # Adjust x position
                
                # Add text with white background for better readability
                ax.text(text_x, text_y, event,
                    rotation=45,  # Diagonal text
                    horizontalalignment=config['align'],
                    verticalalignment='bottom',
                    fontsize=18,
                    bbox=dict(facecolor='white',
                            edgecolor='none',
                            alpha=0.9,
                            pad=3))

    def plot_experiment_timeline(self):
        """Plot main experiment timeline with improved layout and spacing"""
        fig = plt.figure(figsize=(24, 20))
        gs = plt.GridSpec(4, 1, height_ratios=[1, 1, 1, 0.1], hspace=0.4)
        
        # Box Rotation plot
        ax1 = fig.add_subplot(gs[0])
        ax1.set_title('Box Orientation During Task', pad=20, fontsize=30)
        rotation_data = self.data['BoxRotation']
        time_data = self.data['TaskTime']
        
        window = 5
        rotation_std = rotation_data.rolling(window=window).std().fillna(0.5)

        # First plot the contact regions
        ymin, ymax = -8, 4  # Use your y-axis limits
        contact_colors = {
            'box': ('#c4ae00', 'Box'),
            'robot1': ('#b80600', 'Left Robot'),
            'robot2': ('#0a0df5', 'Right Robot')
        }

        # Add contact regions first
        for contact_type, (color, label) in contact_colors.items():
            type_mask = (self.data['IsInContact'] == 1) & (self.data['ContactType'] == contact_type)
            ax1.fill_between(time_data, ymin, ymax, where=type_mask,
                            color=color, alpha=0.4),
                            # label=f'Contact with {label}')
        
        ax1.plot(time_data, rotation_data, 'b-', linewidth=3, label='Box Rotation', zorder=5)
        
        ax1.axhline(y=0, color='g', linestyle='--', linewidth=2, label='Perfect Alignment (0°)', zorder=4)
        ax1.axhspan(-2, 2, facecolor='none', hatch='ooo', edgecolor='g', alpha=0.4, label='Target Range (±2°)', zorder=3)
        
        ax1.set_ylabel('Box Rotation (°)', fontsize=25, labelpad=20)
        ax1.set_ylim(-8, 4)  # Adjusted to focus on relevant range
        ax1.tick_params(axis='both', which='major', labelsize=14)
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='center left', fontsize=30, framealpha=0.9, 
                bbox_to_anchor=(1.02, 0.5))

        # Robot Speeds plot
        ax2 = fig.add_subplot(gs[1])
        ax2.set_title('Robot Speed Profiles', pad=20, fontsize=30)

        # First plot the contact regions
        ymax = max(max(self.data['Robot1Speed']), max(self.data['Robot2Speed'])) * 1.2
        contact_colors = {
            'box': ('#c4ae00', 'Box'),
            'robot1': ('#b80600', 'Left Robot'),
            'robot2': ('#0a0df5', 'Right Robot')
        }

        # Add contact regions first
        for contact_type, (color, label) in contact_colors.items():
            type_mask = (self.data['IsInContact'] == 1) & (self.data['ContactType'] == contact_type)
            ax2.fill_between(time_data, 0, ymax, where=type_mask,
                            color=color, alpha=0.4)
                            # label=f'Contact with {label}')

        # Then plot the speed profiles on top
        window = 10
        robot1_speed_smooth = self.data['Robot1Speed'].rolling(window=window, center=True).mean()
        robot2_speed_smooth = self.data['Robot2Speed'].rolling(window=window, center=True).mean()

        speed_std1 = self.data['Robot1Speed'].rolling(window=window).std().fillna(0)
        speed_std2 = self.data['Robot2Speed'].rolling(window=window).std().fillna(0)

        ax2.plot(time_data, robot1_speed_smooth, 'b-', linewidth=3, 
                label='Left Robot Speed', zorder=5)
        ax2.plot(time_data, robot2_speed_smooth, 'g-', linewidth=3, 
                label='Right Robot Speed', zorder=5)
        # ax2.fill_between(time_data, 
        #                 robot1_speed_smooth - speed_std1,
        #                 robot1_speed_smooth + speed_std1,
        #                 color='blue', alpha=0.2, zorder=4)
        # ax2.fill_between(time_data, 
        #                 robot2_speed_smooth - speed_std2,
        #                 robot2_speed_smooth + speed_std2,
        #                 color='green', alpha=0.2, zorder=4)

        ax2.set_ylabel('Speed (m/s)', fontsize=25, labelpad=15)
        ax2.tick_params(axis='both', which='major', labelsize=14)
        ax2.grid(True, alpha=0.3)
        ax2.set_ylim(0, ymax)  # Set y-axis limits to match the contact regions
        ax2.legend(loc='center left', fontsize=30, framealpha=0.9, 
                bbox_to_anchor=(1.02, 0.5))

        # Force plot
        ax3 = fig.add_subplot(gs[2])
        ax3.set_title('Human-Robot Interaction Forces and Contact States', pad=20, fontsize=30)
        
        # First plot the colored contact regions
        ymax = max(self.data['ForceMagnitude']) * 1.2 # Get force range with 10% padding
        contact_colors = {
            'box': ('#c4ae00', 'Box'),
            'robot1': ('#b80600', 'Left Robot'),
            'robot2': ('#0a0df5', 'Right Robot')
        }
        
        for contact_type, (color, label) in contact_colors.items():
            type_mask = (self.data['IsInContact'] == 1) & (self.data['ContactType'] == contact_type)
            ax3.fill_between(time_data, 0, ymax, where=type_mask,
                           color=color, alpha=0.4)
                           # label=f'Contact with {label}')
        
        # Plot the force magnitude line on top
        ax3.plot(time_data, self.data['ForceMagnitude'], 'blue', 
                linewidth=3, label='Haptic Interaction Force',
                zorder=5)  # Ensure force line is on top
        
        ax3.set_ylim(0, ymax)
        ax3.set_xlabel('Time (seconds)', fontsize=25, labelpad=15)
        ax3.set_ylabel('Haptic Interaction\n Force (N)', fontsize=25, labelpad=15, color='black')
        ax3.tick_params(axis='both', which='major', labelsize=14)
        ax3.grid(True, alpha=0.3)
        ax3.legend(loc='center left', fontsize=30, framealpha=0.9, 
                bbox_to_anchor=(1.02, 0.5))
        
        # Call the modified add_event_markers
        self.add_event_markers([ax1, ax2, ax3])

        # Create the bottom space for legend
        ax4 = fig.add_subplot(gs[3])
        ax4.axis('off')

        # Create lines for the contact legend
        contact_lines = []
        contact_labels = []
        contact_colors = {
            'box': ('#c4ae00', 'Box'),        # Light yellow
            'robot1': ('#b80600', 'Left Robot'),  # Light red
            'robot2': ('#0a0df5', 'Right Robot')  # Light blue
        }
        
        for contact_type, (color, label) in contact_colors.items():
            # Create a tall, narrow rectangle to appear as a vertical bar
            rect = plt.Rectangle((0,0), 0.2, 1, fc=color, alpha=0.4)
            contact_lines.append(rect)
            contact_labels.append(f'Human Contact with {label}')

        # Add vertical dotted line for robot start/stop
        contact_lines.append(plt.Line2D([0], [0], color='red', linestyle=':'))
        contact_labels.append('Robots start/stop')


        # Create single row legend at bottom
        ax4.legend(contact_lines, contact_labels, 
                loc='center', 
                ncol=4,  
                fontsize=25,
                handlelength=1,  
                handleheight=2,
                handler_map={plt.Line2D: VerticalLineHandler()})

        plt.tight_layout()
        plt.subplots_adjust(right=0.82, top=0.92, bottom=0.15, hspace=0.5)
        # fig.suptitle('Experiment Timeline', fontsize=24, y=0.95)
        
        return fig

    def save_all_plots(self, output_dir):
        """Save all plots to files with high resolution"""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        # Save the experiment timeline
        timeline_fig = self.plot_experiment_timeline()
        timeline_fig.savefig(output_path / 'experiment_timeline.png', 
                           dpi=300, 
                           bbox_inches='tight',
                           pad_inches=0.5)
        plt.close(timeline_fig)
        print('Saved experiment timeline plot')

if __name__ == "__main__":
    try:
        plotter = ExperimentPlotter("Assets\ExperimentData\experiment_session_20241111_130135.csv")
        plotter.save_all_plots("experiment_plots")
        print("All plots generated successfully!")
    except Exception as e:
        print(f"Error occurred: {str(e)}")
        import traceback
        traceback.print_exc()