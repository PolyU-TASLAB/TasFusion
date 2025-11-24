import pandas as pd
import matplotlib.pyplot as plt
import os
import argparse

class BasePlotter:
    def __init__(self, df, timestamp_col, output_dir="plots",
                 file_prefix="", save_plots=True, show_plots=True):
        self.df = df
        self.timestamp_col = timestamp_col
        self.output_dir = output_dir if output_dir else "." 
        self.file_prefix = file_prefix
        self.save_plots = save_plots
        self.show_plots = show_plots

        if self.save_plots and self.output_dir != "." and not os.path.exists(self.output_dir):
            try:
                os.makedirs(self.output_dir)
                print(f"Created output directory: {os.path.abspath(self.output_dir)}")
            except Exception as e:
                print(f"Warning: Error creating output directory '{self.output_dir}': {e}.")
                if self.output_dir:
                    self.output_dir = "."

    def _sanitize_filename_part(self, name_part):
        return "".join(c if c.isalnum() or c in (' ', '_', '-') else '_' for c in name_part).replace(' ', '_')

    def _prepare_and_finalize_plot(self, fig, ax, title, xlabel, ylabel, legend_loc='best', plot_type_name_for_file="plot"):
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        if any(ax.get_legend_handles_labels()[1]):
            ax.legend(loc=legend_loc)
        ax.grid(True)
        plt.tight_layout()

        if self.save_plots:
            safe_plot_name = self._sanitize_filename_part(plot_type_name_for_file)
            current_output_dir = self.output_dir if self.output_dir and os.path.isdir(self.output_dir) else "."
            if not os.path.isdir(current_output_dir):
                print(f"Warning: Output directory '{current_output_dir}' does not exist or is not a directory. Attempting to save in CWD.")
                current_output_dir = "."
            filename = os.path.join(current_output_dir, f"{self.file_prefix}{safe_plot_name}.png")
            try:
                fig.savefig(filename)
                print(f"Plot saved to: {os.path.abspath(filename)}")
            except Exception as e:
                print(f"Error saving plot to {filename}: {e}")
        
        if self.show_plots:
            plt.show() # Changed from plt.show(fig)
        
        plt.close(fig)

    def plot(self):
        raise NotImplementedError("Derived classes must implement the 'plot' method.")

class Vector3DPlotter(BasePlotter):
    def __init__(self, df, timestamp_col, x_col, y_col, z_col,
                 vector_name, y_axis_unit, **kwargs):
        super().__init__(df, timestamp_col, **kwargs)
        self.x_col = x_col
        self.y_col = y_col
        self.z_col = z_col
        self.vector_name = vector_name
        self.y_axis_unit = y_axis_unit

    def plot(self):
        # --- DEBUG PRINTS ---
        print(f"\n--- Plotting Vector3D: '{self.vector_name}' ---")
        print(f"  Timestamp column: '{self.timestamp_col}'")
        print(f"  X column: '{self.x_col}'")
        print(f"  Y column: '{self.y_col}'")
        print(f"  Z column: '{self.z_col}'")
        print(f"  Available DataFrame columns: {list(self.df.columns)}")
        # --- END DEBUG PRINTS ---

        cols_to_check = [self.timestamp_col, self.x_col, self.y_col, self.z_col]
        # Check for empty string column names before checking existence in DataFrame
        if any(not col_name for col_name in cols_to_check): # Checks if any col_name is None or empty
            print(f"Warning: For '{self.vector_name}', one or more expected column names are empty. Skipping plot.")
            print(f"  Received: timestamp='{self.timestamp_col}', x='{self.x_col}', y='{self.y_col}', z='{self.z_col}'")
            return

        if not all(col in self.df.columns for col in cols_to_check):
            missing = [col for col in cols_to_check if col not in self.df.columns]
            print(f"Warning: For '{self.vector_name}', missing required columns in DataFrame: {missing}. Skipping plot.")
            return

        ts_np = self.df[self.timestamp_col].to_numpy()
        x_np = self.df[self.x_col].to_numpy()
        y_np = self.df[self.y_col].to_numpy()
        z_np = self.df[self.z_col].to_numpy()

        fig, ax = plt.subplots(figsize=(12, 6))
        
        ax.plot(ts_np, x_np, label=f'{self.x_col} (X)')
        ax.plot(ts_np, y_np, label=f'{self.y_col} (Y)')
        ax.plot(ts_np, z_np, label=f'{self.z_col} (Z)')
        
        title = f'{self.vector_name} over Time'
        xlabel = f'{self.timestamp_col.replace("_", " ").capitalize()} (seconds)'
        ylabel = f'{self.vector_name} ({self.y_axis_unit})'
        
        self._prepare_and_finalize_plot(fig, ax, title, xlabel, ylabel,
                                        plot_type_name_for_file=f"{self.vector_name}_3D_vector")

class OrientationQuaternionPlotter(BasePlotter):
    def __init__(self, df, timestamp_col, w_col, x_col, y_col, z_col, plot_name="Orientation_Quaternion", **kwargs):
        super().__init__(df, timestamp_col, **kwargs)
        self.w_col = w_col
        self.x_col = x_col
        self.y_col = y_col
        self.z_col = z_col
        self.plot_name = plot_name 

    def plot(self):
        # --- DEBUG PRINTS ---
        print(f"\n--- Plotting Quaternion: '{self.plot_name}' ---")
        print(f"  Timestamp column: '{self.timestamp_col}'")
        print(f"  W column: '{self.w_col}'")
        print(f"  X column: '{self.x_col}'")
        print(f"  Y column: '{self.y_col}'")
        print(f"  Z column: '{self.z_col}'")
        print(f"  Available DataFrame columns: {list(self.df.columns)}")
        # --- END DEBUG PRINTS ---

        cols_to_check = [self.timestamp_col, self.w_col, self.x_col, self.y_col, self.z_col]
        if any(not col_name for col_name in cols_to_check):
            print(f"Warning: For Quaternion Plot '{self.plot_name}', one or more expected column names are empty. Skipping plot.")
            print(f"  Received: timestamp='{self.timestamp_col}', w='{self.w_col}', x='{self.x_col}', y='{self.y_col}', z='{self.z_col}'")
            return

        if not all(col in self.df.columns for col in cols_to_check):
            missing = [col for col in cols_to_check if col not in self.df.columns]
            print(f"Warning: For Quaternion Plot '{self.plot_name}', missing required columns in DataFrame: {missing}. Skipping plot.")
            return

        ts_np = self.df[self.timestamp_col].to_numpy()
        w_np = self.df[self.w_col].to_numpy()
        x_np = self.df[self.x_col].to_numpy()
        y_np = self.df[self.y_col].to_numpy()
        z_np = self.df[self.z_col].to_numpy()

        fig, ax = plt.subplots(figsize=(12, 6))
        
        ax.plot(ts_np, w_np, label=f'{self.w_col} (w)')
        ax.plot(ts_np, x_np, label=f'{self.x_col} (x)')
        ax.plot(ts_np, y_np, label=f'{self.y_col} (y)')
        ax.plot(ts_np, z_np, label=f'{self.z_col} (z)')
        
        title = f'{self.plot_name.replace("_", " ")} (Quaternion) over Time'
        xlabel = f'{self.timestamp_col.replace("_", " ").capitalize()} (seconds)'
        ylabel = 'Quaternion Component Value'
        
        self._prepare_and_finalize_plot(fig, ax, title, xlabel, ylabel,
                                        plot_type_name_for_file=self.plot_name)

class ScalarDataPlotter(BasePlotter):
    def __init__(self, df, timestamp_col, data_col, scalar_name, y_axis_unit, **kwargs):
        super().__init__(df, timestamp_col, **kwargs)
        self.data_col = data_col
        self.scalar_name = scalar_name
        self.y_axis_unit = y_axis_unit

    def plot(self):
        # --- DEBUG PRINTS ---
        print(f"\n--- Plotting Scalar: '{self.scalar_name}' ---")
        print(f"  Timestamp column: '{self.timestamp_col}'")
        print(f"  Data column: '{self.data_col}'")
        print(f"  Available DataFrame columns: {list(self.df.columns)}")
        # --- END DEBUG PRINTS ---

        cols_to_check = [self.timestamp_col, self.data_col]
        if any(not col_name for col_name in cols_to_check):
            print(f"Warning: For Scalar '{self.scalar_name}', one or more expected column names are empty. Skipping plot.")
            print(f"  Received: timestamp='{self.timestamp_col}', data_col='{self.data_col}'")
            return

        if not all(col in self.df.columns for col in cols_to_check):
            missing = [col for col in cols_to_check if col not in self.df.columns]
            print(f"Warning: For Scalar '{self.scalar_name}', missing required columns in DataFrame: {missing}. Skipping plot.")
            return

        ts_np = self.df[self.timestamp_col].to_numpy()
        data_np = self.df[self.data_col].to_numpy()

        fig, ax = plt.subplots(figsize=(12, 6))
        
        ax.plot(ts_np, data_np, label=f'{self.data_col}')
        
        title = f'{self.scalar_name} over Time'
        xlabel = f'{self.timestamp_col.replace("_", " ").capitalize()} (seconds)'
        ylabel = f'{self.scalar_name} ({self.y_axis_unit})'
        
        self._prepare_and_finalize_plot(fig, ax, title, xlabel, ylabel,
                                        plot_type_name_for_file=f"{self.scalar_name}_scalar")

def main():
    parser = argparse.ArgumentParser(description='Plot various types of timestamped data from a CSV file using an object-oriented approach.')
    parser.add_argument('file_path', type=str, help='Path to the CSV file.')
    parser.add_argument('--timestamp-col', type=str, default='timestamp', help="Name of the timestamp column (default: 'timestamp').")
    
    parser.add_argument('--output-dir', type=str, default=None, 
                        help="Directory to save plots. If not specified, plots are saved in the same directory as the input file.")
    parser.add_argument('--output-prefix', type=str, default='', help="Prefix for saved plot filenames (e.g., 'my_run_').")
    parser.add_argument('--no-save', action='store_false', dest='save_plots', help='Do not save the plots to files.')
    parser.add_argument('--no-show', action='store_false', dest='show_plots', help='Do not display the plots interactively.')
    parser.set_defaults(save_plots=True, show_plots=True)

    parser.add_argument('--vector3d', action='append', nargs=5,
                        metavar=('NAME', 'X_COL', 'Y_COL', 'Z_COL', 'UNIT'),
                        help="Plot a 3D vector. Provide: Name, X_col, Y_col, Z_col, Unit. Can be used multiple times.")
    
    parser.add_argument('--quaternion', action='append', nargs='+',
                        metavar='QUAT_PARAMS',
                        help="Plot quaternion orientation. Provide 4 or 5 parameters: "
                             "[PLOT_NAME] W_COL X_COL Y_COL Z_COL. "
                             "PLOT_NAME is optional. If plotting multiple unnamed quaternions, they will be indexed.")
    
    parser.add_argument('--scalar', action='append', nargs=3,
                        metavar=('NAME', 'DATA_COL', 'UNIT'),
                        help="Plot a scalar value. Provide: Name, Data_col, Unit. Can be used multiple times.")

    args = parser.parse_args()

    if not os.path.exists(args.file_path):
        print(f"Error: Input file '{args.file_path}' not found.")
        return
    if not os.path.isfile(args.file_path):
        print(f"Error: '{args.file_path}' is not a valid file.")
        return
    
    output_directory = args.output_dir
    if output_directory is None: 
        output_directory = os.path.dirname(os.path.abspath(args.file_path))
        if not output_directory: 
            output_directory = "." 
        print(f"No --output-dir specified. Defaulting to input file's directory: {os.path.abspath(output_directory)}")

    try:
        df = pd.read_csv(args.file_path)
    except Exception as e:
        print(f"Error reading file '{args.file_path}': {e}")
        return
    if df.empty:
        print(f"Error: No data found in file '{args.file_path}'.")
        return
    if args.timestamp_col not in df.columns:
        print(f"Error: Timestamp column '{args.timestamp_col}' not found in the file. Available columns: {list(df.columns)}")
        return

    plotter_common_args = {
        "output_dir": output_directory, 
        "file_prefix": args.output_prefix,
        "save_plots": args.save_plots,
        "show_plots": args.show_plots
    }

    plot_requested = False
    if args.vector3d:
        plot_requested = True
        for name, x_col, y_col, z_col, unit in args.vector3d:
            plotter = Vector3DPlotter(df, args.timestamp_col, x_col, y_col, z_col, name, unit, **plotter_common_args)
            plotter.plot()
            
    if args.quaternion:
        plot_requested = True
        for i, quat_params in enumerate(args.quaternion):
            plot_name = f"Orientation_Quaternion_{i+1}" 
            if len(quat_params) == 5: 
                plot_name, w_col, x_col, y_col, z_col = quat_params
            elif len(quat_params) == 4: 
                w_col, x_col, y_col, z_col = quat_params
                if len(args.quaternion) == 1: 
                    plot_name = "Orientation_Quaternion" 
            else:
                print(f"Warning: Invalid number of parameters for --quaternion: {quat_params}. Expected 4 or 5. Skipping.")
                continue
            
            current_plotter_args = plotter_common_args.copy()
            plotter = OrientationQuaternionPlotter(df, args.timestamp_col, w_col, x_col, y_col, z_col,
                                                   plot_name=plot_name, **current_plotter_args)
            plotter.plot()

    if args.scalar:
        plot_requested = True
        for name, data_col, unit in args.scalar:
            plotter = ScalarDataPlotter(df, args.timestamp_col, data_col, name, unit, **plotter_common_args)
            plotter.plot()
            
    if not plot_requested:
        print("No plot types specified. Use --vector3d, --quaternion, or --scalar arguments to generate plots.")
        parser.print_help()

if __name__ == '__main__':
    main()