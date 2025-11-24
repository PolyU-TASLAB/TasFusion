#!/bin/bash

# Shell script to plot IMU biases using the oop_plotter.py Python script.
# This script now relies on oop_plotter.py's default behavior to save plots
# in the same directory as the input data file if --output-dir is not specified.

# --- Determine the directory where this script is located ---
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# --- Configuration ---
PYTHON_SCRIPT_NAME="oop_plotter.py" # Name of your generalized Python plotting script
PYTHON_SCRIPT_FULL_PATH="${SCRIPT_DIR}/${PYTHON_SCRIPT_NAME}"

# Default path to the bias data file.
# This path is relative to where you RUN this shell script from (Current Working Directory),
# OR it should be an absolute path.
DEFAULT_DATA_FILE_PATH="src/data/bias.txt" # src/data/bias.txt

# Column names and plot parameters for IMU biases
TIMESTAMP_COLUMN="timestamp"

ACC_BIAS_PLOT_NAME="AccelerometerBias"
ACC_BIAS_X_COL="acc_bias_x"
ACC_BIAS_Y_COL="acc_bias_y"
ACC_BIAS_Z_COL="acc_bias_z"
ACC_BIAS_UNIT="m/s^2"

GYRO_BIAS_PLOT_NAME="GyroscopeBias"
GYRO_BIAS_X_COL="gyro_bias_x"   # Check for typos here
GYRO_BIAS_Y_COL="gyro_bias_y"   # Check for typos here
GYRO_BIAS_Z_COL="gyro_bias_z"   # Check for typos here
GYRO_BIAS_UNIT="rad/s"

# Output prefix for these specific bias plots.
# The Python script will handle the output directory by default.
OUTPUT_PREFIX="imu_"

# --- Determine the data file path to use ---
TARGET_DATA_FILE_PATH=""

if [ -z "$1" ]; then
  # No command-line argument provided to this shell script, use the default data path
  echo "No data file path provided to shell script. Using default: ${DEFAULT_DATA_FILE_PATH}"
  TARGET_DATA_FILE_PATH="${DEFAULT_DATA_FILE_PATH}"
else
  # Command-line argument provided, use it as the data file path
  echo "Data file path provided to shell script: $1"
  TARGET_DATA_FILE_PATH="$1"
fi

# --- Sanity Checks ---
if ! command -v python3 &> /dev/null; then
    echo "Error: python3 command could not be found. Please ensure it's installed and in your PATH."
    exit 1
fi

if [ ! -f "${PYTHON_SCRIPT_FULL_PATH}" ]; then
    echo "Error: Python script '${PYTHON_SCRIPT_FULL_PATH}' not found."
    echo "Please ensure that '${PYTHON_SCRIPT_NAME}' is in the same directory as this shell script ('${SCRIPT_DIR}')."
    exit 1
fi

# --- Execute the Python script with arguments for plotting biases ---
echo "Executing Python plotter for IMU biases..."
echo "  Data file: \"${TARGET_DATA_FILE_PATH}\""
echo "  Output prefix: \"${OUTPUT_PREFIX}\""
echo "  (Plots will be saved in the same directory as the data file by default, unless Python script specifies otherwise or --output-dir is added below)"

python3 "${PYTHON_SCRIPT_FULL_PATH}" \
    "${TARGET_DATA_FILE_PATH}" \
    --timestamp-col "${TIMESTAMP_COLUMN}" \
    --vector3d "${ACC_BIAS_PLOT_NAME}" "${ACC_BIAS_X_COL}" "${ACC_BIAS_Y_COL}" "${ACC_BIAS_Z_COL}" "${ACC_UNIT}" \
    --vector3d "${GYRO_BIAS_PLOT_NAME}" "${GYRO_BIAS_X_COL}" "${GYRO_BIAS_Y_COL}" "${GYRO_BIAS_Z_COL}" "${GYRO_BIAS_UNIT}" \
    --output-prefix "${OUTPUT_PREFIX}"
    # Note: --output-dir is intentionally omitted here to use the Python script's new default.
    # You can add it back if you want this bash script to force a specific output directory:
    # --output-dir "my_specific_bias_plots_dir"
    # Or, you could make the output directory an optional second argument to this bash script:
    # TARGET_OUTPUT_DIR="${2:-}" # Use second arg if present
    # if [ -n "${TARGET_OUTPUT_DIR}" ]; then
    #   PYTHON_ARGS_EXTRA="--output-dir \"${TARGET_OUTPUT_DIR}\""
    # fi
    # ... and then append $PYTHON_ARGS_EXTRA to the python3 command.

# Check the exit status of the Python script
EXIT_STATUS=$?
if [ ${EXIT_STATUS} -ne 0 ]; then
  echo "Python script exited with error status ${EXIT_STATUS}."
else
  echo "Python script executed successfully."
  echo "Plots should be saved in the same directory as '${TARGET_DATA_FILE_PATH}' with prefix '${OUTPUT_PREFIX}' (if saving was enabled)."
fi

exit ${EXIT_STATUS}