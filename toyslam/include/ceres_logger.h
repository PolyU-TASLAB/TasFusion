/*
Ceres Logger
A simple C++ class for logging optimization results and metrics from Ceres Solver.
This class provides methods to log optimized parameters, solver summaries, and metadata
to specified files. It ensures thread safety and allows for easy configuration of output formats.

Added by Xiangru, with help of Gemini
Date: 2025-05-06
*/
#pragma once // Use pragma once for header guard

#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <iomanip> // For std::setprecision, std::fixed
#include <ctime>   // For timestamp
#include <sstream> // For formatting timestamp
#include <stdexcept> // If exceptions are needed
#include <mutex>   // For thread safety if needed
#include <iostream> // For cerr error output
#include <sys/stat.h> // For file existence check

#include "ceres/ceres.h" // Required for ceres::Solver::Summary


// Helper function to check if a file exists
bool fileExists(const std::string& filename) {
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

/**
 * @class CeresLogger
 * @brief Logs Ceres Solver results and metrics to two separate, persistent files.
 *
 * This class manages two log files:
 * 1. Results File: Stores optimized parameters and metadata for each run.
 * 2. Metrics File: Stores Solver Summary (computation metrics) and metadata for each run.
 *
 * It supports logging static configuration once at the beginning and appending
 * records for each subsequent optimization run.
 */
class CeresLogger {
public:
    /**
     * @brief Default Constructor.
     * Initializes logger with empty filenames and default settings.
     * Filenames MUST be set later using the initialize() method before logging.
     */
    CeresLogger()
        : precision_(8),
          summary_set_(false)
    {
        // Filenames are default-initialized to empty strings
        // std::cout << "CeresLogger Default Constructed" << std::endl; // For debugging
    }
    /**
     * @brief Constructor. Initializes the logger with paths for the two log files.
     * @param results_filename Path for the file storing optimization results (parameters).
     * @param metrics_filename Path for the file storing computation metrics (Solver Summary).
     */
    explicit CeresLogger(std::string results_filename, std::string metrics_filename)
        : results_filename_(std::move(results_filename)),
          metrics_filename_(std::move(metrics_filename)),
          precision_(8), // Default floating-point precision for output
          summary_set_(false) {}

    // Default destructor is sufficient
    ~CeresLogger() = default;

    /**
     * @brief Initializes or re-initializes the logger with specific filenames.
     * MUST be called after default construction and before the first log() call
     * if the default constructor was used.
     * @param results_filename Path for the results log file (parameters).
     * @param metrics_filename Path for the metrics log file (summary).
     */
    void initialize(std::string results_filename, std::string metrics_filename) {
        std::lock_guard<std::mutex> lock(mtx_); // Lock for thread safety during initialization
        results_filename_ = std::move(results_filename);
        metrics_filename_ = std::move(metrics_filename);
        // Reset other states if re-initializing? Typically not needed if only called once.
        // metadata_.clear();
        // optimized_parameters_.clear();
        // summary_set_ = false;
        // std::cout << "CeresLogger Initialized with filenames" << std::endl; // For debugging
    }

    // Delete copy constructor and assignment operator to prevent accidental copying
    // If copying is needed, implement proper deep copy logic.
    CeresLogger(const CeresLogger&) = delete;
    CeresLogger& operator=(const CeresLogger&) = delete;

    // Allow move constructor and assignment (optional, but good practice)
    CeresLogger(CeresLogger&&) = default;
    CeresLogger& operator=(CeresLogger&&) = default;


    // --- Configuration and Data Addition Methods ---

    /**
     * @brief Sets the Solver Summary obtained after calling ceres::Solve.
     * This marks the logger as ready to log a full optimization run entry.
     * @param summary The summary object from ceres::Solve.
     */
    void setSummary(const ceres::Solver::Summary& summary) {
        std::lock_guard<std::mutex> lock(mtx_); // Lock for thread safety
        summary_ = summary; // Stores a copy
        summary_set_ = true;
    }

    /**
     * @brief Adds a block of optimized parameters (from std::vector) to be logged.
     * @param name Descriptive name for the parameter block (e.g., "State_0_Pose").
     * @param values Vector containing the optimized parameter values.
     */
    void addParameterBlock(const std::string& name, const std::vector<double>& values) {
        std::lock_guard<std::mutex> lock(mtx_);
        optimized_parameters_[name] = values;
    }

    /**
     * @brief Adds a block of optimized parameters (from a C-style array) to be logged.
     * @param name Descriptive name for the parameter block.
     * @param values Pointer to the start of the C-style array of doubles.
     * @param count Number of elements in the array.
     */
    void addParameterBlock(const std::string& name, const double* values, size_t count) {
        std::lock_guard<std::mutex> lock(mtx_);
        // Create a vector from the C-array data
        optimized_parameters_[name] = std::vector<double>(values, values + count);
    }

    /**
     * @brief Sets all optimized parameters at once, replacing any previously added for the current run.
     * @param parameters Map of parameter block names to their value vectors.
     */
    void setParameterBlocks(const std::map<std::string, std::vector<double>>& parameters) {
        std::lock_guard<std::mutex> lock(mtx_);
        optimized_parameters_ = parameters;
    }

    /**
     * @brief Adds a single metadata entry (key-value pair). Use this for both static and dynamic metadata.
     * @param key Metadata key (e.g., "Optimization Run", "Config: IMU Acc Noise").
     * @param value Metadata value as a string.
     */
    void addMetadata(const std::string& key, const std::string& value) {
        std::lock_guard<std::mutex> lock(mtx_);
        metadata_[key] = value;
    }

    /**
     * @brief Sets all metadata at once, replacing any previously added for the current logging cycle.
     * @param metadata Map of metadata keys to values.
     */
    void setMetadata(const std::map<std::string, std::string>& metadata) {
        std::lock_guard<std::mutex> lock(mtx_);
        metadata_ = metadata;
    }

    /**
     * @brief Sets the precision for logging floating-point numbers.
     * @param precision Number of digits after the decimal point.
     */
    void setPrecision(int precision) {
        std::lock_guard<std::mutex> lock(mtx_);
        precision_ = precision;
    }

    // --- Getter Methods ---

    /**
     * @brief Gets the filename used for storing results (parameters).
     * @return Constant reference to the results filename string.
     */
    const std::string& getResultsFilename() const { return results_filename_; }

    /**
     * @brief Gets the filename used for storing metrics (solver summary).
     * @return Constant reference to the metrics filename string.
     */
    const std::string& getMetricsFilename() const { return metrics_filename_; }


    // --- Core Logging Method ---

    /**
     * @brief Logs the current entry. Clears files on the first call (static config), appends otherwise.
     * Handles two modes:
     * 1. Initial Static Log: If setSummary() has NOT been called, assumes this call
     * is for static config. Opens files in OVERWRITE mode (std::ios::trunc),
     * writes static header and metadata.
     * 2. Regular Run Log: If setSummary() HAS been called, assumes this call is for
     * a specific run. Opens files in APPEND mode (std::ios::app), writes run separator,
     * dynamic metadata, summary (metrics file), and parameters (results file).
     * Resets internal state after logging.
     * @return true if writing was successful, false otherwise.
     */
    bool log() {
        std::lock_guard<std::mutex> lock(mtx_); // Lock for thread safety

        // Ensure filenames are set before proceeding
        if (results_filename_.empty() || metrics_filename_.empty()) {
            std::cerr << "Error: Logger filenames not set. Call initialize() before log()." << std::endl;
            return false;
        }

        // Determine if this is the initial call for static config
        bool is_initial_static_log = !summary_set_;

        // Basic validation for regular run logs
        if (!is_initial_static_log && !summary_set_) {
            std::cerr << "Error: log() called for a run entry, but Solver Summary was not set." << std::endl;
            return false;
        }
        // Prevent logging empty initial static config
        if (is_initial_static_log && metadata_.empty() && optimized_parameters_.empty()) {
             std::cerr << "Warning: log() called for initial static configuration, but no metadata or parameters were added." << std::endl;
             resetForNextRunInternal();
             return true; // Nothing to write, consider it success.
        }

        // ★★★ Determine file open mode based on whether it's the initial log ★★★
        std::ios_base::openmode results_mode = std::ios::out; // Default to overwrite for safety
        std::ios_base::openmode metrics_mode = std::ios::out; // Default to overwrite

        if (is_initial_static_log) {
            // For the very first write (static config), TRUNCATE the files
            results_mode = std::ios::out | std::ios::trunc;
            metrics_mode = std::ios::out | std::ios::trunc;
             // std::cout << "Opening logs in TRUNCATE mode for static config." << std::endl; // Debug
        } else {
            // For subsequent writes (runs), APPEND to the files
            results_mode = std::ios::app;
            metrics_mode = std::ios::app;
             // std::cout << "Opening logs in APPEND mode for run entry." << std::endl; // Debug
        }

        // ★★★ Ensure files exist before opening ★★★
        if (!fileExists(results_filename_)) {
            std::ofstream temp_file(results_filename_);
            temp_file.close();
        }
        if (!fileExists(metrics_filename_)) {
            std::ofstream temp_file(metrics_filename_);
            temp_file.close();
        }

        // Open files using the determined mode
        std::ofstream results_file(results_filename_, results_mode);
        std::ofstream metrics_file(metrics_filename_, metrics_mode);

        // Check if files opened successfully
        if (!results_file.is_open()) {
            std::cerr << "Error: Could not open results log file ("
                      << (is_initial_static_log ? "overwrite" : "append")
                      << "): " << results_filename_ << std::endl;
            if (metrics_file.is_open()) metrics_file.close();
            resetForNextRunInternal();
            return false;
        }
        if (!metrics_file.is_open()) {
            std::cerr << "Error: Could not open metrics log file ("
                      << (is_initial_static_log ? "overwrite" : "append")
                      << "): " << metrics_filename_ << std::endl;
            results_file.close();
            resetForNextRunInternal();
            return false;
        }

        // Apply formatting settings
        results_file << std::fixed << std::setprecision(precision_);
        metrics_file << std::fixed << std::setprecision(precision_);

        // --- Write Header/Separator ---
        if (is_initial_static_log) {
            writeStaticHeader(results_file);
            writeStaticHeader(metrics_file);
        } else {
            writeRunSeparator(results_file);
            writeRunSeparator(metrics_file);
        }

        // --- Write Metadata Section ---
        writeMetadataSection(results_file);
        writeMetadataSection(metrics_file);

        // --- Write Specific Sections (Only for dynamic run logs) ---
        if (!is_initial_static_log) {
            writeSummarySection(metrics_file);
            writeParametersSection(results_file);
        }

        // --- Add extra spacing & Close files ---
        results_file << "\n" << std::endl;
        metrics_file << "\n" << std::endl;
        results_file.close();
        metrics_file.close();

        bool success = results_file.good() && metrics_file.good();

        // --- Reset internal state ---
        resetForNextRunInternal();

        return success;
    }


private:
    // --- Member Variables ---
    std::string results_filename_; ///< Path to the results log file (parameters).
    std::string metrics_filename_; ///< Path to the metrics log file (summary).
    ceres::Solver::Summary summary_; ///< Stores the latest solver summary.
    std::map<std::string, std::vector<double>> optimized_parameters_; ///< Stores parameters for the current log cycle.
    std::map<std::string, std::string> metadata_; ///< Stores metadata for the current log cycle.
    int precision_; ///< Floating point precision for output.
    bool summary_set_; ///< Flag indicating if summary has been set for the current run cycle.
    std::mutex mtx_; ///< Mutex for thread safety.

    // --- Private Helper Methods ---

    /**
     * @brief Writes a standard separator and header for a dynamic optimization run entry.
     * Uses "Optimization Run" metadata key to identify the run number.
     */
    void writeRunSeparator(std::ofstream& stream) const {
        stream << "======================================================================\n";
        std::string run_number_str = "(Run number metadata missing)";
        auto it = metadata_.find("Optimization Run");
        if (it != metadata_.end()) {
            run_number_str = it->second;
        } else {
             // Warn if the crucial run number is missing for a dynamic entry
            //  std::cerr << "Warning: 'Optimization Run' metadata missing for this log entry." << std::endl;
        }

        // Get current timestamp for this specific log entry
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::stringstream timestamp_ss;
        timestamp_ss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S %Z"); // e.g., 2025-05-06 12:43:39 PDT

        stream << "--- Optimization Run: " << run_number_str
               << " (Logged at: " << timestamp_ss.str() << ") ---\n";
        stream << "======================================================================\n\n";
    }

    /**
     * @brief Writes a distinct header for the initial static configuration block.
     */
    void writeStaticHeader(std::ofstream& stream) const {
         stream << "//////////////////////////////////////////////////////////////////////\n";
         stream << "//                  STATIC CONFIGURATION PARAMETERS                   //\n";
         auto t = std::time(nullptr); // Timestamp when static config was logged
         auto tm = *std::localtime(&t);
         std::stringstream timestamp_ss;
         timestamp_ss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S %Z");
         stream << "// Logged at: " << timestamp_ss.str() << "                            //\n";
         stream << "//////////////////////////////////////////////////////////////////////\n\n";
    }

    /**
     * @brief Writes the stored metadata key-value pairs to the given stream.
     */
    void writeMetadataSection(std::ofstream& stream) const {
        if (!metadata_.empty()) {
            stream << "[Metadata]" << std::endl;
            // Optionally sort by key for consistent order in the log file
            std::map<std::string, std::string> sorted_metadata = metadata_;
            for (const auto& pair : sorted_metadata) {
                stream << pair.first << ": " << pair.second << std::endl;
            }
            stream << std::endl;
        }
    }

    /**
     * @brief Writes the detailed Ceres Solver Summary information to the given stream.
     * This represents the computation metrics.
     */
    void writeSummarySection(std::ofstream& stream) const {
        stream << "[Solver Summary & Metrics]" << std::endl;
        stream << "Termination Type: " << ceres::TerminationTypeToString(summary_.termination_type) << std::endl;
        stream << "Message: " << summary_.message << std::endl;
        stream << "Total Time (s): " << summary_.total_time_in_seconds << std::endl;
        stream << std::endl;
        stream << "Cost:" << std::endl;
        stream << "  Initial: " << summary_.initial_cost << std::endl;
        stream << "  Final: " << summary_.final_cost << std::endl;
        stream << "  Change: " << summary_.initial_cost - summary_.final_cost << std::endl;
        stream << std::endl;
        stream << "Iterations:" << std::endl;
        // Use num_successful_steps + num_unsuccessful_steps for minimizer iterations
        stream << "  Minimizer: " << summary_.num_successful_steps + summary_.num_unsuccessful_steps << " (" << summary_.iterations.size() << " recorded)" << std::endl;
        // ★★★ Corrected Line 1 ★★★
        stream << "  Linear Solves: " << summary_.num_linear_solves << std::endl; // Changed label and member name
        stream << std::endl;
        stream << "Evaluations:" << std::endl;
        // ★★★ Corrected Line 2 ★★★
        stream << "  Residual Evaluations: " << summary_.num_residual_evaluations << std::endl; // Changed label and member name
        // ★★★ Corrected Line 3 ★★★
        stream << "  Jacobian Evaluations: " << summary_.num_jacobian_evaluations << std::endl; // Changed label and member name
        stream << std::endl;
        stream << "Time Breakdown (s):" << std::endl;
        stream << "  Preprocessor: " << summary_.preprocessor_time_in_seconds << std::endl;
        stream << "  Minimizer: " << summary_.minimizer_time_in_seconds << std::endl;
        // Ensure these time breakdowns are available in your Ceres version as well
        stream << "    - Residual Evaluation: " << summary_.residual_evaluation_time_in_seconds << std::endl;
        stream << "    - Jacobian Evaluation: " << summary_.jacobian_evaluation_time_in_seconds << std::endl;
        stream << "    - Linear Solver: " << summary_.linear_solver_time_in_seconds << std::endl;
        stream << "  Postprocessor: " << summary_.postprocessor_time_in_seconds << std::endl;
        stream << std::endl;
        stream << "Linear Solver Type Used: " << ceres::LinearSolverTypeToString(summary_.linear_solver_type_used) << std::endl;
        stream << std::endl;
    }

    /**
     * @brief Writes the stored optimized parameter blocks to the given stream.
     * This represents the optimization results.
     */
    void writeParametersSection(std::ofstream& stream) const {
        // (Implementation is identical to the previous version - no changes needed here)
        stream << "[Optimized Parameters]" << std::endl;
        if (optimized_parameters_.empty()) {
            stream << "No optimized parameters provided or added for this run." << std::endl;
        } else {
             // Optionally sort by key for consistent order
            std::map<std::string, std::vector<double>> sorted_params = optimized_parameters_;
            for (const auto& pair : sorted_params) {
                stream << pair.first << ":" << std::endl;
                stream << "  Values [";
                for (size_t i = 0; i < pair.second.size(); ++i) {
                    stream << pair.second[i] << (i == pair.second.size() - 1 ? "" : ", ");
                }
                stream << "]" << std::endl;
                stream << "  Dimension: " << pair.second.size() << std::endl;
            }
        }
        stream << std::endl;
    }

    /**
     * @brief Resets the internal state after logging, preparing for the next run cycle.
     * Clears stored metadata, parameters, and the summary flag.
     * (Internal helper, assumes mutex is already held by the caller - log()).
     */
    void resetForNextRunInternal() {
        metadata_.clear();
        optimized_parameters_.clear();
        summary_set_ = false;
        // summary_ object content will be overwritten by the next setSummary call
    }
}; // End of CeresLogger class