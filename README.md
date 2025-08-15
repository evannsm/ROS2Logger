# ROS2Logger

A custom ROS 2 logging utility that makes structured experiment logging painless.

## ✨ Features
1. **Custom log data types**
   - Append values like a list  
   - Supports scalar sequences and sequences of vectors
2. **Automatic shutdown hooks**
   - On node exit (normal or error), automatically collects all log-type variables from your node
3. **Column ordering**
   - Columns in the log file are sorted by:
     1. Your provided ordering integer
     2. Alphabetical order (for ties)
4. **Data directory creation**
   - Creates a `/data_analysis` folder alongside the ROS 2 node that invoked the logger
5. **Built-in analysis tools**
   - Populates `/data_analysis` with:
     - `DataAnalysis.ipynb` notebook containing:
       - RMSE computation
       - Plotting utilities
       - PDF export for Overleaf/LaTeX
     - `/data_analysis/log_files` subdirectory for logs

> **Note:**  
> By default, the logger requires `time`, `x`, `y`, `z`, and `yaw` to be present to ensure minimum dataset completeness for quadrotor experiments.  
> You can modify this requirement in the code.

---

## 🚀 Quick Start

1. **Create** a ROS 2 workspace with a `src/` directory.  
2. **Clone** this repo into `src/`:
   ```bash
   git clone git@github.com:evannsm/ROS2Logger.git
3. **Build** the workspace (symbolic links preferred):

4. Use the test_logger package for a quick and easy example of this system in action, and learn to operate this Logger package!
