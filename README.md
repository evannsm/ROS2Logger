# ros2_logger

A ROS 2 Python library for structured experiment data logging. Automatically discovers log variables from your node at shutdown, writes structured CSVs, and generates Jupyter analysis notebooks.

## Key Features

- **Custom log data types** — `ColumnarLog` provides append-like list behavior for both scalar and vector sequences
- **Automatic shutdown hooks** — on node exit (normal or error), collects all log-type variables and writes them to CSV
- **Column ordering** — log columns are sorted first by a user-provided ordering integer, then alphabetically
- **Auto directory creation** — creates a `data_analysis/` folder alongside the ROS 2 node that invoked the logger
- **Built-in analysis tools** — populates `data_analysis/` with:
  - `DataAnalysis.ipynb` — full analysis and plotting notebook
  - `SingleLogAnalysis.ipynb` — single log file analysis
  - `ControllerComparison.ipynb` — compare runs across controllers
  - `utilities.py` — RMSE computation, plotting utilities, PDF export for Overleaf/LaTeX
  - `log_files/` — subdirectory for CSV log files

> **Note:** By default the logger requires `time`, `x`, `y`, `z`, and `yaw` columns to be present, ensuring minimum dataset completeness for quadrotor experiments. This can be modified in the source.

## Quick Start

1. **Clone** into your ROS 2 workspace `src/` directory:
   ```bash
<<<<<<< HEAD
<<<<<<< Updated upstream
   git clone git@github.com:evannsmc/ROS2Logger.git ros2_logger
=======
   git clone git@github.com:evannsmc/ROS2Logger.git
>>>>>>> Stashed changes
=======
   git clone git@github.com:evannsmc/ROS2Logger.git
>>>>>>> 82534d9 (update GitHub username from evannsm to evannsmc)
   ```
2. **Build** the workspace:
   ```bash
   colcon build --symlink-install
   ```
<<<<<<< HEAD
3. **Try the example** — use the [`test_logger`](https://github.com/evannsmc/test_logger) package for a working example.

## Usage

```python
from ros2_logger.logger import Logger
from ros2_logger.logtypes import ColumnarLog

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.logger = Logger(self)

        # Define log columns (name, ordering_int)
        self.time_log = ColumnarLog("time", 0)
        self.x_log    = ColumnarLog("x", 1)
        self.y_log    = ColumnarLog("y", 2)

    def control_loop(self):
        self.time_log.append(self.get_clock().now())
        self.x_log.append(state.x)
        self.y_log.append(state.y)
```

On shutdown, the logger automatically discovers all `ColumnarLog` attributes, writes them as columns to a CSV, and generates analysis notebooks in `data_analysis/`.

## Package Structure

```
ros2_logger/
├── ros2_logger/
│   ├── logger.py                    # Main Logger class
│   ├── logtypes.py                  # ColumnarLog and related types
│   ├── shutdown_helpers.py          # Shutdown hook management
│   └── analysis_helpers_move/
│       ├── utilities.py             # RMSE, plotting, PDF export
│       └── generate_test_data.py    # Test data generation
├── package.xml
└── setup.py
```

## License

MIT
=======
4. **Use** my [`test_logger`](https://github.com/evannsmc/test_logger) package for a quick and easy example of this system in action, and learn to operate this Logger package!
>>>>>>> 82534d9 (update GitHub username from evannsm to evannsmc)
