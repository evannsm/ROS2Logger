# logger.py
import csv, os
import shutil
import numpy as np
from typing import Optional, Union
from .logtypes import LogType, VectorLogType

class Logger:
    def __init__(self, filename: str, base_dir: str):

        if filename.endswith(os.sep):
            print(f"[logger][WARN] Filename '{filename}' ends with a slash (as though it were a directory).\n Assuming this is a type and removing trailing slash.")
            filename = filename.rstrip(os.sep)
        self.filename = filename

        base_path = os.path.join(base_dir, 'data_analysis/log_files/') # Append the 'data_analysis' folder to the path
        parts = base_path.split(os.sep) # Split the path into components
        parts = ["src" if part in ("build","install") else part for part in parts] # Replace 'build' with 'src' if it exists in the path
        base_path = os.sep.join(parts) # Reconstruct the new path

        self.full_path = os.path.join(base_path, self.filename) # Combine the base path with the filename
        os.makedirs(os.path.dirname(self.full_path), exist_ok=True) # Ensure the directory exists, and creates it if it doesn't

        print(f"[logger] Writing to: {self.full_path}")


        self.data_analysis_notebook = 'DataAnalysis.ipynb'
        self.data_utilities = 'utilities.py'
        source_path = os.path.join(os.path.dirname(os.path.abspath(__file__)))
        destination_path = os.path.dirname(os.path.dirname(base_path))
        self.copy_file(source_path, destination_path, self.data_analysis_notebook)
        self.copy_file(source_path, destination_path, self.data_utilities)


    def copy_file(self, source_path: str, destination_path: str, file_to_copy: str):
        source_path = os.path.join(source_path, file_to_copy)
        if not os.path.isfile(os.path.join(destination_path, file_to_copy)):
            try:
                shutil.copy(source_path, destination_path)
                print(f"'{os.path.basename(source_path)}' has been successfully copied to '{destination_path}'")
            except FileNotFoundError:
                print(f"Error: Source file '{source_path}' not found.")
            except IsADirectoryError:
                print(f"Error: Destination '{destination_path}' is a directory, but a file path was expected.")
            except PermissionError:
                print(f"Error: Permission denied to copy to '{destination_path}'.")
            except Exception as e:
                print(f"An unexpected error occurred: {e}")
            finally:
                return

        print(f"File {file_to_copy} already exists!")


    def _discover_logs(self, obj):
        return [
            val for _, val in vars(obj).items() if isinstance(val, (LogType, VectorLogType))
        ]

    def log(self, source):
        log_objects = self._discover_logs(source)
        if not log_objects:
            print("[logger] No LogType or VectorLogTypes found; nothing to write.")
            return

        required = ["time", "x", "y", "z"]
        have = {log.name for log in log_objects}
        missing = [r for r in required if r not in have]
        if missing:
            raise ValueError(f"[logger] Missing required logs: {missing}")

        # Expand all logs into flat columns
        flat_cols = []  # list of (header, col(N,1), order, tie_key, required_rank)
        def req_rank_from_header(header: str) -> int:
            # required logs are scalar with exact names; vector headers won't match these
            return required.index(header) if header in required else 999

        for log in log_objects:
            for header, col, order, tie_key in log.iter_columns():
                if col.ndim != 2 or col.shape[1] != 1:
                    raise ValueError(f"[logger] Column '{header}' must be (N,1); got {col.shape}")
                flat_cols.append((header, col, order, tie_key, req_rank_from_header(header)))

        # Sort: required first (fixed), then by (order, header name tie-break via tie_key then header)
        flat_cols.sort(key=lambda x: (x[4], x[2], x[3], x[0]))

        headers   = [h for (h, *_ ) in flat_cols]
        arrays    = [c for (_h, c, *_ ) in flat_cols]
        lengths   = [a.shape[0] for a in arrays]
        max_len   = max(lengths) if lengths else 0


        def pad_nan(a, n):
            if a.shape[0] == n: return a
            return np.vstack([a, np.full((n - a.shape[0], 1), np.nan)])

        arrays = [pad_nan(a, max_len) for a in arrays]
        data = np.hstack(arrays) if arrays else np.empty((0,0))

        with open(self.full_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(headers)
            for r in range(max_len):
                w.writerow([v.item() if hasattr(v, "item") else v for v in data[r, :]])

        print(f"[logger] Wrote {max_len} rows × {len(headers)} cols to {self.full_path}")
