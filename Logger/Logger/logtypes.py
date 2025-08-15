from dataclasses import dataclass, field
from typing import List, Iterable, Any, Tuple
import numpy as np

class ColumnarLog:
    """Any log that can expand into one or more (header, column) pairs."""
    name: str
    order: int
    def iter_columns(self) -> List[Tuple[str, np.ndarray, int, str]]:
        """Return [(header, (N,1) array, order, tie_key)]"""
        raise NotImplementedError

@dataclass
class LogType(ColumnarLog):
    """Scalar log: 1 column"""
    name: str
    order: int
    data: List[Any] = field(default_factory=list)

    def append(self, value: Any) -> None:
        self.data.append(value)

    def extend(self, values: Iterable[Any]) -> None:
        self.data.extend(values)

    def _as_col(self) -> np.ndarray:
        arr = np.asarray(self.data)
        if arr.ndim == 0:      arr = arr.reshape(1, 1)
        elif arr.ndim == 1:    arr = arr.reshape(-1, 1) # ensures N by 1 shape
        elif arr.ndim == 2:
            if arr.shape[0] == 1 and arr.shape[1] != 1:  # converts list from row -> col & ensures N by 1 shape
                arr = arr.T
            elif arr.shape[0] != 1 and arr.shape[1] != 1:
                raise ValueError(
                    f"Log '{self.name}' has vector data that is incompatible with LogType; "
                    f"use VectorLogType for multi-dimensional logs."
                )
        else:
            raise ValueError(
                f"Log '{self.name}' has vector data that is incompatible with LogType; "
                f"use VectorLogType for multi-dimensional logs."
            )
        return arr

    def iter_columns(self): # returns a tuple of (header, array, order, tie_key)
        # tie_key is the column header for alphabetical tie-breaking
        return [(self.name, self._as_col(), self.order, self.name)]

@dataclass
class VectorLogType(ColumnarLog):
    """
    Vector/structured log: k named columns.
    Example: VectorLogType("u", 30, ["force","moment_x","moment_y","moment_z"])
    Append with .append(force, mx, my, mz) or .append_dict(force=..., moment_x=...)
    """
    name: str
    order: int
    subnames: List[str]
    _rows: List[List[Any]] = field(default_factory=list)

    def append(self, *values: Any) -> None:
        if len(values) != len(self.subnames):
            raise ValueError(f"{self.name}.append expected {len(self.subnames)} values, got {len(values)}")
        self._rows.append(list(values))

    def append_dict(self, **kwargs: Any) -> None:
        row = [kwargs[s] for s in self.subnames]  # this will raise a KeyError if s not in kwargs
        self._rows.append(row)

    def iter_columns(self): # returns a tuple of (header, array, order, tie_key) for each subname
        if len(self._rows) == 0:
            # yield empty columns so padding logic can still work
            cols = []
            for sub in self.subnames:
                header = f"{self.name}_{sub}"
                cols.append((header, np.empty((0,1)), self.order, sub))
            return cols

        arr = np.asarray(self._rows)  # shape (N, k)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        if arr.shape[1] != len(self.subnames):
            raise ValueError(f"{self.name} has {arr.shape[1]} columns, expected {len(self.subnames)}")

        cols = []
        for idx, subname in enumerate(self.subnames):
            header = f"{self.name}_{subname}"     # e.g., "u_force"
            col = arr[:, idx].reshape(-1, 1)    # (N,1)
            # tie_key = subname (so columns with same order sort alphabetically by subname)
            cols.append((header, col, self.order, subname))
        return cols
