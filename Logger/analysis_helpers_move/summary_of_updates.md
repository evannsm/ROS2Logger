# Controller Comparison Notebook - Complete Summary

## ✅ Overview

Created a comprehensive data analysis notebook for comparing NR/NR Enhanced controllers against NMPC across different trajectories, with separate visualization for Simulation and Hardware platforms.

---

## 📊 Key Features

### 1. **Trajectory Corrections**
- **Lemniscate A (Fig8 H)**: Horizontal figure-8 in XY plane ✓
- **Lemniscate B (Fig8 VS)**: Infinity symbol (∞) orientation in XZ plane ✓
  - X varies 1.6m, Z varies 0.8m (horizontal infinity)
  - Formula: `z = -1.0 + scale * 0.5 * sin(2*angle)`
- **Lemniscate C (Fig8 VT)**: Number 8 orientation in XZ plane ✓
  - Z varies 2.4m, X varies 1.2m (vertical figure-8)
  - Formula: `x = scale * 0.5 * sin(2*angle)`, `z = -1.0 + scale * sin(angle)`

### 2. **Automatic Axis Labels**
Each subfigure shows proper labels based on `detect_trajectory_plane()`:
- **Circle A, Lemniscate A, Triangle** → x [m], y [m] (XY plane)
- **Circle B, Lemniscate B, Lemniscate C** → x [m], z [m] (XZ plane)
- **Sawtooth** → y [m], z [m] (YZ plane)

The plane is automatically detected by analyzing variance in reference trajectory dimensions.

### 3. **Plot Styling**
- ✓ White background (default matplotlib style)
- ✓ Red solid lines for actual trajectories
- ✓ Blue dashed lines for reference trajectories
- ✓ No legends in subplots (cleaner appearance)
- ✓ DejaVu Sans font (with EB Garamond fallback)
- ✓ Configurable gridlines (on/off toggle)
- ✓ Font sizes: Title=14, Ticks=10 (configurable)

### 4. **Subfigure Labels**
Format: `a) NR Enhanced: Circle A`, `b) NR Enhanced: Circle B`, etc.
- Top row (a-g): NR Enhanced (or NR Standard)
- Bottom row (h-n): NMPC
- Matched trajectories in each column for easy comparison

### 5. **Figure Dimensions**
- **Width**: 3.5" per trajectory × 7 trajectories = 24.5" total
- **Height**: 10" (increased for journal format)
- **Aspect ratio**: Optimized for two-column journal papers
- **Output**: 300 DPI PDF files

### 6. **Configuration Options**
All settings in one place at the top of the notebook:
```python
NR_CONTROLLER_CHOICE = 'NR Enhanced'  # or 'NR Standard'
subfigure_title_fontsize = 14
tick_size = 10
show_gridlines = True  # Toggle gridlines on/off
```

---

## 📁 File Structure

### Created/Updated Files
1. **`ControllerComparison.ipynb`** - Main comparison notebook
2. **`generate_test_data.py`** - Test data generator (updated format)
3. **`utilities.py`** - Helper functions (updated for string metadata)
4. **`log_files/*.csv`** - 28 synthetic test log files

### Generated Outputs
- `output/simulation_comparison.pdf` - Simulation results (2×7 grid)
- `output/hardware_comparison.pdf` - Hardware results (2×7 grid)
- `output/controller_comparison_rmse.csv` - RMSE summary table

---

## 🔧 Data Format

### Metadata Handling
The system now handles **both** string and numeric metadata formats:

**String format (like real data):**
```csv
time,x,y,z,yaw,platform,controller,trajectory,traj_double,traj_spin,...
0.17,0.02,-0.003,-0.81,0.001,sim,nr_enhanced,circle_horz,NoSpd,NoSpin,...
```

**Numeric format (legacy):**
```csv
time,x,y,z,yaw,platform,controller,trajectory,traj_double,traj_spin,...
0.17,0.02,-0.003,-0.81,0.001,0,1,1,False,False,...
```

### String Mappings
```python
# Platform
'sim' → 'Simulation'
'hw' → 'Hardware'

# Controller
'nr' → 'NR Standard'
'nr_enhanced' → 'NR Enhanced'
'mpc' → 'MPC'

# Trajectory
'circle_horz' → 'Circle H'
'circle_vert' → 'Circle V'
'fig8_horz' → 'Fig8 H'
'fig8_vert_short' → 'Fig8 VS'
'fig8_vert_tall' → 'Fig8 VT'
'sawtooth' → 'Sawtooth'
'triangle' → 'Triangle'

# Modifiers
'DblSpd' / '2x' → Double speed
'Spin' → Yawing trajectory
```

### Data Columns (34 total)
Matching real data structure:
- Position: `x, y, z, yaw`
- Velocity: `vx, vy, vz`
- References: `x_ref, y_ref, z_ref, yaw_ref, vx_ref, vy_ref, vz_ref`
- Body rates: `p, q, r`
- Control: `throttle_input, p_input, q_input, r_input`
- Safety: `cbf_v_p, cbf_v_q, cbf_v_r, cbf_v_throttle`
- Metadata: `platform, controller, trajectory, traj_double, traj_spin`
- Timing: `time, traj_time, lookahead_time, comp_time`

---

## 🎯 Included Trajectories

Only 7 trajectories are plotted (as specified):
1. **Circle A** - Horizontal Circle (XY plane)
2. **Circle B** - Vertical Circle (XZ plane)
3. **Lemniscate A** - Horizontal Figure-8 (XY plane)
4. **Lemniscate B** - Infinity ∞ orientation (XZ plane)
5. **Lemniscate C** - Number 8 orientation (XZ plane)
6. **Sawtooth** - Linear ramp (YZ plane)
7. **Triangle** - Triangular path (XY plane)

---

## 🧪 Test Data Generation

### Features
- **28 files** covering all combinations (2 platforms × 2 controllers × 7 trajectories)
- **Realistic noise**: MPC ~20% lower tracking error, Hardware ~50% more noise
- **300 samples per file** (30 seconds at 10 Hz)
- **Proper format**: Matches real data structure exactly

### Generation
```bash
python generate_test_data.py
```

### Validation
All test files include:
- Correct string metadata format
- 34 columns matching real data
- Proper trajectory shapes (circles, lemniscates, etc.)
- Body rates, control inputs, CBF values
- Realistic noise and tracking errors

---

## 🚀 Usage

### 1. Basic Usage
1. Place log files in `log_files/` directory
2. Open `ControllerComparison.ipynb`
3. Set `NR_CONTROLLER_CHOICE` in config cell
4. Run all cells
5. PDFs saved to `output/` directory

### 2. Configuration
Adjust settings in the configuration cell:
```python
NR_CONTROLLER_CHOICE = 'NR Enhanced'  # Choose controller
subfigure_title_fontsize = 14         # Title font size
tick_size = 10                        # Tick label size
show_gridlines = True                 # Show/hide gridlines
```

### 3. Font Setup
If EB Garamond font issues occur:
```bash
rm -rf ~/.cache/matplotlib  # Clear font cache
```

The notebook will fallback to DejaVu Sans automatically.

---

## 🔄 Automatic Features

### Module Reloading
The notebook automatically reloads `utilities.py` to pick up any changes:
```python
import importlib
if 'utilities' in sys.modules:
    importlib.reload(sys.modules['utilities'])
```

### Plane Detection
Automatically detects which 2D plane to plot based on reference trajectory variance:
- Calculates variance in x, y, z dimensions
- Selects two dimensions with highest variance
- Returns 'xy', 'xz', or 'yz'
- Sets appropriate axis labels

### Grid Control
Gridlines are controlled after plotting:
```python
if show_grid:
    ax.grid(True, alpha=0.3)
else:
    ax.grid(False)
```
This overrides the default from `plot_trajectory_2d()`.

---

## 📝 Notes

### Important Reminders
1. **Restart kernel** after updating `utilities.py`
2. **Close notebook** before making changes (or revert when prompted)
3. **Font cache** may need clearing after font changes
4. **Test data** format now matches real data exactly

### Compatibility
- Works with both string and numeric metadata formats
- Handles PlotJuggler prefixed and unprefixed column names
- Supports both old and new trajectory naming conventions

### Performance
- Automatic lookahead alignment (shifts reference backward 1.2s)
- NED to ENU conversion (flips z-axis)
- RMSE calculated after alignment for accuracy

---

## 🎨 Visual Design

### Publication Quality
- Clean, professional appearance
- No overall title (cleaner for journals)
- Subfigure labels in regular weight, serif font
- White background suitable for print
- High resolution (300 DPI)

### Layout
```
┌────────┬────────┬────────┬────────┬────────┬────────┬────────┐
│ a) NR  │ b) NR  │ c) NR  │ d) NR  │ e) NR  │ f) NR  │ g) NR  │
│Circle A│Circle B│Lemnis A│Lemnis B│Lemnis C│Sawtooth│Triangle│
├────────┼────────┼────────┼────────┼────────┼────────┼────────┤
│ h) NMPC│ i) NMPC│ j) NMPC│ k) NMPC│ l) NMPC│ m) NMPC│ n) NMPC│
│Circle A│Circle B│Lemnis A│Lemnis B│Lemnis C│Sawtooth│Triangle│
└────────┴────────┴────────┴────────┴────────┴────────┴────────┘
```

---

## 🛠️ Troubleshooting

### Problem: "No simulation data available"
**Solution:** Restart kernel to reload updated `utilities.py`

### Problem: Font warnings
**Solution:** Clear matplotlib cache: `rm -rf ~/.cache/matplotlib`

### Problem: Gridlines still showing when disabled
**Solution:** Grid control happens after plotting, should work correctly now

### Problem: Metadata not detected
**Solution:** Ensure kernel restarted and utilities module reloaded

---

## ✨ Summary

This notebook provides a complete, publication-ready solution for controller comparison:
- ✅ Automatic trajectory plane detection
- ✅ Matched trajectory layouts for easy comparison
- ✅ Configurable styling (fonts, gridlines, sizes)
- ✅ Both simulation and hardware sections
- ✅ High-quality PDF outputs for journals
- ✅ Compatible with multiple data formats
- ✅ Test data generator included
- ✅ RMSE summary tables

Ready for immediate use with your real experimental data!
