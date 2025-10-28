#!/usr/bin/env python3
"""
Generate synthetic test log files for ControllerComparison notebook.
"""

import numpy as np
import pandas as pd
from pathlib import Path

# Enum values from utilities.py
PLATFORM_ENUM = {
    'Simulation': 0,
    'Hardware': 1
}

CONTROLLER_ENUM = {
    'NR Standard': 0,
    'NR Enhanced': 1,
    'MPC': 2
}

TRAJECTORY_ENUM = {
    'Hover': 0,
    'Circle H': 1,
    'Circle V': 2,
    'Fig8 H': 3,
    'Fig8 VS': 4,
    'Fig8 VT': 5,
    'Triangle': 6,
    'Sawtooth': 7,
    'Helix': 8
}


def generate_circle_horizontal(t, radius=1.0, speed=0.5, noise_level=0.02):
    """Generate horizontal circle trajectory (XY plane)."""
    angle = speed * t
    x_ref = radius * np.cos(angle)
    y_ref = radius * np.sin(angle)
    z_ref = -1.0 * np.ones_like(t)  # NED frame
    yaw_ref = np.zeros_like(t)

    # Add noise to actual values
    x = x_ref + np.random.normal(0, noise_level, len(t))
    y = y_ref + np.random.normal(0, noise_level, len(t))
    z = z_ref + np.random.normal(0, noise_level, len(t))
    yaw = yaw_ref + np.random.normal(0, 0.01, len(t))

    return x, y, z, yaw, x_ref, y_ref, z_ref, yaw_ref


def generate_circle_vertical(t, radius=1.0, speed=0.5, noise_level=0.02):
    """Generate vertical circle trajectory (XZ plane)."""
    angle = speed * t
    x_ref = radius * np.cos(angle)
    y_ref = np.zeros_like(t)
    z_ref = -1.0 + radius * np.sin(angle)  # NED frame
    yaw_ref = np.zeros_like(t)

    # Add noise
    x = x_ref + np.random.normal(0, noise_level, len(t))
    y = y_ref + np.random.normal(0, noise_level * 0.5, len(t))
    z = z_ref + np.random.normal(0, noise_level, len(t))
    yaw = yaw_ref + np.random.normal(0, 0.01, len(t))

    return x, y, z, yaw, x_ref, y_ref, z_ref, yaw_ref


def generate_lemniscate_horizontal(t, scale=1.0, speed=0.3, noise_level=0.025):
    """Generate horizontal lemniscate/figure-8 (XY plane)."""
    angle = speed * t
    x_ref = scale * np.sin(angle)
    y_ref = scale * np.sin(angle) * np.cos(angle)
    z_ref = -1.0 * np.ones_like(t)
    yaw_ref = np.zeros_like(t)

    # Add noise
    x = x_ref + np.random.normal(0, noise_level, len(t))
    y = y_ref + np.random.normal(0, noise_level, len(t))
    z = z_ref + np.random.normal(0, noise_level, len(t))
    yaw = yaw_ref + np.random.normal(0, 0.01, len(t))

    return x, y, z, yaw, x_ref, y_ref, z_ref, yaw_ref


def generate_lemniscate_vertical_short(t, scale=0.8, speed=0.3, noise_level=0.025):
    """Generate vertical short lemniscate (XZ plane) - infinity symbol ∞ orientation."""
    angle = speed * t
    # Horizontal infinity in XZ plane: x varies like figure-8, z varies like sin*cos
    x_ref = scale * np.sin(angle)
    y_ref = np.zeros_like(t)
    z_ref = -1.0 + scale * 0.5 * np.sin(2 * angle)  # ∞ orientation (horizontal)
    yaw_ref = np.zeros_like(t)

    # Add noise
    x = x_ref + np.random.normal(0, noise_level, len(t))
    y = y_ref + np.random.normal(0, noise_level * 0.5, len(t))
    z = z_ref + np.random.normal(0, noise_level, len(t))
    yaw = yaw_ref + np.random.normal(0, 0.01, len(t))

    return x, y, z, yaw, x_ref, y_ref, z_ref, yaw_ref


def generate_lemniscate_vertical_tall(t, scale=1.2, speed=0.3, noise_level=0.03):
    """Generate vertical tall lemniscate (XZ plane) - number 8 orientation (vertical)."""
    angle = speed * t
    # Vertical figure-8 in XZ plane: z varies more than x
    x_ref = scale * 0.5 * np.sin(2 * angle)  # 8 orientation (vertical)
    y_ref = np.zeros_like(t)
    z_ref = -1.0 + scale * np.sin(angle)
    yaw_ref = np.zeros_like(t)

    # Add noise
    x = x_ref + np.random.normal(0, noise_level, len(t))
    y = y_ref + np.random.normal(0, noise_level * 0.5, len(t))
    z = z_ref + np.random.normal(0, noise_level, len(t))
    yaw = yaw_ref + np.random.normal(0, 0.01, len(t))

    return x, y, z, yaw, x_ref, y_ref, z_ref, yaw_ref


def generate_sawtooth(t, amplitude=1.0, period=10.0, noise_level=0.02):
    """Generate sawtooth trajectory (YZ plane)."""
    x_ref = np.zeros_like(t)
    y_ref = amplitude * (2 * (t % period) / period - 1)
    z_ref = -1.0 + amplitude * (2 * (t % period) / period - 1) * 0.5
    yaw_ref = np.zeros_like(t)

    # Add noise
    x = x_ref + np.random.normal(0, noise_level * 0.5, len(t))
    y = y_ref + np.random.normal(0, noise_level, len(t))
    z = z_ref + np.random.normal(0, noise_level, len(t))
    yaw = yaw_ref + np.random.normal(0, 0.01, len(t))

    return x, y, z, yaw, x_ref, y_ref, z_ref, yaw_ref


def generate_triangle(t, side_length=2.0, speed=0.2, noise_level=0.025):
    """Generate triangle trajectory (XY plane)."""
    # Triangle vertices
    vertices = np.array([
        [0, 0],
        [side_length, 0],
        [side_length/2, side_length * np.sqrt(3)/2],
        [0, 0]  # Back to start
    ])

    # Interpolate along edges
    total_perimeter = 3 * side_length
    position = (speed * t) % total_perimeter

    x_ref = np.zeros_like(t)
    y_ref = np.zeros_like(t)

    for i in range(len(t)):
        pos = position[i]
        if pos < side_length:
            # Edge 1: (0,0) to (L,0)
            alpha = pos / side_length
            x_ref[i] = vertices[0, 0] * (1 - alpha) + vertices[1, 0] * alpha
            y_ref[i] = vertices[0, 1] * (1 - alpha) + vertices[1, 1] * alpha
        elif pos < 2 * side_length:
            # Edge 2: (L,0) to (L/2, L*sqrt(3)/2)
            alpha = (pos - side_length) / side_length
            x_ref[i] = vertices[1, 0] * (1 - alpha) + vertices[2, 0] * alpha
            y_ref[i] = vertices[1, 1] * (1 - alpha) + vertices[2, 1] * alpha
        else:
            # Edge 3: (L/2, L*sqrt(3)/2) to (0,0)
            alpha = (pos - 2 * side_length) / side_length
            x_ref[i] = vertices[2, 0] * (1 - alpha) + vertices[3, 0] * alpha
            y_ref[i] = vertices[2, 1] * (1 - alpha) + vertices[3, 1] * alpha

    z_ref = -1.0 * np.ones_like(t)
    yaw_ref = np.zeros_like(t)

    # Add noise
    x = x_ref + np.random.normal(0, noise_level, len(t))
    y = y_ref + np.random.normal(0, noise_level, len(t))
    z = z_ref + np.random.normal(0, noise_level, len(t))
    yaw = yaw_ref + np.random.normal(0, 0.01, len(t))

    return x, y, z, yaw, x_ref, y_ref, z_ref, yaw_ref


def generate_log_file(platform_name, controller_name, trajectory_name,
                     duration=30.0, dt=0.1, output_dir='log_files'):
    """Generate a synthetic log file."""

    # Time array
    t = np.arange(0, duration, dt)
    n_samples = len(t)

    # Get trajectory data
    trajectory_generators = {
        'Circle H': generate_circle_horizontal,
        'Circle V': generate_circle_vertical,
        'Fig8 H': generate_lemniscate_horizontal,
        'Fig8 VS': generate_lemniscate_vertical_short,
        'Fig8 VT': generate_lemniscate_vertical_tall,
        'Sawtooth': generate_sawtooth,
        'Triangle': generate_triangle,
    }

    # Different noise levels for different controllers and platforms
    if controller_name == 'MPC':
        noise_scale = 0.8  # MPC typically has lower tracking error
    else:
        noise_scale = 1.0

    if platform_name == 'Hardware':
        noise_scale *= 1.5  # Hardware has more noise

    # Generate trajectory
    generator = trajectory_generators[trajectory_name]
    x, y, z, yaw, x_ref, y_ref, z_ref, yaw_ref = generator(t)

    # Scale noise if needed
    if noise_scale != 1.0:
        x = x_ref + (x - x_ref) * noise_scale
        y = y_ref + (y - y_ref) * noise_scale
        z = z_ref + (z - z_ref) * noise_scale

    # Generate velocity (numerical derivative)
    vx = np.gradient(x, dt)
    vy = np.gradient(y, dt)
    vz = np.gradient(z, dt)
    vx_ref = np.gradient(x_ref, dt)
    vy_ref = np.gradient(y_ref, dt)
    vz_ref = np.gradient(z_ref, dt)

    # Create DataFrame with all required columns
    df = pd.DataFrame({
        'time': t,
        'x': x,
        'y': y,
        'z': z,
        'yaw': yaw,
        'vx': vx,
        'vy': vy,
        'vz': vz,
        'x_ref': x_ref,
        'y_ref': y_ref,
        'z_ref': z_ref,
        'yaw_ref': yaw_ref,
        'vx_ref': vx_ref,
        'vy_ref': vy_ref,
        'vz_ref': vz_ref,
        'traj_time': t,
        'lookahead_time': 1.2,  # Constant lookahead
        'comp_time': np.random.uniform(0.0005, 0.002, n_samples),  # 0.5-2ms
        'platform': PLATFORM_ENUM[platform_name],
        'controller': CONTROLLER_ENUM[controller_name],
        'trajectory': TRAJECTORY_ENUM[trajectory_name],
        'traj_double': False,
        'traj_spin': False,
    })

    # Add PlotJuggler prefix to column names (as they appear in real data)
    rename_dict = {}
    for col in df.columns:
        if col != 'time':
            rename_dict[col] = f'/plotjuggler/logging/{col}'
    df.rename(columns=rename_dict, inplace=True)
    df.rename(columns={'time': '/plotjuggler/logging/__time'}, inplace=True)

    # Save to CSV
    output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)

    filename = f"{platform_name.lower()}_{controller_name.lower().replace(' ', '_')}_{trajectory_name.lower().replace(' ', '_')}.csv"
    filepath = output_dir / filename
    df.to_csv(filepath, index=False)
    print(f"Generated: {filepath}")

    return filepath


def main():
    """Generate all test log files."""

    platforms = ['Simulation', 'Hardware']
    controllers = ['NR Enhanced', 'MPC']
    trajectories = ['Circle H', 'Circle V', 'Fig8 H', 'Fig8 VS', 'Fig8 VT', 'Sawtooth', 'Triangle']

    print("Generating synthetic test log files...")
    print("=" * 60)

    for platform in platforms:
        for controller in controllers:
            for trajectory in trajectories:
                generate_log_file(platform, controller, trajectory)

    print("=" * 60)
    print(f"Generated {len(platforms) * len(controllers) * len(trajectories)} test files")
    print("\nFiles are ready in: log_files/")
    print("You can now run the ControllerComparison.ipynb notebook!")


if __name__ == '__main__':
    main()
