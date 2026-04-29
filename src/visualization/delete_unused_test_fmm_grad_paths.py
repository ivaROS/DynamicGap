#!/usr/bin/env python3
"""Tests for FMM bilinear gradient descent path method.

Creates synthetic obstacle grids (U-shape corridor, zigzag channel, narrow gap),
runs get_fmm_path_bilinear (bilinear-interpolated gradient with obstacle repulsion
and wall-collision prevention), and saves diagnostic plots showing the FMM distance
field, gradient arrows, and path overlaid.

Usage:
    python3 -m nav_tuner.tests.test_fmm_grad_paths [--output-dir DIR]

Plots are saved to the output directory (default: /tmp/fmm_path_tests/).
"""

import argparse
import os
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np

from fmm_grad_utils import (
    _fmm_obstacle_aware_gradient,
    compute_fmm_distance_map,
    compute_obstacle_fmm_repulsive_gradient,
    compute_obstacle_repulsive_gradient,
    get_fmm_path_bilinear,
)

# ---------------------------------------------------------------------------
# Grid builders
# ---------------------------------------------------------------------------

def _make_u_shape_grid(height: int = 40, width: int = 40) -> Tuple[np.ndarray, Tuple[int,int], Tuple[int,int]]:
    """U-shaped obstacle forcing path to go around a wall.

    Layout (rough ASCII, origin bottom-left):
        . . . . . . . . .
        . . . . . . . . .
        . . XXXXXXXX  . .
        S . X G . . . . .
        . . X . . . . . .
        . . XXXXXXXX  . .
        . . . . . . . . .
        . . . . . . . . .

    Start at top-left, goal at bottom-right. The wall blocks the direct diagonal,
    forcing a U-shaped detour around the open right side.
    """
    grid = np.zeros((height, width), dtype=np.int8)

    # Border walls (1-cell thick)
    grid[0, :] = 1
    grid[-1, :] = 1
    grid[:, 0] = 1
    grid[:, -1] = 1

    # U-shaped wall: horizontal top arm, vertical left side, horizontal bottom arm
    wall_left = 8
    wall_right = 28
    wall_top = 26
    wall_bot = 14

    # Top horizontal arm
    grid[wall_top, wall_left:wall_right] = 1
    # Bottom horizontal arm
    grid[wall_bot, wall_left:wall_right] = 1
    # Left vertical connector
    grid[wall_bot:wall_top + 1, wall_left] = 1

    start_rc = (20, 5)   # top-left area (row, col)
    goal_rc = (25, 12)     # bottom-right area

    return grid, start_rc, goal_rc


def _make_zigzag_grid(height: int = 50, width: int = 30) -> Tuple[np.ndarray, Tuple[int,int], Tuple[int,int]]:
    """Zigzag corridor with alternating walls forcing S-shaped path.

    Three horizontal walls alternate from left and right, creating a serpentine channel.
    """
    grid = np.zeros((height, width), dtype=np.int8)

    # Border
    grid[0, :] = 1
    grid[-1, :] = 1
    grid[:, 0] = 1
    grid[:, -1] = 1

    # Wall 1: from left, gap on right side (row 12)
    grid[12, 1:22] = 1

    # Wall 2: from right, gap on left side (row 24)
    grid[24, 8:width - 1] = 1

    # Wall 3: from left, gap on right side (row 36)
    grid[36, 1:22] = 1

    start_rc = (5, 15)    # bottom section
    goal_rc = (44, 15)     # top section

    return grid, start_rc, goal_rc


def _make_narrow_gap_grid(height: int = 30, width: int = 40) -> Tuple[np.ndarray, Tuple[int,int], Tuple[int,int]]:
    """Wall with a narrow 2-cell gap. Tests tight-space behavior near obstacles.

    Start on left side, goal on right side, wall across the middle with a small gap.
    """
    grid = np.zeros((height, width), dtype=np.int8)

    # Border
    grid[0, :] = 1
    grid[-1, :] = 1
    grid[:, 0] = 1
    grid[:, -1] = 1

    # Vertical wall at col=20, gap at rows 14-15
    grid[1:14, 20] = 1
    grid[16:height - 1, 20] = 1

    start_rc = (5, 5)
    goal_rc = (15, 35)

    return grid, start_rc, goal_rc


def _make_col_periodic_grid(height: int = 20, width: int = 30) -> Tuple[np.ndarray, Tuple[int,int], Tuple[int,int]]:
    """Column-periodic grid: wrap-around path is shorter than any non-periodic route.

    No left/right border walls so FMM can propagate through the column seam.

    Axes: periodic=[False, True] (rows not periodic, cols periodic).

        XXXXXXXXXXXXXXXXX
        . . . X . X . . .
        . . . X . X . . X
        . . . X . X . . X
        . G . X . X . S X
        . . . X . XXXXXXX
        . . . X . X . . .
        . . . X . X . . .

    Optimal path: start(col 24) → right → col 29 → wrap → col 0 → left → goal(col 5)
    Distance ≈ 11 cells. Direct (non-periodic) path is completely blocked.
    """
    grid = np.zeros((height, width), dtype=np.int8)

    # Top and bottom borders only — no left/right walls (periodic columns)
    grid[0, :] = 1
    # grid[-1, :] = 1

    # Full-height vertical wall (interior rows only) blocking direct path
    wall_col_left, wall_col_right = 12, 17
    grid[:, wall_col_left] = 1
    grid[:, wall_col_right] = 1
    grid[height // 2 - 2, wall_col_right + 1:] = 1
    grid[height // 2 - 2:3*height // 4, -1] = 1

    start_rc = (height // 2, width - 3)
    goal_rc = (5, 3)

    return grid, start_rc, goal_rc


def _make_torus_grid(height: int = 25, width: int = 25) -> Tuple[np.ndarray, Tuple[int,int], Tuple[int,int]]:
    """Fully periodic (torus) grid: shortest path cuts through both seams.

    No border walls on any side. Start near top-left corner, goal near
    bottom-right corner. The direct path spans ~20 cells in each axis;
    the wrap-around path spans only ~5 cells in each axis.

    Axes: periodic=[True, True].

    Geometry:
        start (row 2, col 2) — 2 rows from top seam, 2 cols from left seam
        goal (row 22, col 22) — 3 rows from bottom seam, 3 cols from right seam
        wrap distance:  sqrt((2+3)^2 + (2+3)^2) ≈ 7.1 cells
        direct distance: sqrt(20^2 + 20^2) ≈ 28.3 cells

    FMM with periodic=[True, True] should choose the short diagonal through
    both the row seam (top↔bottom) and the column seam (left↔right).
    """
    grid = np.zeros((height, width), dtype=np.int8)
    # No borders — purely open torus

    start_rc = (2, 2)
    goal_rc = (22, 22)

    return grid, start_rc, goal_rc


def _make_spiral_grid(height: int = 70, width: int = 70,
                      n_turns: int = 3,
                      gap_width: int = 3,
                      passage_width: int = 4,
                      ) -> Tuple[np.ndarray, Tuple[int, int], Tuple[int, int]]:
    """Concentric rectangular rings with alternating gaps that force a spiralling path to the centre.

    Each ring has a single gap:
      - Even rings (0, 2, …): gap in the RIGHT wall, near the TOP.
      - Odd  rings (1, 3, …): gap in the LEFT  wall, near the BOTTOM.

    To pass through each ring the path must travel almost all the way around it,
    producing a true inward spiral.  Start is placed near the bottom-left of the
    outer corridor; the goal is at the grid centre.

    Args:
        height, width:    grid dimensions (cells).
        n_turns:          number of concentric rings to draw.
        gap_width:        opening height in each ring wall (cells).
        passage_width:    free-space corridor width between rings (cells).

    Layout (n_turns=3, passage_width=4):
        step = passage_width + 1 = 5
        Ring 0: inset=5,  gap right wall rows 6–8   (near top-right)
        Ring 1: inset=10, gap left  wall rows 56–58 (near bottom-left)
        Ring 2: inset=15, gap right wall rows 16–18 (near top-right)
        Goal at centre (~35, 35); start at (68, 2) outer corridor bottom-left.
    """
    grid = np.zeros((height, width), dtype=np.int8)

    # Border
    # grid[0, :] = 1
    # grid[-1, :] = 1
    # grid[:, 0] = 1
    # grid[:, -1] = 1

    step = passage_width + 1  # wall (1 cell) + passage

    for i in range(n_turns):
        inset = (i + 1) * step
        r0, r1 = inset, height - 1 - inset
        c0, c1 = inset, width - 1 - inset

        if r0 >= r1 - gap_width or c0 >= c1 - gap_width:
            break  # no room for another ring

        # Solid ring walls
        grid[r0, c0:c1 + 1] = 1          # top
        grid[r1, c0:c1 + 1] = 1          # bottom
        grid[r0:r1 + 1, c0] = 1          # left
        grid[r0:r1 + 1, c1] = 1          # right

        # Gap: avoid corner cells so each gap is a clean slot
        if i % 2 == 0:
            # Right wall, near top
            grid[r0 + 1:r0 + 1 + gap_width, c1] = 0
        else:
            # Left wall, near bottom
            grid[r1 - gap_width:r1, c0] = 0

    # Start: outer corridor, bottom-left (far from ring-0 gap at top-right)
    start_rc = (height - 2, 2)
    goal_rc = (height // 2, width // 2)

    return grid, start_rc, goal_rc


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def _split_path_at_seams(path: np.ndarray, h: int, w: int,
                          periodic: Optional[List] = None) -> List[np.ndarray]:
    """Split a grid-coord path into continuous segments at wrap-seam crossings.

    When a path crosses a periodic seam the coordinates jump by nearly the full
    grid width/height. Detect those jumps and split so each segment can be drawn
    as a continuous line without the teleport artifact.
    """
    if path is None or len(path) < 2:
        return [path] if path is not None else []
    wrap_cols = periodic is not None and len(periodic) > 1 and periodic[1]
    wrap_rows = periodic is not None and len(periodic) > 0 and periodic[0]

    segments = []
    seg_start = 0
    for i in range(1, len(path)):
        jump = False
        if wrap_cols and abs(path[i, 0] - path[i - 1, 0]) > w * 0.5:
            jump = True
        if wrap_rows and abs(path[i, 1] - path[i - 1, 1]) > h * 0.5:
            jump = True
        if jump:
            segments.append(path[seg_start:i])
            seg_start = i
    segments.append(path[seg_start:])
    return segments


def _plot_comparison(grid: np.ndarray, dist_map: np.ndarray,
                     gy: np.ndarray, gx: np.ndarray,
                     path_discrete: np.ndarray, path_bilinear: np.ndarray,
                     start_rc: Tuple[int, int], goal_rc: Tuple[int, int],
                     title: str, save_path: str,
                     periodic: Optional[List] = None):
    """Plot FMM field, gradient arrows, obstacle grid, and both paths."""
    fig, axes = plt.subplots(1, 2, figsize=(18, 8))

    h, w = grid.shape

    for ax_idx, (ax, label) in enumerate(zip(axes, ['Full view', 'Zoomed paths'])):
        # FMM distance heatmap (inf → nan for display)
        display_d = np.where(np.isinf(dist_map), np.nan, dist_map)
        im = ax.imshow(display_d, origin='lower', cmap='viridis_r', alpha=0.6)

        # Obstacle overlay
        obstacle_display = np.where(grid == 1, 1.0, np.nan)
        ax.imshow(obstacle_display, origin='lower', cmap='Greys', alpha=0.8,
                  vmin=0, vmax=1)

        # Gradient arrows (subsample for readability)
        step = max(1, min(h, w) // 20)
        rows_q = np.arange(0, h, step)
        cols_q = np.arange(0, w, step)
        cc, rr = np.meshgrid(cols_q, rows_q)
        # Only show arrows in free space
        mask = np.isfinite(dist_map[rr, cc])
        ax.quiver(cc[mask], rr[mask],
                  -gx[rr[mask], cc[mask]], -gy[rr[mask], cc[mask]],
                  color='white', alpha=0.4, scale=30, width=0.003, headwidth=3)

        # Paths (in grid coords: col=x, row=y). Split at seam crossings for
        # periodic grids so each segment draws without the teleport artifact.
        h_g, w_g = grid.shape
        if path_discrete is not None and len(path_discrete) > 0:
            for seg_i, seg in enumerate(_split_path_at_seams(path_discrete, h_g, w_g, periodic)):
                label = f'Discrete-snap ({len(path_discrete)} pts)' if seg_i == 0 else '_nolegend_'
                ax.plot(seg[:, 0], seg[:, 1], 'b-', linewidth=2.0, label=label, zorder=5)
        if path_bilinear is not None and len(path_bilinear) > 0:
            for seg_i, seg in enumerate(_split_path_at_seams(path_bilinear, h_g, w_g, periodic)):
                label = f'Bilinear ({len(path_bilinear)} pts)' if seg_i == 0 else '_nolegend_'
                ax.plot(seg[:, 0], seg[:, 1], 'r--', linewidth=2.0, label=label, zorder=5)

        # Start/goal markers
        ax.plot(start_rc[1], start_rc[0], 'g^', markersize=14, label='Start', zorder=6)
        ax.plot(goal_rc[1], goal_rc[0], 'r*', markersize=16, label='Goal', zorder=6)

        ax.legend(loc='upper right', fontsize=8)
        ax.set_title(f'{title} — {label}')
        ax.set_xlabel('col')
        ax.set_ylabel('row')

    # Zoom second axes to path bounding box with margin
    all_pts = []
    if path_discrete is not None:
        all_pts.append(path_discrete)
    if path_bilinear is not None:
        all_pts.append(path_bilinear)
    if all_pts:
        all_pts = np.vstack(all_pts)
        margin = 5
        axes[1].set_xlim(all_pts[:, 0].min() - margin, all_pts[:, 0].max() + margin)
        axes[1].set_ylim(all_pts[:, 1].min() - margin, all_pts[:, 1].max() + margin)

    fig.colorbar(im, ax=axes, label='FMM distance (cells)', shrink=0.7)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(save_path)), exist_ok=True)
    fig.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved: {save_path}")


# ---------------------------------------------------------------------------
# Test runners
# ---------------------------------------------------------------------------

def _grid_to_world(rc: Tuple[int, int], resolution: float,
                   world_bounds: Tuple[float, float, float, float]) -> Tuple[float, float]:
    """(row, col) → (x, y) world coords."""
    x = rc[1] * resolution + world_bounds[0]
    y = rc[0] * resolution + world_bounds[1]
    return (x, y)


def _run_scenario(name: str, grid: np.ndarray,
                  start_rc: Tuple[int, int], goal_rc: Tuple[int, int],
                  output_dir: str,
                  periodic: Optional[List] = None):
    """Run both path methods on a grid and save comparison plot.

    @param  periodic  Per-axis periodicity [wrap_rows, wrap_cols] or None.
                      get_fmm_path_bilinear auto-selects EDT (non-periodic) or
                      FMM narrow-band (periodic) repulsion, so grid is always passed.
    """
    print(f"\n=== Scenario: {name} ===")
    print(f"  Grid shape: {grid.shape}, start_rc={start_rc}, goal_rc={goal_rc}")
    if periodic:
        print(f"  Periodic axes: {periodic}")

    # Use resolution=1.0 so grid coords == world coords (simple)
    h, w = grid.shape
    resolution = 1.0
    world_bounds = (0.0, 0.0, float(w), float(h))

    # Compute FMM distance map (no resolution scaling, units = cells)
    dist_map = compute_fmm_distance_map(
        grid, goal_rc[0], goal_rc[1], resolution=resolution, periodic=periodic
    )
    start_dist = dist_map[start_rc[0], start_rc[1]]
    print(f"  FMM distance at start: {start_dist:.2f} cells")

    if not np.isfinite(start_dist):
        print(f"  ERROR: Start is unreachable! Skipping.")
        return

    # Compute gradient field
    gy, gx = _fmm_obstacle_aware_gradient(dist_map, periodic=periodic)

    start_world = _grid_to_world(start_rc, resolution, world_bounds)
    goal_world = _grid_to_world(goal_rc, resolution, world_bounds)

    # Discrete-snap method disabled — it clips through obstacles and gives invalid paths.
    path_discrete_grid = None

    # Bilinear path with wall repulsion. get_fmm_path_bilinear auto-selects:
    # EDT (non-periodic) or FMM narrow-band (periodic), so grid is always passed.
    print("  Running bilinear path...")
    path_bilinear_world = get_fmm_path_bilinear(
        dist_map, start_world, goal_world, resolution, world_bounds,
        grid=grid,
        periodic=periodic,
    )
    if path_bilinear_world is not None:
        print(f"  Bilinear path: {len(path_bilinear_world)} waypoints")
        path_bilinear_grid = np.column_stack([
            (path_bilinear_world[:, 0] - world_bounds[0]) / resolution,
            (path_bilinear_world[:, 1] - world_bounds[1]) / resolution,
        ])

        def _path_length_periodic(p_grid, periodic_axes, grid_h, grid_w):
            """Sum step distances using minimum-image convention on periodic axes."""
            diffs = np.diff(p_grid, axis=0)  # (N-1, 2): col diff, row diff
            if periodic_axes is not None and periodic_axes[1]:
                diffs[:, 0] -= grid_w * np.round(diffs[:, 0] / grid_w)
            if periodic_axes is not None and periodic_axes[0]:
                diffs[:, 1] -= grid_h * np.round(diffs[:, 1] / grid_h)
            return float(np.sum(np.sqrt(np.sum(diffs ** 2, axis=1))))

        path_len = _path_length_periodic(path_bilinear_grid, periodic, h, w)
        print(f"  Path length: {path_len:.2f} cells (FMM distance: {start_dist:.2f})")
        if periodic:
            n_segs_c = sum(
                1 for i in range(1, len(path_bilinear_grid))
                if periodic[1] and abs(path_bilinear_grid[i, 0] - path_bilinear_grid[i - 1, 0]) > w * 0.5
            )
            n_segs_r = sum(
                1 for i in range(1, len(path_bilinear_grid))
                if periodic[0] and abs(path_bilinear_grid[i, 1] - path_bilinear_grid[i - 1, 1]) > h * 0.5
            )
            print(f"  Seam crossings: col={n_segs_c}, row={n_segs_r}")
    else:
        print("  Bilinear path: FAILED")
        path_bilinear_grid = None

    save_path = os.path.join(output_dir, f"{name}.png")
    _plot_comparison(grid, dist_map, gy, gx,
                     path_discrete_grid, path_bilinear_grid,
                     start_rc, goal_rc, name, save_path,
                     periodic=periodic)


def _bench_repulsion_methods(radii: Optional[List[float]] = None, n_reps: int = 10):
    """Timing and accuracy comparison of EDT vs FMM-narrow obstacle repulsion.

    Runs both compute_obstacle_repulsive_gradient() (EDT, no periodic) and
    compute_obstacle_fmm_repulsive_gradient() (FMM narrow-band, periodic-capable)
    on all scenario grids plus one large synthetic grid, printing timing and direction
    cosine similarity within the narrow band.

    @param  radii   wall_repulsion_radius values to sweep (default [1.5, 3.0, 6.0]).
    @param  n_reps  Number of repetitions per method per grid (default 10).
    """
    import time
    if radii is None:
        radii = [1.5, 3.0, 6.0]

    scenarios = [
        ("u_shape",    _make_u_shape_grid()[0]),
        ("zigzag",     _make_zigzag_grid()[0]),
        ("narrow_gap", _make_narrow_gap_grid()[0]),
        ("col_periodic", _make_col_periodic_grid()[0]),
        ("torus",      _make_torus_grid()[0]),
    ]

    def make_large_grid(h: int = 320, w: int = 320, n_obs: int = 50) -> np.ndarray:
        rng = np.random.default_rng(42)
        g = np.zeros((h, w), dtype=np.int8)
        g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 1
        cx = rng.integers(5, h - 5, n_obs)
        cy = rng.integers(5, w - 5, n_obs)
        for x, y in zip(cx, cy):
            g[x - 2:x + 3, y - 2:y + 3] = 1
        return g

    scenarios.append(("large_320x320", make_large_grid()))

    print(f"\n{'='*70}")
    print("Repulsion method benchmark")
    print(f"{'='*70}")
    print(f"{'Grid':<16} {'Shape':<12} {'Radius':<8} {'EDT ms':>8} {'FMM ms':>8} {'Speedup':>8} {'MeanCos':>9} {'MinCos':>8}")
    print(f"{'-'*70}")

    for name, grid in scenarios:
        free_mask = grid == 0

        # EDT baseline (radius-independent — always computes full field)
        t0 = time.perf_counter()
        for _ in range(n_reps):
            dir_gy_e, dir_gx_e, obs_e = compute_obstacle_repulsive_gradient(grid)
        t_edt = (time.perf_counter() - t0) / n_reps * 1000

        for radius in radii:
            t0 = time.perf_counter()
            for _ in range(n_reps):
                dir_gy_f, dir_gx_f, obs_f = compute_obstacle_fmm_repulsive_gradient(
                    grid, wall_repulsion_radius=radius
                )
            t_fmm = (time.perf_counter() - t0) / n_reps * 1000

            # Cosine similarity of repulsion directions within the EDT narrow band
            in_band = (obs_e <= radius) & free_mask
            if in_band.any():
                cos_sim = (dir_gy_e[in_band] * dir_gy_f[in_band]
                           + dir_gx_e[in_band] * dir_gx_f[in_band])
                mean_cos = float(np.mean(cos_sim))
                min_cos  = float(np.min(cos_sim))
            else:
                mean_cos = float('nan')
                min_cos  = float('nan')

            speedup = t_edt / t_fmm if t_fmm > 0 else float('inf')
            shape_str = f"{grid.shape[0]}x{grid.shape[1]}"
            print(f"{name:<16} {shape_str:<12} {radius:<8.1f} {t_edt:>8.2f} {t_fmm:>8.2f} "
                  f"{speedup:>8.2f}x {mean_cos:>9.4f} {min_cos:>8.4f}")

    print(f"{'='*70}")
    print("Note: EDT is the non-periodic baseline; FMM-narrow supports periodic=.")
    print("SpeedRatio < 1x means FMM is faster; > 1x means EDT is faster.")
    print("MeanCos: mean cosine similarity of repulsion direction vs EDT in the narrow band.")
    print("MinCos: minimum (worst-case agreement); 0 = perpendicular (band-edge artifact).")


def main():
    parser = argparse.ArgumentParser(description="FMM gradient descent path comparison tests.")
    parser.add_argument('--output-dir', type=str, default='/tmp/fmm_path_tests',
                        help='Directory to save comparison plots (default: /tmp/fmm_path_tests).')
    parser.add_argument('--bench', action='store_true',
                        help='Run repulsion method timing benchmark instead of path plots.')
    args = parser.parse_args()

    if args.bench:
        _bench_repulsion_methods()
        return

    output_dir = os.path.expanduser(args.output_dir)

    # Scenario 1: U-shape
    grid_u, start_u, goal_u = _make_u_shape_grid()
    _run_scenario("u_shape", grid_u, start_u, goal_u, output_dir)

    # Scenario 2: Zigzag
    grid_z, start_z, goal_z = _make_zigzag_grid()
    _run_scenario("zigzag", grid_z, start_z, goal_z, output_dir)

    # Scenario 3: Narrow gap
    grid_n, start_n, goal_n = _make_narrow_gap_grid()
    _run_scenario("narrow_gap", grid_n, start_n, goal_n, output_dir)

    # Scenario 4: Column-periodic — path wraps through left/right seam
    grid_cp, start_cp, goal_cp = _make_col_periodic_grid()
    _run_scenario("col_periodic", grid_cp, start_cp, goal_cp, output_dir,
                  periodic=[False, True])

    # Scenario 5: Torus — path wraps through both seams (shortest diagonal)
    grid_t, start_t, goal_t = _make_torus_grid()
    _run_scenario("torus", grid_t, start_t, goal_t, output_dir,
                  periodic=[True, True])

    # Scenario 6: Spiral — concentric rings with alternating gaps force an inward spiral
    grid_s, start_s, goal_s = _make_spiral_grid(n_turns=5)
    _run_scenario("spiral", grid_s, start_s, goal_s, output_dir)

    grid_s, start_s, goal_s = _make_spiral_grid(n_turns=7, passage_width=2)
    _run_scenario("tight_spiral", grid_s, start_s, goal_s, output_dir)

    grid_s, start_s, goal_s = _make_spiral_grid(n_turns=9, passage_width=1)
    _run_scenario("tightest_spiral", grid_s, start_s, goal_s, output_dir)

    print(f"\nAll plots saved to: {output_dir}")


if __name__ == '__main__':
    main()
