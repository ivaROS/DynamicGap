"""
Fast Marching Method (FMM) utilities for geodesic distance computation and gradient-descent path extraction.

These are pure computation functions operating on occupancy grids and FMM distance fields.
Caching and file I/O remain in maze_map_utils.py (ObstacleMap).

Coordinate conventions match maze_map_utils.py:
- Grid: (row, col) where row=y-index, col=x-index (NumPy standard)
- World: (x, y) in meters
"""

import math
from typing import Optional, Tuple

import numpy as np
import skfmm
from scipy.ndimage import distance_transform_edt


def compute_fmm_distance_map(grid: np.ndarray, goal_row: int, goal_col: int,
                              resolution: Optional[float] = None,
                              periodic: Optional[list] = None) -> np.ndarray:
    """Uses skfmm to solve the obstacle-aware Eikonal equation to compute geodesic distances
    from a goal cell to all free cells.

    Obstacles (grid == 1) are masked so FMM cannot propagate through them.
    Unreachable and obstacle cells in the result hold np.inf.

    @param  grid       Binary occupancy grid (1=obstacle, 0=free).
    @param  goal_row   Goal cell row index.
    @param  goal_col   Goal cell column index.
    @param  resolution Grid resolution in meters per cell; sets the dx parameter for skfmm
                       so the result is in meters.
    @param  periodic   Per-axis periodicity: [wrap_rows, wrap_cols]. Passed directly to
                       skfmm.distance. None (default) means no wrapping.
    @return float32 array shaped like grid with geodesic distance (meters) to goal.
    @raises ValueError  If goal is out of bounds or inside an obstacle.
    """
    height, width = grid.shape
    if not (0 <= goal_row < height and 0 <= goal_col < width):
        raise ValueError("Goal is out of bounds")
    if grid[goal_row, goal_col] == 1:
        raise ValueError("Goal is inside an obstacle")

    phi = np.ones((height, width), dtype=np.float64)
    phi[goal_row, goal_col] = -1.0
    phi_masked = np.ma.MaskedArray(phi, mask=grid.astype(bool))

    kwargs = {}
    if resolution is not None:
        kwargs['dx'] = resolution
    if periodic is not None:
        kwargs['periodic'] = periodic
    dists = skfmm.distance(phi_masked, **kwargs)

    if isinstance(dists, np.ma.MaskedArray):
        return dists.filled(np.inf).astype(np.float32)
    return dists.astype(np.float32)


def _fmm_obstacle_aware_gradient(dists: np.ndarray,
                                  periodic: Optional[list] = None) -> Tuple[np.ndarray, np.ndarray]:
    """
    @brief  Computes gradient of an FMM distance field, using one-sided differences at obstacle boundaries.

    np.gradient uses central differences, so a free cell adjacent to an obstacle (inf) produces
    (inf - finite) / 2 = inf. Simply zeroing instead would drop the wall-adjacent cell's gradient
    (parallel or pointing away from the wall), which will cause gradient descent to not be guided
    alonge the wall and instead stop still if that gradient was purely parallel.

    Instead, similarly to how np.gradient handles the edges of arrays, this will fall back to
    one-sided (forward or backward) differences whenever one neighbor is non-finite, computing
    a well-defined gradient everywhere in free space.

    At grid boundaries, the neighbor slots default to nan (treated as obstacle/non-finite),
    so the one-sided fallback applies — identical to treating the grid edge as a wall. When
    an axis is periodic, those nan boundary slots are filled with the opposite-end row/column
    values instead, so central differences run across the wrap seam naturally. The existing
    both/fwd/bwd masking handles this without any additional logic.

    @note   Use the resulting gradient array with bilinear interpolation for smooth continuous
            gradient sampling.

    @param  dists     FMM distance field: finite values in free space, np.inf for obstacles.
    @param  periodic  Per-axis periodicity: [wrap_rows, wrap_cols]. None (default) means no wrapping.
    @return (gy, gx): gradient grid arrays in row and column directions, zero at obstacle cells.
    """
    wrap_rows = periodic[0] if periodic is not None else False
    wrap_cols = periodic[1] if periodic is not None else False
    h, w = dists.shape
    finite = np.isfinite(dists)

    gy = np.zeros_like(dists)
    gx = np.zeros_like(dists)

    # --- Row Grads (gy = ∂d/∂row) ---
    d_prev_r = np.full((h, w), np.nan)
    d_next_r = np.full((h, w), np.nan)
    d_prev_r[1:, :] = dists[:-1, :]
    d_next_r[:-1, :] = dists[1:, :]
    if wrap_rows:
        d_prev_r[0, :] = dists[-1, :]   # row 0's previous wraps to last row
        d_next_r[-1, :] = dists[0, :]   # last row's next wraps to row 0
    prev_ok_r = np.isfinite(d_prev_r)
    next_ok_r = np.isfinite(d_next_r)

    both_r = finite & prev_ok_r & next_ok_r # can central diff because not in an obstacle and both neighbors are valid
    gy[both_r] = (d_next_r[both_r] - d_prev_r[both_r]) / 2.0

    fwd_r = finite & next_ok_r & ~prev_ok_r # obstacle/out-of-bounds above, can only do forward diff
    gy[fwd_r] = d_next_r[fwd_r] - dists[fwd_r]

    bwd_r = finite & prev_ok_r & ~next_ok_r
    gy[bwd_r] = dists[bwd_r] - d_prev_r[bwd_r]

    # --- Column Grads (gx = ∂d/∂col) ---
    d_prev_c = np.full((h, w), np.nan)
    d_next_c = np.full((h, w), np.nan)
    d_prev_c[:, 1:] = dists[:, :-1]
    d_next_c[:, :-1] = dists[:, 1:]
    if wrap_cols:
        d_prev_c[:, 0] = dists[:, -1]   # col 0's previous wraps to last col
        d_next_c[:, -1] = dists[:, 0]   # last col's next wraps to col 0
    prev_ok_c = np.isfinite(d_prev_c)
    next_ok_c = np.isfinite(d_next_c)

    both_c = finite & prev_ok_c & next_ok_c # can central diff because not in an obstacle and both neighbors are valid
    gx[both_c] = (d_next_c[both_c] - d_prev_c[both_c]) / 2.0

    fwd_c = finite & next_ok_c & ~prev_ok_c
    gx[fwd_c] = d_next_c[fwd_c] - dists[fwd_c]

    bwd_c = finite & prev_ok_c & ~next_ok_c
    gx[bwd_c] = dists[bwd_c] - d_prev_c[bwd_c]

    return gy, gx


def compute_obstacle_repulsive_gradient(grid: np.ndarray
                                        ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute obstacle distance field and normalized repulsive direction per cell.

    Uses the Euclidean Distance Transform for per-cell distance to the nearest
    obstacle, then computes the normalized gradient of the EDT (unit vectors pointing
    away from the nearest obstacle). No distance-based weighting is baked in — the
    caller applies weights at query time based on actual (possibly sub-cell) distance
    and its own d_influence / k_rep parameters.

    @note   Periodic boundary conditions are NOT supported. distance_transform_edt has
            no periodic mode, so cells within ~wall_repulsion_radius of a wrap seam will
            underestimate their distance to any obstacle on the other side, causing the
            repulsive field to be too weak near seams. To fix this, pad the grid with
            np.pad(grid, pad_width, mode='wrap') before the EDT call and slice the result
            back to the original map bounds; pad_width should be at least wall_repulsion_radius
            in cells.

    @param  grid         Binary occupancy grid (1=obstacle, 0=free).
    @return (edt_dir_gy, edt_dir_gx, obs_dist):
            - edt_dir_gy, edt_dir_gx: unit direction arrays pointing away from nearest
              obstacle. Zero at obstacle cells.
            - obs_dist: EDT distance field (distance to nearest obstacle per cell).
    """
    free_mask = grid == 0
    obs_dist = np.asarray(distance_transform_edt(free_mask), dtype=np.float32)

    # Gradient of EDT points away from nearest obstacle (increasing distance direction)
    edt_gy, edt_gx = np.gradient(obs_dist) # can use central difference here since inside obs is just zero

    # Normalize to unit vectors
    edt_mag = np.sqrt(edt_gy ** 2 + edt_gx ** 2)
    edt_mag = np.where(edt_mag < 1e-10, 1.0, edt_mag)
    edt_dir_gy = (edt_gy / edt_mag).astype(np.float32)
    edt_dir_gx = (edt_gx / edt_mag).astype(np.float32)

    # Zero out obstacle cells
    edt_dir_gy[~free_mask] = 0.0
    edt_dir_gx[~free_mask] = 0.0

    return edt_dir_gy, edt_dir_gx, obs_dist


def compute_obstacle_fmm_repulsive_gradient(grid: np.ndarray,
                                             wall_repulsion_radius: float,
                                             periodic: Optional[list] = None,
                                             ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute obstacle distance field and repulsive direction using FMM narrow-band marching.

    Drop-in periodic-capable replacement for compute_obstacle_repulsive_gradient().
    Uses skfmm.distance with a narrow band limited to wall_repulsion_radius so only
    the region that can exert repulsive force is marched, and the result naturally
    supports the same periodic parameter as compute_fmm_distance_map().

    phi setup: +1 in free space, -1 inside obstacles. The zero level set sits at obstacle
    boundaries; FMM marches outward into free space up to wall_repulsion_radius cells and
    inward into obstacles. The returned signed distance is positive in free space (matches
    EDT convention in the repulsion-active region) and negative inside obstacles.

    Gradient is computed via _fmm_obstacle_aware_gradient (not np.gradient), which
    uses one-sided differences at obstacle boundaries and across periodic seams.
    Outside the narrow band, the gradient-input field is filled with a constant plateau
    (= wall_repulsion_radius), making the gradient — and therefore repulsion — zero there.

    Distances are in cells (no cell-resultion scaling). wall_repulsion_radius must is also in cells

    @note  ~1.6–3.8× slower than compute_obstacle_repulsive_gradient() (EDT). The sole
           trade-off: this function supports periodic boundaries; EDT does not.

    @param  grid                   Binary occupancy grid (1=obstacle, 0=free).
    @param  wall_repulsion_radius  Narrow-band radius in cells. Must match the
                                   wall_repulsion_radius used in get_next_continuous_descent_state
                                   so that the gradient is nonzero exactly where repulsion activates.
    @param  periodic               Per-axis periodicity: [wrap_rows, wrap_cols]. None (default)
                                   means no wrapping. Must match the periodic argument used in
                                   compute_fmm_distance_map for the same grid.
    @return (edt_dir_gy, edt_dir_gx, obs_dist):
            - edt_dir_gy, edt_dir_gx: float32 unit direction arrays pointing away from nearest
              obstacle. Zero at obstacle cells and outside the narrow band.
            - obs_dist: float32 distance field in cells. 0 at obstacle surface, positive inside
              the narrow band, wall_repulsion_radius + 1 beyond the band.
              Same interface as compute_obstacle_repulsive_gradient().
    """
    free_mask = grid == 0

    if not np.any(~free_mask):
        # No obstacles: repulsion direction is zero and distance is infinity everywhere.
        inf_dist = np.full(grid.shape, wall_repulsion_radius + 1.0, dtype=np.float32)
        zeros = np.zeros(grid.shape, dtype=np.float32)
        return zeros, zeros, inf_dist

    phi = np.ones(grid.shape, dtype=np.float64)
    phi[~free_mask] = -1.0

    kwargs: dict = {'narrow': wall_repulsion_radius}
    if periodic is not None:
        kwargs['periodic'] = periodic
    d_narrow = skfmm.distance(phi, **kwargs)  # MaskedArray when band < grid; ndarray otherwise

    # Normalise to MaskedArray so .filled() is always available.
    if not isinstance(d_narrow, np.ma.MaskedArray):
        d_narrow = np.ma.MaskedArray(d_narrow, mask=False)

    # obs_dist: clip below at 0 so obstacle cells = 0 (matching EDT convention).
    # Outside-band free cells get wall_repulsion_radius + 1 so bilinear d_obs never
    # activates the repulsion check (d_obs <= wall_repulsion_radius) there.
    obs_dist = np.maximum(d_narrow.filled(wall_repulsion_radius + 1.0), 0.0).astype(np.float32)

    # Gradient input: plateau at the band boundary → gradient = 0 outside band.
    # Obstacles → inf so _fmm_obstacle_aware_gradient uses one-sided differences at walls
    # (same as it does for the goal distance map).
    d_for_grad = d_narrow.filled(wall_repulsion_radius).astype(np.float64)
    d_for_grad[~free_mask] = np.inf

    gy, gx = _fmm_obstacle_aware_gradient(d_for_grad, periodic=periodic)

    mag = np.sqrt(gy ** 2 + gx ** 2)
    mag = np.where(mag < 1e-10, 1.0, mag)
    edt_dir_gy = (gy / mag).astype(np.float32)
    edt_dir_gx = (gx / mag).astype(np.float32)

    edt_dir_gy[~free_mask] = 0.0
    edt_dir_gx[~free_mask] = 0.0

    return edt_dir_gy, edt_dir_gx, obs_dist


def get_safe_gradient(r: float, c: float,
                      gy: np.ndarray, gx: np.ndarray,
                      is_valid_mask: np.ndarray,
                      periodic: Optional[list] = None,
                      ) -> Tuple[float, float]:
    """Bilinear interpolation of a precomputed gradient field, excluding obstacle cells.

    Given a continuous grid position (r, c), computes the bilinear weights for the four
    surrounding integer-corner cells, zeros the weight of any corner that is invalid
    (obstacle or out-of-bounds), renormalizes the remaining weights, and returns the
    weighted gradient.

    When some corners are obstacles, their weights are redistributed to the valid corners.
    This naturally transitions from 4-point bilinear (open space) to 3-point, 2-point,
    or 1-point nearest-neighbor as the point approaches a wall. For periodic axes, corner
    indices wrap via modular arithmetic instead of clamping, giving correct bilinear
    interpolation across seam boundaries.

    @param  r               Continuous row position in grid coordinates.
    @param  c               Continuous column position in grid coordinates.
    @param  gy              Row-direction gradient array from _fmm_obstacle_aware_gradient().
    @param  gx              Column-direction gradient array from _fmm_obstacle_aware_gradient().
    @param  is_valid_mask   Boolean array (same shape as gy/gx), True for free-space cells.
    @param  periodic        Per-axis periodicity: [wrap_rows, wrap_cols]. None means no wrapping.
    @return (grad_y, grad_x): interpolated gradient components. Both math.inf if all four
            corners are invalid (point is inside or fully surrounded by obstacles).
    """
    h, w = is_valid_mask.shape
    r_floor = math.floor(r)
    c_floor = math.floor(c)
    dr = r - r_floor
    dc = c - c_floor
    if periodic is not None and periodic[0]:
        r0 = r_floor % h
        r1 = (r0 + 1) % h
    else:
        r0 = max(0, r_floor)
        r1 = min(r0 + 1, h - 1)
    if periodic is not None and periodic[1]:
        c0 = c_floor % w
        c1 = (c0 + 1) % w
    else:
        c0 = max(0, c_floor)
        c1 = min(c0 + 1, w - 1)
    corners = ((r0, c0), (r0, c1), (r1, c0), (r1, c1))
    raw_w = ((1 - dr) * (1 - dc), (1 - dr) * dc, dr * (1 - dc), dr * dc)
    weight_sum = 0.0
    grad_y = 0.0
    grad_x = 0.0
    for (ri, ci), w_i in zip(corners, raw_w):
        if is_valid_mask[ri, ci]:
            weight_sum += w_i
            grad_y += w_i * gy[ri, ci]
            grad_x += w_i * gx[ri, ci]
    if weight_sum == 0.0:
        return math.inf, math.inf
    return grad_y / weight_sum, grad_x / weight_sum


def get_next_continuous_descent_state(r: float, c: float,
                                      gy: np.ndarray, gx: np.ndarray,
                                      is_valid_mask: np.ndarray,
                                      base_step: float = 0.5,
                                      edt_dir_gy: Optional[np.ndarray] = None,
                                      edt_dir_gx: Optional[np.ndarray] = None,
                                      obs_dist: Optional[np.ndarray] = None,
                                      wall_repulsion_radius: float = 1.0,
                                      wall_repulsion_strength: float = 5.0,
                                      periodic: Optional[list] = None,
                                      ) -> Tuple[float, float, bool]:
    """Compute a single gradient-descent step on an FMM field with wall-collision prevention.

    Uses get_safe_gradient() for obstacle-aware bilinear gradient interpolation, then
    blends in a repulsive gradient that pushes away from nearby obstacles. The repulsive
    component activates only when the bilinear-interpolated obstacle distance is within
    wall_repulsion_radius, allowing sub-cell activation thresholds.

    Step logic:
    1. Compute interpolated FMM gradient via get_safe_gradient().
    2. Bilinear-interpolate obs_dist at (r, c). If within wall_repulsion_radius, blend
       in the EDT direction (unit vector away from nearest wall, valid corners only)
       weighted by linear falloff.
    3. Normalize combined gradient and step by base_step in the -∇ direction.
    4. Failsafe: if position lands inside an obstacle cell, push back toward pre-step side.

    @param  r                        Current continuous row position in grid coordinates.
    @param  c                        Current continuous column position in grid coordinates.
    @param  gy                       Row-direction gradient array from _fmm_obstacle_aware_gradient().
    @param  gx                       Column-direction gradient array from _fmm_obstacle_aware_gradient().
    @param  is_valid_mask            Boolean array, True for free-space cells.
    @param  base_step                Step size in cells (default 0.5).
    @param  edt_dir_gy               Normalized EDT direction (row) from compute_obstacle_repulsive_gradient(), or None.
    @param  edt_dir_gx               Normalized EDT direction (col) from compute_obstacle_repulsive_gradient(), or None.
    @param  obs_dist                 EDT distance field from compute_obstacle_repulsive_gradient(), or None.
    @param  wall_repulsion_radius    Distance in cells within which the repulsive field activates (default 1.0).
                                     Sub-cell values work because obs_dist is bilinear-interpolated.
    @param  wall_repulsion_strength  Repulsive gain at zero distance (default 5.0). Scales linearly to 0 at radius.
    @param  periodic                 Per-axis periodicity: [wrap_rows, wrap_cols]. None means no wrapping.
                                     Note: the failsafe copysign direction may be incorrect for a step that
                                     crosses a wrap seam — a rare edge case acceptable in practice.
    @return (r_next, c_next, stuck):
            - r_next, c_next: new continuous grid position after the step.
            - stuck: True if no valid step could be taken (all corners are obstacles or
              gradient magnitude is near zero, indicating goal reached or local minimum).
    """
    h, w = is_valid_mask.shape
    wrap_rows = periodic[0] if periodic is not None else False
    wrap_cols = periodic[1] if periodic is not None else False
    grad_y, grad_x = get_safe_gradient(r, c, gy, gx, is_valid_mask, periodic=periodic)

    if not math.isfinite(grad_y):
        return r, c, True  # inside obstacle

    mag = math.hypot(grad_y, grad_x)
    if mag < 1e-10:
        return r, c, True  # at goal or local minimum

    # Repulsive blending: bilinear-interpolate obs_dist (all corners, including
    # obstacles at 0) for true sub-cell proximity, and EDT direction (valid
    # corners only, so obstacle zero-vectors don't dilute the push direction).
    if edt_dir_gy is not None and edt_dir_gx is not None and obs_dist is not None:
        r_floor = math.floor(r)
        c_floor = math.floor(c)
        dr = r - r_floor
        dc = c - c_floor
        if wrap_rows:
            r0 = r_floor % h;  r1 = (r0 + 1) % h
        else:
            r0 = max(0, r_floor);  r1 = min(r0 + 1, h - 1)
        if wrap_cols:
            c0 = c_floor % w;  c1 = (c0 + 1) % w
        else:
            c0 = max(0, c_floor);  c1 = min(c0 + 1, w - 1)
        corners = ((r0, c0), (r0, c1), (r1, c0), (r1, c1))
        raw_w = ((1 - dr) * (1 - dc), (1 - dr) * dc, dr * (1 - dc), dr * dc)
        # obs_dist: all corners (obstacle cells have obs_dist=0, which is correct)
        d_obs = sum(raw_w[i] * float(obs_dist[corners[i]]) for i in range(4))
        if d_obs <= wall_repulsion_radius:
            # EDT direction: valid corners only (obstacle cells have zero direction)
            valid_w_sum = 0.0
            rep_y = 0.0
            rep_x = 0.0
            for (ri, ci), w_i in zip(corners, raw_w):
                if is_valid_mask[ri, ci]:
                    valid_w_sum += w_i
                    rep_y += w_i * float(edt_dir_gy[ri, ci])
                    rep_x += w_i * float(edt_dir_gx[ri, ci])
            if valid_w_sum > 0:
                rep_y /= valid_w_sum
                rep_x /= valid_w_sum
                weight = wall_repulsion_strength * (wall_repulsion_radius - d_obs) / wall_repulsion_radius
                grad_y -= weight * rep_y
                grad_x -= weight * rep_x

    # Re-normalize after repulsion blending
    mag = math.hypot(grad_y, grad_x)
    if mag < 1e-10:
        return r, c, True

    desc_r = -grad_y / mag
    desc_c = -grad_x / mag

    if wrap_rows:
        r_next = (r + base_step * desc_r) % h
    else:
        r_next = max(0.0, min(h - 1.0, r + base_step * desc_r))
    if wrap_cols:
        c_next = (c + base_step * desc_c) % w
    else:
        c_next = max(0.0, min(w - 1.0, c + base_step * desc_c))

    # Failsafe clamp: if post-step position landed inside an obstacle cell,
    # push back toward the PRE-STEP position. Using the pre-step side ensures
    # we never get pushed across a wall to the other side, regardless of step
    # size or local geometry. round() gives the unique cell containing the point,
    # so one check suffices (the 4-corner loop was redundant).
    ri = round(r_next) % h if wrap_rows else round(r_next)
    ci = round(c_next) % w if wrap_cols else round(c_next)
    if 0 <= ri < h and 0 <= ci < w and not is_valid_mask[ri, ci]:
        pen_r = 0.5 - abs(r_next - ri)
        pen_c = 0.5 - abs(c_next - ci)
        if pen_r > pen_c:
            r_next = ri + math.copysign(0.5001, r - ri)
        else:
            c_next = ci + math.copysign(0.5001, c - ci)

    return r_next, c_next, False


def get_fmm_path_bilinear(fmm_goal_distance_map: np.ndarray,
                           start_world: Tuple[float, float],
                           goal_world: Tuple[float, float],
                           resolution: float,
                           world_bounds: Tuple[float, float, float, float],
                           grid: Optional[np.ndarray] = None,
                           base_step: float = 0.5,
                           max_steps: int = 50000,
                           wall_repulsion_radius: float = 1.0,
                           wall_repulsion_strength: float = 4.0,
                           periodic: Optional[list] = None,
                           ) -> Optional[np.ndarray]:
    """Trace an optimal path via bilinear-interpolated gradient descent with wall-collision prevention.

    Bilinearly interpolates the FMM gradient at each sub-pixel position and blends a
    repulsive field near obstacles to prevent getting stuck in concave corners.

    @note   be careful when tuning wall_repulsion_radius and wall_repulsion_strength. In worlds with 
            tight concave turns, too low will lead to collisions with the walls. In worlds with 
            narrow gaps/passages, too large will repel the path from the passage and lead to oscilations
            and possible block off passages entirely.

    When grid= is provided, the repulsion field is automatically chosen:
    - Non-periodic (default): compute_obstacle_repulsive_gradient() — EDT, fast, full field.
    - Periodic: compute_obstacle_fmm_repulsive_gradient() — FMM narrow-band, handles wrap
      seams correctly. Avoid wall_repulsion_radius >= 4 in periodic mode since large radii can
      cause the repulsion weight to exceed the FMM gradient magnitude in tight passages,
      reversing the descent direction and triggering oscillation. FMM is only used for periodic
      since it is ~3x slower

    @param  fmm_goal_distance_map  float32 distance field from compute_fmm_distance_map().
    @param  start_world            (x, y) start position in world coordinates.
    @param  goal_world             (x, y) goal position in world coordinates.
    @param  resolution             Grid resolution in meters per cell.
    @param  world_bounds           (xmin, ymin, xmax, ymax) in meters.
    @param  grid                   Binary occupancy grid for repulsive field. If None, no
                                   repulsive gradient is used.
    @param  base_step              Step size in cells (default 0.5).
    @param  max_steps              Hard cap on iteration count (default 50000).
    @param  wall_repulsion_radius    Distance in cells within which the repulsive field
                                     activates (default 1.0). Uses bilinear-interpolated
                                     obstacle distance, so sub-cell values work. Values below
                                     ~0.75 may be too late to deflect paths at concave corners.
                                     Additionally, if using the FMM narrow-band repulsion for 
                                     periodic maps, should keeping below 4 to avoid approximation
                                     errors in that method that can cause oscillation in tight
                                     gaps.

    @param  wall_repulsion_strength  Repulsive gain at zero distance (default 4.0). Scales
                                     linearly to 0 at wall_repulsion_radius. Higher values
                                     will push paths away more aggressively, but can also
                                     make them jagged as they try to move back toward obstacles
                                     and straddle the activation boundary.
    @param  periodic                 Per-axis periodicity: [wrap_rows, wrap_cols]. Must match
                                     the periodic argument used in compute_fmm_distance_map.
                                     When set, FMM narrow-band repulsion is used instead of
                                     EDT so wrap-seam distances are computed correctly.
                                     Note: paths that cross a wrap seam will have a discontinuity
                                     in the returned world-coordinate array (the path "teleports"
                                     to the other side). Callers that need a continuous
                                     representation must unwrap the coordinates themselves.
    @return Path as (N, 2) float32 array of world (x, y) coords from start to goal,
            or None if start is out of bounds, inside an obstacle, or unreachable.
    """
    if wall_repulsion_radius < 0.75:
        print(f"Warning: wall_repulsion_radius={wall_repulsion_radius} may be too low to reliably prevent corner-cutting, especially with larger base_step or lower wall_repulsion_strength.")
    if periodic is not None and wall_repulsion_radius >= 4:
        print(f"Warning: wall_repulsion_radius={wall_repulsion_radius} >= 4 in periodic (FMM) mode "
              f"may cause oscillation in tight passages. Consider keeping it <= 3.")

    h, w = fmm_goal_distance_map.shape
    xmin, ymin = world_bounds[0], world_bounds[1]

    # keep float for continuous descent
    r = (start_world[1] - ymin) / resolution
    c = (start_world[0] - xmin) / resolution

    start_row = int(round(r))
    start_col = int(round(c))
    if not (0 <= start_row < h and 0 <= start_col < w):
        return None
    if not np.isfinite(fmm_goal_distance_map[start_row, start_col]):
        return None

    goal_row = int(round((goal_world[1] - ymin) / resolution))
    goal_col = int(round((goal_world[0] - xmin) / resolution))

    gy, gx = _fmm_obstacle_aware_gradient(fmm_goal_distance_map, periodic=periodic)
    is_valid = np.isfinite(fmm_goal_distance_map)

    edt_dir_gy = None
    edt_dir_gx = None
    obs_dist_field = None
    if grid is not None:
        if periodic is not None:
            # FMM narrow-band: handles wrap-around distances correctly.
            edt_dir_gy, edt_dir_gx, obs_dist_field = compute_obstacle_fmm_repulsive_gradient(grid, wall_repulsion_radius, periodic=periodic)
        else:
            # EDT: faster (~2 or 3 times on large grids), computes full field, no periodic support.
            edt_dir_gy, edt_dir_gx, obs_dist_field = compute_obstacle_repulsive_gradient(grid)

    path_world = [(c * resolution + xmin, r * resolution + ymin)]

    for _ in range(max_steps):
        # Goal convergence: use minimum-image distance on periodic axes so the path
        # terminates correctly when start and goal are on opposite sides of a seam.
        dr = r - goal_row
        dc = c - goal_col
        if periodic is not None and periodic[0]:
            dr -= h * round(dr / h)
        if periodic is not None and periodic[1]:
            dc -= w * round(dc / w)
        if dr * dr + dc * dc < 0.9:
            goal_x = goal_col * resolution + xmin
            goal_y = goal_row * resolution + ymin
            path_world.append((goal_x, goal_y))
            break

        r, c, stuck = get_next_continuous_descent_state(
            r, c, gy, gx, is_valid, base_step=base_step,
            edt_dir_gy=edt_dir_gy, edt_dir_gx=edt_dir_gx,
            obs_dist=obs_dist_field,
            wall_repulsion_radius=wall_repulsion_radius,
            wall_repulsion_strength=wall_repulsion_strength,
            periodic=periodic,
        )
        if stuck:
            break

        path_world.append((c * resolution + xmin, r * resolution + ymin))

    return np.array(path_world, dtype=np.float32)