import numpy as np

def resample_curve(curve, num_points, step_distance, dt):
    positions = []
    times = []

    dist_accum = 0.0
    next_target = 0.0
    current_seg = 0

    # start point
    positions.append(curve[0])
    times.append(0.0)

    for i in range(1, num_points):
        next_target += step_distance

        while (current_seg + 1 < len(curve) and
               dist_accum + np.linalg.norm(curve[current_seg + 1] - curve[current_seg]) < next_target):
            dist_accum += np.linalg.norm(curve[current_seg + 1] - curve[current_seg])
            current_seg += 1

        if current_seg + 1 >= len(curve):
            break   # ← this is the problematic line

        seg = curve[current_seg + 1] - curve[current_seg]
        seg_len = np.linalg.norm(seg)
        t = (next_target - dist_accum) / max(seg_len, 1e-6)

        pt = curve[current_seg] + t * seg
        positions.append(pt)
        times.append(i * dt)

    return positions, times


# ------------------ TEST ------------------

# Create a short curve )
curve = [
    np.array([0.0, 0.0]),
    np.array([0.5, 0.0]),
    np.array([1.0, 0.0]),
    np.array([2.0, 0.0]),
    np.array([3.0, 0.0]),
    np.array([4.0, 0.0]),
    np.array([4.0, 0.0])
]

num_points   = 11
dt           = 0.5
v_cmd        = 1.0
step_distance = v_cmd * dt

positions, times = resample_curve(curve, num_points, step_distance, dt)

print("Expected points:", num_points)
print("Actual positions:", len(positions))
print("Actual times:", len(times))

print("\nPositions:")
for p in positions:
    print(p)
