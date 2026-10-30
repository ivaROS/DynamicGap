# Recorded Arena results

| Scenario | Capture time (s) | Final distance (m) | Outward regression (m) | Move Base success |
| --- | ---: | ---: | ---: | --- |
| final_empty_long | 43.12 | 0.245 | 0.000 | yes |
| final_diagonal | 47.47 | 0.250 | 0.000 | yes |
| final_reverse | 47.52 | 0.247 | 0.000 | yes |
| final_short | 2.55 | 0.246 | 0.000 | yes |
| final_wall_adjacent | 2.60 | 0.249 | 0.000 | yes |
| final_very_close_wall | 3.05 | 0.242 | 0.000 | yes |
| final_facing_away | 5.21 | 0.241 | 0.000 | yes |
| final_crossing_person | 4.66 | 0.243 | 0.000 | yes |
| normal_crossing_person | 4.96 | 0.249 | 0.000 | yes |
| normal_empty_long | 42.76 | 0.247 | 0.000 | yes |

Capture begins 0.5 s after the scenario reset. The goal tolerance is 0.25 m. Reset jumps after first arrival are excluded.

final_crossing_person: minimum measured robot/person centre separation 0.814 m; combined footprint radii 0.525 m.
normal_crossing_person: minimum measured robot/person centre separation 0.634 m; combined footprint radii 0.525 m.

The final matrix and normal-process confirmations all passed. No Move Base exit occurred during these trials. Runtime actionlib/pedsim warnings remain; the cause of the earlier pre-change exit was not established.
