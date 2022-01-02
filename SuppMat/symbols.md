### Gap Detection

| Variable | Meaning |
| :------: | :------ |
| ![\mathcal{L}](https://latex.codecogs.com/svg.latex?\mathcal{L}) | A full laser scan-like measurement. |
| ![d_{\max}](https://latex.codecogs.com/svg.latex?d_{\max}) | Maximum range of laser scan measurement. |
| ![\theta(i)](https://latex.codecogs.com/svg.latex?\theta(i)) | Angle associated with index i in ![\mathcal{L}](https://latex.codecogs.com/svg.latex?\mathcal{L}). |
| ![\zeta_i](https://latex.codecogs.com/svg.latex?\zeta_i) | Polar coordiante with range and associated angle with index i. |
| ![\text{d}(\cdot,\cdot)](https://latex.codecogs.com/svg.latex?\text{d}(\cdot,\cdot)) | Euclidean distance. |
| ![l_{l/r}](https://latex.codecogs.com/svg.latex?l_{l/r}) | Distance to left and right side of gap point. |
| ![\alpha](https://latex.codecogs.com/svg.latex?\alpha) | Angle formed by the shorter side of gap and the gap arc (the angle's facing side is the longer side). |
| ![\tau_\alpha](https://latex.codecogs.com/svg.latex?\tau_\alpha) | Threshold value on ![\alpha](https://latex.codecogs.com/svg.latex?\alpha) to distinguish between radial and swept gaps. |
| ![G](https://latex.codecogs.com/svg.latex?G) | A single detected gap. |
| ![\mathcal{G}](https://latex.codecogs.com/svg.latex?\mathcal{G}) | The set of raw detected gaps. |
| ![c_a](https://latex.codecogs.com/svg.latex?c_a) | Maximum acceptable angular difference between pending and terminating gap. |
| ![c_d](https://latex.codecogs.com/svg.latex?c_d) | Maximum acceptable range difference between pending and terminating gap. |
| ![\epsilon = (\epsilon^1, \epsilon^2)](https://latex.codecogs.com/svg.latex?\epsilon=(\epsilon^1,\epsilon^2)) | Parameter to control the rotation of gaps in Radial Gap conversion. Scales with robot geometry since it should guarantee line-of-sight visibility to collision-free regions on the other side of the gap. The corresponding angle of the vector should be scale free relative to robot geometry. |

### Potential Field Construction

| Variable | Meaning |
| :------: | :------ |
| ![\mathcal{G}_SGP](https://latex.codecogs.com/svg.latex?\mathcal{G}_{\text{SGP}}) | Set of gaps after Swept Gap Prioritization (SGP) |
| ![\alpha_g](https://latex.codecogs.com/svg.latex?\alpha_g) | Gap angular extend in Fig. 2 |
| ![\tau_{\text{GA}}](https://latex.codecogs.com/svg.latex?\tau_{\text{GA}}) | Parameter for closing the gap for convexity. Typically 90 or 180 degrees. |
| ![G'](https://latex.codecogs.com/svg.latex?G') | Reduced gap with angular extent equal to ![\tau_{\text{GA}}](https://latex.codecogs.com/svg.latex?\tau_{\text{GA}}). |
| ![G^*](https://latex.codecogs.com/svg.latex?G^*) | Chosen gap. |
| ![x^*_{\text{LG}}](https://latex.codecogs.com/svg.latex?x^*_{\text{LG}}) | Local goal point for chosen gap ![G^*](https://latex.codecogs.com/svg.latex?G^*). |
| ![\Phi(x)](https://latex.codecogs.com/svg.latex?\Phi(x)) |Attractive potential to local goal point determined for the chosen gap. |
| ![\Theta(x)](https://latex.codecogs.com/svg.latex?\Theta(x)) | Circulation vector field for chosen gap.|
| ![\mathbb{J}](https://latex.codecogs.com/svg.latex?\mathbb{J}) | Rotation matrix ![R(-\pi/2)](https://latex.codecogs.com/svg.latex?R(-\pi/2)) in ![\mathfrak{so}(2)](https://latex.codecogs.com/svg.latex?\mathfrak{so}(2)) skew-symmetric form. |
| ![\text{d}_\theta(\cdot,\cdot)](https://latex.codecogs.com/svg.latex?\text{d}_\theta(\cdot,\cdot)) | Angular distance for planar polar coordinate vectors. |
| ![p_{l/r}](https://latex.codecogs.com/svg.latex?p_{l/r}) | Point of left or right side of gap. |
| ![e_\perp](https://latex.codecogs.com/svg.latex?e_\perp) |The circulation term on the gap edge (it is purely perpendicular and inward pointing for the gap point associated to the gap edge). |
| ![f_\phi](https://latex.codecogs.com/svg.latex?f_\phi) | The vector contributed by the circulation term that is on the conjugate gap edge. |
| ![\phi](https://latex.codecogs.com/svg.latex?\phi) | The corresponding angle of vector ![f_\phi](https://latex.codecogs.com/svg.latex?f_\phi). |
| ![e_\rho](https://latex.codecogs.com/svg.latex?e_\rho) | The radially directed outward vector in polar coordinate frame centered at robot location.  |
| ![\hat{\nabla}](https://latex.codecogs.com/svg.latex?\hat{\nabla}) | Operator that computes the gradient and normalizes it unit length if non-zero. |

### Trajectory Synthesis and Scoring

| Variable | Meaning |
| :------: | :------ |
| ![\mathcal{X}](https://latex.codecogs.com/svg.latex?\mathcal{X}) | A set of trajectories integrated based on potential field. |
| ![p](https://latex.codecogs.com/svg.latex?p) | A single pose. |
| ![\mathcal{T}](https://latex.codecogs.com/svg.latex?\mathcal{T}) | A trajectory defined by a collection of a poses. |
| ![p_{\text{end}}](https://latex.codecogs.com/svg.latex?p_{\text{end}}) | The last pose on a trajectory ![\mathcal{T}](https://latex.codecogs.com/svg.latex?\mathcal{T}). |
| ![p^*](https://latex.codecogs.com/svg.latex?p^*) | The local navigation goal. |
| ![J](https://latex.codecogs.com/svg.latex?J) | Function to calculate the cost of a trajectory |
| ![w_1](https://latex.codecogs.com/svg.latex?w_1) | Amplification factor so that the score associated to the terminal state is comparable in scaling to the score generated by integrating along the trajectory. |
| ![w_2](https://latex.codecogs.com/svg.latex?w_2) | Decay factor on penalty for getting close to an obstacle point. |
| ![c_{\text{obs}}](https://latex.codecogs.com/svg.latex?c_{\text{obs}}) | Amplification factor and sign correction for the obstacle proximity penalty.|
| ![r_{\max}](https://latex.codecogs.com/svg.latex?r_{\max}) | Maximum range to apply cost function on a pose. |


### Safety Extension
| Variable | Meaning |
| :------: | :------ |
| ![u = (u^1, u^2, u^3)^\top](https://latex.codecogs.com/svg.latex?u=(u^1,u^2,u^3)^\top) | Control signal of ![\mathfrak{se}(2)](https://latex.codecogs.com/svg.latex?\mathfrak{se}(2)). |
| ![\xi_u](https://latex.codecogs.com/svg.latex?\xi_u) | Control signal for consstrained model with forward and rotation motion only. |
| ![\nu](https://latex.codecogs.com/svg.latex?\nu) | Linear velocity input. |
| ![\omega](https://latex.codecogs.com/svg.latex?\omega) | Rotational velocity input. |
| ![\lambda_y](https://latex.codecogs.com/svg.latex?\lambda_y) | Conversion gain for mapping lateral controls to turn rates. |
| ![\mathcal{C}](https://latex.codecogs.com/svg.latex?\mathcal{C}) | Set of obstacle boundary curves in robot-centered coordinate frame. |
| ![\psi(x)](https://latex.codecogs.com/svg.latex?\psi(x)) | Potential function for projection operator. |
| ![r_{\min}](https://latex.codecogs.com/svg.latex?r_{\min}) | Value where potential function passes value 1. |
| ![r_{\text{nom}}](https://latex.codecogs.com/svg.latex?r_{\text{nom}) | Value where potential function passes value 0. |
| ![\text{Proj}(u; x)](https://latex.codecogs.com/svg.latex?\text{Proj}(u;x)) | Projection operator. |
| ![u'](https://latex.codecogs.com/svg.latex?u') | Control input to the system after applying projection operator. |
| ![\Omega_O](https://latex.codecogs.com/svg.latex?\Omega_O) | Unsafe set established by the obstacles, through potential function. |

