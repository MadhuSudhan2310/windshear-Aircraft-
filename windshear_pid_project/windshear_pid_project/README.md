# Effect of Wind Shear on a PID-Controlled Aircraft (Simulink)
M.Tech avionics mini-project: longitudinal flight control under wind shear.

## 1. Problem statement
A PID autopilot holds pitch attitude (inner loop) and altitude (outer loop) of a transport
aircraft in cruise. A microburst-like wind shear (headwind → tailwind swing plus a downdraft)
hits at t = 20 s. Quantify how shear degrades control performance and how the controller
architecture and gains change the outcome.

## 2. Files
| File | Purpose |
|---|---|
| `windshear_params.m` | All parameters (aircraft, shear, PID gains). Edit here. |
| `build_windshear_pid_model.m` | Builds `windshear_pid_aircraft.slx` from basic blocks |
| `run_windshear_study.m` | Studies A/B/C, figures, `windshear_results.csv` |
| `reference_python/` | Independent Python implementation used to tune gains and sanity-check results |
| `reference_shear_response.png` | Reference plot of the expected behaviour |

## 3. How to run
```matlab
cd windshear_pid_project
build_windshear_pid_model     % creates and opens the model
run_windshear_study           % runs all studies
```
Needs MATLAB + Simulink only (no Aerospace Blockset). The Simulink model was generated
programmatically; the Python reference was run and verified, but I could not execute MATLAB in my
environment, so if `build_windshear_pid_model` reports a block-parameter error on your release,
adjust that line (PID block parameter names are the most likely to differ between releases).

## 4. Mathematical model
States: α (angle of attack), q (pitch rate), θ (pitch), h (altitude deviation), ΔVa (airspeed deviation).

- α̇ = −Zα·α_aero + q
- q̇ = k·(Mα·α_aero + Mδ·δe) + Mq·q, with k = (Va/V0)²
- θ̇ = q, ḣ = Va·(θ − α)
- α_aero = α − w_g/Va  (downdraft w_g > 0 reduces α → loss of lift and a nose-down moment change)
- ΔV̇a = −ΔVa/τV + u̇_wind  (headwind loss changes airspeed instantly, then recovers)

Wind shear enters through **three** mechanisms (this is the core of the analysis):
1. **Vertical gust** → α_aero disturbance → lift/moment disturbance.
2. **Headwind change** → ΔVa → dynamic pressure k changes → *loop gain of the plant changes*
   (the PID is tuned at V0 only, so this probes gain robustness).
3. **Kinematic coupling** → ḣ depends on α, so a downdraft disturbs flight path even when θ is held.

Shear profile (smooth, one-cosine): u_wind = Ah·cos(πs) − Ah, w_g = Aw·sin²(πs), s = clip((t − t0)/T, 0, 1).
Ah = 15 m/s, Aw = 8 m/s, T = 20 s. The aircraft data are representative, not a specific type —
replace with your aircraft's derivatives.

## 5. Controller
Cascade: `h_cmd − h → PID_altitude → θ_cmd (±10°) → PID_pitch → δe (±25°) → servo 1/(0.05s+1)`.
Both PIDs use filtered derivative and clamping anti-windup.
Inner: Kp = 2, Ki = 0.8, Kd = 0.8, N = 20. Outer: Kp = 0.006, Ki = 0.0006, Kd = 0.015, N = 10.

## 6. Results (from the verified reference simulation)
Shear Ah = 15 m/s, Aw = 8 m/s:

| Case | Peak altitude dev. | Peak θ | Peak δe |
|---|---|---|---|
| Pitch-hold only | **80.5 m** (does not recover: −79 m at t = 100 s) | 0.22° | 0.67° |
| Pitch + altitude PID | **6.2 m** (returns to 0) | 3.3° | 0.21° |

Intensity sweep (altitude-loop gains Kph = 0.004, Kih = 0.0004, Kdh = 0.01 in the Python sweep):
peak altitude error grows roughly linearly with shear strength — from 27 m (Ah = 5) to 133 m (Ah = 25)
with pitch-hold only, versus 2.9 m to 16.3 m with the altitude loop.

**Key findings to discuss in your report**
1. A pitch-attitude PID alone looks excellent on θ (0.22°) yet loses ~80 m of altitude: the downdraft
   changes flight-path angle γ = θ − α, which attitude hold does not see. *Controlling the wrong
   variable hides the shear effect.*
2. The outer altitude loop trades attitude excursion (θ up to 3.3°) for altitude accuracy. Good
   disturbance rejection needs the controller to act on the variable that matters.
3. Elevator demand stays small, so the saturation/anti-windup is not exercised at this intensity;
   raise Ah/Aw (e.g., 40 / 20 m/s) to study saturation and integrator windup.
4. Raising inner-loop gains improves θ rejection (peak θ 0.52° → 0.16° for Kp 1 → 3) but costs
   bandwidth margin against actuator lag and noise.

## 7. Suggested extensions for the M.Tech report
- Replace the representative derivatives with a real aircraft (e.g., Boeing 747 / F-16 data) and use the full 4-state model with phugoid.
- Add Dryden turbulence (Aerospace Blockset) or the Oseguera–Bowles microburst profile.
- Compare tuning methods: Ziegler–Nichols, `pidtune`, and an optimization (`fmincon`) minimizing ITAE under shear.
- Gain/phase margin versus Va (linearize with `linmod` at several k) to explain loop-gain robustness.
- Add airspeed/thrust control and a rate limiter on the elevator.
- Compare against LQR or gain-scheduled PID with a V-based schedule.
