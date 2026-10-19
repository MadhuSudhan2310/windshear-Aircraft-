## Aircraft Flight Dynamics: Formulas, Symbols, Units, and Assumptions

This section presents fundamental aircraft aerodynamic, performance, and flight-dynamics equations. All equations use SI units unless stated otherwise.

### 1. Symbols and Units

| Symbol | Description | SI unit |
|---|---|---|
| \(m\) | Aircraft mass | kg |
| \(W\) | Aircraft weight | N |
| \(L\) | Lift force | N |
| \(D\) | Drag force | N |
| \(T\) | Thrust force | N |
| \(g\) | Gravitational acceleration | m/s² |
| \(\rho\) | Air density | kg/m³ |
| \(V\) | True airspeed | m/s |
| \(S\) | Wing reference area | m² |
| \(b\) | Wingspan | m |
| \(C_L\) | Lift coefficient | Dimensionless |
| \(C_D\) | Drag coefficient | Dimensionless |
| \(C_{L_{\max}}\) | Maximum lift coefficient | Dimensionless |
| \(q_\infty\) | Dynamic pressure | Pa (N/m²) |
| \(\alpha\) | Angle of attack | rad or degrees |
| \(\phi\) | Roll angle | rad |
| \(\theta\) | Pitch angle | rad |
| \(\psi\) | Yaw angle | rad |
| \(\gamma\) | Flight-path angle | rad |
| \(p,q,r\) | Body-axis angular velocities | rad/s |
| \(I_x,I_y,I_z\) | Moments of inertia | kg·m² |
| \(M_x,M_y,M_z\) | Body-axis moments | N·m |
| \(P\) | Mechanical power | W |
| \(AR\) | Wing aspect ratio | Dimensionless |
| \(n\) | Load factor | Dimensionless |
| \(M\) | Mach number | Dimensionless |
| \(a\) | Local speed of sound | m/s |
| \(ROC\) | Rate of climb | m/s |
| \(K_p,K_i,K_d\) | PID controller gains | Depend on controller input/output units |
| \(e(t)\) | Control error | Depends on controlled variable |
| \(R\) | Aircraft range | m |
| \(W_i,W_f\) | Initial and final aircraft weights | N |

**Notation:** \(q_\infty\) denotes dynamic pressure, while \(q\) denotes the body-axis pitch rate. The symbol \(M\) denotes Mach number in the Mach equation and body-axis moment in rotational equations when used with axis subscripts.

### 2. Aircraft Weight and Force Balance

**Weight**

\[
W=mg
\]

Symbols and units:

- \(W\): weight, N
- \(m\): mass, kg
- \(g\): gravitational acceleration, m/s²

Assumptions:
- Gravitational acceleration is treated as constant.
- Aircraft mass is constant over the short interval considered.

**Steady, level flight**

\[
L=W,\qquad T=D
\]

Assumptions:
- Constant altitude and airspeed.
- No net vertical or longitudinal acceleration.
- Thrust acts approximately along the flight direction.

### 3. Lift Equation

\[
L=\frac{1}{2}\rho V^2SC_L
\]

| Symbol | Meaning | Unit |
|---|---|---|
| \(L\) | Lift | N |
| \(\rho\) | Air density | kg/m³ |
| \(V\) | True airspeed | m/s |
| \(S\) | Wing area | m² |
| \(C_L\) | Lift coefficient | Dimensionless |

Assumptions:
- Airflow conditions permit the use of a representative lift coefficient.
- The reference wing area is defined consistently.
- Compressibility and unsteady effects are neglected unless included in \(C_L\).

### 4. Drag Equation

\[
D=\frac{1}{2}\rho V^2SC_D
\]

The approximate parabolic drag polar is:

\[
C_D=C_{D0}+kC_L^2
\]

\[
k=\frac{1}{\pi e AR}
\]

\[
AR=\frac{b^2}{S}
\]

Symbols:
- \(C_D\): total drag coefficient, dimensionless
- \(C_{D0}\): zero-lift drag coefficient, dimensionless
- \(k\): induced-drag factor, dimensionless
- \(e\): Oswald efficiency factor, dimensionless
- \(AR\): wing aspect ratio, dimensionless
- \(b\): wingspan, m
- \(S\): wing reference area, m²

Assumptions:
- The drag polar is approximated by a quadratic relationship.
- The efficiency factor is treated as constant over the operating range.
- The approximation may be inaccurate near stall or at high Mach numbers.

### 5. Dynamic Pressure

\[
q_\infty=\frac{1}{2}\rho V^2
\]

Units:

\[
\frac{kg}{m^3}\frac{m^2}{s^2}
=\frac{N}{m^2}=Pa
\]

Assumptions:
- Air density and airspeed are defined for the local flight condition.
- Airspeed is measured relative to the surrounding air mass.

### 6. Stall Speed

\[
V_s=\sqrt{\frac{2W}{\rho SC_{L_{\max}}}}
\]

Symbols:
- \(V_s\): stall speed, m/s
- \(W\): aircraft weight, N
- \(\rho\): air density, kg/m³
- \(S\): wing area, m²
- \(C_{L_{\max}}\): maximum lift coefficient, dimensionless

Assumptions:
- The aircraft is in steady, approximately level flight.
- The stall occurs when the required lift coefficient reaches \(C_{L_{\max}}\).
- Configuration, icing, contamination, and transient effects are not modelled separately.

For a coordinated level turn:

\[
V_{s,n}=V_s\sqrt{n}
\]

where \(n\) is the load factor.

### 7. Load Factor and Turning Flight

\[
n=\frac{L}{W}
\]

For a coordinated, level turn:

\[
n=\frac{1}{\cos\phi}
\]

Symbols:
- \(n\): load factor, dimensionless
- \(\phi\): bank angle, degrees or radians
- \(L\): lift, N
- \(W\): weight, N

Assumptions:
- The turn is coordinated and level.
- Altitude and speed are approximately constant.
- The bank angle is below \(90^\circ\).

### 8. Longitudinal Equations of Motion

A simplified point-mass model is:

\[
m\dot V=T-D-W\sin\gamma
\]

\[
mV\dot\gamma=L-W\cos\gamma
\]

Symbols and units:
- \(\dot V\): airspeed rate of change, m/s²
- \(\dot\gamma\): flight-path angular rate, rad/s
- \(V\): true airspeed, m/s
- \(\gamma\): flight-path angle, rad
- Forces: N

Assumptions:
- The aircraft is represented as a point mass.
- Lift is perpendicular to the flight path.
- Thrust is aligned with the flight path.
- Sideslip, rotational dynamics, and detailed control-surface effects are neglected.

### 9. Climb Performance

\[
ROC=V\sin\gamma
\]

For steady climb, the approximate excess-power relation is:

\[
ROC\approx\frac{P_{\text{available}}-P_{\text{required}}}{W}
\]

Symbols:
- \(ROC\): rate of climb, m/s
- \(P_{\text{available}}\): available propulsion power, W
- \(P_{\text{required}}\): power required for flight, W
- \(W\): aircraft weight, N

Unit check:

\[
\frac{W}{N}=\frac{J/s}{N}=m/s
\]

Assumptions:
- The aircraft is in steady climb.
- The excess-power expression is used as an energy approximation.
- For jet aircraft, the corresponding thrust-based performance relationship may be more appropriate.

### 10. Mechanical Power

\[
P=TV
\]

Symbols:
- \(P\): useful propulsive power, W
- \(T\): thrust, N
- \(V\): velocity along the thrust direction, m/s

Assumptions:
- Thrust and velocity are aligned.
- This is useful propulsive power, not necessarily engine shaft power or electrical input power.
- Propulsion efficiency and installation losses must be considered separately when calculating input power.

### 11. Aircraft Rotational Dynamics

For a rigid aircraft using body-axis angular velocities \(p,q,r\), one general form of the Euler rotational equations is:

\[
I_x\dot p-I_{xz}\dot r
=M_x+(I_y-I_z)qr-I_{xz}pq
\]

\[
I_y\dot q
=M_y+(I_z-I_x)pr+I_{xz}(p^2-r^2)
\]

\[
I_z\dot r-I_{xz}\dot p
=M_z+(I_x-I_y)pq+I_{xz}qr
\]

Symbols and units:
- \(I_x,I_y,I_z,I_{xz}\): inertia terms, kg·m²
- \(p,q,r\): angular velocities, rad/s
- \(\dot p,\dot q,\dot r\): angular accelerations, rad/s²
- \(M_x,M_y,M_z\): external moments, N·m

Assumptions:
- The aircraft is a rigid body.
- The axes are body-fixed axes.
- The equations use a consistent inertia-tensor sign convention.
- Aerodynamic, propulsion, and control-surface moments must be included in the applied moments.

### 12. Euler Attitude Kinematics

For a 3-2-1 yaw-pitch-roll Euler-angle sequence:

\[
\dot\phi=p+(q\sin\phi+r\cos\phi)\tan\theta
\]

\[
\dot\theta=q\cos\phi-r\sin\phi
\]

\[
\dot\psi=\frac{q\sin\phi+r\cos\phi}{\cos\theta}
\]

Symbols:
- \(\phi\): roll angle, rad
- \(\theta\): pitch angle, rad
- \(\psi\): yaw angle, rad
- \(p,q,r\): body-axis angular rates, rad/s

Assumptions:
- Euler angles use the specified 3-2-1 sequence.
- Angles are represented in radians in mathematical calculations.
- The representation becomes singular when \(\cos\theta=0\), corresponding to pitch angles of \(\pm90^\circ\).

### 13. Mach Number and Speed of Sound

\[
M=\frac{V}{a}
\]

\[
a=\sqrt{\gamma_g R_a T}
\]

Symbols:
- \(M\): Mach number, dimensionless
- \(V\): true airspeed, m/s
- \(a\): local speed of sound, m/s
- \(\gamma_g\): ratio of specific heats, dimensionless
- \(R_a\): specific gas constant for air, J/(kg·K)
- \(T\): absolute temperature, K

Assumptions:
- Air is approximated as a calorically perfect ideal gas.
- Temperature is absolute temperature in kelvin.
- For standard dry air near ordinary atmospheric conditions, \(\gamma_g\) is approximately 1.4 and \(R_a\) approximately 287 J/(kg·K).

### 14. Wind and Airspeed Relationship

\[
\vec V_g=\vec V_a+\vec V_w
\]

Symbols:
- \(\vec V_g\): ground-relative velocity, m/s
- \(\vec V_a\): air-relative velocity, m/s
- \(\vec V_w\): wind velocity, m/s

Assumptions:
- All velocity vectors use the same reference frame and coordinate axes.
- Aerodynamic forces depend on air-relative velocity, not ground speed alone.

### 15. PID Flight Controller

\[
u(t)=K_pe(t)+K_i\int e(t)\,dt+K_d\frac{de(t)}{dt}
\]

\[
e(t)=r(t)-y(t)
\]

Symbols:
- \(u(t)\): controller output
- \(e(t)\): reference-minus-measured error
- \(r(t)\): reference signal
- \(y(t)\): measured output
- \(K_p,K_i,K_d\): controller gains

Units:
- Gain units depend on the units of the input error and output command.
- For a dimensionless transfer from error to command, gains still depend on whether the derivative and integral terms are expressed per second or second-integrated.

Assumptions:
- The controller is applied to a defined feedback loop.
- Output saturation and anti-windup should be handled where required.
- Derivative filtering may be necessary to limit noise sensitivity.

### 16. Breguet Range Equations

For a jet aircraft:

\[
R=\frac{V}{c_T}\frac{L}{D}
\ln\left(\frac{W_i}{W_f}\right)
\]

For a propeller aircraft:

\[
R=\frac{\eta_p}{c_P}\frac{L}{D}
\ln\left(\frac{W_i}{W_f}\right)
\]

Symbols:
- \(R\): range, m
- \(V\): representative cruise speed, m/s
- \(c_T\): thrust-specific fuel consumption, with units consistent with the jet equation
- \(c_P\): power-specific fuel consumption, with units consistent with the propeller equation
- \(\eta_p\): propeller efficiency, dimensionless
- \(L/D\): lift-to-drag ratio, dimensionless
- \(W_i,W_f\): initial and final aircraft weights, N

Assumptions:
- The equations represent idealized cruise conditions.
- Aerodynamic efficiency and consumption parameters are treated as constant over the cruise segment.
- Climb, descent, reserve fuel, and operational constraints are not explicitly included.

### 17. General Modelling Assumptions

Unless otherwise specified:

1. SI units are used throughout.
2. Gravitational acceleration is constant at approximately \(9.81\,m/s^2\).
3. The aircraft is treated as a rigid body.
4. Air density is constant only when a constant-density approximation is explicitly used.
5. Aerodynamic coefficients are assumed valid for the modelled flight regime.
6. Small-disturbance or point-mass equations are not a substitute for a full nonlinear six-degree-of-freedom model.
7. Controller gains and simulation results must be validated for the selected aircraft.

### 18. References

- Aircraft Flight Dynamics and Control textbooks.
- Standard flight mechanics and aerodynamics references.
- MATLAB and Simulink documentation for implementation-specific models.

These equations are intended for academic study, aircraft performance calculations, and preliminary flight-control simulation. Always verify coordinate conventions, units, and modelling assumptions before implementing the equations in MATLAB or Simulink.
