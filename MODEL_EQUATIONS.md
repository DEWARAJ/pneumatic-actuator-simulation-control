# Equations and validation scope

All variables use SI units and pressures are absolute. Let x increase on cylinder extension, Aa be the cap-end piston area, and Ab the rod-side annular area.

```text
Va = Vdead + Aa*x
Vb = Vdead + Ab*(stroke - x)
pa = ma*R*T/Va
pb = mb*R*T/Vb
d(ma)/dt = supply_to_A - A_to_exhaust - cross_port_leak
d(mb)/dt = supply_to_B - B_to_exhaust + cross_port_leak
dx/dt = v
M*dv/dt = Aa*(pa-patm) - Ab*(pb-patm)
          - k*(x-x0) - b*v - Fc*tanh(v/vsmooth) - Fload + Fstop
u_raw = Kp*(xref-x) + Ki*I - Kd*v
u = clip(u_raw, -1, 1)
d(z)/dt = (u-z)/valve_tau
```

The ambient-pressure terms include the atmospheric force on the exposed rod area. Integral action stops when the error would push an already saturated command further into saturation. Positive spool routes supply to A and B to atmosphere; negative spool reverses the routing. Zero spool closes the ports. Opening area is Amax*min(abs(z),1).

The final sampled controller replaces true velocity feedback with a causal alpha-beta estimate. At each 5 ms sample, using measured position xm:

```text
x_prediction = x_hat + dt*v_hat
residual = xm - x_prediction
x_hat = x_prediction + alpha*residual
v_hat = v_hat + (beta/dt)*residual
u_raw = Kp*(xref-x_hat) + Ki*I - Kd*v_hat
```

The recorded configuration uses alpha=0.7 and beta=0.6. Ground-truth simulated velocity is retained only for estimator-error analysis. A separate pressure interlock counts consecutive samples below 200 kPa absolute. Four low samples latch the fault; the controller then commands zero valve opening and stops integrating until an operator reset.

For upstream pressure pu, downstream pressure pd, pressure ratio r=pd/pu, gamma=g, and common temperature T, mass-flow magnitude is:

```text
q = Cd*A*pu/sqrt(R*T) * f(r)
rcritical = (2/(g+1))^(g/(g-1))
f(r) = sqrt(g)*(2/(g+1))^((g+1)/(2*(g-1)))                 [choked]
f(r) = sqrt(2*g/(g-1)*(r^(2/g)-r^((g+1)/g)))              [subsonic]
```

Flow sign follows the pressure difference. Each chamber satisfies the isothermal mass/volume relation:

```text
dp/dt = (R*T/V)*dm/dt - (p/V)*dV/dt
```

The software integrates mass instead of pressure. Two extra states integrate net boundary supply and exhaust flows; they permit a global mass-conservation check without summing low-resolution CSV samples.

## Checks

`validation.json` records flow reversal, equal-pressure flow, choked-flow scaling and branch continuity, unforced mechanical equilibrium, a sealed-chamber pressure-volume relation, positive chamber states, mass conservation, and agreement between RK45 and DOP853 with tighter tolerances. The MATLAB implementation supplies a second-language cross-check when its licensed runtime is available.

These checks demonstrate internal consistency of the implementation. They do **not** demonstrate accuracy against real pneumatic hardware. The sealed-volume check directly checks the constitutive relation; it is not a separate experimental benchmark. Physical validation needs identified geometry, valve flow curves, supply conditions, and measured pressure/displacement traces.

## Sources

- [MathWorks: Modeling Gas Systems](https://www.mathworks.com/help/simscape/ug/modeling-gas-systems.html) — gas-system components and modeling assumptions.
- [MathWorks: Pneumatic Piston Chamber](https://www.mathworks.com/help/simscape/ref/pneumaticpistonchamber.html) — ideal-gas chamber continuity. This legacy block is not the recommended block choice; use the current Gas library for any Simscape implementation.
- [Siemens pneumatic systems training](https://training.plm.automation.siemens.com/ilt/iltdescription.cfm?pID=TR-AMSIMPN____LMS__2020.2_B3) — Amesim pneumatic modeling/library workflow.
- [Original MATLAB soft-robot source](https://github.com/DEWARAJ/earthwork-inspired-soft-robot/blob/master/peristaltic.m) — motivation and existing-model limitations.
