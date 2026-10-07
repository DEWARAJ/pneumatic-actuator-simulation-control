"""Isothermal pneumatic actuator reference. SI units; synthetic design parameters.

Run: python reference_model.py
Dependencies: numpy, scipy, matplotlib. Outputs go to results/.
This is not a native Amesim model or a validated physical robot model.
"""
from dataclasses import dataclass, asdict, replace
from pathlib import Path
import json
import numpy as np
from scipy.integrate import solve_ivp


@dataclass(frozen=True)
class Parameters:
    R: float = 287.05
    gamma: float = 1.4
    temperature: float = 293.15
    atmosphere: float = 101325.0
    supply: float = 600000.0  # ABSOLUTE pressure, not gauge
    bore: float = 0.020
    rod: float = 0.008
    stroke: float = 0.060
    dead_volume: float = 5e-6  # each chamber, m^3
    mass: float = 0.40
    spring: float = 100.0
    damping: float = 35.0
    friction: float = 1.0
    friction_velocity: float = 0.003
    valve_area: float = 0.50e-6
    discharge: float = 0.70
    valve_tau: float = 0.025
    leakage_area: float = 0.0
    initial_position: float = 0.010
    load: float = 2.0
    kp: float = 40.0
    ki: float = 8.0
    kd: float = 2.0
    stop_stiffness: float = 2e5
    stop_damping: float = 200.0

    @property
    def areas(self):
        return np.pi * self.bore**2 / 4, np.pi * (self.bore**2-self.rod**2) / 4


def reference(t):
    return 0.010 if t < 0.5 else (0.035 if t < 3.0 else 0.015)


def mass_flow(p1, p2, area, p):
    """Signed quasi-steady ideal-gas orifice flow, with choked/subsonic branches.

    All boundaries use the same prescribed temperature. Reversible port flow.
    Not a fitted valve characteristic, ISO 6358 calibration, or gas energy model.
    """
    if min(p1, p2) <= 0:
        raise ValueError("Absolute pressure must stay positive")
    if area <= 0 or abs(p1-p2) < 1e-9:
        return 0.0
    upstream, downstream = max(p1, p2), min(p1, p2)
    ratio = downstream/upstream
    critical = (2/(p.gamma+1))**(p.gamma/(p.gamma-1))
    if ratio <= critical:
        factor = np.sqrt(p.gamma)*(2/(p.gamma+1))**((p.gamma+1)/(2*(p.gamma-1)))
    else:
        factor = np.sqrt(max(0.0, 2*p.gamma/(p.gamma-1)*
                            (ratio**(2/p.gamma)-ratio**((p.gamma+1)/p.gamma))))
    magnitude = p.discharge*area*upstream/np.sqrt(p.R*p.temperature)*factor
    return magnitude if p1 > p2 else -magnitude


def volumes(x, p):
    a, b = p.areas
    # Penalty contact allows tiny numerical penetration; chamber volumes stay valid.
    return p.dead_volume+a*x, p.dead_volume+b*(p.stroke-x)


def initial_state(p):
    va, vb = volumes(p.initial_position, p)
    return np.array([p.initial_position, 0, p.atmosphere*va/(p.R*p.temperature),
                     p.atmosphere*vb/(p.R*p.temperature), 0, 0, 0, 0], dtype=float)


def rhs(t, y, p):
    x, v, ma, mb, integral, spool, _, _ = y
    va, vb = volumes(x, p)
    if min(va, vb, ma, mb) <= 0:
        raise ValueError("Nonphysical chamber state; check parameters/solver")
    pa, pb = ma*p.R*p.temperature/va, mb*p.R*p.temperature/vb
    error = reference(t)-x
    raw = p.kp*error+p.ki*integral-p.kd*v
    command = np.clip(raw, -1, 1)
    # Conditional integral action prevents accumulation into saturation.
    dint = 0.0 if (raw > 1 and error > 0) or (raw < -1 and error < 0) else error
    aperture = min(abs(spool), 1.0)*p.valve_area
    if spool >= 0:
        sa, sb = mass_flow(p.supply, pa, aperture, p), 0.0
        ea, eb = 0.0, mass_flow(pb, p.atmosphere, aperture, p)
    else:
        sa, sb = 0.0, mass_flow(p.supply, pb, aperture, p)
        ea, eb = mass_flow(pa, p.atmosphere, aperture, p), 0.0
    leak = mass_flow(pa, pb, p.leakage_area, p)
    aa, ab = p.areas
    external = p.load if t >= 2.0 else 0.0
    force = aa*(pa-p.atmosphere)-ab*(pb-p.atmosphere)
    force -= p.spring*(x-p.initial_position)+p.damping*v
    force -= p.friction*np.tanh(v/p.friction_velocity)+external
    if x < 0:
        force += -p.stop_stiffness*x-p.stop_damping*min(v, 0)
    elif x > p.stroke:
        force -= p.stop_stiffness*(x-p.stroke)+p.stop_damping*max(v, 0)
    return [v, force/p.mass, sa-ea-leak, sb-eb+leak, dint,
            (command-spool)/p.valve_tau, sa+sb, ea+eb]


def simulate(p, rtol=1e-7, method="RK45"):
    # Restart at reference/load discontinuities; do not interpolate across them.
    state = initial_state(p)
    ts, ys = [], []
    for left, right in [(0, .5), (.5, 2), (2, 3), (3, 5)]:
        grid = np.linspace(left, right, round((right-left)/.002)+1)
        # Left limit at each segment's terminal discontinuity.
        fun = lambda t, y: rhs(min(t, np.nextafter(right, left)), y, p)
        sol = solve_ivp(fun, [left, right], state, t_eval=grid, method=method,
                        rtol=rtol, atol=np.array([1e-10,1e-9,1e-13,1e-13,1e-10,1e-9,1e-13,1e-13]),
                        max_step=.002)
        if not sol.success:
            raise RuntimeError(sol.message)
        ts.append(sol.t if left == 0 else sol.t[1:])
        ys.append(sol.y if left == 0 else sol.y[:, 1:])
        state = sol.y[:, -1]
    return np.concatenate(ts), np.concatenate(ys, axis=1).T


def summarize(t, y, p):
    desired = np.array([reference(tt) for tt in t])
    error = desired-y[:,0]
    active = t >= .5
    steady = ((t >= 1.7)&(t < 2)) | (t >= 4.7)
    va, vb = volumes(y[:,0], p)
    pa = y[:,2]*p.R*p.temperature/va
    pb = y[:,3]*p.R*p.temperature/vb
    conservation = y[:,2]+y[:,3]-(y[0,2]+y[0,3])-y[:,6]+y[:,7]
    extension = (t >= .5)&(t < 2)
    # Settling within +/-0.5 mm, before the load disturbance.
    ext_t, ext_e = t[extension], abs(error[extension])
    outside = np.flatnonzero(ext_e > .0005)
    settle = None if len(outside) and outside[-1] == len(ext_e)-1 else (
        float(ext_t[outside[-1]+1]-.5) if len(outside) else 0.0)
    return dict(rmse_mm=float(np.sqrt(np.mean(error[active]**2))*1000),
                steady_mean_abs_error_mm=float(np.mean(abs(error[steady]))*1000),
                extension_overshoot_mm=float(max(0, np.max(y[extension,0])-.035)*1000),
                extension_settling_s=settle,
                signed_supply_mass_g=float(y[-1,6]*1000),
                min_position_mm=float(np.min(y[:,0])*1000),
                max_position_mm=float(np.max(y[:,0])*1000),
                min_pressure_bar_abs=float(min(pa.min(),pb.min())/1e5),
                max_pressure_bar_abs=float(max(pa.max(),pb.max())/1e5),
                max_mass_balance_residual_kg=float(max(abs(conservation))))


def validate(p, t, y):
    checks = {}
    a = 0.3e-6
    q = mass_flow(p.supply, p.atmosphere, a, p)
    checks["zero_flow_equal_pressure"] = mass_flow(p.atmosphere,p.atmosphere,a,p) == 0
    checks["flow_reversal"] = abs(q+mass_flow(p.atmosphere,p.supply,a,p)) < 1e-14
    checks["choked_area_scaling"] = abs(mass_flow(p.supply,p.atmosphere,2*a,p)-2*q) < 1e-14
    checks["choked_back_pressure_independence"] = abs(mass_flow(p.supply,2*p.atmosphere,a,p)-q) < 1e-14
    critical = (2/(p.gamma+1))**(p.gamma/(p.gamma-1))
    q1=mass_flow(p.supply,p.supply*critical*(1-1e-6),a,p)
    q2=mass_flow(p.supply,p.supply*critical*(1+1e-6),a,p)
    checks["flow_branch_continuity"] = abs(q1-q2)/q1 < 1e-8
    closed=replace(p,kp=0,ki=0,kd=0,load=0)
    equilibrium=np.array(rhs(0,initial_state(closed),closed))
    checks["unforced_equilibrium"] = bool(np.max(abs(equilibrium)) < 1e-10)
    # Independent sealed chamber analytical pressure-volume relation.
    va0,_=volumes(p.initial_position,p)
    ma=initial_state(p)[2]
    va1,_=volumes(.025,p)
    checks["sealed_isothermal_pressure_volume"] = abs(ma*p.R*p.temperature/va1-p.atmosphere*va0/va1)<1e-9
    metrics=summarize(t,y,p)
    checks["global_mass_conservation"] = metrics["max_mass_balance_residual_kg"] < 1e-10
    checks["positive_mass_and_volume"] = bool(np.min(y[:,2:4])>0 and min(np.min(v) for v in volumes(y[:,0],p))>0)
    checks["nominal_no_endstop_contact"] = bool(y[:,0].min()>=0 and y[:,0].max()<=p.stroke)
    t2,y2=simulate(p,rtol=1e-9,method="DOP853")
    delta=float(np.max(abs(y2[:,0]-y[:,0]))*1000)
    checks["independent_solver_and_tolerance_agreement"] = delta < .01
    checks={name:bool(value) for name,value in checks.items()}
    return dict(checks=checks,passed=all(checks.values()),
                max_position_difference_mm=delta,
                scope="Analytical/software/numerical checks only; no hardware or Amesim validation")


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    output=Path(__file__).resolve().parent/"results"
    output.mkdir(exist_ok=True)
    baseline=Parameters()
    tuned=replace(baseline,kp=100,ki=30,kd=5)
    runs={}
    cases={"baseline":baseline,"tuned":tuned,"heavy_load":replace(tuned,mass=.8,load=5),
           "low_supply":replace(tuned,supply=400000),
           "high_friction":replace(tuned,friction=3),
           "valve_lag":replace(tuned,valve_tau=.075),
           "cross_port_leak":replace(tuned,leakage_area=.005e-6)}
    metrics={}
    for name,p in cases.items():
        t,y=simulate(p)
        va,vb=volumes(y[:,0],p)
        columns=np.column_stack([t,[reference(tt) for tt in t],y[:,0],y[:,1],
                    y[:,2]*p.R*p.temperature/va,y[:,3]*p.R*p.temperature/vb,y[:,5],y[:,6],y[:,7]])
        np.savetxt(output/f"{name}.csv",columns,delimiter=",",comments="",
                   header="time_s,reference_m,position_m,velocity_m_s,pressure_a_pa_abs,pressure_b_pa_abs,valve_spool,signed_supply_mass_kg,signed_exhaust_mass_kg")
        runs[name]=(t,y,p)
        metrics[name]=summarize(t,y,p)
    validation=validate(tuned,*runs["tuned"][:2])
    (output/"validation.json").write_text(json.dumps(validation,indent=2),encoding="utf-8")
    (output/"metrics.json").write_text(json.dumps(metrics,indent=2),encoding="utf-8")
    (output/"parameters.json").write_text(json.dumps({k:asdict(v) for k,v in cases.items()},indent=2),encoding="utf-8")
    plt.rcParams.update({"font.size":10,"axes.spines.top":False,"axes.spines.right":False})
    fig,axes=plt.subplots(3,1,figsize=(10,9),sharex=True)
    for name in ("baseline","tuned"):
        t,y,p=runs[name]
        axes[0].plot(t,y[:,0]*1000,label=name)
    axes[0].plot(t,[reference(tt)*1000 for tt in t],"k--",label="reference")
    axes[0].set_ylabel("Position (mm)");axes[0].legend()
    va,vb=volumes(y[:,0],p)
    axes[1].plot(t,y[:,2]*p.R*p.temperature/va/1e5,label="chamber A")
    axes[1].plot(t,y[:,3]*p.R*p.temperature/vb/1e5,label="chamber B")
    axes[1].set_ylabel("Pressure (bar absolute)");axes[1].legend()
    axes[2].plot(t,y[:,5],label="tuned valve spool")
    axes[2].set_ylabel("Spool (-1 to +1)");axes[2].set_xlabel("Time (s)")
    for ax in axes:
        ax.axvline(2,color="grey",ls=":");ax.grid(alpha=.2)
    fig.suptitle("Pneumatic actuator — synthetic isothermal reference\n2 N resisting-load step at 2 s; no physical or Amesim validation")
    fig.tight_layout();fig.savefig(output/"response.png",dpi=160);fig.savefig(output/"response.svg");plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,4))
    for name in cases:
        t,y,p=runs[name];ax.plot(t,(np.array([reference(tt) for tt in t])-y[:,0])*1000,label=name)
    ax.set(xlabel="Time (s)",ylabel="Tracking error (mm)",title="Sensitivity scenarios — same tuned controller, except baseline")
    ax.legend(ncol=3);ax.grid(alpha=.2);fig.tight_layout();fig.savefig(output/"sensitivity.png",dpi=160);plt.close(fig)
    print(json.dumps({"metrics":metrics,"validation":validation},indent=2))
    if not validation["passed"]:
        raise SystemExit("Validation failed")


if __name__ == "__main__":
    main()
