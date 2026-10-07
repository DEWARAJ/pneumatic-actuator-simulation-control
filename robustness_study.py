"""Sampled controller, delayed/noisy sensing, fixed-step plant integration.

Run: python robustness_study.py
Does not overwrite the independently verified continuous reference results.
"""
from dataclasses import dataclass, asdict, replace
from pathlib import Path
import json
import math
import time
import numpy as np
from reference_model import Parameters, initial_state, reference, summarize, volumes, rhs

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "robustness_results"
LIMITS = dict(rmse_mm=4.5, extension_overshoot_mm=1.0,
              retraction_overshoot_mm=1.0, extension_settling_s=.7,
              late_mean_abs_error_mm=.5)


@dataclass(frozen=True)
class Scenario:
    name: str
    mass: float = .4
    load: float = 2.0
    friction: float = 1.0
    supply: float = 600000
    valve_tau: float = .025
    noise_mm: float = 0
    delay_ms: float = 0
    drop_pressure: float = 0
    valve_limit: float = 1.0
    emergency_load: float = 0


TRAIN = [Scenario("nominal"), Scenario("heavy",mass=.8,load=5),
         Scenario("low_supply",supply=400000), Scenario("slow_valve",valve_tau=.075)]
HOLDOUT = [Scenario("high_friction",friction=3),
           Scenario("position_noise",noise_mm=.10),
           Scenario("feedback_delay",delay_ms=10),
           Scenario("pressure_drop",drop_pressure=250000),
           Scenario("combined",mass=.7,load=4,friction=2,noise_mm=.10,delay_ms=10,
                    valve_tau=.05,drop_pressure=350000)]
FAILURE = Scenario("insufficient_supply",drop_pressure=120000,emergency_load=15)
SATURATION = Scenario("restricted_valve",valve_limit=.08)


def flow(p1,p2,area,p):
    """Equivalent math-library implementation for many fixed-step evaluations."""
    if min(p1,p2)<=0:
        raise ValueError("Nonpositive absolute pressure")
    if area<=0 or abs(p1-p2)<1e-9:
        return 0.0
    pu=max(p1,p2); ratio=min(p1,p2)/pu; g=p.gamma
    critical=(2/(g+1))**(g/(g-1))
    if ratio<=critical:
        factor=math.sqrt(g)*(2/(g+1))**((g+1)/(2*(g-1)))
    else:
        factor=math.sqrt(max(0,2*g/(g-1)*(ratio**(2/g)-ratio**((g+1)/g))))
    q=p.discharge*area*pu/math.sqrt(p.R*p.temperature)*factor
    return q if p1>p2 else -q


def plant(t,y,p,scenario,command,dint):
    x,v,ma,mb,_,z,_,_=y
    aa,ab=p.areas
    va=p.dead_volume+aa*x; vb=p.dead_volume+ab*(p.stroke-x)
    if min(va,vb,ma,mb)<=0:
        raise ValueError("Nonphysical state")
    pa=ma*p.R*p.temperature/va; pb=mb*p.R*p.temperature/vb
    supply=scenario.drop_pressure if scenario.drop_pressure and 2<=t<2.8 else p.supply
    aperture=min(abs(z),1)*p.valve_area
    if z>=0:
        sa=flow(supply,pa,aperture,p); sb=0
        ea=0; eb=flow(pb,p.atmosphere,aperture,p)
    else:
        sa=0; sb=flow(supply,pb,aperture,p)
        ea=flow(pa,p.atmosphere,aperture,p); eb=0
    leak=flow(pa,pb,p.leakage_area,p)
    load=(scenario.emergency_load if scenario.emergency_load else p.load) if 2<=t else 0
    force=aa*(pa-p.atmosphere)-ab*(pb-p.atmosphere)
    force-=p.spring*(x-p.initial_position)+p.damping*v+p.friction*math.tanh(v/p.friction_velocity)+load
    if x<0:
        force-=p.stop_stiffness*x+p.stop_damping*min(v,0)
    elif x>p.stroke:
        force-=p.stop_stiffness*(x-p.stroke)+p.stop_damping*max(v,0)
    return np.array([v,force/p.mass,sa-ea-leak,sb-eb+leak,dint,
                     (command-z)/p.valve_tau,sa+sb,ea+eb])


def run(gains,scenario,seed=20261005,dt=.001,antiwindup=True,control_dt=.005):
    p=replace(Parameters(),kp=gains[0],ki=gains[1],kd=gains[2],
              mass=scenario.mass,load=scenario.load,friction=scenario.friction,
              supply=scenario.supply,valve_tau=scenario.valve_tau)
    tick=round(control_dt/dt)
    if not math.isclose(tick*dt,control_dt,abs_tol=1e-12):
        raise ValueError("Controller period must be an integer number of plant steps")
    delay=round(scenario.delay_ms/1000/control_dt)
    if not math.isclose(delay*control_dt,scenario.delay_ms/1000,abs_tol=1e-12):
        raise ValueError("Delay must be an integer number of controller periods")
    rng=np.random.default_rng(seed)
    count=round(5/dt); ts=np.arange(count+1)*dt
    y=initial_state(p); trace=np.empty((count+1,8)); trace[0]=y
    commands=np.zeros(count+1); measured=np.zeros(count+1); raw_trace=np.zeros(count+1)
    # Complete feedback packet delay: position plus velocity.
    history=[]; command=0.; dint=0.; sensed=y[0]; raw=0.
    for k in range(count):
        t=ts[k]
        if k%tick==0:
            history.append((y[0]+rng.normal(0,scenario.noise_mm/1000),y[1]))
            sensed,sensed_v=history[max(0,len(history)-1-delay)]
            error=reference(t)-sensed
            raw=p.kp*error+p.ki*y[4]-p.kd*sensed_v
            command=max(-scenario.valve_limit,min(scenario.valve_limit,raw))
            dint=0. if antiwindup and ((raw>scenario.valve_limit and error>0) or
                    (raw< -scenario.valve_limit and error<0)) else error
        commands[k]=command; measured[k]=sensed; raw_trace[k]=raw
        # Piecewise events align with integer steps. Left limit at endpoints.
        end=np.nextafter(t+dt,t)
        f1=plant(t,y,p,scenario,command,dint)
        f2=plant(t+dt/2,y+dt*f1/2,p,scenario,command,dint)
        f3=plant(t+dt/2,y+dt*f2/2,p,scenario,command,dint)
        f4=plant(end,y+dt*f3,p,scenario,command,dint)
        y=y+dt*(f1+2*f2+2*f3+f4)/6
        trace[k+1]=y
    commands[-1]=command; measured[-1]=sensed; raw_trace[-1]=raw
    metrics=summarize(ts,trace,p)
    metrics["late_mean_abs_error_mm"]=metrics["steady_mean_abs_error_mm"]
    metrics["retraction_overshoot_mm"]=float(max(0,.015-trace[ts>=3,0].min())*1000)
    metrics["saturated_fraction"]=float(np.mean(abs(raw_trace[:-1])>=scenario.valve_limit))
    metrics["max_abs_integral_m_s"]=float(abs(trace[:,4]).max())
    metrics["within_stroke"]=bool(trace[:,0].min()>=0 and trace[:,0].max()<=p.stroke)
    metrics["acceptance"]={name:(metrics[name] is not None and metrics[name]<=limit)
                            for name,limit in LIMITS.items()}
    metrics["accepted"]=all(metrics["acceptance"].values()) and metrics["within_stroke"]
    return ts,trace,commands,measured,raw_trace,metrics


def score(metrics):
    # Declared before search. Constraint violations dominate modest speed gains.
    violation=sum(1 if metrics[name] is None else max(0,metrics[name]/limit-1)
                  for name,limit in LIMITS.items())
    if not metrics["within_stroke"]:
        violation+=10
    return 10*violation+metrics["rmse_mm"]/LIMITS["rmse_mm"]


def save_run(label,result,scenario,gains):
    t,y,cmd,sensed,raw,metrics=result
    p=replace(Parameters(),mass=scenario.mass,load=scenario.load,friction=scenario.friction,
              supply=scenario.supply,valve_tau=scenario.valve_tau)
    va,vb=volumes(y[:,0],p)
    columns=np.column_stack([t,[reference(tt) for tt in t],y[:,0],y[:,1],
          y[:,2]*p.R*p.temperature/va,y[:,3]*p.R*p.temperature/vb,cmd,y[:,5],sensed,raw,y[:,4],y[:,6],y[:,7]])
    np.savetxt(OUT/f"{label}.csv",columns,delimiter=",",comments="",
        header="time_s,reference_m,position_m,velocity_m_s,pressure_a_pa_abs,pressure_b_pa_abs,command,spool,measured_position_m,raw_command,integral_m_s,signed_supply_mass_kg,signed_exhaust_mass_kg")
    return metrics


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    OUT.mkdir(exist_ok=True)
    previous=(100,30,5)
    # Finite reproducible grid, not a claim of global optimization.
    initial_grid=[(kp,ki,kd) for kp in (40,70,100) for ki in (10,30) for kd in (5,10,15)]
    # Expanded after inspecting training failures, with unchanged acceptance limits.
    lower_gain_grid=[(kp,ki,kd) for kp in (30,40,50) for ki in (2,5) for kd in (1.5,2,2.5)]
    candidates=list(dict.fromkeys(initial_grid+lower_gain_grid+[previous]))
    search=[]; start=time.monotonic()
    for i,gains in enumerate(candidates):
        metrics=[run(gains,s)[-1] for s in TRAIN]
        objective=max(score(m) for m in metrics)+.1*np.mean([score(m) for m in metrics])
        search.append(dict(gains=gains,objective=float(objective),training_metrics=metrics))
        print(f"Candidate {i+1}/{len(candidates)} {gains}: {objective:.4f}",flush=True)
    winner=min(search,key=lambda c:c["objective"]); selected=tuple(winner["gains"])
    (OUT/"search.json").write_text(json.dumps(search,indent=2),encoding="utf-8")
    report={"limits":LIMITS,"controller_period_s":.005,"plant_step_s":.0005,
            "noise_model":"seeded independent position samples; std 0.1 mm; ideal velocity measurement",
            "seed":20261005,"previous_gains":previous,"selected_gains":selected,
            "selection":"minimax constraint penalty on four training cases; held-out cases unused in selection",
            "scenarios":{},"antiwindup":{},"validation":{}}
    # Confirm the independent fast RHS agrees with the continuous reference
    # when given the same command, feedback, plant parameters, and load timing.
    p=Parameters(); state=initial_state(p)
    state[0]=.025; state[1]=.01; va,vb=volumes(state[0],p)
    state[2]=180000*va/(p.R*p.temperature); state[3]=160000*vb/(p.R*p.temperature)
    state[4]=.0004; state[5]=.4
    discrepancy=0.
    for test_time in (.2,.7,2.2,3.5):
        error=reference(test_time)-state[0]
        raw=p.kp*error+p.ki*state[4]-p.kd*state[1]
        actual=plant(test_time,state,p,Scenario("equivalence"),max(-1,min(1,raw)),error)
        discrepancy=max(discrepancy,float(np.max(abs(actual-rhs(test_time,state,p)))))
    report["validation"]["reference_rhs_equivalence"]={"max_derivative_difference":discrepancy,"passed":discrepancy<1e-12}
    for scenario in TRAIN+HOLDOUT+[FAILURE]:
        row={"scenario":asdict(scenario),"split":"training" if scenario in TRAIN else
             ("failure_demonstration" if scenario==FAILURE else "holdout")}
        for label,gains in [("previous",previous),("robust",selected)]:
            result=run(gains,scenario,dt=.0005)
            row[label]=save_run(f"{scenario.name}_{label}",result,scenario,gains)
        report["scenarios"][scenario.name]=row
        print(f"Evaluated {scenario.name}",flush=True)
    # Same gains/plant/command/noise seed: change anti-windup alone.
    saturation_gains=(100,200,5)
    for enabled in (False,True):
        key="enabled" if enabled else "disabled"
        result=run(saturation_gains,SATURATION,dt=.0005,antiwindup=enabled)
        report["antiwindup"][key]=save_run(f"antiwindup_{key}",result,SATURATION,saturation_gains)
    report["antiwindup"]["gains"]=saturation_gains
    report["antiwindup"]["scenario"]=asdict(SATURATION)
    report["antiwindup"]["saturation_exercised"]=report["antiwindup"]["enabled"]["saturated_fraction"]>.05
    # Exact noise schedule remains on the fixed controller clock under dt refinement.
    for scenario in (TRAIN[0],TRAIN[1],HOLDOUT[2],HOLDOUT[4],SATURATION):
        gains=selected if scenario!=SATURATION else saturation_gains
        coarse=run(gains,scenario,dt=.0005)
        fine=run(gains,scenario,dt=.00025)
        delta=float(np.max(abs(coarse[1][:,0]-fine[1][::2,0]))*1000)
        report["validation"][scenario.name]={"step_refinement_difference_mm":delta,"passed":delta<.01}
    report["validation"]["mass_conservation"]={"passed":all(
        row["robust"]["max_mass_balance_residual_kg"]<1e-10 for row in report["scenarios"].values())}
    report["validation"]["antiwindup_saturation_exercised"]={"passed":report["antiwindup"]["saturation_exercised"]}
    report["noise_seed_study"]={"scope":"five reproducible seeds; no statistical reliability claim",
                               "position_noise":{},"combined":{}}
    for scenario in (HOLDOUT[1],HOLDOUT[4]):
        for seed in range(20261005,20261010):
            metrics=run(selected,scenario,dt=.0005,seed=seed)[-1]
            report["noise_seed_study"][scenario.name][str(seed)]=metrics
    report["validation"]["all_passed"]=all(v["passed"] for v in report["validation"].values())
    report["elapsed_s"]=time.monotonic()-start
    (OUT/"report.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    fig,axes=plt.subplots(2,2,figsize=(12,7),sharex=True)
    for ax,case in zip(axes.ravel(),("heavy","combined","feedback_delay","insufficient_supply")):
        for label in ("previous","robust"):
            data=np.genfromtxt(OUT/f"{case}_{label}.csv",delimiter=",",names=True)
            ax.plot(data["time_s"],data["position_m"]*1000,label=label)
        ax.plot(data["time_s"],data["reference_m"]*1000,"k--",label="target")
        ax.set(title=case.replace("_"," "),ylabel="Position (mm)");ax.grid(alpha=.2)
    axes[0,0].legend();axes[1,0].set_xlabel("Time (s)");axes[1,1].set_xlabel("Time (s)")
    fig.suptitle("Robustness study — synthetic isothermal plant, sampled feedback\nHeld-out tests and insufficient-supply failure remain visible")
    fig.tight_layout();fig.savefig(OUT/"robustness.png",dpi=160);plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
    for key in ("disabled","enabled"):
        data=np.genfromtxt(OUT/f"antiwindup_{key}.csv",delimiter=",",names=True)
        axes[0].plot(data["time_s"],data["position_m"]*1000,label=key)
        axes[1].plot(data["time_s"],data["integral_m_s"],label=key)
        axes[2].plot(data["time_s"],data["command"],label=key)
    axes[0].plot(data["time_s"],data["reference_m"]*1000,"k--",label="target")
    for ax,label in zip(axes,("Position (mm)","Integral (m s)","Valve command")):
        ax.set_ylabel(label);ax.grid(alpha=.2);ax.legend()
    axes[2].set_xlabel("Time (s)")
    fig.suptitle("Anti-windup isolation test — same gains, valve limited to +/-0.08")
    fig.tight_layout();fig.savefig(OUT/"antiwindup.png",dpi=160);plt.close(fig)
    print(json.dumps({"selected":selected,"validation":report["validation"],
                      "elapsed_s":report["elapsed_s"]},indent=2),flush=True)
    if not report["validation"]["all_passed"]:
        raise SystemExit("Step refinement or conservation check failed")


if __name__=="__main__":
    main()
