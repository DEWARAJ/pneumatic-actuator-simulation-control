"""Repair known combined-disturbance failure; preserve the earlier study.

Known failed case is now a tuning case. Fresh test cases never enter selection.
Run: python controller_revision.py
"""
from dataclasses import asdict
from pathlib import Path
import json
import numpy as np
import robustness_study as study

OUT=Path(__file__).resolve().parent/"revision_results"
EARLIER_TESTS=[study.Scenario("fresh_mixed_a",mass=.65,load=3,friction=2,noise_mm=.15,
                      delay_ms=5,valve_tau=.06,drop_pressure=300000),
       study.Scenario("fresh_mixed_b",mass=.55,load=3.5,friction=2.5,noise_mm=.12,
                      delay_ms=15,valve_tau=.04,drop_pressure=400000),
       study.Scenario("fresh_low_pressure",mass=.6,load=3,supply=350000,
                      friction=2,noise_mm=.15,delay_ms=5,valve_tau=.04)]
TRAIN=study.TRAIN+[study.HOLDOUT[-1],EARLIER_TESTS[1]]
REGRESSION=study.HOLDOUT[:-1]+[EARLIER_TESTS[0],EARLIER_TESTS[2]]
FRESH=[study.Scenario("fresh_delay_mix",mass=.62,load=3.8,friction=2.2,
                      noise_mm=.18,delay_ms=10,valve_tau=.055,drop_pressure=320000),
       study.Scenario("fresh_mass_mix",mass=.75,load=4.5,friction=1.8,
                      noise_mm=.12,delay_ms=5,valve_tau=.045,drop_pressure=330000),
       study.Scenario("fresh_sensor_mix",mass=.5,load=3,friction=2.7,
                      noise_mm=.2,delay_ms=15,valve_tau=.035,drop_pressure=380000)]


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    OUT.mkdir(exist_ok=True)
    old=(40,2,2)
    candidates=[(kp,ki,kd) for kp in (30,35,40) for ki in (1,2) for kd in (1.5,2,2.5)]
    search=[]
    for i,gains in enumerate(candidates):
        metrics=[study.run(gains,case)[-1] for case in TRAIN]
        objective=max(study.score(m) for m in metrics)+.1*np.mean([study.score(m) for m in metrics])
        search.append(dict(gains=gains,objective=float(objective),all_training_accepted=all(m["accepted"] for m in metrics),metrics=metrics))
        print(f"Candidate {i+1}/{len(candidates)} {gains}: {objective:.4f}, all accepted={search[-1]['all_training_accepted']}",flush=True)
    eligible=[item for item in search if item["all_training_accepted"]]
    if not eligible:
        raise RuntimeError("No candidate meets the unchanged training limits")
    winner=min(eligible,key=lambda item:item["objective"])
    selected=tuple(winner["gains"])
    (OUT/"search.json").write_text(json.dumps(search,indent=2),encoding="utf-8")
    report=dict(previous_gains=old,selected_gains=selected,limits=study.LIMITS,
        selection="18 candidates; require all six tuning cases to meet unchanged limits, then minimize worst penalty plus 0.1 mean score",
        controller_period_s=.005,plant_step_s=.0005,
        known_combined_case="Combined and fresh_mixed_b promoted from observed failures into tuning; neither is held-out evidence in this revision",
        scenarios={},seed_study={},validation={})
    # Reuse CSV writer but route outputs into the revision directory.
    study.OUT=OUT
    for case in TRAIN+REGRESSION+FRESH+[study.FAILURE]:
        split="tuning" if case in TRAIN else ("fresh_test" if case in FRESH else
              ("physical_authority_failure" if case==study.FAILURE else "regression"))
        row=dict(split=split,scenario=asdict(case))
        for label,gains in (("previous",old),("revised",selected)):
            result=study.run(gains,case,dt=.0005,seed=20261015)
            row[label]=study.save_run(case.name+"_"+label,result,case,gains)
        report["scenarios"][case.name]=row
        print(f"Evaluated {case.name}: accepted={row['revised']['accepted']}",flush=True)
    for case in [study.HOLDOUT[-1],EARLIER_TESTS[1]]+FRESH:
        report["seed_study"][case.name]={}
        for seed in range(20261015,20261020):
            report["seed_study"][case.name][str(seed)]=study.run(selected,case,dt=.0005,seed=seed)[-1]
    for case in [study.HOLDOUT[-1],EARLIER_TESTS[1],FRESH[2]]:
        coarse=study.run(selected,case,dt=.0005,seed=20261015)
        fine=study.run(selected,case,dt=.00025,seed=20261015)
        delta=float(max(abs(coarse[1][:,0]-fine[1][::2,0]))*1000)
        report["validation"][case.name]=dict(position_step_refinement_mm=delta,passed=delta<.01)
    report["validation"]["mass_conservation"]=dict(passed=all(row["revised"]["max_mass_balance_residual_kg"]<1e-10 for row in report["scenarios"].values()))
    report["validation"]["all_passed"]=all(check["passed"] for check in report["validation"].values())
    # Check exact same seed as the originally observed failure as well.
    report["original_failure_seed_check"]=study.run(selected,study.HOLDOUT[-1],dt=.0005,seed=20261005)[-1]
    (OUT/"report.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    fig,axes=plt.subplots(2,2,figsize=(12,7),sharex=True)
    for ax,case in zip(axes.ravel(),["combined"]+[c.name for c in FRESH]):
        for label in ("previous","revised"):
            data=np.genfromtxt(OUT/f"{case}_{label}.csv",delimiter=",",names=True)
            ax.plot(data["time_s"],data["position_m"]*1000,label=label)
        ax.plot(data["time_s"],data["reference_m"]*1000,"k--",label="target")
        ax.set(title=case.replace("_"," "),ylabel="Position (mm)");ax.grid(alpha=.2)
    axes[0,0].legend();axes[1,0].set_xlabel("Time (s)");axes[1,1].set_xlabel("Time (s)")
    fig.suptitle("Controller revision — known failure plus fresh test cases\nUnchanged acceptance limits; synthetic isothermal plant")
    fig.tight_layout();fig.savefig(OUT/"revision.png",dpi=160);plt.close(fig)
    print(json.dumps(dict(selected_gains=selected,validation=report["validation"],
                         original_failure_repaired=report["original_failure_seed_check"]["accepted"]),indent=2))
    if not report["validation"]["all_passed"] or not report["original_failure_seed_check"]["accepted"]:
        raise SystemExit("Revision validation failed")


if __name__=="__main__":
    main()
