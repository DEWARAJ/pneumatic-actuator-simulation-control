"""Realistic sensing and low-supply protection for the sampled controller.

The earlier studies remain unchanged. This stage estimates velocity from sampled,
possibly noisy/delayed position and latches a protective low-supply fault.
Run: python control_upgrade.py
"""
from dataclasses import asdict, replace
from pathlib import Path
import json
import math
import numpy as np

import robustness_study as study
from reference_model import Parameters, initial_state, reference, summarize, volumes
import controller_revision as revision

OUT = Path(__file__).resolve().parent / "upgrade_results"
CONTROL_DT = .005
PRESSURE_TRIP_PA = 200000.0
PRESSURE_RESET_PA = 240000.0
TRIP_SAMPLES = 4


def available_supply(t, scenario, p):
    return scenario.drop_pressure if scenario.drop_pressure and 2 <= t < 2.8 else p.supply


def run(gains, scenario, estimator_alpha=.7, estimator_beta=.2, seed=20261015, dt=.001,
        pressure_protection=True):
    """Run with a causal alpha-beta state estimator and pressure interlock."""
    p = replace(Parameters(), kp=gains[0], ki=gains[1], kd=gains[2],
                mass=scenario.mass, load=scenario.load, friction=scenario.friction,
                supply=scenario.supply, valve_tau=scenario.valve_tau)
    tick = round(CONTROL_DT / dt)
    if not math.isclose(tick * dt, CONTROL_DT, abs_tol=1e-12):
        raise ValueError("Controller period must be an integer number of plant steps")
    delay = round(scenario.delay_ms / 1000 / CONTROL_DT)
    if not math.isclose(delay * CONTROL_DT, scenario.delay_ms / 1000, abs_tol=1e-12):
        raise ValueError("Delay must be an integer number of controller periods")

    rng = np.random.default_rng(seed)
    count = round(5 / dt)
    ts = np.arange(count + 1) * dt
    y = initial_state(p)
    trace = np.empty((count + 1, 8)); trace[0] = y
    commands = np.zeros(count + 1)
    measured = np.zeros(count + 1)
    position_estimate = np.zeros(count + 1)
    velocity_estimate = np.zeros(count + 1)
    raw_trace = np.zeros(count + 1)
    supply_trace = np.zeros(count + 1)
    fault_trace = np.zeros(count + 1, dtype=int)

    history = []
    command = 0.; dint = 0.; sensed = y[0]
    position_hat = y[0]; velocity_hat = 0.; raw = 0.
    low_count = 0; fault_latched = False

    for k in range(count):
        t = ts[k]
        supply = available_supply(t, scenario, p)
        if k % tick == 0:
            history.append(y[0] + rng.normal(0, scenario.noise_mm / 1000))
            sensed = history[max(0, len(history) - 1 - delay)]
            predicted_position = position_hat + CONTROL_DT * velocity_hat
            residual = sensed - predicted_position
            position_hat = predicted_position + estimator_alpha * residual
            velocity_hat += estimator_beta / CONTROL_DT * residual

            if pressure_protection and not fault_latched:
                low_count = low_count + 1 if supply < PRESSURE_TRIP_PA else 0
                if low_count >= TRIP_SAMPLES:
                    fault_latched = True
            # Latching represents a required operator reset. Reset threshold is
            # recorded for a future state-machine extension, not automatic reset.
            error = reference(t) - position_hat
            raw = p.kp * error + p.ki * y[4] - p.kd * velocity_hat
            if fault_latched:
                command = 0.
                dint = 0.
            else:
                command = max(-scenario.valve_limit, min(scenario.valve_limit, raw))
                dint = 0. if ((raw > scenario.valve_limit and error > 0) or
                              (raw < -scenario.valve_limit and error < 0)) else error

        commands[k] = command; measured[k] = sensed
        position_estimate[k] = position_hat; velocity_estimate[k] = velocity_hat; raw_trace[k] = raw
        supply_trace[k] = supply; fault_trace[k] = int(fault_latched)
        end = np.nextafter(t + dt, t)
        f1 = study.plant(t, y, p, scenario, command, dint)
        f2 = study.plant(t + dt / 2, y + dt * f1 / 2, p, scenario, command, dint)
        f3 = study.plant(t + dt / 2, y + dt * f2 / 2, p, scenario, command, dint)
        f4 = study.plant(end, y + dt * f3, p, scenario, command, dint)
        y = y + dt * (f1 + 2 * f2 + 2 * f3 + f4) / 6
        trace[k + 1] = y

    commands[-1] = command; measured[-1] = sensed
    position_estimate[-1] = position_hat; velocity_estimate[-1] = velocity_hat; raw_trace[-1] = raw
    supply_trace[-1] = available_supply(ts[-1], scenario, p)
    fault_trace[-1] = int(fault_latched)
    metrics = summarize(ts, trace, p)
    metrics["late_mean_abs_error_mm"] = metrics["steady_mean_abs_error_mm"]
    metrics["retraction_overshoot_mm"] = float(max(0, .015 - trace[ts >= 3, 0].min()) * 1000)
    metrics["saturated_fraction"] = float(np.mean(abs(raw_trace[:-1]) >= scenario.valve_limit))
    metrics["max_abs_integral_m_s"] = float(abs(trace[:, 4]).max())
    metrics["max_velocity_estimation_error_m_s"] = float(np.max(abs(velocity_estimate - trace[:, 1])))
    metrics["velocity_estimation_rmse_m_s"] = float(np.sqrt(np.mean((velocity_estimate - trace[:, 1]) ** 2)))
    metrics["within_stroke"] = bool(trace[:, 0].min() >= 0 and trace[:, 0].max() <= p.stroke)
    metrics["fault_latched"] = bool(fault_latched)
    trip_indices = np.flatnonzero(fault_trace)
    metrics["fault_trip_time_s"] = float(ts[trip_indices[0]]) if len(trip_indices) else None
    metrics["acceptance"] = {name: metrics[name] is not None and metrics[name] <= limit
                             for name, limit in study.LIMITS.items()}
    metrics["accepted"] = all(metrics["acceptance"].values()) and metrics["within_stroke"]
    return (ts, trace, commands, measured, position_estimate, velocity_estimate,
            raw_trace, supply_trace, fault_trace, metrics)


def save_run(label, result, scenario):
    t, y, cmd, sensed, xhat, vhat, raw, supply, fault, metrics = result
    p = replace(Parameters(), mass=scenario.mass, load=scenario.load,
                friction=scenario.friction, supply=scenario.supply,
                valve_tau=scenario.valve_tau)
    va, vb = volumes(y[:, 0], p)
    data = np.column_stack([t, [reference(tt) for tt in t], y[:, 0], y[:, 1],
        sensed, xhat, vhat, y[:, 2] * p.R * p.temperature / va,
        y[:, 3] * p.R * p.temperature / vb, supply, cmd, raw, y[:, 4], fault])
    np.savetxt(OUT / f"{label}.csv", data, delimiter=",", comments="", header=
        "time_s,reference_m,position_m,true_velocity_m_s,measured_position_m,estimated_position_m,estimated_velocity_m_s,"
        "pressure_a_pa_abs,pressure_b_pa_abs,supply_pressure_pa_abs,command,raw_command,integral_m_s,low_supply_fault")
    return metrics


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    OUT.mkdir(exist_ok=True)
    previous_gains = (35, 1, 2)
    # Controller and estimator are selected together using known tuning cases only.
    candidates = [(kp, 1, kd, alpha, beta) for kp in (30, 35)
                  for kd in (1.5, 2) for alpha in (.3, .5, .7) for beta in (.4, .6)]
    tuning = revision.TRAIN
    search = []
    for kp, ki, kd, alpha, beta in candidates:
        gains = (kp, ki, kd)
        metrics = [run(gains, case, estimator_alpha=alpha, estimator_beta=beta)[-1] for case in tuning]
        score = max(study.score(m) for m in metrics) + .1 * np.mean([study.score(m) for m in metrics])
        search.append(dict(gains=gains, estimator_alpha=alpha, estimator_beta=beta, objective=float(score),
                           all_tuning_accepted=all(m["accepted"] for m in metrics), metrics=metrics))
    (OUT / "estimator_search.json").write_text(json.dumps(search, indent=2), encoding="utf-8")
    eligible = [item for item in search if item["all_tuning_accepted"]]
    if not eligible:
        raise RuntimeError("No estimator setting meets unchanged tuning limits")
    winner = min(eligible, key=lambda item: item["objective"])
    gains = tuple(winner["gains"])
    selected_alpha = winner["estimator_alpha"]
    selected_beta = winner["estimator_beta"]

    report = dict(previous_controller_gains=previous_gains, controller_gains=gains,
        estimator="causal alpha-beta position/velocity estimator",
        selected_estimator_alpha=selected_alpha, selected_estimator_beta=selected_beta,
        candidates_evaluated=len(candidates),
        controller_period_s=CONTROL_DT, limits=study.LIMITS,
        pressure_protection=dict(trip_pa_abs=PRESSURE_TRIP_PA, reset_pa_abs=PRESSURE_RESET_PA,
                                 debounce_samples=TRIP_SAMPLES, debounce_s=TRIP_SAMPLES*CONTROL_DT,
                                 action="latch fault; command neutral; freeze integrator; operator reset required"),
        scenarios={}, seed_study={}, validation={})
    cases = revision.TRAIN + revision.REGRESSION + revision.FRESH
    for case in cases:
        split = "tuning" if case in revision.TRAIN else ("fresh_test" if case in revision.FRESH else "regression")
        result = run(gains, case, estimator_alpha=selected_alpha, estimator_beta=selected_beta, dt=.0005)
        report["scenarios"][case.name] = dict(split=split, scenario=asdict(case),
                                                upgraded=save_run(case.name, result, case))
    failure_unprotected = run(gains, study.FAILURE, estimator_alpha=selected_alpha,
                              estimator_beta=selected_beta,
                              dt=.0005, pressure_protection=False)
    failure_protected = run(gains, study.FAILURE, estimator_alpha=selected_alpha,
                            estimator_beta=selected_beta,
                            dt=.0005, pressure_protection=True)
    report["supply_failure"] = dict(scenario=asdict(study.FAILURE),
        unprotected=save_run("insufficient_supply_unprotected", failure_unprotected, study.FAILURE),
        protected=save_run("insufficient_supply_protected", failure_protected, study.FAILURE))

    seed_cases = [study.HOLDOUT[-1], revision.EARLIER_TESTS[1]] + revision.FRESH
    for case in seed_cases:
        report["seed_study"][case.name] = {}
        for seed in range(20261015, 20261020):
            report["seed_study"][case.name][str(seed)] = run(
                gains, case, estimator_alpha=selected_alpha, estimator_beta=selected_beta,
                dt=.0005, seed=seed)[-1]
    for case in [study.HOLDOUT[-1], revision.EARLIER_TESTS[1], revision.FRESH[2]]:
        coarse = run(gains, case, estimator_alpha=selected_alpha, estimator_beta=selected_beta, dt=.0005)
        fine = run(gains, case, estimator_alpha=selected_alpha, estimator_beta=selected_beta, dt=.00025)
        delta = float(np.max(abs(coarse[1][:, 0] - fine[1][::2, 0])) * 1000)
        report["validation"][case.name] = dict(position_step_refinement_mm=delta, passed=delta < .01)
    report["validation"]["operating_cases"] = dict(
        passed=all(v["upgraded"]["accepted"] for v in report["scenarios"].values()),
        count=len(report["scenarios"]))
    report["validation"]["seed_runs"] = dict(
        passed=all(m["accepted"] for v in report["seed_study"].values() for m in v.values()),
        count=sum(len(v) for v in report["seed_study"].values()))
    pf = report["supply_failure"]["protected"]
    report["validation"]["pressure_interlock"] = dict(
        passed=pf["fault_latched"] and pf["fault_trip_time_s"] is not None and
               2.0 <= pf["fault_trip_time_s"] <= 2.0 + TRIP_SAMPLES*CONTROL_DT + 1e-9,
        trip_time_s=pf["fault_trip_time_s"])
    report["validation"]["mass_conservation"] = dict(passed=all(
        v["upgraded"]["max_mass_balance_residual_kg"] < 1e-10 for v in report["scenarios"].values()))
    report["validation"]["all_passed"] = all(v["passed"] for v in report["validation"].values())
    (OUT / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
    for ax, name in zip(axes.ravel(), ("position_noise", "combined", "fresh_sensor_mix", "fresh_delay_mix")):
        data = np.genfromtxt(OUT/f"{name}.csv", delimiter=",", names=True)
        ax.plot(data["time_s"], data["true_velocity_m_s"], label="true velocity")
        ax.plot(data["time_s"], data["estimated_velocity_m_s"], label="estimated velocity", alpha=.85)
        ax.set(title=name.replace("_", " "), ylabel="Velocity (m/s)"); ax.grid(alpha=.2)
    axes[0, 0].legend(); axes[1, 0].set_xlabel("Time (s)"); axes[1, 1].set_xlabel("Time (s)")
    fig.suptitle("Causal velocity estimation from sampled position")
    fig.tight_layout(); fig.savefig(OUT/"velocity_estimator.png", dpi=160); plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for label in ("unprotected", "protected"):
        data = np.genfromtxt(OUT/f"insufficient_supply_{label}.csv", delimiter=",", names=True)
        axes[0].plot(data["time_s"], data["position_m"]*1000, label=label)
        axes[1].plot(data["time_s"], data["command"], label=label)
    axes[0].plot(data["time_s"], data["reference_m"]*1000, "k--", label="target")
    axes[2].plot(data["time_s"], data["supply_pressure_pa_abs"]/1e5, label="supply")
    axes[2].plot(data["time_s"], data["low_supply_fault"], label="latched fault")
    for ax, ylabel in zip(axes, ("Position (mm)", "Valve command", "bar abs / fault")):
        ax.set_ylabel(ylabel); ax.grid(alpha=.2); ax.legend()
    axes[2].set_xlabel("Time (s)")
    fig.suptitle("Low-supply protection: neutral valve and frozen integrator")
    fig.tight_layout(); fig.savefig(OUT/"supply_protection.png", dpi=160); plt.close(fig)
    print(json.dumps(dict(controller_gains=gains, selected_estimator_alpha=selected_alpha,
                          selected_estimator_beta=selected_beta,
                          validation=report["validation"],
                          protected_failure=pf), indent=2))
    if not report["validation"]["all_passed"]:
        raise SystemExit("Upgrade validation failed")


if __name__ == "__main__":
    main()
