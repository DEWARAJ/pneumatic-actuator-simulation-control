# Sampled sensing and low-supply protection upgrade

The final controller no longer reads true piston velocity. It uses a causal alpha-beta estimator driven only by the 5 ms position samples available to the controller. It also monitors measured supply pressure and latches a protective fault after four consecutive samples below 2 bar absolute.

The selected configuration retains **Kp=35, Ki=1, Kd=2** and uses alpha=0.7 and beta=0.6. Twenty-four controller/estimator combinations were evaluated using the six previously known tuning cases. Six regression cases and three fresh combinations remained outside selection. The performance limits from the controller-repair stage were not changed.

## Results

| Case | Split | RMSE (mm) | Extension overshoot (mm) | Velocity-estimate RMSE (m/s) | Result |
|---|---|---:|---:|---:|---|
| nominal | Tuning | 3.770 | 0.140 | 0.010 | Pass |
| heavy | Tuning | 3.880 | 0.530 | 0.010 | Pass |
| low supply | Tuning | 4.090 | 0.170 | 0.010 | Pass |
| slow valve | Tuning | 4.170 | 0.280 | 0.000 | Pass |
| combined | Tuning | 4.000 | 0.600 | 0.020 | Pass |
| earlier 15 ms delay failure | Tuning | 3.870 | 0.970 | 0.030 | Pass |
| high friction | Regression | 3.850 | 0.320 | 0.010 | Pass |
| position noise | Regression | 3.780 | 0.150 | 0.020 | Pass |
| feedback delay | Regression | 3.660 | 0.080 | 0.030 | Pass |
| pressure drop | Regression | 3.770 | 0.140 | 0.010 | Pass |
| mixed regression A | Regression | 4.070 | 0.390 | 0.020 | Pass |
| low-pressure regression | Regression | 4.330 | 0.350 | 0.020 | Pass |
| fresh delay mix | Fresh test | 4.010 | 0.450 | 0.030 | Pass |
| fresh mass mix | Fresh test | 3.980 | 0.390 | 0.020 | Pass |
| fresh sensor mix | Fresh test | 3.810 | 0.590 | 0.040 | Pass |

All **15/15 operating cases** and **25/25 repeated noise-seed runs** meet the fixed limits. Three step-refinement checks and the mass-conservation check also pass. Exact full-precision values, scenarios, seeds, and acceptance fields are recorded in `upgrade_results/report.json`.

![Velocity estimate compared with simulated truth](upgrade_results/velocity_estimator.png)

## Protection behavior

The interlock threshold is 200 kPa absolute, with a 20 ms debounce. Once tripped, it commands the closed-center valve to neutral, freezes the PI integrator, reports the fault, and requires an operator reset. In the insufficient-supply scenario it trips at **2.015 s**.

The unprotected simulation crosses the lower stroke boundary to -0.101 mm. The protected simulation remains within the 0–60 mm stroke and reduces signed supply use from 0.0468 g to 0.0194 g. It holds near 27 mm instead of tracking the later 15 mm command, so the case remains a declared performance failure. This is the intended distinction between safe failure handling and restored actuator authority.

![Protected and unprotected insufficient-supply response](upgrade_results/supply_protection.png)

## Simulink execution

`pneumatic_sampled_controller.slx` now contains the same alpha-beta estimator, anti-windup logic, and latched supply interlock. The plant output includes supply pressure as the eighth feedback signal, and a separate diagnostic output records estimated position, estimated velocity, and fault state. The model executed in Simulink and matched the Python nominal and heavy position traces within **0.000295 mm** and **0.000263 mm**, below the 0.01 mm comparison limit. See `upgrade_results/simulink_validation.json`.

![Upgraded modular Simulink architecture](upgrade_results/simulink_architecture.png)

## Reproduce and interpret

Run `python control_upgrade.py` after installing `requirements.txt`. In MATLAB, add `matlab/` to the path and run `build_modular_controller`. The Python run regenerates the estimator search, all scenario traces, seed study, protection comparison, plots, and validation report. The MATLAB builder regenerates and executes the modular model.

The estimator is evaluated against simulated ground-truth velocity for engineering analysis, but ground truth is not used by the controller. Sensor noise and delay remain synthetic. The interlock assumes a supply-pressure measurement and ideal thresholding; it has no pressure-sensor noise or diagnostic coverage model. Hardware calibration, thermal dynamics, native Amesim implementation, FMI export, and safety certification remain outside the evidence.
