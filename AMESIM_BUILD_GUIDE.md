# Amesim implementation and comparison

## Access comes first

The [official Amesim trial page](https://trials.sw.siemens.com/en-US/trials/simcenter-amesim) was inspected. It offers a hosted environment without installation, but states that the complimentary trial is available only to qualified businesses. An individual job applicant's eligibility is therefore **not established**. Ask Siemens whether individual evaluation access is available; do not assume eligibility. Confirm trial duration, ability to create/save/export custom models, and pneumatic/mechanical libraries before starting.

An academic license is another route only if your institution confirms access, including any alumni restriction. The Student Edition requires checking current student eligibility and library/interface restrictions.

Do not buy a license just for the application without first establishing its cost and needed capabilities. Trial or institution access may be sufficient. Account creation, license acceptance, downloads, and any payment are separate user actions; none has been completed here.

## Build a subsystem before a robot

Create a new model called `pneumatic_actuator_position_control`. Use the installed version's documentation to select actual library components; this guide intentionally does not invent component IDs or submodel names.

1. Add regulated supply and atmospheric boundaries; set 600000 Pa and 101325 Pa **absolute**, respectively.
2. Add a double-acting cylinder or equivalent variable-volume chamber/mechanical converter assembly. Bore 20 mm, rod 8 mm, stroke 60 mm, dead volume 5 cm³ per chamber, initial extension 10 mm.
3. Connect its mechanical port to a 0.4 kg translational load, a 100 N/m spring referenced to the initial position, and 35 N*s/m damping.
4. Add four controlled restrictions representing supply/exhaust routing. Use maximum opening area 0.5 mm² and Cd=0.7 where the selected component permits those parameters. Add a first-order command response of 25 ms.
5. Instrument both chamber pressures, piston displacement/velocity, supply mass flow, exhaust mass flow, and valve command.
6. First test a locked piston with a small valve opening, then a sealed chamber, then motion without position control. Resolve units, pressure conventions, and force signs before closing the loop.
7. Add the baseline PI plus velocity-feedback controller, saturation, and anti-windup. Use the reference/load timing documented in the README. Compare the tuned gains only after baseline operation is stable.

## Compare like with like

The Python/MATLAB reference is isothermal. A default Amesim chamber may use energy balance and heat transfer. Configure equivalent assumptions if supported, or explicitly document that the models differ; do not demand matching traces from different physics.

Real valve submodels may use sonic conductance, critical ratio, spool overlap, leakage, or calibrated curves rather than an ideal nozzle area. Record that choice. Smoothed friction and the end-stop model also need matching assumptions. A comparison is meaningful only after these differences are reconciled.

Export a CSV on the same time grid or interpolate onto it. Preserve units, pressure conventions, solver settings, exact parameter values, Amesim version, and original project file. Compare position, both chamber pressures, and integrated mass supply. Establish an acceptance threshold before claiming agreement; begin with 0.1 mm position and 1% of supply pressure for matched assumptions, and explain any threshold change.

## Controller integration

Begin with an in-Amesim controller. Then verify which MATLAB/Simulink interface your license and version support. Keep the pneumatic/mechanical plant in Amesim and move the controller into Simulink, exchanging position/velocity feedback and valve command. Test communication-step sensitivity and compare against the monolithic run. FMI/FMU is optional and requires separate capability verification; an FMU has not been produced.

## Evidence to capture

- A schematic showing actual component connections.
- The saved native model and reproducible parameter set.
- Baseline versus tuned position/pressure plots.
- One load/friction/supply sensitivity table, including failed cases.
- A clear assumptions/limitations page and comparison CSVs.
- A short narrated demonstration you can reproduce during an interview.

Only label Amesim results as completed after an actual licensed run and the checks above.
