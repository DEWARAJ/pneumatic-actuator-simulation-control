"""Compare a future, matched-assumption Amesim CSV against tuned.csv.

Required columns: time_s,position_m,pressure_a_pa_abs,pressure_b_pa_abs.
Usage: python compare_amesim.py export.csv
No Amesim export currently exists. Thresholds are declared before comparison.
"""
from pathlib import Path
import argparse
import json
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv",type=Path)
    args=parser.parse_args()
    root=Path(__file__).resolve().parent
    baseline=np.genfromtxt(root/"results"/"tuned.csv",delimiter=",",names=True)
    measured=np.genfromtxt(args.csv,delimiter=",",names=True)
    needed=("time_s","position_m","pressure_a_pa_abs","pressure_b_pa_abs")
    if measured.ndim!=1 or len(measured)<2 or not set(needed)<=set(measured.dtype.names or ()):
        raise ValueError("At least two rows and all required column names are needed")
    if not all(np.isfinite(measured[n]).all() for n in needed):
        raise ValueError("Nonfinite export values")
    if not (np.diff(measured["time_s"])>0).all():
        raise ValueError("Export time must be strictly increasing")
    if measured["time_s"][0]>baseline["time_s"][0] or measured["time_s"][-1]<baseline["time_s"][-1]:
        raise ValueError("Export must cover the complete reference interval; no extrapolation")
    if min(measured["pressure_a_pa_abs"].min(),measured["pressure_b_pa_abs"].min())<=0:
        raise ValueError("Absolute pressures must be positive")
    report={"scope":"Matched-assumption model comparison, not hardware validation",
            "limits":{"position_mm":.1,"pressure_pa":6000},"metrics":{}}
    for name in needed[1:]:
        error=np.interp(baseline["time_s"],measured["time_s"],measured[name])-baseline[name]
        scale=1000 if name=="position_m" else 1
        limit=.1 if name=="position_m" else 6000
        report["metrics"][name]={"max_abs_error":float(max(abs(error))*scale),
                                  "rmse":float(np.sqrt(np.mean(error**2))*scale),
                                  "unit":"mm" if scale==1000 else "Pa",
                                  "passed":bool(max(abs(error))*scale<=limit)}
    report["passed"]=all(item["passed"] for item in report["metrics"].values())
    target=root/"results"/"amesim_comparison.json"
    target.write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report["passed"] else 1)


if __name__=="__main__":
    main()
