import subprocess
import re
import csv
import os
from pathlib import Path

script_dir = Path(__file__).resolve().parent
models_base_dir = script_dir.parent / "models"

test_cases = [
    {"model": models_base_dir / "ur10.urdf", "floating": 0},
    {"model": models_base_dir / "franka-fr3.urdf", "floating": 0},
    {"model": models_base_dir / "unitree-go2.urdf", "floating": 1},
    {"model": models_base_dir / "spot.urdf", "floating": 1},
    {"model": models_base_dir / "atlas.urdf", "floating": 1},
    {"model": models_base_dir / "unitree-h1-2.urdf", "floating": 1}
]

is_windows = os.name == 'nt'
exe_ext = ".exe" if is_windows else ""

# Automatically locate build directory (supports both build and cmake-build-release)
possible_build_dirs = [
    script_dir.parent / "build" / "benchmark",
    script_dir.parent / "cmake-build-release" / "benchmark"
]

rdl_bin = None
pin_bin = None
for bdir in possible_build_dirs:
    r_cand = bdir / f"benchmark_rdl{exe_ext}"
    p_cand = bdir / f"benchmark_pinocchio{exe_ext}"
    if r_cand.exists() and p_cand.exists():
        rdl_bin = r_cand
        pin_bin = p_cand
        break

if not rdl_bin or not pin_bin:
    print("[Error] Binaries not found in 'build/benchmark' or 'cmake-build-release/benchmark'. Please compile first.")
    exit(1)

results = []

for case in test_cases:
    model_file = case["model"]
    floating_base = case["floating"]

    if not model_file.exists():
        print(f"[Warning] Model file {model_file} not found, skipping...")
        continue

    print(f"--> Testing model: {model_file.name} (Floating Base: {floating_base})")

    def run_binary(bin_path, name):
        cmd = [str(bin_path), str(model_file), str(floating_base)]
        try:
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            if res.returncode != 0:
                print(f"[Error] {name} failed for {model_file.name}:\n{res.stderr}")
                return None
            return res.stdout + res.stderr
        except Exception as e:
            print(f"[Error] Exception running {name}: {e}")
            return None

    pin_out = run_binary(pin_bin, "Pinocchio")
    rdl_out = run_binary(rdl_bin, "RDL")

    if not rdl_out or not pin_out:
        continue

    dof_match = re.search(r'(?:Total Joints \(nv\)|DOF \(nv\)):\s*(\d+)', rdl_out, re.IGNORECASE)
    dof_val = dof_match.group(1) if dof_match else "0"
    base_name = model_file.stem
    model_label = f"{base_name}({dof_val}dof)"

    def extract_val(output, framework, key):
        pattern = rf'-->\s*{framework}\s*{key}:\s*([eE\+\-\.\d]+)\s*us'
        match = re.search(pattern, output, re.IGNORECASE)
        return float(match.group(1)) if match else 0.0

    data = {
        "Model": base_name,
        "DOF": dof_val,
        "ModelLabel": model_label,
        # FK
        "RDL_FK": extract_val(rdl_out, "RDL", "FK Median Time"),
        "RDL_FK_P99": extract_val(rdl_out, "RDL", "FK P99 Time"),
        "Pin_FK": extract_val(pin_out, "Pinocchio", "FK Median Time"),
        "Pin_FK_P99": extract_val(pin_out, "Pinocchio", "FK P99 Time"),
        # Total Times
        "RDL_CRBA_Total": extract_val(rdl_out, "RDL", "CRBA Total Time"),
        "RDL_CRBA_Total_P99": extract_val(rdl_out, "RDL", "CRBA Total P99"),
        "Pin_CRBA_Total": extract_val(pin_out, "Pinocchio", "CRBA Total Time"),
        "Pin_CRBA_Total_P99": extract_val(pin_out, "Pinocchio", "CRBA Total P99"),
        "RDL_RNEA_Total": extract_val(rdl_out, "RDL", "RNEA Total Time"),
        "RDL_RNEA_Total_P99": extract_val(rdl_out, "RDL", "RNEA Total P99"),
        "Pin_RNEA_Total": extract_val(pin_out, "Pinocchio", "RNEA Total Time"),
        "Pin_RNEA_Total_P99": extract_val(pin_out, "Pinocchio", "RNEA Total P99"),
        "RDL_ABA_Total": extract_val(rdl_out, "RDL", "ABA Total Time"),
        "RDL_ABA_Total_P99": extract_val(rdl_out, "RDL", "ABA Total P99"),
        "Pin_ABA_Total": extract_val(pin_out, "Pinocchio", "ABA Total Time"),
        "Pin_ABA_Total_P99": extract_val(pin_out, "Pinocchio", "ABA Total P99"),
        # Pure Times
        "RDL_CRBA_Pure": extract_val(rdl_out, "RDL", "CRBA Pure Time"),
        "RDL_CRBA_Pure_P99": extract_val(rdl_out, "RDL", "CRBA Pure P99"),
        "Pin_CRBA_Pure": extract_val(pin_out, "Pinocchio", "CRBA Pure Time"),
        "Pin_CRBA_Pure_P99": extract_val(pin_out, "Pinocchio", "CRBA Pure P99"),
        "RDL_RNEA_Pure": extract_val(rdl_out, "RDL", "RNEA Pure Time"),
        "RDL_RNEA_Pure_P99": extract_val(rdl_out, "RDL", "RNEA Pure P99"),
        "Pin_RNEA_Pure": extract_val(pin_out, "Pinocchio", "RNEA Pure Time"),
        "Pin_RNEA_Pure_P99": extract_val(pin_out, "Pinocchio", "RNEA Pure P99"),
        "RDL_ABA_Pure": extract_val(rdl_out, "RDL", "ABA Pure Time"),
        "RDL_ABA_Pure_P99": extract_val(rdl_out, "RDL", "ABA Pure P99"),
        "Pin_ABA_Pure": extract_val(pin_out, "Pinocchio", "ABA Pure Time"),
        "Pin_ABA_Pure_P99": extract_val(pin_out, "Pinocchio", "ABA Pure P99"),
    }

    results.append(data)
    print(f"  [{model_label}] FK P50 -> RDL: {data['RDL_FK']:.2f}us | Pin: {data['Pin_FK']:.2f}us")

if results:
    csv_filename = script_dir / "time_benchmark_results.csv"
    with open(csv_filename, "w", newline="", encoding="utf-8") as csvfile:
        fieldnames = list(results[0].keys())
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)
    print(f"\n[Completed] Benchmarks finished. Data saved to {csv_filename}")
else:
    print("[Error] No benchmark results collected.")