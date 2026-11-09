import subprocess
import re
import csv
import os

test_cases = [
    {"model": "../models/ur10.urdf", "floating": 0},
    {"model": "../models/franka-fr3.urdf", "floating": 0},
    {"model": "../models/unitree-go2.urdf", "floating": 1},
    {"model": "../models/spot.urdf", "floating": 1},
    {"model": "../models/atlas.urdf", "floating": 1}
]

rdl_bin = "../build/benchmark/benchmark_rdl"
pin_bin = "../build/benchmark/benchmark_pinocchio"
results = []

if not os.path.exists(rdl_bin) or not os.path.exists(pin_bin):
    print("[Error] Binaries not found. Please compile benchmark_rdl_2 and benchmark_pinocchio_2 first.")
    exit(1)

for case in test_cases:
    model_file = case["model"]
    floating_base = case["floating"]
    
    if not os.path.exists(model_file):
        print(f"[Warning] Model file {model_file} not found, skipping...")
        continue
        
    print(f"--> Testing model: {model_file} (Floating Base: {floating_base})")
    
    def run_binary(bin_path, name):
        cmd = [bin_path, model_file, str(floating_base)]
        try:
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            if res.returncode != 0:
                print(f"[Error] {name} failed for {model_file}")
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
    base_name = os.path.splitext(os.path.basename(model_file))[0]
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
        "RDL_FK": extract_val(rdl_out, "RDL", "FK Average Time"),
        "Pin_FK": extract_val(pin_out, "Pinocchio", "FK Average Time"),
        # Total Times
        "RDL_CRBA_Total": extract_val(rdl_out, "RDL", "CRBA Total Time"),
        "Pin_CRBA_Total": extract_val(pin_out, "Pinocchio", "CRBA Total Time"),
        "RDL_RNEA_Total": extract_val(rdl_out, "RDL", "RNEA Total Time"),
        "Pin_RNEA_Total": extract_val(pin_out, "Pinocchio", "RNEA Total Time"),
        "RDL_ABA_Total": extract_val(rdl_out, "RDL", "ABA Total Time"),
        "Pin_ABA_Total": extract_val(pin_out, "Pinocchio", "ABA Total Time"),
        # Pure Times
        "RDL_CRBA_Pure": extract_val(rdl_out, "RDL", "CRBA Pure Time"),
        "Pin_CRBA_Pure": extract_val(pin_out, "Pinocchio", "CRBA Pure Time"),
        "RDL_RNEA_Pure": extract_val(rdl_out, "RDL", "RNEA Pure Time"),
        "Pin_RNEA_Pure": extract_val(pin_out, "Pinocchio", "RNEA Pure Time"),
        "RDL_ABA_Pure": extract_val(rdl_out, "RDL", "ABA Pure Time"),
        "Pin_ABA_Pure": extract_val(pin_out, "Pinocchio", "ABA Pure Time"),
    }
    
    results.append(data)
    print(f"  [{model_label}] FK -> RDL: {data['RDL_FK']}us | Pin: {data['Pin_FK']}us")

csv_filename = "time_benchmark_results.csv"
with open(csv_filename, "w", newline="") as csvfile:
    fieldnames = list(results[0].keys())
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(results)

print(f"\n[Completed] Benchmarks finished with pure/FK timing info. Data saved to {csv_filename}")