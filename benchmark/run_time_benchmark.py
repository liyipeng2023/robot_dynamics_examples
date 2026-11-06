import subprocess
import re
import csv
import os

# 定义测试模型及对应的浮动基座标志 (1: 有浮动基座, 0: 固定基座)
test_cases = [
    {"model": "../models/ur10.urdf", "floating": 0},
    {"model": "../models/franka-fr3.urdf", "floating": 0},
    {"model": "../models/unitree-go2.urdf", "floating": 0},
    {"model": "../models/spot.urdf", "floating": 1},
    {"model": "../models/atlas.urdf", "floating": 1}
]

rdl_bin = "../build/benchmark/benchmark_rdl"
pin_bin = "../build/benchmark/benchmark_pinocchio"
results = []

if not os.path.exists(rdl_bin) or not os.path.exists(pin_bin):
    print("[Error] Binaries not found. Please compile benchmark_rdl and benchmark_pinocchio first.")
    exit(1)

for case in test_cases:
    model_file = case["model"]
    floating_base = case["floating"]
    
    if not os.path.exists(model_file):
        print(f"[Warning] Model file {model_file} not found, skipping...")
        continue
        
    print(f"\n==========================================")
    print(f"Testing model: {model_file} (Floating Base: {floating_base})")
    print(f"==========================================")
    
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

    rdl_out = run_binary(rdl_bin, "RDL")
    pin_out = run_binary(pin_bin, "Pinocchio")
    
    if not rdl_out or not pin_out:
        continue

    # 1. 自动提取自由度 nv (DOF)
    dof_match = re.search(r'(?:Total Joints \(nv\)|DOF \(nv\)):\s*(\d+)', rdl_out, re.IGNORECASE)
    dof_val = dof_match.group(1) if dof_match else "0"
    
    # 2. 构造带dof的模型名称，如 atlas(36dof)
    base_name = os.path.splitext(os.path.basename(model_file))[0]
    # 去除多余后缀（如 -fr3 等视情况保留）
    model_label = f"{base_name}({dof_val}dof)"

    # 3. 提取算法耗时辅助函数
    def extract_time(output, framework, algo):
        pattern = rf'-->\s*{framework}\s*{algo}\s*Average Time:\s*([eE\+\-\.\d]+)\s*us'
        match = re.search(pattern, output, re.IGNORECASE)
        return float(match.group(1)) if match else 0.0

    data = {
        "Model": base_name,
        "DOF": dof_val,
        "ModelLabel": model_label,
        "RDL_CRBA": extract_time(rdl_out, "RDL", "CRBA"),
        "Pin_CRBA": extract_time(pin_out, "Pinocchio", "CRBA"),
        "RDL_RNEA": extract_time(rdl_out, "RDL", "RNEA"),
        "Pin_RNEA": extract_time(pin_out, "Pinocchio", "RNEA"),
        "RDL_ABA": extract_time(rdl_out, "RDL", "ABA"),
        "Pin_ABA": extract_time(pin_out, "Pinocchio", "ABA"),
    }
    
    results.append(data)
    print(f"  [{model_label}] CRBA -> RDL: {data['RDL_CRBA']}us | Pin: {data['Pin_CRBA']}us")
    print(f"  [{model_label}] RNEA -> RDL: {data['RDL_RNEA']}us | Pin: {data['Pin_RNEA']}us")
    print(f"  [{model_label}] ABA  -> RDL: {data['RDL_ABA']}us  | Pin: {data['Pin_ABA']}us")

# 保存为 CSV
csv_filename = "time_benchmark_results.csv"
with open(csv_filename, "w", newline="") as csvfile:
    fieldnames = [
        "Model", "DOF", "ModelLabel", 
        "RDL_CRBA", "Pin_CRBA", 
        "RDL_RNEA", "Pin_RNEA", 
        "RDL_ABA", "Pin_ABA"
    ]
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(results)

print(f"\n[Completed] Benchmarks finished with DOF info. Data saved to {csv_filename}")