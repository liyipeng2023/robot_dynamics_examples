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
    {"model": models_base_dir / "HumanURDFmodel.urdf", "floating": 1},
    {"model": models_base_dir / "unitree-h1-2.urdf", "floating": 1}
]
# linux
binary_path = "../build/benchmark/compare_rdl_pinocchio"
# windows
#binary_path = script_dir / ".." / "cmake-build-release" / "benchmark" / "compare_rdl_pinocchio.exe"
results = []

if not os.path.exists(binary_path):
    print(f"[Error] Binary not found: {binary_path}. Please compile your C++ program first.")
    exit(1)

for case in test_cases:
    model_file = case["model"]
    floating_base = case["floating"]
    
    if not os.path.exists(model_file):
        print(f"[Warning] Model file {model_file} not found, skipping...")
        continue
        
    print(f"--> Testing model: {model_file} (Floating Base: {floating_base})")
    cmd = [binary_path, model_file, str(floating_base)]
    
    try:
        # 执行 C++ 测试程序并捕获输出
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        output = res.stdout + res.stderr
        
        # 检查是否运行成功
        if res.returncode != 0:
            print(f"[Error] Test failed for {model_file}")
            continue
            
        # 提取自由度 nv (DOF)
        dof_match = re.search(r'(?:Total Joints \(nv\)|DOF \(nv\)|nv):\s*(\d+)', output, re.IGNORECASE)
        dof_val = dof_match.group(1) if dof_match else "0"
        
        # 提取文件名并构造带 dof 的标签，如 atlas(36dof)
        base_name = os.path.splitext(os.path.basename(model_file))[0]
        model_label = f"{base_name}({dof_val}dof)"
        
        # 使用正则表达式提取各项算法的最大误差
        crba_errs = re.findall(r'\[CRBA \(M matrix\)\] Max error:\s*([eE\+\-\.\d]+)', output)
        rnea_errs = re.findall(r'\[RNEA \(tau\)\] Max error:\s*([eE\+\-\.\d]+)', output)
        aba_errs = re.findall(r'\[ABA \(ddq\)\] Max error:\s*([eE\+\-\.\d]+)', output)
        
        # 取多轮迭代中的最大误差作为指标
        def get_max_err(err_list):
            return max([float(e) for e in err_list]) if err_list else 0.0

        max_crba = get_max_err(crba_errs)
        max_rnea = get_max_err(rnea_errs)
        max_aba = get_max_err(aba_errs)
        
        results.append({
            "Model": base_name,
            "DOF": dof_val,
            "ModelLabel": model_label,
            "FloatingBase": floating_base,
            "CRBA_Max_Error": max_crba,
            "RNEA_Max_Error": max_rnea,
            "ABA_Max_Error": max_aba
        })
        print(f"    [Success] [{model_label}] CRBA: {max_crba:.2e} | RNEA: {max_rnea:.2e} | ABA: {max_aba:.2e}")
        
    except subprocess.TimeoutExpired:
        print(f"[Error] Test timeout for {model_file}")
    except Exception as e:
        print(f"[Error] Exception occurred while running {model_file}: {e}")

# 将结果保存为 CSV 文件
csv_filename = "benchmark_results.csv"
with open(csv_filename, "w", newline="") as csvfile:
    fieldnames = ["Model", "DOF", "ModelLabel", "FloatingBase", "CRBA_Max_Error", "RNEA_Max_Error", "ABA_Max_Error"]
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(results)

print(f"\n[Completed] Precision tests finished with DOF info. Data recorded to {csv_filename}")