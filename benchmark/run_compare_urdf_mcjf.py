import subprocess
import re
import csv
import os
import sys
from pathlib import Path

script_dir = Path(__file__).resolve().parent
models_base_dir = script_dir.parent / "models"

# 默认测试模型列表（自动匹配同名的 .urdf 和 .xml 文件）
default_test_cases = [
    #{"model": "ur10", "floating": 0},
    {"model": "franka-fr3", "floating": 0},
    {"model": "unitree-go2", "floating": 1},
    {"model": "spot", "floating": 1},
    {"model": "atlas-v4", "floating": 1},
    {"model": "unitree-h1-2", "floating": 1}
]

# 支持通过命令行指定特定模型名称，例如: python3 run_compare_urdf_xml.py ur10 atlas
if len(sys.argv) > 1:
    specified_names = [os.path.splitext(arg)[0] for arg in sys.argv[1:]]
    floating_lookup = {c["model"]: c["floating"] for c in default_test_cases}
    test_cases = [
        {"model": name, "floating": floating_lookup.get(name, 1)}
        for name in specified_names
    ]
else:
    test_cases = default_test_cases

# linux
binary_path = "../build/benchmark/compare_urdf_mjcf_rdl"
# windows
binary_path = script_dir / ".." / "cmake-build-release" / "benchmark" / "compare_urdf_mjcf_rdl.exe"

results = []

if not os.path.exists(binary_path):
    print(f"[Error] Binary not found: {binary_path}. Please compile your C++ program first.")
    exit(1)

for case in test_cases:
    model_name = case["model"]
    floating_base = case["floating"]

    urdf_file = models_base_dir / f"{model_name}.urdf"
    xml_file = models_base_dir / f"{model_name}.xml"

    if not os.path.exists(urdf_file) or not os.path.exists(xml_file):
        print(f"[Warning] Model files for '{model_name}' (.urdf or .xml) not found in {models_base_dir}, skipping...")
        continue

    # 传入不含后缀的模型路径前缀（或模型名称），由 C++ 程序自动拼接 .urdf 和 .xml
    model_prefix = str(models_base_dir / model_name)
    print(f"--> Testing model: {model_name} (Floating Base: {floating_base})")
    cmd = [str(binary_path), model_prefix, str(floating_base)]

    try:
        # 执行 C++ 测试程序并捕获输出
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        output = res.stdout + res.stderr

        # 检查是否运行成功
        if res.returncode != 0:
            print(f"[Error] Test failed for {model_name}:\n{output.strip()}")
            continue

        # 提取自由度 nv (DOF)
        dof_match = re.search(r'(?:Total Joints \(nv\)|DOF \(nv\)|nv):\s*(\d+)', output, re.IGNORECASE)
        dof_val = dof_match.group(1) if dof_match else "0"

        # 构造带 dof 的标签，如 atlas(36dof)
        model_label = f"{model_name}({dof_val}dof)"

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
            "Model": model_name,
            "DOF": dof_val,
            "ModelLabel": model_label,
            "FloatingBase": floating_base,
            "CRBA_Max_Error": max_crba,
            "RNEA_Max_Error": max_rnea,
            "ABA_Max_Error": max_aba
        })
        print(f"    [Success] [{model_label}] CRBA: {max_crba:.2e} | RNEA: {max_rnea:.2e} | ABA: {max_aba:.2e}")

    except subprocess.TimeoutExpired:
        print(f"[Error] Test timeout for {model_name}")
    except Exception as e:
        print(f"[Error] Exception occurred while running {model_name}: {e}")

# 终端打印汇总报表
if results:
    header = f"{'ModelLabel':<24} | {'Floating':<8} | {'CRBA Max Err':<14} | {'RNEA Max Err':<14} | {'ABA Max Err':<14}"
    sep = "-" * len(header)
    print("\n" + sep)
    print(header)
    print(sep)
    for r in results:
        print(
            f"{r['ModelLabel']:<24} | {r['FloatingBase']:<8} | "
            f"{r['CRBA_Max_Error']:<14.3e} | {r['RNEA_Max_Error']:<14.3e} | {r['ABA_Max_Error']:<14.3e}"
        )
    print(sep)

# 将结果保存为 CSV 文件
csv_filename = "urdf_xml_compare_results.csv"
with open(csv_filename, "w", newline="") as csvfile:
    fieldnames = ["Model", "DOF", "ModelLabel", "FloatingBase", "CRBA_Max_Error", "RNEA_Max_Error", "ABA_Max_Error"]
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(results)

print(f"\n[Completed] URDF vs XML precision tests finished. Data recorded to {csv_filename}")