import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

csv_filename = "time_benchmark_results.csv"

if not pd.io.common.file_exists(csv_filename):
    print(f"[Error] {csv_filename} not found. Please run run_separate_benchmarks.py first.")
    exit(1)

df = pd.read_csv(csv_filename)
if df.empty:
    print("[Warning] No data found in CSV file.")
    exit(0)

# 使用包含DOF的标签作为X轴坐标
model_labels = df["ModelLabel"]
x = np.arange(len(model_labels))
width = 0.35  # 柱子宽度

fig, axes = plt.subplots(1, 3, figsize=(18, 6), sharey=False)

algorithms = [
    ("CRBA", "RDL_CRBA", "Pin_CRBA"),
    ("RNEA", "RDL_RNEA", "Pin_RNEA"),
    ("ABA", "RDL_ABA", "Pin_ABA")
]

for i, (algo_name, rdl_col, pin_col) in enumerate(algorithms):
    ax = axes[i]
    
    rdl_times = df[rdl_col]
    pin_times = df[pin_col]
    
    ax.bar(x - width/2, rdl_times, width, label='RDL (Custom)', color='#2ca02c')
    ax.bar(x + width/2, pin_times, width, label='Pinocchio', color='#1f77b4')
    
    ax.set_ylabel('Execution Time ($\mu s$)', fontsize=12)
    ax.set_title(f'{algo_name} Algorithm Execution Time', fontsize=14, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(model_labels, rotation=25, ha='right', fontsize=10)
    ax.legend(fontsize=10)
    ax.grid(True, linestyle="--", alpha=0.5)

plt.tight_layout()

output_image = "time_benchmark_comparison_with_dof.png"
plt.savefig(output_image, dpi=300)
print(f"[Completed] Time comparison chart with DOF saved as {output_image}")
plt.show()