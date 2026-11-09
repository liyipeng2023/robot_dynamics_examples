import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

csv_filename = "time_benchmark_results.csv"

if not pd.io.common.file_exists(csv_filename):
    print(f"[Error] {csv_filename} not found. Please run run_time_benchmark.py first.")
    exit(1)

df = pd.read_csv(csv_filename)
if df.empty:
    print("[Warning] No data found in CSV file.")
    exit(0)

model_labels = df["ModelLabel"]
x = np.arange(len(model_labels))
width = 0.35

# -------------------------------------------------------------------
# 图 1: 包含 FK 和 各算法总耗时的对比图 (4 个子图)
# -------------------------------------------------------------------
fig1, axes1 = plt.subplots(1, 4, figsize=(22, 5), sharey=False)

full_algos = [
    ("Forward Kinematics (FK)", "RDL_FK", "Pin_FK"),
    ("CRBA (Total)", "RDL_CRBA_Total", "Pin_CRBA_Total"),
    ("RNEA (Total)", "RDL_RNEA_Total", "Pin_RNEA_Total"),
    ("ABA (Total)", "RDL_ABA_Total", "Pin_ABA_Total")
]

for i, (title, rdl_col, pin_col) in enumerate(full_algos):
    ax = axes1[i]
    ax.bar(x - width/2, df[rdl_col], width, label='RDL (Custom)', color='#2ca02c')
    ax.bar(x + width/2, df[pin_col], width, label='Pinocchio', color='#1f77b4')
    
    ax.set_ylabel('Execution Time ($\mu s$)', fontsize=11)
    ax.set_title(title, fontsize=13, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(model_labels, rotation=25, ha='right', fontsize=9)
    ax.legend(fontsize=9)
    ax.grid(True, linestyle="--", alpha=0.5)

plt.tight_layout()
fig1.savefig("time_benchmark_comparison_full.png", dpi=300)
print("[Completed] Saved: time_benchmark_comparison_full.png")

# -------------------------------------------------------------------
# 图 2: 扣除 FK 后的纯算法耗时对比图 (3 个子图)
# -------------------------------------------------------------------
fig2, axes2 = plt.subplots(1, 3, figsize=(18, 5), sharey=False)

pure_algos = [
    ("Pure CRBA (Excl. FK)", "RDL_CRBA_Pure", "Pin_CRBA_Pure"),
    ("Pure RNEA (Excl. FK)", "RDL_RNEA_Pure", "Pin_RNEA_Pure"),
    ("Pure ABA (Excl. FK)", "RDL_ABA_Pure", "Pin_ABA_Pure")
]

for i, (title, rdl_col, pin_col) in enumerate(pure_algos):
    ax = axes2[i]
    ax.bar(x - width/2, df[rdl_col], width, label='RDL (Custom)', color='#2ca02c')
    ax.bar(x + width/2, df[pin_col], width, label='Pinocchio', color='#1f77b4')
    
    ax.set_ylabel('Execution Time ($\mu s$)', fontsize=11)
    ax.set_title(title, fontsize=13, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(model_labels, rotation=25, ha='right', fontsize=10)
    ax.legend(fontsize=10)
    ax.grid(True, linestyle="--", alpha=0.5)

plt.tight_layout()
fig2.savefig("time_benchmark_comparison_pure.png", dpi=300)
print("[Completed] Saved: time_benchmark_comparison_pure.png")

plt.show()