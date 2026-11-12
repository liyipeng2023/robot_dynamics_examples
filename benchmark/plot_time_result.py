import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

script_dir = Path(__file__).resolve().parent
csv_filename = script_dir / "time_benchmark_results.csv"

if not csv_filename.exists():
    print(f"[Error] {csv_filename} not found. Please run run_time_benchmark.py first.")
    exit(1)

df = pd.read_csv(csv_filename)
if df.empty:
    print("[Warning] No data found in CSV file.")
    exit(0)

model_labels = df["ModelLabel"]
x = np.arange(len(model_labels))
width = 0.36

def plot_comparison_chart(title, rdl_p50_col, rdl_p99_col, pin_p50_col, pin_p99_col, output_name):
    fig, ax = plt.subplots(figsize=(8.5, 5.5))

    rdl_p50 = df[rdl_p50_col].values
    pin_p50 = df[pin_p50_col].values

    rdl_err_upper = np.maximum(0.0, df[rdl_p99_col].values - rdl_p50)
    pin_err_upper = np.maximum(0.0, df[pin_p99_col].values - pin_p50)

    rdl_yerr = [np.zeros_like(rdl_p50), rdl_err_upper]
    pin_yerr = [np.zeros_like(pin_p50), pin_err_upper]

    ax.bar(
        x - width/2, rdl_p50, width,
        yerr=rdl_yerr, capsize=4, error_kw={'elinewidth': 1.2, 'ecolor': '#1b5e20'},
        label='RDL (Median + P99)', color='#2ca02c', alpha=0.9
    )
    ax.bar(
        x + width/2, pin_p50, width,
        yerr=pin_yerr, capsize=4, error_kw={'elinewidth': 1.2, 'ecolor': '#0d47a1'},
        label='Pinocchio (Median + P99)', color='#1f77b4', alpha=0.9
    )

    max_y = max(np.max(rdl_p50 + rdl_err_upper), np.max(pin_p50 + pin_err_upper))
    ax.set_ylim(0, max_y * 1.25 if max_y > 0 else 1.0)

    for i in range(len(x)):
        pin_top = pin_p50[i] + pin_err_upper[i]
        ax.text(
            x[i] + width/2, pin_top + max_y * 0.015,
            f"{pin_p50[i]:.2f}",
            ha='center', va='bottom', fontsize=8, color='#1f77b4'
        )

        rdl_top = rdl_p50[i] + rdl_err_upper[i]
        pct = (rdl_p50[i] / pin_p50[i] * 100.0) if pin_p50[i] > 0 else 0.0
        pct_color = '#1b5e20' if pct <= 100.0 else '#d95f02'

        ax.text(
            x[i] - width/2, rdl_top + max_y * 0.015,
            f"{rdl_p50[i]:.2f}\n({pct:.1f}%)",
            ha='center', va='bottom', fontsize=8, fontweight='bold', color=pct_color
        )

    ax.set_ylabel('Execution Time ($\mu s$)', fontsize=11)
    ax.set_title(f"{title} (Median & P99 Tail Latency)", fontsize=12.5, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(model_labels, rotation=22, ha='right', fontsize=9.5)
    ax.legend(fontsize=9.5, loc='upper left')
    ax.grid(True, linestyle="--", alpha=0.45)

    ax.text(
        0.99, 0.96, "Label: RDL Median (RDL / Pinocchio %)\n<100% indicates faster execution",
        transform=ax.transAxes, ha='right', va='top', fontsize=8,
        bbox=dict(boxstyle='round,pad=0.3', facecolor='#f8f9fa', edgecolor='#cccccc')
    )

    plt.tight_layout()
    out_path = script_dir / output_name
    fig.savefig(out_path, dpi=300)
    print(f"[Completed] Saved: {out_path}")

# 1. FK
plot_comparison_chart(
    "Forward Kinematics (FK)",
    "RDL_FK", "RDL_FK_P99", "Pin_FK", "Pin_FK_P99",
    "time_benchmark_comparison_kin.png"
)

# 2. Total Algorithms (incl. FK)
full_algos = [
    ("CRBA_Total", "RDL_CRBA_Total", "RDL_CRBA_Total_P99", "Pin_CRBA_Total", "Pin_CRBA_Total_P99"),
    ("RNEA_Total", "RDL_RNEA_Total", "RDL_RNEA_Total_P99", "Pin_RNEA_Total", "Pin_RNEA_Total_P99"),
    ("ABA_Total",  "RDL_ABA_Total",  "RDL_ABA_Total_P99",  "Pin_ABA_Total",  "Pin_ABA_Total_P99")
]
for title, r50, r99, p50, p99 in full_algos:
    plot_comparison_chart(title, r50, r99, p50, p99, f"time_benchmark_comparison_{title}.png")

# 3. Pure Algorithms (excl. FK)
#pure_algos = [
#    ("Pure_CRBA_Excl_FK", "RDL_CRBA_Pure", "RDL_CRBA_Pure_P99", "Pin_CRBA_Pure", "Pin_CRBA_Pure_P99"),
#    ("Pure_RNEA_Excl_FK", "RDL_RNEA_Pure", "RDL_RNEA_Pure_P99", "Pin_RNEA_Pure", "Pin_RNEA_Pure_P99"),
#    ("Pure_ABA_Excl_FK",  "RDL_ABA_Pure",  "RDL_ABA_Pure_P99",  "Pin_ABA_Pure",  "Pin_ABA_Pure_P99")
#]
#for title, r50, r99, p50, p99 in pure_algos:
#    plot_comparison_chart(title, r50, r99, p50, p99, f"time_benchmark_comparison_{title}.png")

plt.show()