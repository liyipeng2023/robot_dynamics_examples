import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

csv_filename = "benchmark_results.csv"

if not pd.io.common.file_exists(csv_filename):
    print(f"[Error] {csv_filename} not found. Please run the automation script first.")
    exit(1)

# 读取测试数据
df = pd.read_csv(csv_filename)

if df.empty:
    print("[Warning] No data found in CSV file.")
    exit(0)

# 如果 CSV 中包含 ModelLabel 则使用，否则降级使用 Model 列
model_labels = df["ModelLabel"] if "ModelLabel" in df.columns else df["Model"]
x = np.arange(len(model_labels))
width = 0.25  # 柱状图宽度

fig, ax = plt.subplots(figsize=(11, 6))

# 绘制 CRBA、RNEA、ABA 的误差柱状图
rects1 = ax.bar(x - width, df["CRBA_Max_Error"], width, label='CRBA (Inertia Matrix M)', color='#1f77b4')
rects2 = ax.bar(x, df["RNEA_Max_Error"], width, label='RNEA (Inverse Dynamics tau)', color='#ff7f0e')
rects3 = ax.bar(x + width, df["ABA_Max_Error"], width, label='ABA (Forward Dynamics ddq)', color='#2ca02c')

# 图表美化与对数坐标设置
ax.set_ylabel('Max Absolute Error (Log Scale)', fontsize=12)
ax.set_title('RDL vs Pinocchio Algorithm Precision Comparison Across Robot Models', fontsize=14, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels(model_labels, rotation=20, ha='right', fontsize=11)
ax.set_yscale('log')  # 对数坐标轴
ax.legend(fontsize=11)
ax.grid(True, which="both", ls="--", alpha=0.5)

# 添加精度参考线（1e-12）
ax.axhline(y=1.e-12, color='red', linestyle=':', linewidth=1.5, label='Precision Threshold (1e-12)')

plt.tight_layout()

# 保存并展示图片
output_image = "benchmark_comparison.png"
plt.savefig(output_image, dpi=300)
print(f"[Completed] Comparison chart saved as {output_image}")
plt.show()