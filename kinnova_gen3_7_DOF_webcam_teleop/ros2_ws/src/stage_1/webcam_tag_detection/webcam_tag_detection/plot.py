import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

# Data from data_messe.pdf
data = {
    'Age': [8, 8, 8, 11, 11, 11, 12, 10, 10, 9, 9, 13, 13, 64, 13, 27, 11, 12, 11, 7, 6, 64, 45, 6, 11, 7, 32, 7, 8, 6, 6, 29, 11, 7, 6, 11, 7, 8, 9, 9, 39, 43],
    'Time': [110, 90, 120, 120, 120, 95, 118, 105, 120, 113, 120, 30, 30, 60, 90, 40, 40, 90, 48, 80, 70, 120, 60, 52, 92, 70, 54, 95, 120, 46, 120, 40, 120, 32, 41, 70, 120, 160, 120, 41, 35, 39],
    'Success': ["Success" if s == 1 else "Failure" for s in [1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1]],
    'EaseOfUse': [2, 1, 3, 2, 2, 1, 2, 2, 3, 1, 3, 1, 1, 1, 1, 2, 2, 2, 1, 3, 1, 3, 2, 2, 1, 2, 1, 2, 4, 1, 3, 1, 3, 1, 1, 1, 3, 4, 4, 1, 1, 1]
}
df = pd.DataFrame(data)

# Set visual style to match journal standards
plt.rcParams['font.family'] = 'serif'
sns.set_context("paper", font_scale=1.2)
fig, axes = plt.subplots(1, 3, figsize=(18, 6))
fig.suptitle('Fig. 4. Scatter Plots: Participant Performance Relationships (n=42)', fontsize=16, fontweight='bold')

colors = {"Success": "#5b8dbf", "Failure": "#d98a7a"}

# (a) Completion Time vs. Ease of Use
sns.scatterplot(ax=axes[0], data=df, x='EaseOfUse', y='Time', hue='Success', palette=colors, s=100, edgecolor='white', alpha=0.8)
axes[0].set_title('(a) Completion Time vs. Ease of Use')
axes[0].set_xlabel('Ease of Use Score (1=Easiest)')
axes[0].set_ylabel('Completion Time (s)')
axes[0].grid(True, linestyle='--', alpha=0.6)

# (b) Completion Time vs. Age
sns.scatterplot(ax=axes[1], data=df, x='Age', y='Time', hue='Success', palette=colors, s=100, edgecolor='white', alpha=0.8)
axes[1].set_title('(b) Completion Time vs. Age')
axes[1].set_xlabel('Participant Age (years)')
axes[1].set_ylabel('Completion Time (s)')
axes[1].grid(True, linestyle='--', alpha=0.6)

# (c) Ease of Use vs. Age
sns.scatterplot(ax=axes[2], data=df, x='Age', y='EaseOfUse', hue='Success', palette=colors, s=100, edgecolor='white', alpha=0.8)
axes[2].set_title('(c) Ease of Use vs. Age')
axes[2].set_xlabel('Participant Age (years)')
axes[2].set_ylabel('Ease of Use Score')
axes[2].grid(True, linestyle='--', alpha=0.6)

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()