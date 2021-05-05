import matplotlib.pyplot as plt 
from numpy import *  
from matplotlib import rcParams
from matplotlib.cbook import boxplot_stats
import plotly.express as px
import plotly.graph_objects as go
import seaborn as sns
from matplotlib.ticker import FormatStrFormatter


# gammadot_c_50 = loadtxt("MSI_50_steps_c++.txt")
# gammadot_acado_50_6 = loadtxt("MSI_50_steps_acado_6.txt")
# gammadot_acado_50_12 = loadtxt("MSI_50_steps_acado_12.txt")
# gammadot_c_100 = loadtxt("MSI_100_steps_c++.txt")
# gammadot_acado_100 = loadtxt("MSI_100_steps_acado_10.txt")

compTime_rmsprop = [0.115242986,0.110638029,0.109560088,0.11264435,0.113208206,0.111663285,0.113955133,0.113635916,0.107962791,0.112056754]
compTime_gekko = [0.128508977, 0.110821315, 0.108430751, 0.110592638, 0.107887299, 0.133688109, 0.144631113, 0.11833281, 0.113991874, 0.119653876]
compTime_scipy = [0.316157596,0.272387608,0.320058204,0.256322948,0.341020679,0.328206789,0.323953943,0.338770242,0.254394152,0.305696907]
plt.figure(figsize=(6.5,6.5))
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [compTime_rmsprop, compTime_gekko, compTime_scipy], linewidth = 2)
ax.set_xticklabels(["RMSProp", "Gekko-IPOPT", "Scipy-SLSQP"], fontsize=10, weight='bold')
ax.set_yticklabels(ax.get_yticks(), fontsize=10, weight='bold')
ax.set_ylabel(ylabel="s", weight='bold')
ax.set_xlabel(xlabel="(f)", weight='bold')
# #$||\dot\gamma||$
# ax.set_title(label="Laptop Computation Time", fontsize=20)
# ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
# stats = boxplot_stats([gammadot_c_50, gammadot_acado_50_6, gammadot_acado_50_12, gammadot_c_100, gammadot_acado_100])
print (stats)
# ax.legend(["Ours_50_Stp", "ACADO_50_Stp \n 6_It", "ACADO_50_Stp \n 12_It", "Ours_100_Steps", "ACADO_100_Stp \n 10_Iter"], loc="topright") 
# plt.savefig("laptop_comp_time.png", dpi=100)

plt.show()
