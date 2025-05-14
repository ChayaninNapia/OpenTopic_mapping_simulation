import matplotlib.pyplot as plt

# === DATA ===
attempts = list(range(1, 9))  # x-axis: attempt 1 to 8

errors_001 = [1.50358,	0.85353,	0.59237,	0.59284	,0.39998,	0.22952,	0.22050,	0.17814]
errors_005 = [1.38909,	0.71755,	0.48096,	0.4162,	0.28191,	0.33862,	0.21622,	0.2336]
errors_01 = [1.37891    ,0.51163	,0.32883	,0.27392	,0.30478	,0.55401	,0.60548	,0.99677]

# === PLOT ===
plt.figure(figsize=(10, 6))
plt.plot(attempts, errors_001, marker='o', label='Noise 0.01')
plt.plot(attempts, errors_005, marker='s', label='Noise 0.05')
plt.plot(attempts, errors_01, marker='^', label='Noise 0.1')

# === LABELS & STYLES ===
plt.title("Impact of Encoder velocity Noise on Map Quality", fontsize=14)
plt.xlabel("Attempt", fontsize=12)
plt.ylabel("Map Error", fontsize=12)
plt.xticks(attempts)
plt.grid(True, linestyle='--', alpha=0.5)
plt.legend()
plt.tight_layout()

# === SHOW ===
plt.show()

