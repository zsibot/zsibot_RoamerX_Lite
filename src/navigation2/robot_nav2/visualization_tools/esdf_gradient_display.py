import pandas as pd
import matplotlib.pyplot as plt

# Load CSV
df = pd.read_csv("/home/lyh/YihuWS/NAVIGO/code/navigo_workspace/gradient.csv")

# Extract coordinates and gradient
x = df['x'].values
y = df['y'].values
gx = df['gx'].values
gy = df['gy'].values

# Compute norm (for color mapping)
import numpy as np
norm = np.sqrt(gx**2 + gy**2)

# Plot
plt.figure(figsize=(12, 12))
plt.quiver(x, y, gx, gy, norm, cmap='coolwarm', angles='xy', scale_units='xy', scale=1, width=0.003)
plt.colorbar(label="Gradient Magnitude")
plt.title(f"Gradient Field Visualization at z = {df['z'].iloc[0]}")
plt.xlabel("x")
plt.ylabel("y")
# plt.gca().invert_yaxis()  # Optional, flip to match map direction
plt.axis("equal")
plt.grid(True)

plt.tight_layout()
plt.show()
