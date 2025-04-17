import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Your grid data (same as before)
grid_data = [
[
    [1, 1, 1, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 1],
    [0, 1, 1, 0],
    [1, 1, 1, 0],
    [1, 1, 1, 0]
],
[
    [1, 0, 0, 1],
    [0, 0, 1, 0],
    [1, 1, 0, 1],
    [0, 0, 0, 1],
    [0, 0, 1, 0],
    [1, 0, 1, 0]
],
[
    [1, 1, 1, 0],
    [1, 0, 1, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [1, 0, 1, 0]
],
[
    [1, 0, 0, 0],
    [0, 0, 1, 1],
    [1, 0, 0, 1],
    [0, 0, 1, 1],
    [1, 0, 0, 0],
    [0, 0, 1, 1]
],
[
    [1, 0, 0, 0],
    [0, 1, 1, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 1],
    [0, 0, 0, 1],
    [0, 1, 1, 1]
],
[
    [1, 0, 1, 1],
    [1, 0, 0, 1],
    [0, 0, 0, 1],
    [0, 1, 0, 1],
    [0, 1, 0, 1],
    [0, 1, 1, 1]
]

]  # ← Keep your full grid data here

rows = len(grid_data)
cols = len(grid_data[0])

fig, ax = plt.subplots(figsize=(cols / 2, rows / 2))
ax.set_aspect('equal')

# Draw grid and walls
for y in range(rows):
    for x in range(cols):
        cell = grid_data[y][x]
        cx, cy = x, rows - 1 - y  # Flip y-axis

        # Draw walls
        if cell[0]: ax.plot([cx, cx], [cy, cy + 1], color='black')       # West
        if cell[1]: ax.plot([cx, cx + 1], [cy + 1, cy + 1], color='black')  # North
        if cell[2]: ax.plot([cx + 1, cx + 1], [cy, cy + 1], color='black')  # East
        if cell[3]: ax.plot([cx, cx + 1], [cy, cy], color='black')       # South

# Add x-axis labels (top)
for x in range(cols):
    ax.text(x + 0.5, rows + 0.2, str(x), ha='center', va='bottom', fontsize=8, color='blue')

# Add y-axis labels (left)
for y in range(rows):
    ax.text(-0.4, rows - 1 - y + 0.5, str(y), ha='right', va='center', fontsize=8, color='blue')

# Hide axes
ax.set_xlim(-1, cols + 1)
ax.set_ylim(-1, rows + 1)
ax.axis('off')
plt.tight_layout()
plt.show()
