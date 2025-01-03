import re
import matplotlib.pyplot as plt
from geopy.distance import geodesic

# Input string
log_entry = """
2025-01-02 22:07:11.913 DEBUG [903167] [SearchPatternState.cpp:354] Search Pattern Points (Horizontal ZigZag): (37.944574, -91.769295), (37.944943, -91.769289), (37.944573, -91.769272), (37.944943, -91.769266), (37.944573, -91.769250), (37.944943, -91.769243), (37.944573, -91.769227), (37.944942, -91.769221), (37.944573, -91.769204), (37.944942, -91.769198), (37.944573, -91.769181), (37.944942, -91.769175), (37.944572, -91.769159), (37.944942, -91.769152), (37.944572, -91.769136), (37.944942, -91.769130), (37.944572, -91.769113), (37.944941, -91.769107), (37.944572, -91.769090), (37.944941, -91.769084), (37.944571, -91.769068), (37.944941, -91.769061), (37.944571, -91.769045), (37.944941, -91.769039), (37.944571, -91.769022), (37.944940, -91.769016), (37.944571, -91.768999), (37.944940, -91.768993), (37.944570, -91.768977), (37.944940, -91.768970), (37.944570, -91.768954), (37.944940, -91.768948), (37.944570, -91.768931), (37.944939, -91.768925), (37.944570, -91.768908), (37.944939, -91.768902), (37.944569, -91.768886), (37.944939, -91.768879), (37.944569, -91.768863), (37.944939, -91.768857), (37.944569, -91.768840), (37.944938, -91.768834), (37.944569, -91.768817), 
"""
# Extract latitude and longitude points using regex
pattern = r"\((-?\d+\.\d+), (-?\d+\.\d+)\)"
points_string = re.findall(pattern, log_entry)

# The path points are in string format, convert them to float tuples stored in a list.
path_points = [(float(lat), float(lon)) for lat, lon in points_string]

# Calculate distances and prepare data for plotting
distances = []
for i in range(len(path_points) - 1):
    dist = geodesic(path_points[i], path_points[i + 1]).meters
    distances.append(dist)

# Prepare the plot
plt.figure(figsize=(10, 8))
x, y = zip(*[(pt[1], pt[0]) for pt in path_points])  # Longitude, Latitude for plotting

plt.plot(x, y, marker="o", label="Path", color="blue")

# Annotate the distances
for i in range(len(path_points) - 1):
    mid_x = (x[i] + x[i + 1]) / 2
    mid_y = (y[i] + y[i + 1]) / 2
    plt.text(mid_x, mid_y, f"{distances[i]:.2f} m", fontsize=8, color="red")

plt.title("Path with Distances")
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("path_with_distances.png")