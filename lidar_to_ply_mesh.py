import laspy
import lazrs
from plyfile import PlyData, PlyElement
import numpy as np
import open3d as o3d

# 1: Create Function to convert LAS/LAZ to PLY
def laz_to_ply(input_laz_path, output_ply_path):
    # Read the LAZ file
    with laspy.open(input_laz_path) as file:
        las = file.read()

    # Extract the point cloud data
    point_cloud = np.vstack((las.x, las.y, las.z)).transpose()

    # Prepare PLY data
    vertex = np.array([tuple(point) for point in point_cloud], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    el = PlyElement.describe(vertex, 'vertex')

    # Write to a PLY file
    PlyData([el], text=True).write(output_ply_path)

# Example usage from USGS LiDAR (https://apps.nationalmap.gov/lidar-explorer/#/)
laz_to_ply('USGS_LPC_AK_GlacierBay_2019_B19_GB_00478.laz', 'point_cloud.ply')

# 2: Load in created point cloud
pcd = o3d.io.read_point_cloud("point_cloud.ply")

# 3: Estimate normals
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

# 4: Create Mesh
# 4.1: Ball Pivoting Method
radii = [0.005, 0.01, 0.02, 0.04]     # You might need to experiment with the radius parameter
bp_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    pcd, o3d.utility.DoubleVector(radii))

# 4.2: Poisson Reconstruction Method
poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd)[0]
