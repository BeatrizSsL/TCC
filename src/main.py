from preprocess.preprocessing import *
import open3d as o3d
import math
import time
import copy

# # Transforma o mesh para pointcloud
# pcd, path_to_pc = mesh_to_pointcloud("/home/beatriz/TCC - Beatriz Sakata Luiz/data/raw/head.stl",
#                    50000,
#                    sampling_method="uniform")
# print(path_to_pc)
# # o3d.visualization.draw_geometries(pcd)

# # Filtra a pointcloud utilizando o método voxel
# down_pcd = voxel_downsampling(pcd, 5)
# # visualize_point_cloud(down_pcd)

# # Calcula as normais da pointcloud
# down_pcd = estimate_normals(down_pcd,
#                             radius=10,
#                             max_nn=30,
#                             show_normals=True)
# # o3d.visualization.draw_geometries(down_pcd, show_normals=True)

# # Pinta a nuvem de pontos
# paint_pointcloud(down_pcd)
# # visualize_point_cloud(down_pcd)

# # Cria bounding boxes ao redor do objeto
# aabb, obb = create_bounding_boxes(down_pcd)
# o3d.visualization.draw_geometries([down_pcd, aabb, obb])

# Visualize surface matching result
def draw_registration_result(source, target, brain, transformation):

    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    brain_temp = copy.deepcopy(brain)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    new_transformation = np.matmul(transformation, trans_brain_Z)
    brain_temp.transform(new_transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp, brain_temp])


# Carregar as point clouds
head_target = o3d.io.read_point_cloud("/home/beatriz/TCC/data/processed/head.pcd")
head_source = o3d.io.read_point_cloud("/home/beatriz/TCC/data/processed/head_1.pcd")
brain = o3d.io.read_point_cloud("/home/beatriz/TCC/data/processed/brain.pcd")

# Forward head_target position
trans_head = [[1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1]]
head_target.transform(trans_head)

# Scale head_source, because it is a little bit smaller than the head_target
head_source.scale(1.1, center=head_source.get_center())

# Find the inicial position
ang_deg = 168 # Em graus
tx = 0
ty = -173
tz = 160
# trans_brain_X = [[1, 0, 0, tx], 
#                 [0, math.cos(ang_deg*math.pi/180), -math.sin(ang_deg*math.pi/180), ty], 
#                 [0, math.sin(ang_deg*math.pi/180), math.cos(ang_deg*math.pi/180), tz], 
#                 [0, 0, 0, 1]]
# trans_brain_Y = [[math.cos(ang_deg*math.pi/180), 0, -math.sin(ang_deg*math.pi/180), tx], 
#                 [0, 1, 0, ty], 
#                 [math.sin(ang_deg*math.pi/180), 0, math.cos(ang_deg*math.pi/180), tz], 
#                 [0, 0, 0, 1]]
trans_brain_Z = [[math.cos(ang_deg*math.pi/180), -math.sin(ang_deg*math.pi/180), 0, tx], 
                [math.sin(ang_deg*math.pi/180), math.cos(ang_deg*math.pi/180), 0, ty], 
                [0, 0, 1, tz], 
                [0, 0, 0, 1]]
brain.transform(trans_brain_Z)
o3d.visualization.draw_geometries([head_source, head_target, brain])

# ICP: POINT-TO-POINT and POINT-TO-PLANE
threshold = 20
trans_init = np.asarray([[1, 0, 0, 0],
                            [0, 0, 1, 0],
                            [0, 1, 0, 0], 
                            [0, 0, 0, 1]])

print("Initial alignment")
evaluation = o3d.pipelines.registration.evaluate_registration(head_source, head_target, threshold, trans_init)
print("evaluation: ", evaluation.transformation)

t = time.time()
print("Apply point-to-point ICP")
reg_p2p = o3d.pipelines.registration.registration_icp(head_source, head_target, threshold, trans_init, o3d.pipelines.registration.TransformationEstimationPointToPoint(), o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))
print(f"{time.time()-t} segundos")

print(reg_p2p)
print("Transformation is:")
print(reg_p2p.transformation)
draw_registration_result(head_source, head_target, brain, reg_p2p.transformation)