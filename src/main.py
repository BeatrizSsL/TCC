from preprocess.preprocessing import *
import open3d as o3d

# Transforma o mesh para pointcloud
pcd, path_to_pc = mesh_to_pointcloud("/home/beatriz/TCC - Beatriz Sakata Luiz/data/raw/head.stl",
                   50000,
                   sampling_method="uniform")
print(path_to_pc)
# o3d.visualization.draw_geometries(pcd)

# Filtra a pointcloud utilizando o método voxel
down_pcd = voxel_downsampling(pcd, 5)
# visualize_point_cloud(down_pcd)

# Calcula as normais da pointcloud
down_pcd = estimate_normals(down_pcd,
                            radius=10,
                            max_nn=30,
                            show_normals=True)
# o3d.visualization.draw_geometries(down_pcd, show_normals=True)

# Pinta a nuvem de pontos
paint_pointcloud(down_pcd)
# visualize_point_cloud(down_pcd)

# Cria bounding boxes ao redor do objeto
aabb, obb = create_bounding_boxes(down_pcd)
o3d.visualization.draw_geometries([down_pcd, aabb, obb])

# import open3d as o3d
# import pyrealsense2 as rs
# import cv2
# import numpy as np

# # Inicializa a pipeline do RealSense
# pipeline = rs.pipeline()
# config = rs.config()

# # Carrega o arquivo .bag
# config.enable_device_from_file("/home/beatriz/TCC - Beatriz Sakata Luiz/data/raw/rs_05_20.bag")

# # Configura para habilitar o fluxo de dados de profundidade
# config.enable_stream(rs.stream.depth, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, rs.format.rgb8, 30)

# # Inicia a pipeline
# pipeline.start(config)

# # Alinha a profundidade com a cor
# align_to = rs.stream.color
# align = rs.align(align_to)

# try:
#     while True:
#         # Espera por um novo conjunto de frames
#         frames = pipeline.wait_for_frames()

#         # Alinha a profundidade com o fluxo de cor
#         aligned_frames = align.process(frames)
        
#         # Obtém o frame de profundidade e o frame de cor
#         depth_frame = aligned_frames.get_depth_frame()
#         color_frame = aligned_frames.get_color_frame()
        
#         if not depth_frame or not color_frame:
#             continue

#         # Converte os frames para numpy arrays
#         depth_image = np.asanyarray(depth_frame.get_data())
#         color_image = np.asanyarray(color_frame.get_data())
        
#         # Converte a profundidade para uma nuvem de pontos
#         depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
#         pcd = o3d.geometry.PointCloud.create_from_depth_image(
#             o3d.geometry.Image(depth_image),
#             o3d.camera.PinholeCameraIntrinsic(
#                 depth_intrinsics.width,
#                 depth_intrinsics.height,
#                 depth_intrinsics.fx,
#                 depth_intrinsics.fy,
#                 depth_intrinsics.ppx,
#                 depth_intrinsics.ppy
#             ),
#             depth_scale=1.0,
#             project_valid_depth_only=True
#         )
        
#         # Adiciona a cor à nuvem de pontos
#         pcd.colors = o3d.utility.Vector3dVector(color_image.reshape(-1, 3) / 255.0)

#         # Exibe a nuvem de pontos
#         o3d.visualization.draw_geometries([pcd])

# finally:
#     pipeline.stop()



# import open3d as o3d

# # Cria um pipeline de captura da RealSense
# bag_reader = o3d.t.io.RealSenseSensor()

# # Define a configuração
# config = o3d.t.io.RealSenseSensorConfig()
# bag_reader.start_capture(config=config, filename="/home/beatriz/realsense_studies/rs_05_20.bag")

# # Captura um frame de profundidade e converte para nuvem de pontos
# while True:
#     im_rgbd = bag_reader.capture_frame(wait=True)
    
#     if im_rgbd is not None:
#         pcd = o3d.t.geometry.PointCloud.create_from_rgbd_image(im_rgbd)
#         o3d.visualization.draw_geometries([pcd.to_legacy_pointcloud()])

# bag_reader.stop_capture()



# import open3d as o3d
# import pyrealsense2 as rs

# # Inicializa a pipeline do RealSense
# pipeline = rs.pipeline()
# config = rs.config()

# # Carrega o arquivo .bag
# config.enable_device_from_file("/home/beatriz/realsense_studies/rs_05_20.bag", 
#                                repeat_playback=False)

# # Configura para habilitar o fluxo de dados de profundidade
# config.enable_stream(rs.stream.depth, rs.format.z16, 30)

# # Inicia a pipeline
# pipeline.start(config)

# # Cria uma janela de visualização
# vis = o3d.visualization.Visualizer()
# vis.create_window()

# # Cria um ponto de nuvem Open3D
# pcd = o3d.geometry.PointCloud()

# # Configura o processamento
# pcd_points = rs.pointcloud()

# try:
#     while True:
#         # Espera por um novo conjunto de frames
#         frames = pipeline.wait_for_frames()
        
#         # Obtém o frame de profundidade
#         depth_frame = frames.get_depth_frame()
#         if not depth_frame:
#             continue
        
#         # Transforma os dados de profundidade em uma nuvem de pontos
#         pcd_points.map_to(frames.get_color_frame())
#         points = pcd_points.calculate(depth_frame)
        
#         # Converte os dados para o formato Open3D
#         vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
#         pcd.points = o3d.utility.Vector3dVector(vtx)
        
#         # Limpa e desenha a nuvem de pontos
#         vis.clear_geometries()
#         vis.add_geometry(pcd)
#         vis.poll_events()
#         vis.update_renderer()

# finally:
#     pipeline.stop()
#     vis.destroy_window()







# # Import libraries
# import numpy as np
# import open3d as o3d

# # Load a mesh
# ply_file_path_read = "/home/beatriz/TCC - Beatriz Sakata Luiz/data/raw/cube.stl"
# print(ply_file_path_read)
# mesh = o3d.io.read_triangle_mesh(ply_file_path_read)
# mesh.compute_vertex_normals()
# #mesh.scale(1 / np.max(mesh.get_max_bound() - mesh.get_min_bound()),center=mesh.get_center()) # Fit to unit cube
# print(f"Number of Vertices = {len(np.asarray(mesh.vertices))}")
# print(f"Number of Triangales(faces) = {len(np.asarray(mesh.triangles))}")

# # Convert mesh to pcd
# points = mesh.vertices
# point_cloud = o3d.geometry.PointCloud(points)

# # Visualize and save the point cloud
# o3d.visualization.draw_geometries([point_cloud], width=1200, height=800)
# o3d.io.write_point_cloud("bunny-mesh-to-pcd.pcd", point_cloud, write_ascii=True)