# import open3d as o3d
# import numpy as np
# import rosbag
# import cv2
# from sensor_msgs.msg import CompressedImage
# from cv_bridge import CvBridge
# import os

# # Função para converter imagem ROS para Open3D PointCloud
# def ros_image_to_o3d_pointcloud(depth_image, color_image, intrinsic_matrix):
#     # Cria objeto PinholeCameraIntrinsic
#     intrinsic = o3d.camera.PinholeCameraIntrinsic()
#     intrinsic.set_intrinsics(intrinsic_matrix['width'], intrinsic_matrix['height'],
#                              intrinsic_matrix['fx'], intrinsic_matrix['fy'],
#                              intrinsic_matrix['cx'], intrinsic_matrix['cy'])

#     # Cria a nuvem de pontos a partir das imagens de cor e profundidade
#     rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
#         o3d.geometry.Image(color_image),
#         o3d.geometry.Image(depth_image),
#         convert_rgb_to_intensity=False)

#     pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
#     return pcd

# # Função para processar o arquivo .bag e salvar a nuvem de pontos em um arquivo .pcd
# def process_and_save_bag_file(bag_file):
#     # Criação do ROS bag object
#     bag = rosbag.Bag(bag_file)
#     bridge = CvBridge()

#     intrinsic_matrix = {
#         'width': 640,  # Ajuste para o tamanho da sua imagem
#         'height': 480,
#         'fx': 600,     # Ajuste conforme necessário com base no seu sensor
#         'fy': 600,
#         'cx': 320,
#         'cy': 240
#     }

#     depth_image = None
#     color_image = None

#     # Itera sobre as mensagens do bag
#     for topic, msg, t in bag.read_messages(topics=['/camera/aligned_depth_to_color/image_raw/compressed',
#                                                    '/camera/color/image_raw/compressed']):
#         if topic == '/camera/aligned_depth_to_color/image_raw/compressed':
#             # Converte a imagem de profundidade comprimida ROS para OpenCV
#             np_arr = np.frombuffer(msg.data, np.uint8)
#             depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)  # Carrega a imagem comprimida

#         elif topic == '/camera/color/image_raw/compressed':
#             # Converte a imagem de cor comprimida ROS para OpenCV
#             np_arr = np.frombuffer(msg.data, np.uint8)
#             color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Carrega a imagem comprimida

#         # Se ambas as imagens forem obtidas, gera a nuvem de pontos
#         if depth_image is not None and color_image is not None:
#             # Chama a função para converter em nuvem de pontos
#             pcd = ros_image_to_o3d_pointcloud(depth_image, color_image, intrinsic_matrix)

#             # Define o caminho para salvar a nuvem de pontos
#             savepath = bag_file.replace("/raw/", "/processed/")
#             savepath = os.path.splitext(savepath)[0] + ".pcd"
            
#             # Salva a nuvem de pontos no formato .pcd
#             o3d.io.write_point_cloud(savepath, pcd, write_ascii=True)
#             print(f"Nuvem de pontos salva em: {savepath}")
#             break  # Encerra após salvar a primeira nuvem de pontos

#     bag.close()

# # Arquivo .bag
# bag_file = "/home/beatriz/TCC/data/raw/rs_05_20.bag"

# # Processa o arquivo e salva a nuvem de pontos
# process_and_save_bag_file(bag_file)


# pcd_file = "/home/beatriz/TCC/data/processed/rs_05_20.pcd"
# pointcloud = o3d.io.read_point_cloud(pcd_file)
# # pointcloud = o3d.geometry.PointCloud() 
# # downpcd = pointcloud.voxel_down_sample(voxel_size=5)
# o3d.visualization.draw_geometries([pointcloud])


# ===================================================
# import open3d as o3d
# import numpy as np
# import rosbag
# import cv2
# from sensor_msgs.msg import CompressedImage
# from cv_bridge import CvBridge

# # Função para converter imagem ROS para Open3D PointCloud
# def ros_image_to_o3d_pointcloud(depth_image, color_image, intrinsic_matrix):
#     # Cria objeto PinholeCameraIntrinsic
#     intrinsic = o3d.camera.PinholeCameraIntrinsic()
#     intrinsic.set_intrinsics(intrinsic_matrix['width'], intrinsic_matrix['height'],
#                              intrinsic_matrix['fx'], intrinsic_matrix['fy'],
#                              intrinsic_matrix['cx'], intrinsic_matrix['cy'])

#     # Cria a nuvem de pontos a partir das imagens de cor e profundidade
#     rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
#         o3d.geometry.Image(color_image),
#         o3d.geometry.Image(depth_image),
#         convert_rgb_to_intensity=False)

#     pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
#     return pcd

# # Função para ler um arquivo .bag e processar a nuvem de pontos
# def process_bag_file(bag_file):
#     # Criação do ROS bag object
#     bag = rosbag.Bag(bag_file)
#     bridge = CvBridge()

#     intrinsic_matrix = {
#         'width': 640,  # Ajuste para o tamanho da sua imagem
#         'height': 480,
#         'fx': 600,     # Ajuste conforme necessário com base no seu sensor
#         'fy': 600,
#         'cx': 320,
#         'cy': 240
#     }

#     depth_image = None
#     color_image = None

#     # Itera sobre as mensagens do bag
#     for topic, msg, t in bag.read_messages(topics=['/camera/aligned_depth_to_color/image_raw/compressed',
#                                                    '/camera/color/image_raw/compressed']):
#         if topic == '/camera/aligned_depth_to_color/image_raw/compressed':
#             # Converte a imagem de profundidade comprimida ROS para OpenCV
#             np_arr = np.frombuffer(msg.data, np.uint8)
#             depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)  # Carrega a imagem comprimida

#         elif topic == '/camera/color/image_raw/compressed':
#             # Converte a imagem de cor comprimida ROS para OpenCV
#             np_arr = np.frombuffer(msg.data, np.uint8)
#             color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Carrega a imagem comprimida

#         # Se ambas as imagens forem obtidas, gera a nuvem de pontos
#         if depth_image is not None and color_image is not None:
#             # Chama a função para converter em nuvem de pontos
#             pcd = ros_image_to_o3d_pointcloud(depth_image, color_image, intrinsic_matrix)
#             return pcd  # Retorna a nuvem de pontos e finaliza o loop

#     bag.close()

# # Arquivo .bag
# bag_file = "/home/beatriz/TCC/data/raw/rs_05_20.bag"

# # Processa o arquivo e gera a nuvem de pontos
# pcd = process_bag_file(bag_file)

# # # Visualiza a nuvem de pontos
# # if pcd is not None:
# #     o3d.visualization.draw_geometries([pcd])
# # else:
# #     print("Nenhuma nuvem de pontos foi gerada.")


# # Verifica se a nuvem de pontos foi gerada e a salva
# if pcd is not None:
#     # Caminho para salvar a nuvem de pontos
#     save_path = "/home/beatriz/TCC/data/processed/pointcloud.pcd"
    
#     # Salva a nuvem de pontos em um arquivo .pcd
#     o3d.io.write_point_cloud(save_path, pcd, write_ascii=True)
#     print(f"Nuvem de pontos salva em: {save_path}")
    
#     # Visualiza a nuvem de pontos
#     o3d.visualization.draw_geometries([pcd])
# else:
#     print("Nenhuma nuvem de pontos foi gerada.")



# import open3d as o3d
# import numpy as np
# import rosbag
# import cv2
# from sensor_msgs.msg import CompressedImage
# from cv_bridge import CvBridge
# import os

# # Função para converter imagem ROS para Open3D PointCloud
# def ros_image_to_o3d_pointcloud(depth_image, color_image, intrinsic_matrix):
#     # Cria objeto PinholeCameraIntrinsic
#     intrinsic = o3d.camera.PinholeCameraIntrinsic()
#     intrinsic.set_intrinsics(intrinsic_matrix['width'], intrinsic_matrix['height'],
#                              intrinsic_matrix['fx'], intrinsic_matrix['fy'],
#                              intrinsic_matrix['cx'], intrinsic_matrix['cy'])

#     # Cria a nuvem de pontos a partir das imagens de cor e profundidade
#     rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
#         o3d.geometry.Image(color_image),
#         o3d.geometry.Image(depth_image),
#         convert_rgb_to_intensity=False)

#     pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
#     return pcd

# # Função para ler um arquivo .bag e processar a nuvem de pontos
# def process_bag_file(bag_file):
#     # Criação do ROS bag object
#     bag = rosbag.Bag(bag_file)
#     bridge = CvBridge()

#     intrinsic_matrix = {
#         'width': 640,  # Ajuste para o tamanho da sua imagem
#         'height': 480,
#         'fx': 615,     # Ajuste conforme necessário com base no seu sensor
#         'fy': 615,
#         'cx': 320,
#         'cy': 240
#     }

#     depth_image = None
#     color_image = None
#     count = 0  # Contador para a numeração dos arquivos

#     # Itera sobre as mensagens do bag
#     for topic, msg, t in bag.read_messages(topics=['/camera/aligned_depth_to_color/image_raw/compressed',
#                                                    '/camera/color/image_raw/compressed']):
#         if topic == '/camera/aligned_depth_to_color/image_raw/compressed':
#             # Converte a imagem de profundidade comprimida ROS para OpenCV
#             np_arr = np.frombuffer(msg.data, np.uint8)
#             depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)  # Carrega a imagem comprimida

#         elif topic == '/camera/color/image_raw/compressed':
#             # Converte a imagem de cor comprimida ROS para OpenCV
#             np_arr = np.frombuffer(msg.data, np.uint8)
#             color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Carrega a imagem comprimida

#         # Se ambas as imagens forem obtidas
#         if depth_image is not None and color_image is not None:
#             # Chama a função para converter em nuvem de pontos
#             pcd = ros_image_to_o3d_pointcloud(depth_image, color_image, intrinsic_matrix)

#             # Salva a nuvem de pontos a cada 5 segundos
#             if int(t.to_sec()) % 15 == 0:
#                 # Nome do arquivo baseado no tempo em segundos
#                 save_path = f"/home/beatriz/TCC/data/processed/pointcloud_{count}.pcd"
#                 o3d.io.write_point_cloud(save_path, pcd, write_ascii=True)
#                 print(f"Nuvem de pontos salva em: {save_path}")
#                 count += 1  # Incrementa o contador

#     bag.close()

# # Função para ler e visualizar todas as nuvens de pontos em um diretório
# def visualize_all_pointclouds(directory):
#     # Lista todos os arquivos no diretório
#     for filename in os.listdir(directory):
#         if filename.endswith(".pcd"):  # Verifica se o arquivo é um .pcd
#             # Cria o caminho completo para o arquivo
#             file_path = os.path.join(directory, filename)
#             print(f"Lendo nuvem de pontos de: {file_path}")
            
#             # Carrega a nuvem de pontos
#             pcd = o3d.io.read_point_cloud(file_path)
            
#             # Visualiza a nuvem de pontos
#             o3d.visualization.draw_geometries([pcd], window_name=filename)


# # Arquivo .bag
# # bag_file = "/home/beatriz/TCC/data/raw/rs_05_20.bag"

# # Processa o arquivo e gera a nuvem de pontos
# # process_bag_file(bag_file)

# # Diretório onde as nuvens de pontos estão salvas
# pointcloud_directory = "/home/beatriz/TCC/data/processed/"

# # Chama a função para visualizar todas as nuvens de pontos
# visualize_all_pointclouds(pointcloud_directory)

import open3d as o3d
import numpy as np
import rosbag
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import os

# Função para converter imagem ROS para Open3D PointCloud
def ros_image_to_o3d_pointcloud(depth_image, color_image, intrinsic_matrix):
    # Cria objeto PinholeCameraIntrinsic
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(intrinsic_matrix['width'], intrinsic_matrix['height'],
                             intrinsic_matrix['fx'], intrinsic_matrix['fy'],
                             intrinsic_matrix['cx'], intrinsic_matrix['cy'])

    # Cria a nuvem de pontos a partir das imagens de cor e profundidade
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(color_image),
        o3d.geometry.Image(depth_image),
        convert_rgb_to_intensity=False)

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
    return pcd

# Função para ler um arquivo .bag e processar a nuvem de pontos
def process_bag_file(bag_file, num_pointclouds):
    # Criação do ROS bag object
    bag = rosbag.Bag(bag_file)
    bridge = CvBridge()

    intrinsic_matrix = {
        'width': 640,  # Ajuste para o tamanho da sua imagem
        'height': 480,
        'fx': 615,     # Ajuste conforme necessário com base no seu sensor
        'fy': 615,
        'cx': 320,
        'cy': 240
    }

    # intrinsic_matrix = {
    #     'width': 1280,  # Ajuste para o tamanho da sua imagem
    #     'height': 720,
    #     'fx': 895,     # Ajuste conforme necessário com base no seu sensor
    #     'fy': 895,
    #     'cx': 640,
    #     'cy': 360
    # }

    # intrinsic_matrix = {
    #     'width': 1920,  # Ajuste para o tamanho da sua imagem
    #     'height': 1080,
    #     'fx': 1300,     # Ajuste conforme necessário com base no seu sensor
    #     'fy': 1300,
    #     'cx': 960,
    #     'cy': 540
    # }

    depth_image = None
    color_image = None
    total_frames = 0  # Contador de frames
    pointclouds_saved = 0  # Contador de nuvens de pontos salvas

    # Itera sobre as mensagens do bag e conta os frames
    for topic, msg, t in bag.read_messages(topics=['/camera/aligned_depth_to_color/image_raw/compressed',
                                                   '/camera/color/image_raw/compressed']):
        total_frames += 1

    # Calcula o intervalo de frames para salvar
    interval = total_frames // num_pointclouds

    print(f"Total de frames: {total_frames}, Intervalo para salvar nuvens de pontos: {interval}")

    # Reseta o bag para a leitura novamente
    bag.close()
    bag = rosbag.Bag(bag_file)

    # Itera novamente para processar as nuvens de pontos
    for i, (topic, msg, t) in enumerate(bag.read_messages(topics=['/camera/aligned_depth_to_color/image_raw/compressed',
                                                                  '/camera/color/image_raw/compressed'])):
        if topic == '/camera/aligned_depth_to_color/image_raw/compressed':
            # Converte a imagem de profundidade comprimida ROS para OpenCV
            np_arr = np.frombuffer(msg.data, np.uint8)
            depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)  # Carrega a imagem comprimida

        elif topic == '/camera/color/image_raw/compressed':
            # Converte a imagem de cor comprimida ROS para OpenCV
            np_arr = np.frombuffer(msg.data, np.uint8)
            color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Carrega a imagem comprimida

        # Se ambas as imagens forem obtidas
        if depth_image is not None and color_image is not None:
            # Chama a função para converter em nuvem de pontos
            pcd = ros_image_to_o3d_pointcloud(depth_image, color_image, intrinsic_matrix)

            # Salva a nuvem de pontos a cada 'interval' frames
            if i % interval == 0 and pointclouds_saved < num_pointclouds:
                # Nome do arquivo baseado no número da nuvem de pontos
                save_path = f"/home/beatriz/TCC/data/processed/rs_05_20_{pointclouds_saved}.pcd"
                o3d.io.write_point_cloud(save_path, pcd, write_ascii=True)
                print(f"Nuvem de pontos salva em: {save_path}")
                pointclouds_saved += 1  # Incrementa o contador

    bag.close()

# Função para ler e visualizar todas as nuvens de pontos em um diretório
def visualize_all_pointclouds(directory):
    # Lista todos os arquivos no diretório
    for filename in os.listdir(directory):
        if filename.endswith(".pcd"):  # Verifica se o arquivo é um .pcd
            # Cria o caminho completo para o arquivo
            file_path = os.path.join(directory, filename)
            print(f"Lendo nuvem de pontos de: {file_path}")
            
            # Carrega a nuvem de pontos
            pcd = o3d.io.read_point_cloud(file_path)
            
            # Visualiza a nuvem de pontos
            o3d.visualization.draw_geometries([pcd], window_name=filename)

# Arquivo .bag
bag_file = "/home/beatriz/TCC/data/raw/rs_05_20.bag"
num_pointclouds_to_save = 5  # Número de nuvens de pontos a salvar

# Processa o arquivo e gera a nuvem de pontos
process_bag_file(bag_file, num_pointclouds_to_save)

# Diretório onde as nuvens de pontos estão salvas
pointcloud_directory = "/home/beatriz/TCC/data/processed/"

# Chama a função para visualizar todas as nuvens de pontos
visualize_all_pointclouds(pointcloud_directory)
