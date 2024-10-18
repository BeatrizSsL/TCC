import os
import open3d as o3d
import numpy as np

def mesh_to_pointcloud(filepath, num_points=None, sampling_method="poisson"):
   
    """
    Converte um arquivo de malha (.stl, .ply, .obj) para uma nuvem de pontos (.pcd),
    salvando o resultado na pasta data/processed.

    Parâmetros:
    param filepath (str): Caminho absoluto do arquivo a ser convertido.
    param num_points (int, opcional): Número de pontos a serem amostrados. Se None, será ignorado para o método de vertex sampling.
    param sampling_method (str): Método de amostragem a ser utilizado ("poisson", "uniform" ou "vertex").

    Output:
    str: Caminho completo do arquivo .pcd salvo na pasta data/processed.
    """
    
    # Carrega o arquivo da malha
    mesh = o3d.io.read_triangle_mesh(filepath)
    mesh.compute_vertex_normals()

    # Verifica se a malha foi carregada
    if not mesh.is_empty():
        print("Malha carregada com sucesso.")
        print(f"Número de vértices: {len(mesh.vertices)}")
        print(f"Número de triângulos: {len(mesh.triangles)}\n")

    # Realiza a amostragem de pontos
    if sampling_method == "poisson":
        pointcloud = mesh.sample_points_poisson_disk(num_points or 50000)  # 50k pontos padrão se num_points for None
    elif sampling_method == "uniform":
        pointcloud = mesh.sample_points_uniformly(num_points or 50000)  # 50k pontos padrão se num_points for None
    elif sampling_method == "vertex":
        points = mesh.vertices  # Aqui assumimos que você quer os vértices
        pointcloud = o3d.geometry.PointCloud(points)
    else:
        raise ValueError("Método de amostragem inválido. Escolha 'poisson', 'uniform' ou 'vertex'.\n")


    # Salva a nuvem de pontos em um arquivo .pcd
    savepath = filepath.replace("/raw/", "/processed/")
    savepath = os.path.splitext(savepath)[0] + ".pcd"
    o3d.io.write_point_cloud(savepath, pointcloud, write_ascii=True)

    return pointcloud, savepath


def voxel_downsampling(pointcloud, voxel_size=0.05):
   
    """
    Filtra a nuvem de pontos utilizando um downsample por voxel e exibe a nuvem filtrada.
    
    Parâmetros:
    pcd (open3d.geometry.PointCloud): Nuvem de pontos original.
    voxel_size (float, padrão: 0.05): Tamanho do voxel para o downsample.
    
    Output:
    open3d.geometry.PointCloud: Nuvem de pontos filtrada.
    """
    
    # Número de pontos antes da filtragem
    original_point_count = len(pointcloud.points)
    print(f"Número de pontos antes da filtragem: {original_point_count}")

    print(f"Filtra a nuvem de pontos com um voxel de {voxel_size}")
    downpcd = pointcloud.voxel_down_sample(voxel_size=voxel_size)

    # Número de pontos depois da filtragem
    filtered_point_count = len(downpcd.points)
    print(f"Número de pontos depois da filtragem: {filtered_point_count}\n")

    return downpcd


def estimate_normals(pointcloud, radius=0.1, max_nn=30, show_normals=True):
    
    """
    Recalcula as normais da nuvem de pontos.

    Parâmetros:
    pcd (open3d.geometry.PointCloud): Nuvem de pontos.
    radius (float, padrão: 0.1): Raio de pesquisa usado para estimar as normais.
    max_nn (int, padrão: 30): Número máximo de vizinhos considerados para a estimativa das normais.
    show_normals (bool, padrão: True): Se True, as normais serão exibidas na visualização.

    Output:
    open3d.geometry.PointCloud: Nuvem de pontos com as normais recalculadas.
    """
    
    print(f"Recalculando as normais da nuvem de pontos com raio {radius} e {max_nn} vizinhos\n")
    
    # Recalcula as normais da nuvem de pontos
    pointcloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius,
                                                          max_nn=max_nn)
    )
        
    return pointcloud

def paint_pointcloud(pointcloud, color=[1, 0.706, 0]):
    
    """
    Pinta o objeto 3D.

    Parâmetros:
    pointcloud (open3d.geometry.TriangleMesh): Objeto 3D a ser pintado.

    Output:
    None
    """
    
    print("Pintando o objeto\n")
    pointcloud.paint_uniform_color(color)


def create_bounding_boxes(pointcloud):
    
    """
    Calcula a bounding box alinhada ao eixo (AABB) e a bounding box orientada (OBB)
    da nuvem de pontos.

    Parâmetros:
    pointcloud (open3d.geometry.PointCloud): Nuvem de pontos da qual calcular as caixas delimitadoras.
   
    Output:
    tuple: Contém a AABB e a OBB (open3d.geometry.AxisAlignedBoundingBox, open3d.geometry.OrientedBoundingBox).
    """

    print("Calculando os bounding boxes do objeto\n")
    # Calcula a bounding box alinhada ao eixo
    aabb = pointcloud.get_axis_aligned_bounding_box()
    aabb.color = (1, 0, 0)  # Define a cor da AABB como vermelho

    # Calcula a bounding box orientada
    obb = pointcloud.get_oriented_bounding_box()
    obb.color = (0, 1, 0)  # Define a cor da OBB como verde
    
    return aabb, obb
