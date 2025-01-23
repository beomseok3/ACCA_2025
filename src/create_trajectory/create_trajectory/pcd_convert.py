# import pcl
# import numpy as np
# pc = pcl.load("/home/ps/맵/school/pcd/GlobalMap.pcd")
# pc_array = pc.to_array()
# for i in pc_array:
#     li = list(i)
#     ind = pc_array.index(i)
#     li[0] += 196394.986783
#     li[1] += 493770.804259
#     tu = tuple(li)
#     pc_array[ind] = tu
# pcl.save(pc_array,"/home/ps/맵/school/pcd/GlobalMap_converted.pcd")

# # pcl.save(pc, "/home/ps/맵/school/pcd/GlobalMap_converted.pcd")

import open3d as o3d
import numpy as np

# PCD 파일 읽기
pcd_path = "/home/acca/backups/pointcloud_map_origin.pcd"
pcd = o3d.io.read_point_cloud(pcd_path)

points = np.asarray(pcd.points)

translation_vector = np.array([196394.986783, 493770.804259, 0.0])  

translated_points = points + translation_vector

translated_pcd = o3d.geometry.PointCloud()
translated_pcd.points = o3d.utility.Vector3dVector(translated_points)

output_path = "/home/acca/backups/pointcloud_map_origin_translated.pcd"
o3d.io.write_point_cloud(output_path, translated_pcd)