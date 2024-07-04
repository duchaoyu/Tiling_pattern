import numpy as np
from compas.datastructures import Mesh
from compas.geometry import Vector, centroid_points
from compas.utilities import pairwise
import os
import math as m
import open3d as o3d

path = os.path.abspath('/Users/duch/Documents/Github/phd/optimization/data/tmesh.json')
o_path = os.path.abspath('/Users/duch/Documents/Github/phd/optimization/data/tmesh_out.json')
I_mesh = Mesh.from_json(path)
O_mesh = Mesh.from_json(o_path)

tri_mesh = Mesh()
for vkey in I_mesh.vertices():
    xyz = I_mesh.vertex_coordinates(vkey)
    tri_mesh.add_vertex(x=xyz[0], y=xyz[1], z=xyz[2])

for fkey in I_mesh.faces():
    f_vkeys = I_mesh.face_vertices(fkey)
    if len(f_vkeys) <= 3:
        tri_mesh.add_face(f_vkeys)
    elif len(f_vkeys) == 3:
        tri_mesh.add_face(f_vkeys[0:3])
        tri_mesh.add_face(f_vkeys[1:])
    else:
        centroid = centroid_points([I_mesh.vertex_coordinates(vkey) for vkey in f_vkeys])
        key = tri_mesh.add_vertex(x=centroid[0], y=centroid[1], z=centroid[2])
        for i, j in pairwise(f_vkeys + f_vkeys[:1]):
            tri_mesh.add_face([i, j, key])



o3dmesh = o3d.geometry.TriangleMesh()
np_vertices = np.asarray([tri_mesh.vertex_coordinates(key) for key in tri_mesh.vertices()])
faces = [tri_mesh.face_vertices(fkey) for fkey in tri_mesh.faces()]
np_triangles = np.asarray(faces, dtype=np.int32)
o3dmesh.vertices = o3d.utility.Vector3dVector(np_vertices)
o3dmesh.triangles = o3d.utility.Vector3iVector(np_triangles)

# Create a scene and add the triangle mesh
t_mesh = o3d.t.geometry.TriangleMesh.from_legacy(o3dmesh)
scene = o3d.t.geometry.RaycastingScene()
_ = scene.add_triangles(t_mesh)  # we do not need the geometry ID fo


query_points = np.array([O_mesh.vertex_coordinates(key) for key in O_mesh.vertices()]).astype(np.float32)
unsigned_distances = scene.compute_distance(query_points)

print(unsigned_distances.numpy().tolist())