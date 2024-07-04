import pygad
import numpy as np
from compas.datastructures import Mesh
from compas.geometry import Vector, centroid_points
from compas.numerical import dr_numpy
from compas.utilities import pairwise
import os
import math as m
import open3d as o3d

path = os.path.abspath('/Users/duch/Documents/Github/phd/optimization/data/tmesh.json')
o_path = os.path.abspath('/Users/duch/Documents/Github/phd/optimization/data/tmesh_out.json')
I_mesh = Mesh.from_json(path)


best_fitness = m.inf



# ======



# =========

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



# function_inputs = [1.0 for i in range(input_length)]


def fitness_func(solution, solution_idx):
    global best_fitness
    pressure = solution[-1]

    mesh = I_mesh.copy()

    # initiate attributes
    dva = {
        'is_fixed': False,
        'x': 0.0,
        'y': 0.0,
        'z': 0.0,
        'px': 0.0,
        'py': 0.0,
        'pz': 0.0,
        'rx': 0.0,
        'ry': 0.0,
        'rz': 0.0,
    }

    dea = {
        'qpre': 1.0,
        'fpre': 0.0,
        'lpre': 0.0,
        'linit': 0.0,
        'E': 210, # steel
        'radius': 5,
    }
        
    mesh.update_default_vertex_attributes(dva)
    mesh.update_default_edge_attributes(dea)

    v_bdrs = set()
    for v_bdr in mesh.vertices_on_boundaries():
        v_bdrs.update(v_bdr)
    v_bdrs = list(v_bdr)
    mesh.vertices_attribute('is_fixed', True, v_bdrs)

    k_i = mesh.key_index()

    for i, edge in enumerate(mesh.edges()):
        mesh.edge_attribute(edge, 'qpre', solution[i])
        
    

    def callback(k, xyz, crits, args):

        for key, attr in mesh.vertices(True):
            index = k_i[key]
            attr['x'] = xyz[index][0]
            attr['y'] = xyz[index][1]
            attr['z'] = xyz[index][2]

    # density of compacted earth earth: 1.36 g/cm3 -> 13.6 kN/m3
    d = 13.6
    t = 0.025 # thickness: 0.025m 
    # pressure: kN/m2
        
    # DR
    def inflate(mesh, pressure): 
        xyz      = mesh.vertices_attributes(['x', 'y', 'z'])
        edges    = [(k_i[u], k_i[v]) for u, v in mesh.edges()]
        fixed    = [k_i[key] for key in mesh.vertices_where({'is_fixed': True})]
        
        for vkey in mesh.vertices_where({'is_fixed': False}):
            v_n = mesh.vertex_normal(vkey)
            v_area = mesh.vertex_area(vkey)

            if m.isinf(v_area): return -m.inf


            px = v_n[0] * v_area * pressure
            py = v_n[1] * v_area * pressure
            pz = v_n[2] * v_area * pressure

            pz -= v_area * t * d  # weight of the tile
            mesh.vertex_attributes(vkey, ['px', 'py', 'pz'], [px, py, pz])
        
        loads = mesh.vertices_attributes(['px', 'py', 'pz'])
        qpre = mesh.edges_attribute('qpre')
        fpre     = mesh.edges_attribute('fpre')
        lpre     = mesh.edges_attribute('lpre')
        linit    = mesh.edges_attribute('linit')
        radius   = mesh.edges_attribute('radius')
        stiffness = mesh.edges_attribute('E')

        xyz, q, f, l, r = dr_numpy(xyz, edges, fixed, loads, qpre, E=stiffness, radius=radius, kmax=50, callback=callback)

        for index, (key, attr) in enumerate(mesh.vertices(True)):
            mesh.vertex_attribute(key, 'residual', Vector(*r[key]))
            
        for index, ((u, v), attr) in enumerate(mesh.edges(True)):
            mesh.edge_attribute((u,v),'f',f[index][0])
            mesh.edge_attribute((u,v),'l',l[index][0])

    for k in range(5):
        inflate(mesh, pressure=pressure)

    # ======

    query_points = np.array([mesh.vertex_coordinates(key) for key in mesh.vertices()]).astype(np.float32)
    unsigned_distances = scene.compute_distance(query_points)
    fitness = unsigned_distances.sum().item()
    print(fitness, best_fitness, solution)
    if fitness < best_fitness:
        # print(solution)
        best_fitness = fitness
        mesh.to_json(o_path)
    return -fitness

fitness_function = fitness_func

num_generations = 1000
num_parents_mating = 4

sol_per_pop = 20
num_genes = len(list(I_mesh.edges())) + 1

init_range_low = 0.1
init_range_high = 1.5

parent_selection_type = "sss"
keep_parents = 1

crossover_type = "single_point"

mutation_type = "random"
mutation_percent_genes = 10

ga_instance = pygad.GA(num_generations=num_generations,
                       num_parents_mating=num_parents_mating,
                       fitness_func=fitness_function,
                       sol_per_pop=sol_per_pop,
                       num_genes=num_genes,
                       init_range_low=init_range_low,
                       init_range_high=init_range_high,
                       parent_selection_type=parent_selection_type,
                       keep_parents=keep_parents,
                       crossover_type=crossover_type,
                       mutation_type=mutation_type,
                       mutation_percent_genes=mutation_percent_genes)


# ga_instance.run()