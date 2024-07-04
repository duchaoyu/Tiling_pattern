import string
from collections import defaultdict

import gurobipy as gp
from gurobipy import GRB

# vertices_rows = 60
# vertices_columns = 60

import os 
from compas.datastructures import Mesh
file = "/Users/duch/Documents/Github/libigl/build/_deps/libigl_tests_tata-src/quad_output.obj"
mesh = Mesh.from_obj(file)
adj_lists = mesh.face_adjacency()


def ordered_pair(v1, v2):
    return (v1, v2) if v1 < v2 else (v2, v1)

edges = set()
# a dictionary whose key is the node(mesh face)
#                    value is the edge(two adjacent mesh faces)
edges_by_vert = defaultdict(set) 
for a in adj_lists.keys():
    for b in adj_lists[a]:
        e = ordered_pair(a, b)
        edges.add(e)
        edges_by_vert[a].add(e)
        edges_by_vert[b].add(e)

print(len(adj_lists), 'Vertexes')
print('Edges:', edges)

try:
    # initiate the model
    m = gp.Model("optimal_bricks")

    # add variables, all the edges are variables
    edges_picked_vars = m.addVars(edges, vtype=GRB.BINARY, lb = 0, ub = 1)

    # add constraint, every node can only be connected once
    for a in edges_by_vert.keys():
        m.addConstr(sum([edges_picked_vars[e] for e in edges_by_vert[a]]) <= 1, str(a))

    # Find pairs of edges which are directly next to each other and parallel, e.g. (a,b) and (c,d), and (a,c) and (b,d):
    # a-b-*
    # | | |
    # *-c-d
    # We want to avoid such combination of edges because it is undesirable to have bricks next to each other like this.
    #   -   -  v**
    #     | c2 | d2
    #   - v -  u**
    # | a | b  |
    #   - u -  v* -
    #     | c1 | d1 |
    #       -  u* -
    # The code below finds the pair (a,b) & (c,d)
    running_bond = set()
    for a in adj_lists.keys():
        face_halfedges = mesh.face_halfedges(a)
        for b in adj_lists[a]:
            for (u, v) in face_halfedges:
                if mesh.halfedge_face(v, u) == b:
                    break
            v_hat = mesh.face_vertex_after(b, u, 1)
            u_hat2 = mesh.face_vertex_before(b, v, 1)
            if mesh.halfedge_face(v_hat, u) != None:
                c1 = mesh.halfedge_face(v_hat, u)
                u_hat = mesh.face_vertex_before(c1, v_hat, 1)
                if mesh.halfedge_face(v_hat, u_hat) != None:
                    d1 = mesh.halfedge_face(v_hat, u_hat)
                    e1 = ordered_pair(a, b)
                    e2 = ordered_pair(c1, d1)
                    running_bond.add(ordered_pair(e1, e2))
                else:
                    continue
            else:
                continue
            if mesh.halfedge_face(v, u_hat2) != None:
                c2 =  mesh.halfedge_face(v, u_hat2)
                v_hat2 = mesh.face_vertex_after(c2, u_hat2, 1)
                if mesh.halfedge_face(v_hat2, u_hat2) != None:
                    d2 = mesh.halfedge_face(v_hat2, u_hat2)
                    e3 = ordered_pair(c2, d2)
                    running_bond.add(ordered_pair(e1, e3))
                else:
                    continue
            else:
                continue

    running_bond_pairs_vars = []
    for e1, e2 in running_bond:
        edge_str = str(f"({e1})+({e2})")
        running_bond_exists = m.addVar(lb=0, ub=1, name=edge_str)
        m.addConstr(running_bond_exists <= edges_picked_vars[e1], "constr_" + edge_str)
        m.addConstr(running_bond_exists <= edges_picked_vars[e2], "constr_" + edge_str)
        running_bond_pairs_vars.append(running_bond_exists)

    print("parallel_brick_pairs_vars", len(running_bond_pairs_vars))


    edge_neighbors = set()
    for a in adj_lists.keys():
        for b in adj_lists[a]:
            for c in adj_lists[a]:
                if b != c:
                    for d in adj_lists[c]:
                        if d in adj_lists[b]:
                            e1 = ordered_pair(a, b)
                            e2 = ordered_pair(c, d)
                            edge_neighbors.add(ordered_pair(e1, e2))
    
    parallel_brick_pairs_vars = []
    for e1, e2 in edge_neighbors:
        edge_str = str(f"({e1})+({e2})")
        edge_pair_exists = m.addVar(lb=0, ub=1, name=edge_str)
        m.addConstr(edge_pair_exists >= edges_picked_vars[e1] + edges_picked_vars[e2] - 1, "constr_" + edge_str)
        parallel_brick_pairs_vars.append(edge_pair_exists)

    print(len(parallel_brick_pairs_vars))


    # It seems more important to minimize the number of irregular bricks, so we assign a 5x higher wait to this part of
    # the objective function
    # m.setObjective(sum(edges_picked_vars.values()) + 1000.0 * sum(running_bond_pairs_vars) - 0.2 * sum(parallel_brick_pairs_vars), GRB.MAXIMIZE)
    m.setObjective(sum(edges_picked_vars.values()) + 2000.0 * sum(running_bond_pairs_vars) , GRB.MAXIMIZE)

    m.optimize()
    print('Obj: %g' % m.ObjVal)
    bricks_cnt = sum(int(v.X) for v in edges_picked_vars.values())
    print(f"{bricks_cnt} bricks, {len(adj_lists) - bricks_cnt*2} unconnected vertex(es)")
    parallel_bricks_cnt = sum(round(v.X) for v in parallel_brick_pairs_vars)
    print(f"{parallel_bricks_cnt} pair(s) of parallel bricks")

    chosen_edges = []
    for e in edges_picked_vars.keys():
        if edges_picked_vars[e].X == 1:
            chosen_edges.append(e)

    print(chosen_edges)

    # for var in running_bond_pairs_vars:
    #     print(var)

except gp.GurobiError as e:
    print('Error code ' + str(e.errno) + ': ' + str(e))

except AttributeError:
    print('Encountered an attribute error')
