import string
from collections import defaultdict

import gurobipy as gp
from gurobipy import GRB

vertices_rows = 40
vertices_columns = 40

vertices = []
vert_by_idx = []
curr_vertex_idx = 0
for r in range(vertices_rows):
    v_r = []
    for c in range(vertices_columns):
        v_r.append(curr_vertex_idx)
        vert_by_idx.append((r, c))
        curr_vertex_idx += 1
        vertices.append(v_r)


def get_edge(v1, v2):
    return (v1, v2) if v1 < v2 else (v2, v1)


edges = set()
edges_by_vert = defaultdict(list)
for r in range(vertices_rows):
    for c in range(vertices_columns):
        v1 = (r, c)
        for d in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            r2 = r + d[0]
            c2 = c + d[1]
            if 0 <= r2 < vertices_rows and 0 <= c2 < vertices_columns:
                v2 = (r2, c2)
                e = get_edge(v1, v2)
                edges.add(e)
                edges_by_vert[(r, c)].append(e)


adj_lists =   {0: [1, 10], 1: [2, 0, 11], 2: [3, 1, 12], 3: [4, 2, 13], 4: [5, 3, 14], 5: [6, 4, 15], 6: [7, 5, 16], 7: [8, 6, 17], 8: [9, 7, 18], 9: [8, 19], 10: [11, 0, 20], 11: [12, 1, 10, 21], 12: [13, 2, 11, 22], 13: [14, 3, 12, 23], 14: [15, 4, 13, 24], 15: [16, 5, 14, 25], 16: [17, 6, 15, 26], 17: [18, 7, 16, 27], 18: [19, 8, 17, 28], 19: [9, 18, 29], 20: [21, 10, 30], 21: [22, 11, 20, 31], 22: [23, 12, 21, 32], 23: [24, 13, 22, 33], 24: [25, 14, 23, 34], 25: [26, 15, 24, 35], 26: [27, 16, 25, 36], 27: [28, 17, 26, 37], 28: [29, 18, 27, 38], 29: [19, 28, 39], 30: [31, 20, 40], 31: [32, 21, 30, 41], 32: [33, 22, 31, 42], 33: [34, 23, 32, 43], 34: [35, 24, 33, 44], 35: [36, 25, 34, 45], 36: [37, 26, 35, 46], 37: [38, 27, 36, 47], 38: [39, 28, 37, 48], 39: [29, 38, 49], 40: [41, 30, 50], 41: [42, 31, 40, 51], 42: [43, 32, 41, 52], 43: [44, 33, 42, 53], 44: [45, 34, 43, 54], 45: [46, 35, 44, 55], 46: [47, 36, 45, 56], 47: [48, 37, 46, 57], 48: [49, 38, 47, 58], 49: [39, 48, 59], 50: [51, 40, 60], 51: [52, 41, 50, 61], 52: [53, 42, 51, 62], 53: [54, 43, 52, 63], 54: [55, 44, 53, 64], 55: [56, 45, 54, 65], 56: [57, 46, 55, 66], 57: [58, 47, 56, 67], 58: [59, 48, 57, 68], 59: [49, 58, 69], 60: [61, 50, 70], 61: [62, 51, 60, 71], 62: [63, 52, 61, 72], 63: [64, 53, 62, 73], 64: [65, 54, 63, 74], 65: [66, 55, 64, 75], 66: [67, 56, 65, 76], 67: [68, 57, 66, 77], 68: [69, 58, 67, 78], 69: [59, 68, 79], 70: [71, 60, 80], 71: [72, 61, 70, 81], 72: [73, 62, 71, 82], 73: [74, 63, 72, 83], 74: [75, 64, 73, 84], 75: [76, 65, 74, 85], 76: [77, 66, 75, 86], 77: [78, 67, 76, 87], 78: [79, 68, 77, 88], 79: [69, 78, 89], 80: [81, 70, 90], 81: [82, 71, 80, 91], 82: [83, 72, 81, 92], 83: [84, 73, 82, 93], 84: [85, 74, 83, 94], 85: [86, 75, 84, 95], 86: [87, 76, 85, 96], 87: [88, 77, 86, 97], 88: [89, 78, 87, 98], 89: [79, 88, 99], 90: [91, 80], 91: [92, 81, 90], 92: [93, 82, 91], 93: [94, 83, 92], 94: [95, 84, 93], 95: [96, 85, 94], 96: [97, 86, 95], 97: [98, 87, 96], 98: [99, 88, 97], 99: [89, 98]}

try:

    # Create a new model
    m = gp.Model("matching1")

    edge_picked_var = m.addVars(edges, vtype=GRB.BINARY, lb = 0, ub = 1)

    for u in edges_by_vert.keys():
        m.addConstr(sum([edge_picked_var[e] for e in edges_by_vert[u]]) <= 1, str(u))

    penalties = []
    for u1, u2 in edges:
        if u1[0] == u2[0] and u1[0] < vertices_rows - 1:
            v1 = (u1[0] + 1, u1[1])
            v2 = (u2[0] + 1, u2[1])
            v = m.addVar(lb=0, ub=1, name=str((u1, u2)) + "+" + str((v1, v2)))
            m.addConstr(v >= edge_picked_var[(u1, u2)] + edge_picked_var[(v1, v2)] - 1)
            penalties.append(v)
        if u1[1] == u2[1] and u1[1] < vertices_columns - 1:
            v1 = (u1[0], u1[1] + 1)
            v2 = (u2[0], u2[1] + 1)
            v = m.addVar(lb=0, ub=1, name=str((u1, u2)) + "+" + str((v1, v2)))
            m.addConstr(v >= edge_picked_var[(u1, u2)] + edge_picked_var[(v1, v2)] - 1)
            penalties.append(v)

    print('Penalties done')

    # Set objective
    # m.setObjective(sum(edge_picked_var.values()) - sum(penalties), GRB.MAXIMIZE)
    m.setObjective(sum(edge_picked_var.values()), GRB.MAXIMIZE)

    # Optimize model
    m.optimize()
    print('Obj: %g' % m.ObjVal)

    all_colors = list(string.ascii_lowercase + string.ascii_uppercase + string.digits)
    curr_color_idx = 0

    colors_by_vertex = {}
    for e in edge_picked_var.keys():
        # print(e, int(edge_picked_var[e].X))
        if 0.001 < edge_picked_var[e].X < 0.999:
            print(e, edge_picked_var[e].X)
        if edge_picked_var[e].X == 1:
            v1, v2 = e
            colors_by_vertex[v1] = all_colors[curr_color_idx]
            colors_by_vertex[v2] = all_colors[curr_color_idx]
            curr_color_idx += 1
            if curr_color_idx >= len(all_colors):
                curr_color_idx = 0

    for r in range(vertices_rows):
        print(''.join(colors_by_vertex[(r, c)] if (r,c) in colors_by_vertex else ' ' for c in range(vertices_columns)))

except gp.GurobiError as e:
    print('Error code ' + str(e.errno) + ': ' + str(e))

except AttributeError:
    print('Encountered an attribute error')
