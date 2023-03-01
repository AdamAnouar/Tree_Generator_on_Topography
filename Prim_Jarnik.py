""" 
    This code is meant to be an implementation of the Prim_Jarnik algorithm using the Rhino grasshopper libraries.
    This code is meant to be copy pasted inside a ghPython component.
    Inputs :
            edges : List of lines of delauney network connecting all the points
            seed : Int value to change the random seed for adding additional edges to the final MinSpanningTree
            num_additional_edges : Int value for the number of edges to add to the final MinSpanningTree

    Outputs :
            MSTgrid : List of edges of the final MinSpanningTree

"""


import Rhino.Geometry as rg
import random

#Extract end points from every edge and
#Create a dict : for every point and its neighbor nodes; node = [neighbor_pt + cost] 
#which will be the playground for our prim's Algorithm


def Playground(edges):
    
    Edges_pts = {}
    set_points = set()
    
    for edge in edges:
        ptS = edge.PointAt(0)
        ptE = edge.PointAt(1)
        Edges_pts.setdefault(ptS, []).append([ptE,edge.Length, ptS])
        Edges_pts.setdefault(ptE,[]).append([ptS,edge.Length, ptE])
        set_points.add(ptS)
        set_points.add(ptE)
        
    return Edges_pts, set_points

#Function to find the smallest cost among a list of nodes

def find(candidates):
    cost_candidates = [node[1] for node in candidates]
    chosen_index = cost_candidates.index(min(cost_candidates))
    node = candidates[chosen_index]
    return node


#Prim's Algorithm logic

def Prim_Algorithm(Edges_pts, N):

    MST = []
    Visited = []
    candidates = []
    Visited.append(random.choice(list(Edges_pts)))
    
    
    while len(MST) < N:
        for selected in Visited:
            for node in Edges_pts[selected]:
                if node[0] not in Visited:
                    candidates.append(node)
        chosen_node = find(candidates)
        MST.append([chosen_node[0],chosen_node[2]])
        Visited.append(chosen_node[0])
        candidates = []
    return MST


#Extract the Minimum Spanning Grid

def MSTgrid_Creator(MST):
    MSTgrid = []
    
    for list in MST:
        line = rg.Line(list[0],list[1])
        MSTgrid.append(line)
    return MSTgrid


#Randomly add segments to increase the porosity of the network

def add_porosity(edges, MSTgrid, num_additional_edges):
    global seed
    random.seed(seed)
    
    out_lines = []
    for line in edges:
        if line not in MSTgrid:
            out_lines.append(line)
    
    add_lines = random.sample(out_lines, num_additional_edges)
    
    MSTgrid.extend(add_lines)
    return MSTgrid


#Runnning the code

try:
    Edges_pts, set_points = Playground(edges)
    #NV is the number of unique vertices  and N is the number of edges our mst should have
    NV = len(set_points)
    N = NV - 1
    MST = Prim_Algorithm(Edges_pts, N)
    MSTgrid = MSTgrid_Creator(MST)
    MSTgrid = add_porosity(edges, MSTgrid, num_additional_edges)
except Exception:
    print("Sorry, missing or wrong Data")
