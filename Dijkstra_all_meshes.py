""" 
    This code is meant to be an implementation of Dijkstra algorithm using the Rhino grasshopper libraries.
    and have a great control over the output by Adding multiple parameters
    It can be used to search in any kind of mesh however it is slow with heavy meshes
    This code is meant to be copy pasted inside a ghPython component.
    Inputs :
            mesh_pts : List of mesh points
            PointA : The start point
            PointB : The goal point to reach
            max_Angle : Angle in rad for maximum slope
            excluded_pts : List of points in the mesh to exclude from being visited by the search algorithm
    
    Outputs :
            sorted_pts : List of the sorted mesh points
            optimal_sequence : List of indices of the final sequence of points
            closed : List of indices of all the visited points during the process

"""

import Rhino.Geometry as rg
import math
from ghpythonlib.treehelpers import list_to_tree



#We sort the points of the topography in the X and the Y axis
def sort_pts(mesh_pts):
    pt_sort = []
    for i, pt in enumerate(mesh_pts):
        pt_sort.append(pt.X*10000 + pt.Y*10)
    sortedp = sorted(zip(mesh_pts, pt_sort))
    sorted_pts = []
    #unpack tuples and retrieve only the points
    for pt, val in sortedp:
        sorted_pts.append(pt)
        
    return sorted_pts


#We split the plain list into list of lists to make neighbor search easier
def pts_branched(sorted_pts):
    Grid = []
    new_list = []
    for i, pt in enumerate(sorted_pts):
        if (i != 0 and pt.X != sorted_pts[i-1].X):
            Grid.append(new_list)
            new_list = []
        elif i == len(sorted_pts)-1:
            new_list.append([i, pt.Z])
            Grid.append(new_list)
            break
        new_list.append([i,pt.Z])
#    print len(Grid[2])
    return Grid


#find index of pointA (Start point) and pointB (Goal point) in the new ordered list
def Start_End_Points(sorted_pts):
    Start_point = []
    Goal_point = []
    for i, pt in enumerate(sorted_pts):
        if pt == PointA:
            Start_point.append(i)
        if pt == PointB:
            Goal_point.append(i)
    return Start_point[0], Goal_point[0]


#The Tree will be the playground of the Astar
#Tree is a dict that has every node as a key, and its lists of [neighbor + cost] as a value
#in the for loop a node = [pt_index, pt.Z]

def Playground(Grid):
    global max_Angle
    Tree = {}
    for i, list in enumerate(Grid):
        for j, node in enumerate(list):
            if i > 0 and abs(abs(node[1]) - abs(Grid[i-1][j][1])) / math.sqrt(12**2 + (abs(node[1]) - abs(Grid[i-1][j][1]))**2) < math.sin(max_Angle):
                Tree.setdefault(node[0], []).append([Grid[i-1][j][0], math.sqrt(math.pow(abs(abs(Grid[i-1][j][1])-abs(Grid[i][j][1])),2) + math.pow(12,2))])
            if j > 0 and abs(abs(node[1]) - abs(Grid[i][j-1][1])) / math.sqrt(12**2 + (abs(node[1]) - abs(Grid[i][j-1][1]))**2) < math.sin(max_Angle):
                Tree.setdefault(node[0], []).append([Grid[i][j-1][0], math.sqrt(math.pow(abs(abs(Grid[i][j-1][1])-abs(Grid[i][j][1])),2) + math.pow(12,2))])
            if i < len(Grid) - 1 and abs(abs(node[1]) - abs(Grid[i+1][j][1])) / math.sqrt(12**2 + (abs(node[1]) - abs(Grid[i+1][j][1]))**2) < math.sin(max_Angle):
                Tree.setdefault(node[0], []).append([Grid[i+1][j][0], math.sqrt(math.pow(abs(abs(Grid[i+1][j][1])-abs(Grid[i][j][1])),2) + math.pow(12,2))])
            if j < len(list) - 1 and abs(abs(node[1]) - abs(Grid[i][j+1][1])) / math.sqrt(12**2 + (abs(node[1]) - abs(Grid[i][j+1][1]))**2) < math.sin(max_Angle):
                Tree.setdefault(node[0], []).append([Grid[i][j+1][0], math.sqrt(math.pow(abs(abs(Grid[i][j+1][1])-abs(Grid[i][j][1])),2) + math.pow(12,2))])
    return Tree


#A second version that allows you to add breps to exclude points

def Playground_with_pt_exclusion(Grid):
    global max_Angle, excluded_pts
    Tree = {}
    for i, list in enumerate(Grid):
        for j, node in enumerate(list):
            if i > 0 and abs(abs(node[1]) - abs(Grid[i-1][j][1])) / math.sqrt(12**2 + (abs(node[1]) - abs(Grid[i-1][j][1]))**2) < math.sin(max_Angle) and sorted_pts[node[0]] not in excluded_pts and sorted_pts[Grid[i-1][j][0]] not in excluded_pts:
                Tree.setdefault(node[0], []).append([Grid[i-1][j][0], math.sqrt(math.pow(abs(abs(Grid[i-1][j][1])-abs(Grid[i][j][1])),2) + math.pow(12,2))])
            if j > 0 and abs(abs(node[1]) - abs(Grid[i][j-1][1])) / math.sqrt(12**2 + (abs(node[1]) - abs(Grid[i][j-1][1]))**2) < math.sin(max_Angle) and sorted_pts[node[0]] not in excluded_pts and sorted_pts[Grid[i][j-1][0]] not in excluded_pts:
                Tree.setdefault(node[0], []).append([Grid[i][j-1][0], math.sqrt(math.pow(abs(abs(Grid[i][j-1][1])-abs(Grid[i][j][1])),2) + math.pow(12,2))])
            if i < len(Grid) - 1 and abs(abs(node[1]) - abs(Grid[i+1][j][1])) / math.sqrt(12**2 + (abs(node[1]) - abs(Grid[i+1][j][1]))**2) < math.sin(max_Angle) and sorted_pts[node[0]] not in excluded_pts and sorted_pts[Grid[i+1][j][0]] not in excluded_pts:
                Tree.setdefault(node[0], []).append([Grid[i+1][j][0], math.sqrt(math.pow(abs(abs(Grid[i+1][j][1])-abs(Grid[i][j][1])),2) + math.pow(12,2))])
            if j < len(list) - 1 and abs(abs(node[1]) - abs(Grid[i][j+1][1])) / math.sqrt(12**2 + (abs(node[1]) - abs(Grid[i][j+1][1]))**2) < math.sin(max_Angle) and sorted_pts[node[0]] not in excluded_pts and sorted_pts[Grid[i][j+1][0]] not in excluded_pts:
                Tree.setdefault(node[0], []).append([Grid[i][j+1][0], math.sqrt(math.pow(abs(abs(Grid[i][j+1][1])-abs(Grid[i][j][1])),2) + math.pow(12,2))])
    return Tree


#A more reliable way to generate the Tree if the def Playground is not working properly, it may be slower

def Playground_alternative(srt_pts, end_pts, sorted_pts):
    Tree = {}
    closed = []
    for srt_pt,end_pt in zip(srt_pts, end_pts):
        if ((abs(srt_pt.Z) - abs(end_pt.Z)) / math.sqrt((12**2) + (abs(srt_pt.Z) - abs(end_pt.Z))**2)) < math.sin(max_Angle):
            Tree.setdefault(sorted_pts.index(end_pt),[]).append([sorted_pts.index(srt_pt), math.sqrt((abs(srt_pt.X) - abs(end_pt.X))**2 + (abs(srt_pt.Y) - abs(end_pt.Y))**2 + (abs(srt_pt.Z) - abs(end_pt.Z))**2)])
            Tree.setdefault(sorted_pts.index(srt_pt),[]).append([sorted_pts.index(end_pt), math.sqrt((abs(srt_pt.X) - abs(end_pt.X))**2 + (abs(srt_pt.Y) - abs(end_pt.Y))**2 + (abs(srt_pt.Z) - abs(end_pt.Z))**2)])
    return Tree


#Defining Dijkstra’s logic :  searching for neighbor nodes iteratively until finding goal node. Then using weight to loop over the sequence of points from goal to start to find the shortest path.



def DijkstraAlgorithm():
    global Tree, Start_point, Goal_point
    cost = {Start_point: 0}
    closed = []
    came_from = {}
    opened = [[Start_point, cost[Start_point]]]
    #Dijkstra’s Search logic
    while True:
        #pick a node
        node = opened[0][0]
        closed.append(opened[0])
        del opened[0]
        #End the loop if we reach the goal node
        if closed[-1][0] == Goal_point:
            print "Goal point found !"
            break
        for item in Tree[node]:
            if item[0] in [closed_item[0] for closed_item in closed]:
                continue
            cost_node = cost[node] + item[1]
            temp = [item[0], cost_node]
            if temp[0] in [open[0] for open in opened]:
                if cost[node] + item[1] < cost[temp[0]]:
                    cost.update({item[0]: cost[node] + item[1]})
                    chosen_index = [open[0] for open in opened].index(temp[0])
                    del opened[chosen_index]
                    opened.append(temp)
                    came_from[temp[0]] = node
            elif temp[0] not in [open[0] for open in opened]:
                cost.update({item[0]: cost[node] + item[1]})
                opened.append(temp)
                came_from[temp[0]] = node
        #Stopping the loop before having an error
        if len(opened) == 0:
            print "Failed to find the Goal point..."
            break
    #Loop back to find the sequence 
    trace_node = Goal_point
    optimal_sequence = [Goal_point]
    while trace_node in came_from.Keys:
        trace_node = came_from[trace_node]
        optimal_sequence.append(trace_node)
    
    
    optimal_sequence.reverse()
    
    return  closed, optimal_sequence



#Running the Code
if Network == 0:
    if len(excluded_pts) == 0:
        try:
            sorted_pts = sort_pts(mesh_pts)
            Grid = pts_branched(sorted_pts)
            Start_point, Goal_point = Start_End_Points(sorted_pts)
            Tree = Playground(Grid)
            closed, optimal_sequence = DijkstraAlgorithm()
            closed = list_to_tree(closed)
        except Exception:
            print("Sorry, missing or wrong Data")
            
    elif len(excluded_pts) != 0:
        try:
            sorted_pts = sort_pts(mesh_pts)
            Grid = pts_branched(sorted_pts)
            Start_point, Goal_point = Start_End_Points(sorted_pts)
            Tree = Playground_with_pt_exclusion(Grid)
            closed, optimal_sequence = DijkstraAlgorithm()
            closed = list_to_tree(closed)
        except Exception:
            print("Sorry, missing or wrong Data")

if Network == 1:
    try:
        sorted_pts = sort_pts(mesh_pts)
        Grid = pts_branched(sorted_pts)
        Start_point, Goal_point = Start_End_Points(sorted_pts)
        Tree = Playground_alternative(srt_pts, end_pts, sorted_pts)
        Heuristic = heuristic(sorted_pts)
        closed, optimal_sequence = DijkstraAlgorithm()
        closed = list_to_tree(closed)
    except Exception:
        print("Sorry, missing or wrong Data")


