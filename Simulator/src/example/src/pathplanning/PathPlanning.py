import networkx as nx
from scipy.spatial import distance
from pathlib import Path

graphmlPath = Path("").absolute() 

class PathPlanning:

    """
    This class calculates the path that our vehicle is going to follow, given a start and a finish node.
    The path planning is implemented using A* algorithm, utilizing the BFMC map graph.
    """

    # Competition
    map_path = str(graphmlPath) + "/Competition_track.graphml"

    intersection_nodes = ['346', '270', '32', '36', '34', '6', '25', '27', '23',
                        '370', '81', '2', '15', '16', '77', '72', '70', '79',
                        '311', '314', '61', '467', '4', '59', '54', '52', '63',
                        '68', '423', '303', '305', '45', '43', '41', '230']
    central_nodes = ['347', '271', '37', '39', '38', '11', '29', '30', '28',
                    '371', '84', '9', '20', '20', '82', '75', '74', '83',
                    '312', '315', '65', '468', '10', '64', '57', '56', '66',
                    '73', '424', '304', '306', '48', '47', '46', '269']

    # LAB
    # map_path = str(graphmlPath) + "/Lab_track.graphml"

    # intersection_nodes = ['1', '4', '38', '39', '46', '52', '53', '24', '19']
    # central_nodes = ['2', '23']

    turning_left = False
    turn_angle = 0
    G = nx.read_graphml(map_path)

    @staticmethod
    def shortest_path(source, target):
        return list(nx.all_shortest_paths(PathPlanning.G, source=source, target=target))[0]

    @staticmethod
    def find_closest_node(x, y):
        distances_dict = {"Nodes": [], "Distances":[]}
        for node in PathPlanning.G.nodes():
            node_x = PathPlanning.G.nodes[node]['x']
            node_y = PathPlanning.G.nodes[node]['y']

            distance_from_node = distance.euclidean((x,y), (node_x,node_y))

            distances_dict["Nodes"].append(node)
            distances_dict["Distances"].append(distance_from_node)

        
        min_distance = min(distances_dict["Distances"])
        min_index  = distances_dict["Distances"].index(min_distance)
        min_node = distances_dict["Nodes"][min_index]
        
        return min_node

    @staticmethod
    def remove_central_nodes(path):
        new_path = path[:]
        for node in PathPlanning.central_nodes:
            if node in new_path:
                new_path.remove(node)
        return new_path

    @staticmethod
    def find_inter_nodes():
        inter_nodes = []
        for node in PathPlanning.G.nodes:
            neighbors_list = list(PathPlanning.G.neighbors(node))
            if len(neighbors_list) > 0 and len(list(PathPlanning.G.neighbors(neighbors_list[0]))) > 1:
                inter_nodes.append(node)
        
    @staticmethod
    def find_central_nodes():
        inter_nodes = []
        for node in PathPlanning.G.nodes:
            neighbors_list = list(PathPlanning.G.neighbors(node))
            if len(neighbors_list) > 0 and len(list(PathPlanning.G.neighbors(neighbors_list[0]))) > 1:
                inter_nodes.append(neighbors_list[0])

    @staticmethod
    def check_intersection(path):

        for i in range(min(len(path), 3)):
            if path[i] in PathPlanning.intersection_nodes:
                return True

        return False

    @staticmethod
    def find_target(path):
        for i in range(min(3, len(path))):
            #print("Node in find target: " + str(path[i]))
            if path[i] in PathPlanning.intersection_nodes:
                
                if i+1 >= len(path):
                    return None, True
                    break

                target_node = path[i+1] 

                return target_node, False
        #if path[0] or path[1] in PathPlanning.intersection_nodes:
        #    return True, path[1]
        return None, False

    @staticmethod
    def update_path(path, x, y, finish):
        closest_node = PathPlanning.find_closest_node(x,y)
        #print("CLOSEST NODE: "*50 + str(closest_node))
        path_updated = False
        if closest_node == finish:
            return path, True
        if(closest_node in path):
            path = path[path.index(closest_node):]
            print("PATH"*50, path)
            path_updated = True
        
        if path_updated == False:
            closest_node = PathPlanning.find_closest_node_from_path(x, y, path)
            #print("Closest node in path: ", str(closest_node))
            if abs(PathPlanning.G.nodes[closest_node]['x'] - x) < 0.3 and  abs(PathPlanning.G.nodes[closest_node]['y'] - y) < 0.3:
                path = path[path.index(closest_node):]

        finished = False

        if len(path) < 2:
            finished = True

        return path, finished

    @staticmethod
    def distance_from_node(node_x, node_y, x, y):

        distance_node = distance.euclidean((x,y), (node_x,node_y))

        return distance_node

    @staticmethod
    def in_range(node_x,node_y, x, y):
        threshold = 0.1

        if( node_x - threshold <= x < node_x + threshold and node_y - threshold <= y <= node_y + threshold):
            return True
        else:
            return False

    @staticmethod
    def in_vertical_range(node_y, y):
        threshold = 0.2
        if(node_y - threshold <= y <= node_y + threshold):
            return True
        else:
            False

    @staticmethod
    def find_closest_node_from_path(x, y, path):
        distances_dict = {"Nodes": [], "Distances":[]}
        for node in path:
            node_x = PathPlanning.G.nodes[node]['x']
            node_y = PathPlanning.G.nodes[node]['y']

            distance_from_node = distance.euclidean((x,y), (node_x,node_y))
            
            distances_dict["Nodes"].append(node)
            distances_dict["Distances"].append(distance_from_node)

        
        min_distance = min(distances_dict["Distances"])
        min_index  = distances_dict["Distances"].index(min_distance)
        min_node = distances_dict["Nodes"][min_index]

        return min_node