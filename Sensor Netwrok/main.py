import random
import math 
import heapq
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np

#Node Class
class Node():
    def __init__(self, x, y, is_data_node=False, data_packets=0, storage_capacity=0):
        self.x = x
        self.y = y
        self.is_data_node = is_data_node
        self.data_packets = data_packets
        self.storage_capacity = storage_capacity
    
    def setLocation(self, x, y):
        self.x = x
        self.y = y

    def getLocation(self):
        return [self.x, self.y]
    
    def get_x(self):
        return self.x

    def get_y(self):
        return self.y
    
    def is_DN(self):
        if self.is_data_node:
            return True
        else:
            return False
    
    def get_data_packets(self):
        return self.data_packets

    def get_storage_capacity(self):
        return self.storage_capacity

#Generates Nodes 
def generate_nodes(N, width, length, p, q, m):
    nodes = []
    dn = random.sample(range(N), p)
    for i in range(N):
        x = random.randint(0, width)
        y = random.randint(0, length)
        if i in dn:   #data node 
            node = Node(x, y, True, q, 0)
        else:       #storage node
            node = Node(x, y, False, 0, m)
        nodes.append(node)
    return nodes

#Calculates the Euclidean Distance
def distance(x1, y1, x2, y2):
    return math.sqrt(((x2 - x1 )**2) + ((y2-y1)**2))
5
#Calculates the weight of the edge
def to_transmission(distance):
    elec=100
    amp=0.1
    k=3200
    return (2*elec*k) + (amp*k*pow(distance,2))

#Creates Edges
def find_edges(nodes, Tr):
    edges = {}
    for i in range(len(nodes)):
        if i not in edges:
            edges[i]={}
        for j in range(i+1, len(nodes)):
            if j not in edges:
                edges[j] = {}
            d=distance(nodes[i].get_x(), nodes[i].get_y(), nodes[j].get_x(), nodes[j].get_y())
            if d <= Tr:
                tr_ = to_transmission(d)
                edges[i][j] = tr_
                edges[j][i] = tr_
    return edges

# Helper function: convert edges from dict to array
def get_edges_array(edges):
    edges_array = []
    for src in edges:
        for dst in edges[src]:
            edges_array.append((src, dst))      
    return edges_array

#Helper for main
def ask_question(str_question):
    while True:
        print(str_question)
        input_ = input()
        if is_number(input_) and int(input_)>=0:
            var = int(input_)
            print("")
            break
        else:
            print("")
            print("Input must be an integer. Please try again")
    return var

#Helper for main
def is_number(x):
    try:
        x = int(x)
        return True
    except:
        return False

#List of DN's and SN's
def list_DN_n_SN(nodes):
    dn=[]
    sn=[]
    for i in range(len(nodes)):
        node = nodes[i]
        if node.is_DN():
            dn.append(i)
        else:
            sn.append(i)
    print("Data Node IDs: ")
    print(dn)
    print("")
    print("Storage Node IDs: ")
    print(sn)
    return dn, sn
 
#Checks Feasibilty   
def is_feasible(p, q, N, m):
    if p*q <= (N-p)*m:
        return True
    else:
        return False 

#Checks Connectivity    
def is_connected(N, nodes, edges):
    adj_list = {i: [] for i in range(len(nodes))}

    for src in edges:
        for dst in edges[src]:
            adj_list[src].append(dst)
            adj_list[dst].append(src)

    visited = set()

    def dfs(node):
        visited.add(node)
        for neighbor in adj_list[node]:
            if neighbor not in visited:
                dfs(neighbor)

    dfs(0)   

    if len(visited) == N:
        return True
    else:
        return False

#Shortest Path using Dijkstra
def dijkstra(graph, start, end):
    heap = [(0, start, [])]
    visited = set()
    while heap:
        (cost, node, path) = heapq.heappop(heap)
        if node not in visited:
            visited.add(node)
            path = path + [node]
            if node == end:
                return (cost, path)
            for neighbor, edge_weight in graph[node].items():
                if neighbor not in visited:
                    heapq.heappush(heap, (cost + edge_weight, neighbor, path))
    return float("inf"), []

#Minimum Spanning Tree
def mst(graph):
    visited = set()
    edges = []
    start_node = next(iter(graph))
    visited.add(start_node)

    for neighbor, weight in graph[start_node].items():
        heapq.heappush(edges, (weight, start_node, neighbor))

    mst_edges = []
    while edges:
        weight, start_node, end_node = heapq.heappop(edges)
        if end_node in visited:
            continue
        visited.add(end_node)
        mst_edges.append((start_node, end_node))
        for neighbor, weight in graph[end_node].items():
            if neighbor not in visited:
                heapq.heappush(edges, (weight, end_node, neighbor))

    return mst_edges, len(mst_edges)

# plot graph
def visalize(nodes, edges_array, path, arrow):
    G = nx.DiGraph()
    G.add_edges_from(edges_array)
    # Specify the edges you want here
    red_edges = path
    black_edges = [edge for edge in G.edges() if edge not in red_edges]

    color=[]
    pos={}
    for i in range(len(nodes)):
        node = nodes[i]
        pos[i] = np.array(node.getLocation())

        if node.is_DN():
            color.append('#8EF1FF')
        else:
            color.append('#FF6666')
        
    nx.draw_networkx_nodes(G, pos, cmap=plt.get_cmap('jet'), 
                            node_size = 500, node_color = color)
    nx.draw_networkx_labels(G, pos)
    nx.draw_networkx_edges(G, pos, edgelist=red_edges, edge_color='r', arrows=arrow, width=5)
    nx.draw_networkx_edges(G, pos, edgelist=black_edges, arrows=False)
    plt.show()

#===========================================================================================================================================
#                                                   Main Function
def main():
    while True:
        print("")
        print("******************************************")
        print("Enter information about the Sensor Network")
        print("******************************************")
       
        width = ask_question("Enter the Width of the Sensor Network: ")    
        length = ask_question("Enter the Length of the Sensor Network: ")   
       
        N = ask_question("Enter the Number of Sensor Nodes: ")   
        Tr = ask_question("Enter the Transmission Range in Meters")  
        while True:
            p = ask_question("Enter the Number of DNs: ")
            if p > N:
                print("The number of DNs must be between 0 to {}".format(N))
            else:
                break
        q = ask_question("Enter the Number of Data Packets in each Data Node DN: ")
        m = ask_question("Enter the Storage Capacity of each Storage Node SN : ")
        print("==================== Generating Sensor Network ===================")
        nodes = generate_nodes(N, width, length, p, q, m)
        edges = find_edges(nodes, Tr)
        
        if is_connected(N, nodes, edges):
            print('The Sensor Network is connected')
            if is_feasible(p, q, N, m):
                print('The Sensor Network is feasible')
                break
            else:
                print('Not enough storage space. Network is not feasible. Please enter again.')
                print("")
        else:
            print('Sensor Network is not connected')
        
    print("")            
    dn, sn = list_DN_n_SN(nodes)  
    print("")
    print("Select your choice ")
    while True:
        algor = ask_question("1) To find the Minimum-Energy Data Offloading Path \n2) To find Minimum Spanning Tree ")
        if algor == 1: 
            while True:
                print("") 
                print("Please enter an ID of a Data Node (DN) as a starting point from the list")
                dn_id = ask_question("DN ID: ")
                if dn_id in dn:
                    break
                else:
                    print("ID must be for a Data Node from Data Nodes List. Please try again")
            while True:
                print("Please enter an ID of a Storage Node (SN) as a end point from the list")
                sn_id = ask_question("SN ID: ")
                if sn_id in sn:
                    break
                else:
                    print("ID must be for a Storage Node from Storage Node List. Please try again")
            print("You have selected ID:{} for DN and ID:{} for SN".format(dn_id, sn_id))
            print("=== Generating Result ===")
            cost, path = dijkstra(edges,dn_id, sn_id)
            print("The shortest offloading path is ", path)
            print("The energy cost of offloading one data packet from Data Node",dn_id, "to the Storage Node", sn_id, "is", cost, "picoJoules")
            print("The total energy cost of offloading all data packets is", cost*q, "picoJoules")
            # visalization 
            print("")
            print("=== Preparing Graph ===")
            edges_array=get_edges_array(edges)
            new_path=[]
            for i in range(len(path)-1):
                new_path.append((path[i], path[i+1]))
            visalize(nodes, edges_array, new_path, True)
            break
        elif algor == 2:
            print("=== Generating Result ===")
            mstPath, mstLength = mst(edges)
            print("The Minimum Spanning Tree path is ", mstPath)
            print("The total number of edges in Minimum Spanning Tree are", mstLength)
            # visalization
            print("")
            print("=== Preparing graph ===")
            edges_array=get_edges_array(edges)
            visalize(nodes, edges_array, mstPath, False)
            break
        else:
            print("Please enter 1 or 2 to select algorithm")


if __name__ == "__main__":
    main()
    






