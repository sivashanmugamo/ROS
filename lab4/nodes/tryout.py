import math
import time
import numpy as np
import matplotlib.pyplot as plt

world_grid= open('/home/shiva/catkin_ws/src/lab4/world/map.txt', 'r').read()
world_grid= np.array([int(i) for i in world_grid if i in ['0', '1']]).reshape(20, 18)
world_grid= list(world_grid)
# print(world_grid)
class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    # def __eq__(self, other):
    #     return self.position == other.position

def cal_euclidean_dist(pt1, pt2):
    return math.sqrt(math.pow(pt2[0]-pt1[0], 2) + math.pow(pt2[1]-pt1[1], 2))

def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    z= 0
    # Loop until you find the end
    while len(open_list) > 0:
        z+=1
        print('----------'+str(z)+'-----------')

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            print('Goal found')
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        # for new_position in [(0,1),(0,-1),(1,0),(-1,0)]:
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # print(cal_euclidean_dist(current_node.position, node_position))
            # if (cal_euclidean_dist(current_node.position, node_position)-1.4<=0.015 and cal_euclidean_dist(current_node.position, node_position)-1.4 >0):
            #     if (maze[node_position[0]][node_position[1]-1] == 1) and (maze[node_position[0]+1][node_position[1]] == 1):
            #         print('Diagonal node')
            #         continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)
            # print([i.position for i in children])

            # time.sleep(0.5)
        # print([(i.position, i.f) for i in children])

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)
        # print([(k.position, k.f) for k in open_list])

        # if z==6:
        #     break


def main():

    # maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    maze= world_grid

    start = (11, 1)
    end = (1, 13)

    path = astar(maze, start, end)
    print(path)

    for i in path:
        maze[i[0]][i[1]]= 2
    
    plt.imshow(maze)
    plt.show()

if __name__ == '__main__':
    main()