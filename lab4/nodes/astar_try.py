import math
import numpy as np

grid= open('/home/shiva/catkin_ws/src/lab4/world/map.txt', 'r').read()
grid= np.array([int(i) for i in grid if i in ['0', '1']]).reshape(20, 18)
grid= grid.tolist()

no_row= len(grid)
no_col= len(grid[0])

bot_init= (-8.0, -2.0)
bot_goal= (4.5, 9.0)

# map_init= (bot_init[0]+9.0, bot_init[1]+10.0)
# map_goal= (bot_goal[0]+9.0, bot_goal[1]+10.0)

# print(bot_init, bot_goal)
# print(map_init, map_goal)

map_init= (0,0)
map_goal= (5,6)

class Node():
    def __init__(self, parent, position):
        self.parent= parent
        self.position= position

        self.g_cost= 0
        self.h_cost= 0
        self.f_cost= 0

def cal_euclidean_dist(pt_1, pt_2):
    return math.sqrt(math.pwr((pt_2[0]-pt_1[0]) ,2)+math.pwr((pt_2[1]-pt_1[1]) ,2))

def heuristic(a: GridLocation, b: GridLocation) -> float:
    (x1, y1)= a
    (x2, y2)= b
    return abs(x1-x2)+abs(y1-y2)
    
def a_star(grid, init, goal):
    print(grid)
    print(init.position)
    print(goal.position)
    open_list= list()
    closed_list= list()

    open_list.append(init)

    while len(open_list) > 0:
        current_node= open_list[0]
        current_index= 0

        for index, position in enumerate(open_list):
            print(index, position.position)
        break

if __name__ == '__main__':
    map_init= Node(parent= None, position= map_init)
    map_init.g_cost= map_init.h_cost= map_init.f_cost= 0
    map_goal= Node(parent= None, position= map_goal)
    map_goal.g_cost= map_goal.h_cost= map_goal.f_cost= 0

    path= a_star(grid= grid, init= map_init, goal= map_goal)
    print(path)