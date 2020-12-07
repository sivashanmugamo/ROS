# import numpy as np

# world_grid= open('/home/shiva/catkin_ws/src/lab4/world/map.txt', 'r').read()
# world_grid= np.array([int(i) for i in world_grid if i in ['0', '1']]).reshape(20, 18)

# temp_1= list()
# for i in range(-9, 10):
#     if i != 0:
#         temp_2= list()
#         for j in reversed(range(-10, 11)):
#             if j != 0:
#                 temp_2.append((i, j))
#         temp_1.append(temp_2)

# temp= np.array(temp_1).reshape((20, 18, 2))

# for i in range(0, 20):
#     tst= dict()
#     for j in range(0, 18):
#         tst[tuple(temp[i][j])]= world_grid[i][j]
#     print(tst)
