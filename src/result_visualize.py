import matplotlib.pyplot as plt
import numpy as np

f = open("../build/result0.txt", "r")
x =f.readline()
print(x)

frontier_x = []
frontier_y = []

x = f.readline()
while (x != str('-\n')):
    if x=='-':
        break
    x_split = x.strip().split()
    print(x_split)
    frontier_x.append(float(x_split[0]))
    frontier_y.append(float(x_split[1]))
    x = f.readline()


topo_plot = []

trajectories_size = 9
max_len = 0
for i in range(trajectories_size):
    new_top_list = []
    while (x != str('-\n') or x!=''):
        x = f.readline()

        if (x == '-\n' or x == ''):
            break
        x_split = x.strip().split()
        new_top_list.append([float(x_split[0]), float(x_split[1])])
    max_len = max(max_len, len(new_top_list))
    topo_plot.append(new_top_list)

map_np = np.zeros((trajectories_size,max_len,2))
for i in range(trajectories_size):
    j  = 0
    while j  < len(topo_plot[i]):
        map_np[i][j][0] = topo_plot[i][j][0]
        map_np[i][j][1] = topo_plot[i][j][1]
        j+=1
    while(j<max_len):
        map_np[i][j][0] = topo_plot[i][len(topo_plot[i])-1][0]
        map_np[i][j][1] = topo_plot[i][len(topo_plot[i])-1][1]
        j+=1

mean_plot = np.mean(map_np, axis=0)
std = np.std(map_np, axis=0)
print(std.shape)


# while (x != '-\n'):
#     if x == '\n':
#         break
#     x_split = x.strip().split()
#     topo_x.append(float(x_split[0]))
#     topo_y.append(float(x_split[1]))
#     x = f.readline()

plt.plot(frontier_x, frontier_y, 'r')
plt.plot(mean_plot[:,0],mean_plot[ : , 1],'b')
plt.fill_between(mean_plot[:,0], mean_plot[:,1]-std[:,1], mean_plot[:,1]+std[:,1], color='b', alpha=.1)
# plt.plot(topo_x, topo_y, 'b')
plt.show()
