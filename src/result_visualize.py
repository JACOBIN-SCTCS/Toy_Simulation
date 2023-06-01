import matplotlib.pyplot as plt
import numpy as np

def individual_result():
    f = open("../build/result4.txt", "r")
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

    plt.plot(frontier_x, frontier_y, 'r')
    plt.plot(mean_plot[:,0],mean_plot[ : , 1],'b')
    plt.fill_between(mean_plot[:,0], mean_plot[:,1]-std[:,1], mean_plot[:,1]+std[:,1], color='b', alpha=.1)
    # plt.plot(topo_x, topo_y, 'b')
    plt.show()

def frontier_result():
    results = []
    f = open("../build/result_frontiers.txt", "r")
    x = f.readline()

    num_scans = 5
    scenario_count = 10


    for i in range(num_scans):
        current_sensor_range_result = []
        for j in range(scenario_count):
            current_map_result = []
            x = f.readline()
            while (x != str('-\n')):
                if x=='-' or x=='\n' or x == '':
                    break
                x_split = x.strip().split()
                current_map_result.append([float(x_split[0]), float(x_split[1])])
                x = f.readline()
            current_sensor_range_result.append(current_map_result)
        results.append(current_sensor_range_result)


    new_results = []
    for i in range(num_scans):
        max_scenario_length = 0
        for j in range(scenario_count):
            max_scenario_length = max(max_scenario_length,len(results[i][j]))
        current_sensor_result = np.zeros((scenario_count,max_scenario_length,2))

        for j in range(scenario_count):
            k = 0
            while k < len(results[i][j]):
                current_sensor_result[j][k][0] = results[i][j][k][0]
                current_sensor_result[j][k][1] = results[i][j][k][1]
                k+=1
            while k < max_scenario_length:
                current_sensor_result[j][k][0] = results[i][j][len(results[i][j])-1][0]
                current_sensor_result[j][k][1] = results[i][j][len(results[i][j])-1][1]
                k+=1
        new_results.append(current_sensor_result)

    mean_results = []
    std_results = []
    for i in range(num_scans):
        mean_results.append(np.mean(new_results[i],axis=0))
        std_results.append(np.std(new_results[i],axis=0))


    colors = ['b','g','r','c','m','y','k','w']
    for i in range(num_scans):
        plt.plot(mean_results[i][:,0],mean_results[i][:,1],colors[i],label=str(i+1)+" scans")
        plt.fill_between(mean_results[i][:,0],mean_results[i][:,1]-std_results[i][:,1],mean_results[i][:,1]+std_results[i][:,1],color=colors[i],alpha=0.2)
    plt.show()

def topology_result_visualize():
    results = []
    f = open("../build/result_topology.txt", "r")
    x = f.readline()

    num_scans = 5
    scenario_count = 10
    num_runs = 10


    for i in range(num_scans):
        current_sensor_range_result = []
        for j in range(scenario_count):
            current_map_result  = []
            for k in range(num_runs):
                current_map_count_result = []
                x = f.readline()
                while (x != str('-\n')):
                    if x=='-' or x=='\n' or x == '':
                        break
                    x_split = x.strip().split()
                    current_map_count_result.append([float(x_split[0]), float(x_split[1])])
                    x = f.readline()    
                current_map_result.append(current_map_count_result)
            refined_current_map_result = []
            episode_max_length = max(len(sublist) for sublist in current_map_result)
            current_map_result_np = np.zeros((num_runs,episode_max_length,2))
            for k in range(num_runs):
                for l in range(len(current_map_result[k])):
                    current_map_result_np[k][l][0] = current_map_result[k][l][0]
                    current_map_result_np[k][l][1] = current_map_result[k][l][1]
                for l in range(len(current_map_result[k]),episode_max_length):
                    current_map_result_np[k][l][0] = current_map_result[k][len(current_map_result[k])-1][0]
                    current_map_result_np[k][l][1] = current_map_result[k][len(current_map_result[k])-1][1]
            mean_result = np.mean(current_map_result_np,axis=0,keepdims=False)
            for k in range(episode_max_length):
                refined_current_map_result.append([mean_result[k][0],mean_result[k][1]])
            current_sensor_range_result.append(refined_current_map_result)
        results.append(current_sensor_range_result)

    new_results = []
    for i in range(num_scans):
        max_scenario_length = 0
        for j in range(scenario_count):
            max_scenario_length = max(max_scenario_length,len(results[i][j]))
        current_sensor_result = np.zeros((scenario_count,max_scenario_length,2))

        for j in range(scenario_count):
            k = 0
            while k < len(results[i][j]):
                current_sensor_result[j][k][0] = results[i][j][k][0]
                current_sensor_result[j][k][1] = results[i][j][k][1]
                k+=1
            while k < max_scenario_length:
                current_sensor_result[j][k][0] = results[i][j][len(results[i][j])-1][0]
                current_sensor_result[j][k][1] = results[i][j][len(results[i][j])-1][1]
                k+=1
        new_results.append(current_sensor_result)

    mean_results = []
    std_results = []
    for i in range(num_scans):
        mean_results.append(np.mean(new_results[i],axis=0))
        std_results.append(np.std(new_results[i],axis=0))


    colors = ['b','g','r','c','m','y','k','w']
    for i in range(num_scans):
        plt.plot(mean_results[i][:,0],mean_results[i][:,1],colors[i],label=str(i+1)+" scans")
        plt.fill_between(mean_results[i][:,0],mean_results[i][:,1]-std_results[i][:,1],mean_results[i][:,1]+std_results[i][:,1],color=colors[i],alpha=0.2)
    plt.show()

individual_result()