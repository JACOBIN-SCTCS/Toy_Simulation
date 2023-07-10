import matplotlib.pyplot as plt
import numpy as np

def individual_result():
    f = open("../build/result_obs_256_15.txt", "r")
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

def individual_result_error(index_to_plot = 1,save_plot = False):
    plot_unknown_cells_remaining = False
    total_unknown_cells = 64542
    f = open("../build/result_obs_256_15_3.txt", "r")
    trajectories_size = 6
    dict_mapping = {1:'Percentage of area mapped',2:'Error vs steps(Depth = 1)', 3:'Error vs steps(Depth = 2)',
                    4:'Error vs steps(Depth = 3)',5:'Error vs steps(Depth = 4)',6:'Error vs steps(Depth = 5)',7:'Error vs steps(Depth = 6)',
                    8:'Number of unknown cells vs steps (Depth = 1)',9:'Number of unknown cells vs steps (Depth = 2)',10:'Number of unknown cells vs steps (Depth = 3)',11:'Number of unknown cells vs steps (Depth = 4)',
                    12:'Number of unknown cells vs steps (Depth = 5)',13:'Number of unknown cells vs steps (Depth = 6)'}
    yabels = ['%','Error','Error','Error','Error','Error','Error',
              'Unknown cell count','Unknown cell count','Unknown cell count',
              'Unknown cell count','Unknown cell count','Unknown cell count']
    xlabel = 'Robot steps'

    plt.title(dict_mapping[index_to_plot])
    plt.ylabel(yabels[index_to_plot-1])
    plt.xlabel(xlabel)
    x =f.readline()

    frontier_result = []
    x = f.readline()
    while (x != str('-\n')):
        if x=='-':
            break
        x_split = x.strip().split()
        # print(x_split)
        x_split_float = [ float(ele) for ele in x_split]
        frontier_result.append(x_split_float)
        x = f.readline()
    frontier_result_np = np.array(frontier_result)

    topo_plot = []
 
    max_len = 0
    # print(frontier_result_np.shape)
    for i in range(trajectories_size):
        new_top_list = []
        
        while (x != str('-\n') or x!=''):
            x = f.readline()

            if (x == '-\n' or x == ''):
                break
            x_split = x.strip().split()
            x_split_float = [float(ele) for ele in x_split]
            new_top_list.append(x_split_float)
       
        # print(len(new_top_list))
        # print(len(new_top_list[0]))
        max_len = max(max_len, len(new_top_list))
        topo_plot.append(new_top_list)
    # topo_plot_np = np.array(topo_plot)
    # print(topo_plot_np.shape)
    # print(topo_plot_np[0])
    # print(topo_plot_np.shape)

    map_np = np.zeros((trajectories_size,max_len,frontier_result_np.shape[1]))
    for i in range(trajectories_size):
        j  = 0
        while j  < len(topo_plot[i]):
            for k in range(frontier_result_np.shape[1]):
                map_np[i][j][k] = topo_plot[i][j][k]
            j+=1
        while(j<max_len):
            for k in range(frontier_result_np.shape[1]):
                map_np[i][j][k] = topo_plot[i][len(topo_plot[i])-1][k]
            # map_np[i][j][0] = topo_plot[i][len(topo_plot[i])-1][0]
            # map_np[i][j][1] = topo_plot[i][len(topo_plot[i])-1][1]
            j+=1



    mean_plot = np.mean(map_np, axis=0)
    # print(mean_plot.shape)
    std = np.std(map_np, axis=0)
    # print(std.shape)
    if(plot_unknown_cells_remaining and index_to_plot==1):
        modified_y_value_frontier = np.zeros(frontier_result_np.shape[0])
        for i in range(frontier_result_np.shape[0]):
            modified_y_value_frontier[i] = ((100-frontier_result_np[i,1])/100.0)*total_unknown_cells
        plt.plot(frontier_result_np[:,0], modified_y_value_frontier, 'r')
      
        new_topo = np.zeros((map_np.shape[0],map_np.shape[1],2))
        for i in range(map_np.shape[0]):
            for j in range(map_np.shape[1]):
                new_topo[i][j][0] = map_np[i][j][0]
                new_topo[i][j][1] = ((100-map_np[i][j][1])/100.0)*total_unknown_cells
        new_topo_mean = np.mean(new_topo,axis=0)
        new_topo_std = np.std(new_topo,axis=0)
        plt.plot(new_topo_mean[:,0],new_topo_mean[:,1],'b')
        plt.fill_between(new_topo_mean[:,0], new_topo_mean[:,1]-new_topo_std[:,1], new_topo_mean[:,1]+new_topo_std[:,1], color='b', alpha=.1)
        
    else:
        plt.plot(frontier_result_np[:,0], frontier_result_np[:,index_to_plot], 'r')
        plt.plot(mean_plot[:,0],mean_plot[ : , index_to_plot],'b')
        plt.fill_between(mean_plot[:,0], mean_plot[:,index_to_plot]-std[:,index_to_plot], mean_plot[:,index_to_plot]+std[:,index_to_plot], color='b', alpha=.1)
        # plt.plot(topo_x, topo_y, 'b')
    if save_plot:
        plt.savefig('../plots/'+dict_mapping[index_to_plot]+'_'+yabels[index_to_plot-1]+'.png')
    else:
        plt.show()
    plt.cla()

def individual_result_error_include_random_walk(index_to_plot = 1,save_plot = False):
    plot_unknown_cells_remaining = False
    total_unknown_cells = 64542
    # obs_256_15_3.txt was used previously.
    f = open("../build/m_result_multimodal_gaussian_15_16.txt", "r")
    trajectories_size = 5
    dict_mapping = {1:'Percentage of area mapped',2:'Error vs steps(Depth = 1)', 3:'Error vs steps(Depth = 2)',
                    4:'Error vs steps(Depth = 3)',5:'Error vs steps(Depth = 4)',6:'Error vs steps(Depth = 5)',7:'Error vs steps(Depth = 6)',
                    8:'Number of unknown cells vs steps (Depth = 1)',9:'Number of unknown cells vs steps (Depth = 2)',10:'Number of unknown cells vs steps (Depth = 3)',11:'Number of unknown cells vs steps (Depth = 4)',
                    12:'Number of unknown cells vs steps (Depth = 5)',13:'Number of unknown cells vs steps (Depth = 6)'}
    yabels = ['%','Error','Error','Error','Error','Error','Error',
              'Unknown cell count','Unknown cell count','Unknown cell count',
              'Unknown cell count','Unknown cell count','Unknown cell count']
    xlabel = 'Robot steps'

    plt.title(dict_mapping[index_to_plot])
    plt.ylabel(yabels[index_to_plot-1])
    plt.xlabel(xlabel)
    x =f.readline()

    frontier_result = []
    x = f.readline()
    while (x != str('-\n')):
        if x=='-':
            break
        x_split = x.strip().split()
        # print(x_split)
        x_split_float = [ float(ele) for ele in x_split]
        frontier_result.append(x_split_float)
        x = f.readline()
    frontier_result_np = np.array(frontier_result)

    topo_plot = []
    max_len = 0
    # print(frontier_result_np.shape)
    for i in range(trajectories_size):
        new_top_list = [] 
        while (x != str('-\n') or x!=''):
            x = f.readline()

            if (x == '-\n' or x == ''):
                break
            x_split = x.strip().split()
            x_split_float = [float(ele) for ele in x_split]
            new_top_list.append(x_split_float)
        max_len = max(max_len, len(new_top_list))
        topo_plot.append(new_top_list)


    map_np_topo = np.zeros((trajectories_size,max_len,frontier_result_np.shape[1]))
    for i in range(trajectories_size):
        j  = 0
        while j  < len(topo_plot[i]):
            for k in range(frontier_result_np.shape[1]):
                map_np_topo[i][j][k] = topo_plot[i][j][k]
            j+=1
        while(j<max_len):
            for k in range(frontier_result_np.shape[1]):
                map_np_topo[i][j][k] = topo_plot[i][len(topo_plot[i])-1][k]
            # map_np[i][j][0] = topo_plot[i][len(topo_plot[i])-1][0]
            # map_np[i][j][1] = topo_plot[i][len(topo_plot[i])-1][1]
            j+=1
    mean_plot_topo = np.mean(map_np_topo, axis=0)
    # print(mean_plot.shape)
    std_topo = np.std(map_np_topo, axis=0)
    # print(std.shape)
    
    
    rw_plot = []
    max_len = 0
    for i in range(trajectories_size):
        new_rw_list = [] 
        while (x != str('-\n') or x!=''):
            x = f.readline()

            if (x == '-\n' or x == ''):
                break
            x_split = x.strip().split()
            x_split_float = [float(ele) for ele in x_split]
            new_rw_list.append(x_split_float)
        max_len = max(max_len, len(new_rw_list))
        rw_plot.append(new_rw_list)

    map_np_rw = np.zeros((trajectories_size,max_len,frontier_result_np.shape[1]))
    for i in range(trajectories_size):
        j  = 0
        while j  < len(rw_plot[i]):
            for k in range(frontier_result_np.shape[1]):
                map_np_rw[i][j][k] = rw_plot[i][j][k]
            j+=1
        while(j<max_len):
            for k in range(frontier_result_np.shape[1]):
                map_np_rw[i][j][k] = rw_plot[i][len(rw_plot[i])-1][k]
            j+=1
    mean_plot_rw = np.mean(map_np_rw, axis=0)
    # print(mean_plot.shape)
    std_rw = np.std(map_np_rw, axis=0)  
    
    if(plot_unknown_cells_remaining and index_to_plot==1):
        modified_y_value_frontier = np.zeros(frontier_result_np.shape[0])
        for i in range(frontier_result_np.shape[0]):
            modified_y_value_frontier[i] = ((100-frontier_result_np[i,1])/100.0)*total_unknown_cells
        plt.plot(frontier_result_np[:,0], modified_y_value_frontier, 'r')
      
        new_topo = np.zeros((map_np_topo.shape[0],map_np_topo.shape[1],2))
        for i in range(map_np_topo.shape[0]):
            for j in range(map_np_topo.shape[1]):
                new_topo[i][j][0] = map_np_topo[i][j][0]
                new_topo[i][j][1] = ((100-map_np_topo[i][j][1])/100.0)*total_unknown_cells
        new_topo_mean = np.mean(new_topo,axis=0)
        new_topo_std = np.std(new_topo,axis=0)
        plt.plot(new_topo_mean[:,0],new_topo_mean[:,1],'b')
        plt.fill_between(new_topo_mean[:,0], new_topo_mean[:,1]-new_topo_std[:,1], new_topo_mean[:,1]+new_topo_std[:,1], color='b', alpha=.1)
        
    else:
        plt.plot(frontier_result_np[:,0], frontier_result_np[:,index_to_plot], 'r')
        plt.plot(mean_plot_topo[:,0],mean_plot_topo[ : , index_to_plot],'b')
        plt.fill_between(mean_plot_topo[:,0], mean_plot_topo[:,index_to_plot]-std_topo[:,index_to_plot], mean_plot_topo[:,index_to_plot]+std_topo[:,index_to_plot], color='b', alpha=.1)
        plt.plot(mean_plot_rw[:,0],mean_plot_rw[ : , index_to_plot],'g')
        plt.fill_between(mean_plot_rw[:,0], mean_plot_rw[:,index_to_plot]-std_rw[:,index_to_plot], mean_plot_rw[:,index_to_plot]+std_rw[:,index_to_plot], color='g', alpha=.1) 
        # plt.plot(topo_x, topo_y, 'b')
    if save_plot:
        plt.savefig('../plots/'+dict_mapping[index_to_plot]+'_'+yabels[index_to_plot-1]+'.png')
    else:
        plt.show()
    plt.cla()

def frontier_result(return_results = False):
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

    if(return_results):
        return mean_results,std_results
    else:
        colors = ['b','g','r','c','m','y','k','w']
        for i in range(num_scans):
            plt.plot(mean_results[i][:,0],mean_results[i][:,1],colors[i],label=str(i+1)+" scans")
            plt.fill_between(mean_results[i][:,0],mean_results[i][:,1]-std_results[i][:,1],mean_results[i][:,1]+std_results[i][:,1],color=colors[i],alpha=0.2)
        plt.show()

def topology_result_visualize(return_results = False):
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

    if(return_results):
        return mean_results,std_results
    else:
        colors = ['b','g','r','c','m','y','k','w']
        for i in range(num_scans):
            plt.plot(mean_results[i][:,0],mean_results[i][:,1],colors[i],label=str(i+1)+" scans")
            plt.fill_between(mean_results[i][:,0],mean_results[i][:,1]-std_results[i][:,1],mean_results[i][:,1]+std_results[i][:,1],color=colors[i],alpha=0.2)
        plt.show()

def show_frontier_topology():
    frontier_mean, frontier_std = frontier_result(return_results=True)
    topology_mean, topology_std = topology_result_visualize(return_results=True)
    colors = ['b','g','r','c','m','y','k','w']
    for i in range(len(frontier_mean)-1,len(frontier_mean)):
        plt.plot(frontier_mean[i][:,0],frontier_mean[i][:,1],colors[i],label=str(i+1)+" scans")
        plt.fill_between(frontier_mean[i][:,0],frontier_mean[i][:,1]-frontier_std[i][:,1],frontier_mean[i][:,1]+frontier_std[i][:,1],color=colors[i],alpha=0.1)
    for i in range(len(frontier_mean)-1,len(topology_mean)):
        plt.plot(topology_mean[i][:,0],topology_mean[i][:,1],colors[i]+'--',label=str(i+1)+" scans")
        plt.fill_between(topology_mean[i][:,0],topology_mean[i][:,1]-topology_std[i][:,1],topology_mean[i][:,1]+topology_std[i][:,1],color=colors[i],alpha=0.4)
    plt.show()

# individual_result_error()
for i in range(1,14):
    individual_result_error_include_random_walk(i,save_plot=False)
