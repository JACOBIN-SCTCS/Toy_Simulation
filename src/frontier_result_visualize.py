import numpy as np
import matplotlib.pyplot as plt

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

