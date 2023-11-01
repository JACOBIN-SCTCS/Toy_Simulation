import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt


path_computation_time_file = "../build/obs0_time_file.txt"
total_time_file = "../build/time_algorithm_without.txt"

def get_average_percentage():
    s = 0.0
    f = open(path_computation_time_file,"r")
    time_list = []
    for line in f:
        stripped_line = line.strip()
        if (stripped_line == '-'):
            time_list.append(s)
            s = 0
        else:
            s = s + float(stripped_line)
            #current_time_series.append(float(stripped_line))
    time_list.append(s)
    f.close()

    f = open(total_time_file,"r")
    total_time_list = []
    for line in f:
        total_time_list.append(float(line))

    # Compute percentage

    percentage_list = []
    for i in range(len(total_time_list)):

        percentage = ((float(time_list[i+1]))/total_time_list[i])*100
        print("Percentage = " + str(percentage))
        percentage_list.append(percentage)
    
    percentage_list = np.array(percentage_list)
    print("Mean percentage = " + str(np.mean(percentage_list)))
   

def read_file(file_name):
    array = []
    f = open(file_name,"r")
    lines = f.readlines()
    for line in lines:
        array.append(int(line.strip()))
    f.close()
    return array

def plot_histogram():
    arr1 = np.array(read_file("../build/time_algorithm_with.txt"))
    arr2 = np.array(read_file("../build/time_algorithm_without.txt"))



    plt.hist(arr1,bins=50,label='With the idea')
    plt.hist(arr2,bins=50,label='Without the idea')
    plt.axvline(x = np.mean(arr1), color = 'royalblue')
    plt.axvline(x = np.mean(arr2), color = 'darkorange') 
    
    plt.legend(loc = 'upper right')
    plt.show()

    

#get_average_percentage()
plot_histogram()