import matplotlib.pyplot as plt

def visualize_time(time_list, title):
    # Plot all the the time_series in time_list
    for time_series in time_list:
        print(len(time_series))
        index = [i+1 for i in range(len(time_series))]
        plt.plot(index, time_series)
    
    plt.title(title)
    plt.ylabel('time (ms)')
    plt.xlabel('epoch')
    plt.show()

# Path to data file is in build folder
def visualize_time_from_file(file_name, title):
    time_list = []
    f = open(file_name,"r")
    current_time_series = []
    for line in f:
        stripped_line = line.strip()
        if (stripped_line == '-'):
            time_list.append(current_time_series)
            current_time_series = []
        else:
            current_time_series.append(float(stripped_line))
    time_list.append(current_time_series)
    visualize_time(time_list[1:], title)
        
visualize_time_from_file("../build/obs0_time_file.txt","Non-Homologous Path Computation Time per epoch")