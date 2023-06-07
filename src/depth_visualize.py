import numpy as np
import matplotlib.pyplot as plt


filename = "../build/dobs1.txt"
file = open(filename,"r")

lines = file.readlines()
for i in range(1,len(lines)):
    stripped_line = lines[i].strip().split(',')[:-1]
    stripped_line_int = [int(val) for val in stripped_line]
    processed_line = []
    for j in range(len(stripped_line_int)):
        if(stripped_line_int[j]==-1):
            processed_line.append(0.5)
        elif(stripped_line_int[j]==0):
            processed_line.append(0.0)
        else:
            processed_line.append(1.0)
            
    stripped_line_np = np.array(processed_line)
    stripped_line_np = stripped_line_np.reshape(64,64)
    plt.imshow(stripped_line_np,cmap='bone')
    plt.show()
    
