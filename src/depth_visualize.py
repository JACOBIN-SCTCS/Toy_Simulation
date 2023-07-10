import numpy as np
import matplotlib.pyplot as plt
import os
from PIL import Image

depths = [2,4,8,16,32,64]

def get_particular_images(filename = "../build/dobs1_topo.txt"):
    file = open(filename,"r")
    lines = file.readlines()
    for i in range(1,len(lines)):
        stripped_line = lines[i].strip().split(',')[:-1]
        if(len(stripped_line)==0):
            continue
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
    file.close()

def get_image(grid_size,directory_name, filename , depths ,timestep = 100):
    
    if(not os.path.exists(directory_name)):
        os.mkdir(directory_name)

    file = open(filename,"r")
    lines = file.readlines()
    
    current_depth_index = 0
    current_timestep = 0
    for i in range(1,len(lines)):
        stripped_line = lines[i].strip().split(',')[:-1]
        if(len(stripped_line)==0):

            current_timestep =  current_timestep + timestep
            continue
        
        stripped_line_int = [float(val) for val in stripped_line]
        processed_line = []
        
        stripped_line_np = np.zeros((grid_size,grid_size,4))
        
        index = 0
        for j in range(grid_size):
            for k in range(grid_size):
                if(stripped_line_int[index]==-1.0):
                    stripped_line_np[j,k,0] = 0.5
                    stripped_line_np[j,k,1] = 0.5
                    stripped_line_np[j,k,2] = 0.5
                    stripped_line_np[j,k,3] = 1.0

                elif(stripped_line_int[index]==0.0):
                    stripped_line_np[j,k,0] = 0.0
                    stripped_line_np[j,k,1] = 0.0
                    stripped_line_np[j,k,2] = 0.0
                    stripped_line_np[j,k,3] = 1.0

                else:
                    stripped_line_np[j,k,0] = 1.0
                    stripped_line_np[j,k,1] = 1.0
                    stripped_line_np[j,k,2] = 1.0
                    stripped_line_np[j,k,3] = 1.0
                
                index +=1
        # for j in range(len(stripped_line_int)):
        #     if(stripped_line_int[j]==-1):
        #         processed_line.append(0.5)
        #     elif(stripped_line_int[j]==0):
        #         processed_line.append(0.0)
        #     else:
        #         processed_line.append(1.0)
                
        # stripped_line_np = np.array(processed_line)
        # stripped_line_np = stripped_line_np.reshape(grid_size,-1)
        # stripped_line_np_image = Image.fromarray(stripped_line_np,mode='L')
        # stripped_line_np_image.save(directory_name+"/"+str(current_timestep)+"_"+str(depths[current_depth_index])+".png")
        # current_depth_index = (current_depth_index + 1)%len(depths)
        #plt.figure(figsize=(10,10))
        plt.imshow(stripped_line_np,cmap='magma')
        plt.savefig(directory_name+"/"+str(current_timestep)+"_"+str(depths[current_depth_index])+".png")
        current_depth_index = (current_depth_index + 1)%len(depths)
        # plt.show()
    file.close()

def plot_depth_map(grid_size,directory_name, filename , depths ,timestep = 100):
    if(not os.path.exists(directory_name)):
        os.mkdir(directory_name)

    file = open(filename,"r")
    lines = file.readlines()
    
    current_depth_index = 0
    current_timestep = 0
    for i in range(1,len(lines)):
        stripped_line = lines[i].strip().split(',')[:-1]
        if(len(stripped_line)==0):

            current_timestep =  current_timestep + timestep
            continue
        
        stripped_line_int = [float(val) for val in stripped_line]
        processed_line = []
        
        stripped_line_np = np.zeros((grid_size,grid_size,4))
        
        index = 0
        for j in range(grid_size):
            for k in range(grid_size):
                if(stripped_line_int[index]==-1.0):
                    stripped_line_np[j,k,0] = 1.0
                    stripped_line_np[j,k,1] = 1.0
                    stripped_line_np[j,k,2] = 0.65
                    stripped_line_np[j,k,3] = 1.0
                else:
                   
                    probability = stripped_line_int[index]
                    occ_value = 1.0-probability
                    # the above gives the probability of an obstacle being present
                    # we want to convert this to an occupancy value


                    # occ_value =  (1.0 - probability)
                    # if occ_value <= 0.25:
                    #     occ_value = 0.25
                    # elif occ_value >= 0.25 and occ_value <= 0.5:
                    #     occ_value = 0.5
                    # elif occ_value >= 0.5 and occ_value <= 0.75:
                    #     occ_value = 0.75
                    # else:
                    #     occ_value = 1.0
                    
                    stripped_line_np[j,k,0] = occ_value
                    stripped_line_np[j,k,1] = occ_value
                    stripped_line_np[j,k,2] = occ_value
                    stripped_line_np[j,k,3] = 1.0

                # elif(stripped_line_int[index]==0):
                #     stripped_line_np[j,k,0] = 0.0
                #     stripped_line_np[j,k,1] = 0.0
                #     stripped_line_np[j,k,2] = 0.0
                #     stripped_line_np[j,k,3] = 1.0

                # else:
                #     stripped_line_np[j,k,0] = 1.0
                #     stripped_line_np[j,k,1] = 1.0
                #     stripped_line_np[j,k,2] = 1.0
                #     stripped_line_np[j,k,3] = 1.0
                
                index +=1
        # for j in range(len(stripped_line_int)):
        #     if(stripped_line_int[j]==-1):
        #         processed_line.append(0.5)
        #     elif(stripped_line_int[j]==0):
        #         processed_line.append(0.0)
        #     else:
        #         processed_line.append(1.0)
                
        # stripped_line_np = np.array(processed_line)
        # stripped_line_np = stripped_line_np.reshape(grid_size,-1)
        # stripped_line_np_image = Image.fromarray(stripped_line_np,mode='L')
        # stripped_line_np_image.save(directory_name+"/"+str(current_timestep)+"_"+str(depths[current_depth_index])+".png")
        # current_depth_index = (current_depth_index + 1)%len(depths)
        #plt.figure(figsize=(10,10))
        plt.xticks([])
        plt.yticks([])
        plt.imshow(stripped_line_np,cmap='magma')
        plt.savefig(directory_name+"/"+str(current_timestep)+"_"+str(depths[current_depth_index])+".png")
        current_depth_index = (current_depth_index + 1)%len(depths)
        # plt.show()
    file.close()

def plot_original():
    pass

print("rw")
plot_depth_map(256,'../rw_3',"../build/visual_result_obs_256_15_3.txt_rw.txt",depths,300)
print("topo")
plot_depth_map(256,'../topological_3',"../build/visual_result_obs_256_15_3.txt_topo.txt",depths,300)
print("frontier")
plot_depth_map(256,'../frontier_3',"../build/visual_result_obs_256_15_3.txt_frontier.txt",depths,300)

# get_image(256,'../topological',"../build/dobs1_topo.txt",depths,300)
    
