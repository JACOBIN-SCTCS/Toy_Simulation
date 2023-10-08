
import random
import numpy as np

def generate_random_obstacles(n_obstacles=25,grid_size = 60 , number_of_runs=10,obstacle_size = 15):
    for i in range(number_of_runs):
        f = open("../build/obs_"+str(grid_size)+"_"+str(n_obstacles)+"_"+str(i)+".txt","w")
        j= 0
        while j<n_obstacles:
            x = random.randint(0,grid_size-1)
            y = random.randint(0,grid_size-1)

            invalid_point = False
            for k in range(obstacle_size):
                for l in range(obstacle_size):
                    if(x+k <=0 or x+k >= (grid_size-1) or y+l<=0 or y+l>=(grid_size-1)):
                        invalid_point = True
                        break
                if invalid_point:
                    break
            if invalid_point:
                continue
            
            else:
                content_to_write = str(x) + " " + str(y) +"\n"
                f.write(content_to_write)
                j+=1

        f.close()

def generate_grid_1(filename="../build/grid_1.txt",lower_bound = 5 , upper_bound = 55):
    
    f = open(filename,"w")
    for i in range(lower_bound,upper_bound,10):
        for j in range(lower_bound,upper_bound,10):
            content_to_write = str(i) + " " + str(j) +"\n"
            f.write(content_to_write)
    f.close()

def generate_multimodal_gaussian_obstacles(grid_size,n_clusters,num_objects_cluster,sampling_intervals,deviation,file_name):
    f = open(file_name,"w")
    for i in range(n_clusters):
        x_center = np.random.randint(sampling_intervals[0],sampling_intervals[1])
        y_center = np.random.randint(sampling_intervals[2],sampling_intervals[3])

        for j in range(num_objects_cluster):
            x = np.random.randint(x_center-deviation,x_center+deviation+1)
            y = np.random.randint(y_center-deviation,y_center+deviation+1)
            content_to_write = str(x) + " " + str(y) +"\n"
            f.write(content_to_write)
    


def helper_multimodal_gaussian():
    for i in range(16,21):
        file_name = '../build/multimodal_gaussian_15_'+str(i)+'.txt'
        generate_multimodal_gaussian_obstacles(256,12,6,[21,231,21,231],20,file_name)

def generate_maze_like_environment(grid_size,num_walls,size_of_each_wall,file_name):
    obstacle_size = 10
    f = open(file_name,"w")
    i = 0
    while (i< num_walls):
        x = np.random.randint(1,grid_size-(size_of_each_wall*obstacle_size)-1)
        y = np.random.randint(1,grid_size-(size_of_each_wall*obstacle_size)-1)
        wall_build = False
        horizontal_vertical = np.random.randint(0,2)
        if horizontal_vertical == 0:
            # horizontal wall
            current_x = x
            current_y = y
            for j in range(size_of_each_wall):
                invalid_point = False
                
                for k in range(obstacle_size):
                    for l in range(obstacle_size):
                        if(current_x+k <=0 or current_x+k >= (grid_size-1) or current_y+l<=0 or current_y+l>=(grid_size-1)):
                            invalid_point = True
                            break    
                    if invalid_point:
                        break
                
                if not invalid_point:
                    content_to_write = str(current_x) + " " + str(current_y) +"\n"
                    f.write(content_to_write)
                    current_y = current_y + 4
                    wall_build = True
                else:
                    break
        else:
            current_x = x
            current_y = y
            for j in range(size_of_each_wall):
                invalid_point = False
                for k in range(obstacle_size):
                    for l in range(obstacle_size):
                        if(current_x+k <=0 or current_x+k >= (grid_size-1) or current_y+l<=0 or current_y+l>=(grid_size-1)):
                            invalid_point = True
                            break    
                    if invalid_point:
                        break
                if not invalid_point:
                    content_to_write = str(current_x) + " " + str(current_y) +"\n"
                    f.write(content_to_write)
                    current_x = current_x + 4
                    wall_build = True
                else:
                    break
        
        if wall_build:
            i+=1
    f.close()

def helper_generate_maze():
    generate_maze_like_environment(256,20,10,'../build/maze_256_10_2.txt')

def generate_custom_clustered(file_name,grid_size = 256):
    f = open(file_name,"w")
    
    cluster_centers = [[64,64],[64,172],[172,64],[172,172]]
    for i in range(len(cluster_centers)):
        x_center = cluster_centers[i][0]
        y_center = cluster_centers[i][1]

        for j in range(10):
            x = np.random.randint(x_center-35,x_center+35+1)
            y = np.random.randint(y_center-35,y_center+35+1)
            content_to_write = str(x) + " " + str(y) +"\n"
            f.write(content_to_write)
    f.close()
    # for i in range(n_clusters):
    #     x_center = np.random.randint(sampling_intervals[0],sampling_intervals[1])
    #     y_center = np.random.randint(sampling_intervals[2],sampling_intervals[3])

    #     for j in range(num_objects_cluster):
    #         x = np.random.randint(x_center-deviation,x_center+deviation+1)
    #         y = np.random.randint(y_center-deviation,y_center+deviation+1)
    #         content_to_write = str(x) + " " + str(y) +"\n"
    #         f.write(content_to_write)



generate_random_obstacles(30,256,1,10)
# generate_custom_clustered('../build/custom_clustered_256_10_1.txt')
# generate_random_obstacles(60,256,10)
# helper_multimodal_gaussian()
# helper_generate_maze()
# generate_grid_1("../build/grid_256_1.txt",5,245)


