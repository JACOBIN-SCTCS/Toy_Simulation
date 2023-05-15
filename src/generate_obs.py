
import random

n_obstacles = 25
grid_size = 60
number_of_runs = 10

for i in range(number_of_runs):
    f = open("../build/obs"+str(i)+".txt","w")
    
    j= 0
    while j<n_obstacles:
        x = random.randint(0,grid_size-1)
        y = random.randint(0,grid_size-1)

        obs_collide = False
        for k in range(4):
            for l in range(4):
                if(x+k == 10 and y+l==10):
                    obs_collide = True
                    break
            if obs_collide:
                break
        if obs_collide:
            continue
        else:
            content_to_write = str(x) + "," + str(y) +"\n"
            f.write(content_to_write)
            j+=1

    f.close()



