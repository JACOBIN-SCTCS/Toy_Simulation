
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

        invalid_point = False
        for k in range(4):
            for l in range(4):
                if(x+k <0 or x+k >= grid_size or y+l<0 or y+l>=grid_size):
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



