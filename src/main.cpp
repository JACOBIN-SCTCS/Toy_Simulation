#include <iostream>
#include <time.h>
#include "framework.h"

class Robot
{	
	public:
		Robot(int g_size , int w_size, double scale) : grid_size(g_size) , fw(w_size,w_size,scale)
		{
			
			grid.resize(grid_size,std::vector<int>(grid_size,-1));
			std::cout<<grid_size<<std::endl;
		}

		void setInitialRobotPose(int x , int y)
		{
			for(int i = x-lidar_range ; i < x+lidar_range ; i++)
			{
				for(int j = y-lidar_range ; j < y+lidar_range ; j++)
				{
					//grid[i][j] = 2;
					int distance = (i-x)*(i-x) + (j-y)*(j-y);
					if(distance<= (lidar_range*lidar_range))
					{
						grid[i][j] = 1;
					}
				}
			}
			grid[x][y] = 2;
			

			// for(int i=x ; i<x+20;++i)
			// {
			// 	for(int j=y ; j<y+20;++j)
			// 	{
			// 		grid[i][j] = 0;
			// 	}
			// }

			fw.render_screen(grid);
			//fw.draw_point(x,y);
		}
		Framework fw;

	private:
		int grid_size;
		std::vector<std::vector<int>> grid;
		int lidar_range = 4;
		double scale  = 10.0;
};

int main(int argc, char *argv[])
{
	srand (time(NULL));
	
	Robot robot(60,600,10.0);
	robot.setInitialRobotPose(5,5);

	//robot.setInitialRobotPose(5,5);

	// robot.fw.draw_point(300,300);
	// robot.fw.draw_point(400,400);
	//robot.setInitialRobotPose(300,300);
	SDL_Event event;
	while (!(event.type == SDL_QUIT))
	{
		SDL_Delay(10);
		SDL_PollEvent(&event);
	}
	return 0;
}