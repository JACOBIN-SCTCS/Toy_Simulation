#include <iostream>
#include <time.h>
#include "framework.h"
#include "frontier_explore.h"

class Robot
{
public:
	Robot(int g_size, int w_size, double scale, int n_obstacles) : grid_size(g_size), fw(w_size, w_size, scale), num_obstacles(n_obstacles)
	{

		grid.resize(grid_size, std::vector<int>(grid_size, -1));
		for (int i = 0; i < num_obstacles; ++i)
		{
			int x = rand() % grid_size;
			int y = rand() % grid_size;
			obstacles.push_back({x, y});
			 grid[x][y] = 0.0;
		}
		std::cout << "Size of the grid = " << grid_size << std::endl;
	}

	void sensor_model(int x, int y)
	{
		for (int i = x - lidar_range; i <= x + lidar_range; ++i)
		{
			for (int j = y - lidar_range; j <= y + lidar_range; ++j)
			{
				double dist = (x - i) * (x - i) + (y - j) * (y - j);
				if (i >= 0 && i < grid_size && j >= 0 && j < grid_size && dist <= (lidar_range * lidar_range))
				{
					bool flag = false;
					for (int k = 0; k < obstacles.size(); ++k)
					{
						if (obstacles[k][0] == i && obstacles[k][1] == j)
						{
							obstacles_seen.push_back({i, j});
							flag = true;
							break;
						}
					}
					if (flag == true)
						grid[i][j] = 0;
					else
						grid[i][j] = 1;
				}
			}
		}
	}

	void start_exploring(int x, int y)
	{
		for (int i = 0; i < obstacles.size(); ++i)
		{
			if (obstacles[i][0] == x && obstacles[i][1] == y)
			{
				std::cout << "Robot is in obstacle" << std::endl;
				return;
			}
		}
		sensor_model(x, y);
		fw.render_screen(grid);
		
		// for(int i= 0;i < 30;++i)
		// {
		// 	sensor_model(x+i,y+i);
		// 	grid[x+i][y+i] = 2;
		// 	fw.render_screen(grid);
		// }

		FrontierExplore f_explore(&grid);
		f_explore.findFrontiers(x,y);
		for(int i=0;i<f_explore.frontiers.size();++i)
		{
			std::cout << "Frontier " << i << " = " << f_explore.frontiers[i].x << " " << f_explore.frontiers[i].y << std::endl;
		}


	}

	Framework fw;

private:
	int grid_size;
	std::vector<std::vector<int>> grid;
	std::vector<std::vector<int>> obstacles;
	std::vector<std::vector<int>> obstacles_seen;
	int num_obstacles;
	int lidar_range = 4;
	double scale = 10.0;
};

int main(int argc, char *argv[])
{
	srand(time(NULL));

	Robot robot(150, 600, 4.0, 6);
	robot.start_exploring(10, 10);

	// robot.setInitialRobotPose(5,5);

	// robot.fw.draw_point(300,300);
	// robot.fw.draw_point(400,400);
	// robot.setInitialRobotPose(300,300);
	SDL_Event event;
	while (!(event.type == SDL_QUIT))
	{
		SDL_Delay(10);
		SDL_PollEvent(&event);
	}
	return 0;
}