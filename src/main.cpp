#include <iostream>
#include <time.h>
#include "framework.h"
#include "frontier_explore.h"
#include "topolgoical_explore.h"

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
		obstacles.push_back({8,8});
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
					if(i==x && j==y)
					{
						grid[i][j] = 2;
						continue;
					}
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
		int current_x = x ;
		int current_y = y;

		for (int i = 0; i < obstacles.size(); ++i)
		{
			if (obstacles[i][0] == x && obstacles[i][1] == y)
			{
				std::cout << "Robot is in obstacle" << std::endl;
				return;
			}
		}
		sensor_model(current_x, current_y);

		FrontierExplore f_explore(&grid);
		f_explore.findFrontiers(current_x, current_y);

		while(f_explore.frontiers.size() > 0)
		{
			 std::cout << "Frontiers size = " << f_explore.frontiers.size() << std::endl;
			// std::cout << "Before computing path" << std::endl;
			std::vector<std::pair<int, int>> path = f_explore.getPath(current_x, current_y, f_explore.frontiers[0].cells[0].first, f_explore.frontiers[0].cells[0].second);
			// grid[current_x][current_y] = 2;
			grid[f_explore.frontiers[0].x][f_explore.frontiers[0].y] = 3;

			// for (int i = 0; i < f_explore.frontiers[0].cells.size(); ++i)
			// {
			// 	grid[f_explore.frontiers[0].cells[i].first][f_explore.frontiers[0].cells[i].second] = 2;
			// }

			std::cout << "Path size = " << path.size() << std::endl;
			for (int i = 0; i < path.size(); ++i)
			{
				grid[current_x][current_y] = 1;
				current_x = path[i].first;
				current_y = path[i].second;
				grid[current_x][current_y] = 2;
				sensor_model(current_x,current_y);
				fw.render_screen(grid);
				SDL_Delay(500);
			}
			// fw.render_screen(grid);
			current_x = path[path.size() - 1].first;
			current_y = path[path.size() - 1].second;
			sensor_model(current_x, current_y);
			fw.render_screen(grid);
			f_explore.findFrontiers(current_x, current_y);
		}
		
		// for(int i= 0;i < 30;++i)
		// {
		// 	sensor_model(x+i,y+i);
		// 	grid[x+i][y+i] = 2;
		// 	fw.render_screen(grid);
		// }

		// FrontierExplore f_explore(&grid);
		// f_explore.findFrontiers(x,y);
		// std::cout<<"Before computing path"<<std::endl;
		// std::vector<std::pair<int,int>> path = f_explore.getPath(x+29,y+29,f_explore.frontiers[0].cells[0].first,f_explore.frontiers[0].cells[0].second);
		// // std::cout<<"After computing path between source and destination node"<<std::endl;
		// // std::cout << "Length of path = " <<path.size()<<std::endl;
		
		// for(int i=0;i<path.size();++i)
		// {
		// 	grid[path[i].first][path[i].second] = 4;
		// }
		// grid[x][y] = 2;
		// grid[f_explore.frontiers[0].x][f_explore.frontiers[0].y] = 3;

		// for(int i =0;i<f_explore.frontiers[0].cells.size();++i)
		// {
		// 		grid[f_explore.frontiers[0].cells[i].first][f_explore.frontiers[0].cells[i].second] = 2;
		// }
		// std::cout << "Path size = " << path.size() << std::endl;
		// for(int i=0; i<path.size();++i)
		// {
		// 	grid[path[i].first][path[i].second] = 2;
		// 	fw.render_screen(grid);
		// }
		// fw.render_screen(grid);

	}

	void topological_explore(int x , int y)
	{
		int current_x = x ;
		int current_y = y;

		for (int i = 0; i < obstacles.size(); ++i)
		{
			if (obstacles[i][0] == x && obstacles[i][1] == y)
			{
				std::cout << "Robot is in obstacle" << std::endl;
				return;
			}
		}
		sensor_model(current_x, current_y);
		TopolgicalExplore top_explore(&grid, &obstacles_seen);
		std::vector<std::vector<std::pair<int,int>>> paths = top_explore.getPaths(current_x, current_y);

		std::cout<<"Paths size = "<<paths.size()<<std::endl;

		for(int i=0;i<paths.size();++i)
		{
			std::cout<<"\n";
			for(int j=0;j<paths[i].size();++j)
			{
				std::cout<<"("<<paths[i][j].first<<","<<paths[i][j].second<<") "<<std::endl;

				grid[paths[i][j].first][paths[i][j].second] = 3;
				fw.render_screen(grid);
				SDL_Delay(500);
				
			}
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

	Robot robot(60, 600,10.0, 6);
	// robot.start_exploring(10, 10);
	robot.topological_explore(10,10);
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