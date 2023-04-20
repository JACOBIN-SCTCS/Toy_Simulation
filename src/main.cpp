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
			// grid[x][y] = 0.0;
		}
		// obstacles.push_back({8,8});
		//obstacles_seen = obstacles;
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
							bool o_cflag  = false;
							for(int o_c = 0; o_c < obstacles_seen.size(); ++o_c)
							{
								if((obstacles_seen[o_c][0] == i && obstacles_seen[o_c][1] == j))
								{
									//obstacles_seen.push_back({i, j});
									o_cflag = true;
									break;
								}
							}
							if(!o_cflag)
								obstacles_seen.push_back({i, j});
							flag = true;
							// std::cout <<"Found an obstacle"<<std::endl;
							break;
						}
					}
					if (flag == true)
						grid[i][j] = 0;
					else
					{
						if(grid[i][j]==-1)
							grid[i][j] = 1;
					
					}
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
		int iterations = 0;
		std::vector<int> goal = top_explore.getGoalCoordinate(current_x,current_y);

		while(goal.size() > 0)
		{
			std::cout << "Iteration :" << iterations<<std::endl;
			std::vector<std::vector<std::pair<int,int>>> current_paths = top_explore.getPaths(current_x,current_y,goal[0],goal[1]);
			if(current_paths.size() <=0 )
			{
				iterations+=1;
				continue;
			}
			
			if(current_paths.size()>0)
			{
				for(int i=0;i<current_paths[0].size();++i)
				{
					grid[current_paths[0][i].first][current_paths[0][i].second]=3;
					
				}
				if(current_paths.size()>1)
				{
					for(int i=0;i<current_paths[1].size();++i)
					{
						grid[current_paths[1][i].first][current_paths[1][i].second]=3;
					}
				}
			}
			

			for(int i=0;i<current_paths[0].size();++i)
			{
				grid[current_x][current_y] = 1;
				current_x = current_paths[0][i].first;
				current_y = current_paths[0][i].second;
				grid[current_x][current_y] = 2;
				sensor_model(current_x,current_y);
				fw.render_screen(grid);
				SDL_Delay(500);
			}
			std::cout<<"Reached here";
			if(current_paths.size() > 1)
			{
				std::vector<std::pair<int,int>> reversed_path  = current_paths[1];
				std::reverse(reversed_path.begin(),reversed_path.end());
				for(int i=0;i<reversed_path.size();++i)
				{
					grid[current_x][current_y] = 1;
					current_x = reversed_path[i].first;
					current_y = reversed_path[i].second;
					grid[current_x][current_y] = 2;
					sensor_model(current_x,current_y);
					fw.render_screen(grid);
					SDL_Delay(500);
				}
			}
			iterations+=1;
			goal = top_explore.getGoalCoordinate(current_x,current_y);
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

	Robot robot(60, 600,10.0, 20);
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