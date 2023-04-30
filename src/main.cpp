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
		}
		std::cout << "Size of the grid = " << grid_size << std::endl;
	}

	void sensor_model(int x, int y)
	{
		int num_rays = 180;
		double angle = 0;
		int range = 8;

		double resolution = (2*M_PI)/num_rays;
		for(int i=0;i<num_rays;++i)
		{
			for(int j=0;j<range;++j)
			{
				double new_x = x + j*cos(angle);
				double new_y = y + j*sin(angle);

				int new_int_x = int(new_x);
				int new_int_y = int(new_y);
				if(new_int_x<0 || new_int_x >=grid_size || new_int_y<0 || new_int_y>=grid_size)
					break;
				bool touched_an_obstacle=false;
				for(int k=0;k < obstacles.size();++k)
				{
					if(new_int_x == obstacles[k][0] && new_int_y == obstacles[k][1])
					{
						touched_an_obstacle = true;
						break;
					}	
				}
				if(touched_an_obstacle)
				{
					grid[new_int_x][new_int_y] = 0;
					break;
				}
				else
				{
					grid[new_int_x][new_int_y] = 1;
				}
			}
			angle = angle + resolution;
		}
		grid[x][y] = 2;
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
			std::vector<std::pair<int, int>> path = f_explore.getPath(current_x, current_y, f_explore.frontiers[0].cells[0].first, f_explore.frontiers[0].cells[0].second);
			grid[f_explore.frontiers[0].x][f_explore.frontiers[0].y] = 3;

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
			current_x = path[path.size() - 1].first;
			current_y = path[path.size() - 1].second;
			sensor_model(current_x, current_y);
			fw.render_screen(grid);
			f_explore.findFrontiers(current_x, current_y);
		}
	}

	void topological_explore_2(std::vector<int> src, std::vector<int> dest)
	{

		srand(time(NULL));

		for (int i = 0; i < obstacles.size(); ++i)
		{
			if ((obstacles[i][0] == src[0] && obstacles[i][1] == src[1])||(obstacles[i][0] == dest[0] && obstacles[i][1] == dest[1]))
			{
				std::cout << "The source and destination point collides with an obstacle" << std::endl;
				return;
			}
		}
		int start_x = src[0] , start_y = src[1];
		sensor_model(start_x, start_y);
		TopolgicalExplore top_explore(&grid, &obstacles_seen,src,dest);
		FrontierExplore f_explore(&grid);


		std::vector<std::vector<std::pair<int,int>>> already_traversed_paths;
		
		std::vector<Eigen::VectorXd> visited_h_signatures;
		bool reverse_follow = false;
		Eigen::VectorXd current_partial_signature;
		std::vector<std::pair<int,int>> current_travelled_path;
		
		double epsilon = 1.0;
		int t = 0;
		while(true)
		{
			double drawn_number = ((double)rand()/(double)RAND_MAX);
			if(drawn_number <= epsilon)
			{
				;
			}
			else
			{
				// Adopt  a frontier based exploration strategy
				// std::cout << "Frontier based exploration" << std::endl;
				// f_explore.findFrontiers(current_x, current_y);
					;

			}
			t+=1;
			epsilon = epsilon*pow(2.71828,-0.01*t);
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
	robot.start_exploring(10, 10);
	// robot.topological_explore_2({10,10},{59,59});
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