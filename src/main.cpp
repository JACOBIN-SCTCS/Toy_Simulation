#include <iostream>
#include <time.h>
#include "framework.h"
#include "frontier_explore.h"
#include "topolgoical_explore.h"

const int SENSOR_RANGE = 8;

class Robot
{
public:
	Robot(int g_size, int w_size, double scale, int n_obstacles) : grid_size(g_size), fw(w_size, w_size, scale), num_obstacles(n_obstacles)
	{
		grid.resize(grid_size, std::vector<int>(grid_size, -1));
		grid_original.resize(grid_size,std::vector<int>(grid_size,1));
		obstacle_id_grid.resize(grid_size,std::vector<int>(grid_size,1));

		for (int i = 0; i < num_obstacles; ++i)
		{
			int x = rand() % grid_size;
			int y = rand() % grid_size;
			for(int j=0;j<4;++j)
			{
				for(int k=0;k<4;++k)
				{
					if(x+j<grid_size && y+k<grid_size)
					{
						grid_original[x+j][y+k] = 0;
						obstacle_id_grid[x+j][y+k] = i;
					}
				}
			}
			obstacles.push_back({x, y});
		}
		std::cout << "Size of the grid = " << grid_size << std::endl;
	}

	void sensor_model(int x, int y)
	{
		int num_rays = 360;
		double angle = 0;
		int range = SENSOR_RANGE;

		double resolution = (2*M_PI)/num_rays;
		bool touched_an_obstacle[obstacles.size()] = {false};

		for(int i=0;i<num_rays;++i)
		{
			for(int j=0;j<range;++j)
			{
				double new_x = floor(x + j*cos(angle));
				double new_y = floor(y + j*sin(angle));
				int new_int_x = int(new_x);
				int new_int_y = int(new_y);
				if(new_int_x<0 || new_int_x >=grid_size || new_int_y<0 || new_int_y>=grid_size)
					break;	

				if(grid_original[new_int_x][new_int_y]==0)
				{
					grid[new_int_x][new_int_y]=0;
					touched_an_obstacle[obstacle_id_grid[new_int_x][new_int_y]] = true;
					break;
				}
				else
				{
					grid[new_int_x][new_int_y] = 1;
				
				}
			}
			angle = angle + resolution;
		}
		for(int i=0;i<obstacles.size();++i)
		{
			if(touched_an_obstacle[i])
			{
				bool already_seen =  false;
				for(int j=0;j<obstacles_seen.size();++j)
				{
					if(obstacles_seen[j][0] == obstacles[i][0] && obstacles_seen[j][1] == obstacles[i][1])
					{
						already_seen = true;
						break;
					}
				}
				if(! already_seen)
				{
					obstacles_seen.push_back(obstacles[i]);
				}
			}
		}
		grid[x][y] = 2;
	}

	void start_exploring(int x, int y)
	{
		// fw.render_screen(grid_original);
		int current_x = x ;
		int current_y = y;

		if(obstacle_id_grid[current_x][current_y] == 0)
		{
			std::cout << "Robot is in obstacle" << std::endl;
			return;
		}

		sensor_model(current_x, current_y);

		FrontierExplore f_explore(&grid,&obstacles_seen);
		f_explore.findFrontiers(current_x, current_y);

		while(f_explore.frontiers.size() > 0)
		{
			 std::cout << "Frontiers size = " << f_explore.frontiers.size() << std::endl;
			std::vector<std::pair<int, int>> path = f_explore.getPath(current_x, current_y, f_explore.frontiers[0].x, f_explore.frontiers[0].y);
			grid[f_explore.frontiers[0].x][f_explore.frontiers[0].y] = 3;

			std::cout << "Frontier exploration Path size = " << path.size() << std::endl;
			for (int i = 0; i < path.size(); ++i)
			{
				grid[current_x][current_y] = 1;
				current_x = path[i].first;
				current_y = path[i].second;
				grid[current_x][current_y] = 2;
				sensor_model(current_x,current_y);
				fw.render_screen(grid);
				SDL_Delay(500);
				bool towards_an_obstacle = false;
				// for(int j=i+1;j<path.size();++j)
				// {
				// 	if(grid_original[path[j].first][path[j].second]==0  && sqrt((path[j].first-path[i].first)*(path[j].first-path[i].first)+(path[j].second-path[i].second)*(path[j].second-path[i].second))<=SENSOR_RANGE)
				// 	{
				// 		towards_an_obstacle = true;
				// 		break;
				// 	}    
				// }
				if(((i+1)<path.size() && grid_original[path[i+1].first][path[i+1].second]==0)  || towards_an_obstacle)
					break;
			}
			f_explore.findFrontiers(current_x, current_y);
		}
	}

	void topological_explore_2(std::vector<int> start, std::vector<int> goal)
	{
		int current_x = start[0] ;
		int current_y = start[1];
		srand(time(NULL));

		for (int i = 0; i < obstacles.size(); ++i)
		{
			if ((obstacles[i][0] == current_x && obstacles[i][1] == current_y ) ||(obstacles[i][0] == goal[0] && obstacles[i][1] == goal[1] ) )
			{
				std::cout << "The starting and destination points cannot be in an obstacle" << std::endl;
				return;
			}
		}
		int start_x = current_x , start_y = current_y;
		sensor_model(current_x, current_y);
		TopolgicalExplore top_explore(&grid, &obstacles_seen,start,goal);
		FrontierExplore f_explore(&grid,&obstacles_seen);


		std::vector<std::vector<std::pair<int,int>>> already_traversed_paths;
		std::vector<Eigen::VectorXd> visited_h_signatures;
		double epsilon = 1.0;
		int t = 0;
		while(true)
		{
			double drawn_number = ((double)rand()/(double)RAND_MAX);
			if(drawn_number <= epsilon)
			{
				// Adopt a topolgical exploration strategy
				// auto current_path = top_explore.getNonHomologousPaths(current_x,current_y,{});
				std::cout<<"Doing a topological exploration strategy"<<std::endl;
				top_explore.getNonHomologousPaths(current_x,current_y,{});
				std::vector<std::pair<int,int>> p = top_explore.current_path;
				int start_idx = top_explore.current_path_index;
				for(int i=start_idx;i<p.size();++i)
				{
					grid[p[i].first][p[i].second] = 2;
					
				}
				fw.render_screen(grid);
				for(int i=start_idx;i < p.size();++i)
				{
					
					grid[current_x][current_y] = 1;
					current_x = p[i].first;
					current_y = p[i].second;
					grid[current_x][current_y] = 2;
					sensor_model(current_x,current_y);
					fw.render_screen(grid);
					if((i+1)< p.size() && grid[p[i+1].first][p[i+1].second] ==0)
					{
						std::vector<std::pair<int,int>> new_current_path;
						for(int j=0;j<=i;++j)
							new_current_path.push_back(p[j]);
						top_explore.current_path = new_current_path;
						// top_explore.current_path.erase(p.begin()+i+1);
						top_explore.current_path_index = i;
						break;
					}
					SDL_Delay(500);
				}
				if(current_x== top_explore.current_goal[0] && current_y==top_explore.current_goal[1])
				{
					if(top_explore.current_goal[0] == top_explore.start_coordinates[0] && top_explore.current_goal[1] == top_explore.start_coordinates[1])
					{
						top_explore.current_start = {top_explore.start_coordinates[0],top_explore.start_coordinates[1]};
						top_explore.current_goal = {top_explore.goal_coordinates[0],top_explore.goal_coordinates[1]};
						std::reverse(top_explore.current_path.begin(),top_explore.current_path.end());
					}
					else
					{
						top_explore.current_start = {top_explore.goal_coordinates[0],top_explore.goal_coordinates[1]};
						top_explore.current_goal = {top_explore.start_coordinates[0],top_explore.start_coordinates[1]};
					}
					std::vector<std::pair<int,int>> current_path_copy;
					for(int i=0;i<top_explore.current_path.size();++i)
					{
						current_path_copy.push_back({top_explore.current_path[i].first,top_explore.current_path[i].second});
					}
					top_explore.traversed_paths.push_back(current_path_copy);
					top_explore.current_path.clear();
					top_explore.current_path_index = 0;
				}

			}
			else
			{
				std::cout<<"Doing Frontier Based exploration"<<std::endl;
				f_explore.findFrontiers(current_x, current_y);
				
				std::vector<std::pair<int, int>> path = f_explore.getPath(current_x, current_y, f_explore.frontiers[0].x, f_explore.frontiers[0].y);
				grid[f_explore.frontiers[0].x][f_explore.frontiers[0].y] = 3;

				std::cout << "Frontier ExplorationPath size = " << path.size() << std::endl;
				std::vector<std::pair<int,int>> new_current_path_copy;
				for(int i =0;i<top_explore.current_path.size();++i)
					new_current_path_copy.push_back(top_explore.current_path[i]);
					
				for (int i = 0; i < path.size(); ++i)
				{	
					new_current_path_copy.push_back(path[i]);
					top_explore.current_path_index  = new_current_path_copy.size()-1;

					grid[current_x][current_y] = 1;
					current_x = path[i].first;
					current_y = path[i].second;
					grid[current_x][current_y] = 2;
					sensor_model(current_x,current_y);
					fw.render_screen(grid);
					SDL_Delay(500);
				}
				top_explore.current_path = new_current_path_copy;
				// current_x = path[path.size() - 1].first;
				// current_y = path[path.size() - 1].second;
				// sensor_model(current_x, current_y);
				// fw.render_screen(grid);
				// f_explore.findFrontiers(current_x, current_y);
					
			}
			t+=1;
			epsilon = epsilon*pow(2.71828,-0.01*t);
			if(t>=1000)
				break;
		}

	}
	

	Framework fw;

private:
	int grid_size;
	std::vector<std::vector<int>> grid;
	std::vector<std::vector<int>> grid_original;

	std::vector<std::vector<int>> obstacle_id_grid;
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