#include <iostream>
#include <fstream>
#include <time.h>
#include "framework.h"
#include "frontier_explore.h"
#include "topolgoical_explore.h"
#include "modified_topological_explore.h"


class Robot
{
public:
	Robot(int g_size, int w_size, double scale, int n_obstacles, bool window = true , std::string filename = "results.txt", std::string obs_filename="obs0.txt",int sensor_range=8) : grid_size(g_size),  fw(w_size,w_size,scale), num_obstacles(n_obstacles), SENSOR_RANGE(sensor_range)
	{
		use_window  = window;
		output_file_name = filename;
		grid.resize(grid_size, std::vector<int>(grid_size, -1));
		grid_original.resize(grid_size,std::vector<int>(grid_size,1));
		obstacle_id_grid.resize(grid_size,std::vector<int>(grid_size,1));
		
		std::ifstream infile(obs_filename);
		int i=0;
		while(i < num_obstacles)
		{	
			int tmp_x, tmp_y;
			infile >> tmp_x >> tmp_y;
			std::cout << tmp_x <<" "<< tmp_y <<std::endl;
			int x =   tmp_x ; 
			int y =   tmp_y ;
			bool obstacle_invalid = false;
			for(int j=0;j<4;++j)
			{
				for(int k=0;k<4;++k)
				{
					if(x+j >= (grid_size-1) ||  x+j < 0 ||  y+k >= (grid_size-1) ||  y+k < 0)
					{
						obstacle_invalid = true;
						break;
					}
				}
				if(obstacle_invalid)
					break;
			}

			for(int j=0;j<4;++j)
			{
				for(int k=0;k<4;++k)
				{
					grid_original[x+j][y+k] = 0;
					obstacle_id_grid[x+j][y+k] = i;
				}
			}
			obstacles.push_back({x, y});
			i+=1;
		}
		infile.close();

		for(int i=0;i<grid_size;++i)
		{
			for(int j=0;j<grid_size;++j)
			{
				if(grid_original[i][j] == 1)
					total_free_space +=1;
			}
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
					if(grid[new_int_x][new_int_y]==-1 || grid[new_int_x][new_int_y]==3)
						total_cells_mapped+=1;
					
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
		std::fstream f;

		if(!use_window)
		{
			
			f.open(output_file_name, std::ios::app);
			f << "-\n";
		}

		sensor_model(current_x, current_y);

		FrontierExplore f_explore(&grid,&obstacles_seen);
		f_explore.findFrontiers(current_x, current_y);

		while(f_explore.frontiers.size() > 0)
		{
			 std::cout << "Frontiers size = " << f_explore.frontiers.size() << std::endl;
			std::vector<std::pair<int, int>> path = f_explore.getPath(current_x, current_y, f_explore.frontiers[0].x, f_explore.frontiers[0].y);
			// grid[f_explore.frontiers[0].x][f_explore.frontiers[0].y] = 3;

			std::cout << "Frontier exploration Path size = " << path.size() << std::endl;
			for (int i = 0; i < path.size(); ++i)
			{
				timesteps_taken +=1;
				grid[current_x][current_y] = 1;
				current_x = path[i].first;
				current_y = path[i].second;
				grid[current_x][current_y] = 2;
				sensor_model(current_x,current_y);
				if(!use_window)
					f << timesteps_taken << " " << ((double)total_cells_mapped/(total_free_space)) * 100<< "\n";
				if(use_window)
				{
					fw.render_screen(grid);
					SDL_Delay(500);
				}
				// fw.render_screen(grid);
				// SDL_Delay(500);
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
				{
					grid[path[i+1].first][path[i+1].second] = 0;
					break;
				}
			}
			f_explore.findFrontiers(current_x, current_y);
		}
		f.close();
		std::cout<<"Exploration complete"<<std::endl;
	}

	void topological_explore_2(std::vector<int> start, std::vector<int> goal)
	{
		int current_x = start[0] ;
		int current_y = start[1];
		srand(time(NULL));

		// if ((obstacles[i][0] == current_x && obstacles[i][1] == current_y ) ||(obstacles[i][0] == goal[0] && obstacles[i][1] == goal[1] ) )
		if(grid_original[current_x][current_y] ==0 || grid_original[goal[0]][goal[1]] ==0)
		{
			std::cout << "The starting and destination points cannot be in an obstacle" << std::endl;
			return;
		}
		
		std::fstream f;
		if(!use_window)
		{
			f.open(output_file_name, std::ios::app);
		 	f<< "-\n";
		}

		int start_x = current_x , start_y = current_y;
		sensor_model(current_x, current_y);
		TopolgicalExplore top_explore(&grid, &obstacles_seen,start,goal,&grid_original);
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
				if(p.size() == 0)
				{
					std::cout<<"No Topological exploration path found"<<std::endl;
					t+=1;
					epsilon = epsilon*pow(2.71828,-0.01*t);
					if(t>=1000)
						break;
				}
				int start_idx = top_explore.current_path_index;
				// for(int i=start_idx;i<p.size();++i)
				// {
				// 	grid[p[i].first][p[i].second] = 2;
					
				// }
				if(use_window)
					fw.render_screen(grid);
				
				for(int i=start_idx;i < p.size();++i)
				{

					timesteps_taken += 1;
					grid[current_x][current_y] = 1;
					current_x = p[i].first;
					current_y = p[i].second;
					grid[current_x][current_y] = 2;
					sensor_model(current_x,current_y);
					if(!use_window)
						f<< timesteps_taken << " " << ((double)total_cells_mapped/(total_free_space)) * 100<< "\n";
					if (use_window)
						fw.render_screen(grid);
					
					if((i+1)< p.size() && grid_original[p[i+1].first][p[i+1].second] ==0)
					{
						std::vector<std::pair<int,int>> new_current_path;
						for(int j=0;j<=i;++j)
							new_current_path.push_back(p[j]);
						top_explore.current_path = new_current_path;
						// top_explore.current_path.erase(p.begin()+i+1);
						top_explore.current_path_index = i;
						break;
					}
					if(use_window)
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
				frontier:
				std::cout<<"Doing Frontier Based exploration"<<std::endl;
				f_explore.findFrontiers(current_x, current_y);
				if(f_explore.frontiers.size() <=0)
				{
					t+=1000;
					break;
				}
			
				std::vector<std::pair<int, int>> path = f_explore.getPath(current_x, current_y, f_explore.frontiers[0].x, f_explore.frontiers[0].y);
				grid[f_explore.frontiers[0].x][f_explore.frontiers[0].y] = 3;

				std::cout << "Frontier ExplorationPath size = " << path.size() << std::endl;
				std::vector<std::pair<int,int>> new_current_path_copy;
				for(int i =0;i<top_explore.current_path.size();++i)
					new_current_path_copy.push_back(top_explore.current_path[i]);
					
				for (int i = 0; i < path.size(); ++i)
				{	timesteps_taken+=1;
					new_current_path_copy.push_back(path[i]);
					top_explore.current_path_index  = new_current_path_copy.size()-1;

					grid[current_x][current_y] = 1;
					current_x = path[i].first;
					current_y = path[i].second;
					grid[current_x][current_y] = 2;
					sensor_model(current_x,current_y);
					if(!use_window)
						f<< timesteps_taken << " " << ((double)total_cells_mapped/(total_free_space)) * 100<< "\n";
					if(use_window)
					{
						fw.render_screen(grid);
						SDL_Delay(500);
					}
					// }
					if(((i+1)<path.size() && grid_original[path[i+1].first][path[i+1].second]==0))
					{
						grid[path[i+1].first][path[i+1].second] = 0;
						break;
					}
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
		f.close();

	}

	void topological_explore_3(std::vector<int> start)
	{
		int current_x = start[0] ;
		int current_y = start[1];
		srand(time(NULL));

		// if ((obstacles[i][0] == current_x && obstacles[i][1] == current_y ) ||(obstacles[i][0] == goal[0] && obstacles[i][1] == goal[1] ) )
		std::vector<std::vector<int>> goals = {{grid_size-1,grid_size-1},{0,0},{0,grid_size-1},{grid_size-1,0}};
		if(grid_original[current_x][current_y] ==0 || grid_original[grid_size-1][0] ==0 || grid_original[grid_size-1][grid_size-1] ==0 || grid_original[0][grid_size-1] ==0 || grid_original[0][0] == 0)
		{
			std::cout << "The starting and destination points cannot be in an obstacle" << std::endl;
			return;
		}

		std::fstream f;
		if(!use_window)
		{
			f.open(output_file_name, std::ios::app);
			f << "-\n";
		}
    	
		
		int start_x = current_x , start_y = current_y;
		sensor_model(current_x, current_y);
		
		
		TopolgicalExplore top_explore(&grid, &obstacles_seen,start,goals,&grid_original);
		FrontierExplore f_explore(&grid,&obstacles_seen);
		
		std::vector<std::vector<std::pair<int,int>>> already_traversed_paths;
		std::vector<Eigen::VectorXd> visited_h_signatures;
		double epsilon = 1.0;
		int t = 0;
		while(true)
		{
			double drawn_number = ((double)rand()/(double)RAND_MAX);
			std::cout <<"Drawn number  = "  <<drawn_number <<std::endl;
			// std::cout <<"Epsilon = "  <<epsilon <<std::endl;
			if(drawn_number <= epsilon)
			{
				// Adopt a topolgical exploration strategy
				// auto current_path = top_explore.getNonHomologousPaths(current_x,current_y,{});
				std::cout<<"Doing a topological exploration strategy"<<std::endl;
				bool status = top_explore.getNonHomologousPaths(current_x,current_y,{});
				std::vector<std::pair<int,int>> p = top_explore.current_path;
				if(!status)
				{
					std::cout<<"No Topological exploration path found"<<std::endl;
					t+=1;
					epsilon = 1.0 - ((double)total_cells_mapped/(total_free_space) + pow(2.71828,0.01*t)/(total_free_space)); //pow(2.71828,-0.02*t);
					// std::cout<<"Epsilon = "<< epsilon << std::endl;
					goto frontier;
					if(t>=1000)
						break;
				}
				int start_idx = top_explore.current_path_index;
				// for(int i=start_idx;i<p.size();++i)
				// {
				// 	grid[p[i].first][p[i].second] = 2;
					
				// }
				if (use_window)
					fw.render_screen(grid);
				for(int i=start_idx;i < p.size();++i)
				{
					timesteps_taken+=1;
					grid[current_x][current_y] = 1;
					current_x = p[i].first;
					current_y = p[i].second;
					grid[current_x][current_y] = 2;
					sensor_model(current_x,current_y);
					if(!use_window)
						f<< timesteps_taken << " " << ((double)total_cells_mapped/(total_free_space)) * 100<< "\n";
					if(use_window)
						fw.render_screen(grid);
					if((i+1)< p.size() && grid_original[p[i+1].first][p[i+1].second] ==0)
					{
						std::vector<std::pair<int,int>> new_current_path;
						for(int j=0;j<=i;++j)
							new_current_path.push_back(p[j]);
						top_explore.current_path = new_current_path;
						// top_explore.current_path.erase(p.begin()+i+1);
						top_explore.current_path_index = i;
						break;
					}
					if(use_window)
						SDL_Delay(500);
				}
				if(current_x== top_explore.current_goal[0] && current_y==top_explore.current_goal[1])
				{
					if(top_explore.current_goal[0] == top_explore.start_coordinates[0] && top_explore.current_goal[1] == top_explore.start_coordinates[1])
					{
						top_explore.current_start = {top_explore.start_coordinates[0],top_explore.start_coordinates[1]};
						if(!top_explore.use_four_corner_points)
						{
							top_explore.current_goal = {top_explore.goal_coordinates[0],top_explore.goal_coordinates[1]};
						}
						else
						{
							// times visited here
							// int corner_point_idx = rand()%4;
							int weight_sum = 0;
							for(int i=0;i<top_explore.n_times_chosen.size();++i)
								weight_sum += top_explore.n_times_chosen[i];
							
							std::vector<int> tmp_weights;
							for(int i=0;i<top_explore.n_times_chosen.size();++i)
								tmp_weights.push_back(weight_sum - top_explore.n_times_chosen[i]);
							
							
							weight_sum = 0;
							for(int i=0;i<tmp_weights.size();++i)
								weight_sum += tmp_weights[i];
							
							int drawn_number = rand()%weight_sum;

							int chosen_index = 0;
							for(int i=0;i<tmp_weights.size();++i)
							{
								drawn_number -= tmp_weights[i];
								if(drawn_number <=0)
								{
									chosen_index = i;
									break;
								}

							}
							top_explore.n_times_chosen[chosen_index] += 1;
							std::cout<<"Goal point chosen = " << top_explore.goals[chosen_index][0] << " " << top_explore.goals[chosen_index][1] << std::endl;

							top_explore.current_goal = {top_explore.goals[chosen_index][0],top_explore.goals[chosen_index][1]};
						}
						std::reverse(top_explore.current_path.begin(),top_explore.current_path.end());
					}
					else
					{
						if(!top_explore.use_four_corner_points)
							top_explore.current_start = {top_explore.goal_coordinates[0],top_explore.goal_coordinates[1]};
						else
							top_explore.current_start = {top_explore.current_goal[0],top_explore.current_goal[1]};

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
				frontier:
				std::cout<<"Doing Frontier Based exploration"<<std::endl;
				f_explore.findFrontiers(current_x, current_y);
				if(f_explore.frontiers.size() <=0)
				{
					t+=1000;
					break;
				}
				
				std::vector<std::pair<int, int>> path = f_explore.getPath(current_x, current_y, f_explore.frontiers[0].x, f_explore.frontiers[0].y);
				grid[f_explore.frontiers[0].x][f_explore.frontiers[0].y] = 3;

				std::cout << "Frontier ExplorationPath size = " << path.size() << std::endl;
				std::vector<std::pair<int,int>> new_current_path_copy;
				for(int i =0;i<top_explore.current_path.size();++i)
					new_current_path_copy.push_back(top_explore.current_path[i]);
					
				for (int i = 0; i < path.size(); ++i)
				{	timesteps_taken+=1;
					new_current_path_copy.push_back(path[i]);
					top_explore.current_path_index  = new_current_path_copy.size()-1;

					grid[current_x][current_y] = 1;
					current_x = path[i].first;
					current_y = path[i].second;
					grid[current_x][current_y] = 2;
					sensor_model(current_x,current_y);
					if(!use_window)
						f<< timesteps_taken << " " << ((double)total_cells_mapped/(total_free_space)) * 100<< "\n";
					if(use_window)
					{
						fw.render_screen(grid);
						SDL_Delay(500);
					}
					// }
					if(((i+1)<path.size() && grid_original[path[i+1].first][path[i+1].second]==0))
					{
						grid[path[i+1].first][path[i+1].second] = 0;
						break;
					}
				}
				top_explore.current_path = new_current_path_copy;
				// current_x = path[path.size() - 1].first;
				// current_y = path[path.size() - 1].second;
				// sensor_model(current_x, current_y);
				// fw.render_screen(grid);
				// f_explore.findFrontiers(current_x, current_y);
					
			}
			t+=1;
			epsilon = 1.0 - ((double)total_cells_mapped / (total_free_space) + pow(2.71828,0.01*t)/(total_free_space)); //pow(2.71828,-0.02*t);
			// std::cout<<"Epsilon = "<< epsilon << std::endl;
			if(t>=1000)
				break;
		}
		std::cout<<"Done Exploration"<<std::endl;
	}
	
	void topological_explore_4(std::vector<int> start)
	{
		int current_x = start[0] ;
		int current_y = start[1];
		srand(time(NULL));

		std::fstream f;
		if(!use_window)
		{
			f.open(output_file_name, std::ios::app);
			f << "-\n";
		}
    	
		
		int start_x = current_x , start_y = current_y;
		sensor_model(current_x, current_y);
		
		ModifiedTopolgicalExplore top_explore(&grid,&obstacles_seen,start,&grid_original);
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
				if(grid[top_explore.current_goal[0]][top_explore.current_goal[1]]!=-1)
				{
					std::default_random_engine generator;
            		std::vector<int> quadrant_weights = top_explore.get_quadrant_vector();
            		std::discrete_distribution<int> distribution(quadrant_weights.begin(),quadrant_weights.end());
            		int quadrant_select_index = distribution(generator);//rand() % 4;
					int next_quadrant_index = quadrant_select_index;
            		std::vector<std::pair<int,int>> dest_points = top_explore.get_destination_point(quadrant_select_index);
            		int random_index = rand() % dest_points.size();
            		top_explore.current_goal = {dest_points[random_index].first,dest_points[random_index].second};
					top_explore.current_goal_quadrant  = next_quadrant_index;
				}
				bool status = top_explore.getNonHomologousPaths(current_x,current_y,{});
				std::vector<std::pair<int,int>> p = top_explore.current_path;
				if(!status)
				{
					std::cout<<"No Topological exploration path found"<<std::endl;
					t+=1;
					epsilon = 1.0 - ((double)total_cells_mapped/(total_free_space) + pow(2.71828,0.01*t)/(total_free_space));//pow(2.71828,-0.02*t);
					goto frontier;
					// if(t>=1000)
					// 	break;
				}
				int start_idx = top_explore.current_path_index;
				// for(int i=start_idx;i<p.size();++i)
				// {
				// 	grid[p[i].first][p[i].second] = 2;
					
				// }
				if (use_window)
					fw.render_screen(grid);
				for(int i=start_idx;i < p.size();++i)
				{
					timesteps_taken+=1;
					grid[current_x][current_y] = 1;
					current_x = p[i].first;
					current_y = p[i].second;
					grid[current_x][current_y] = 2;
					sensor_model(current_x,current_y);
					if(!use_window)
						f<< timesteps_taken << " " << ((double)total_cells_mapped/(total_free_space)) * 100<< "\n";
					if(use_window)
						fw.render_screen(grid);
					if((i+1)< p.size() && grid_original[p[i+1].first][p[i+1].second] ==0)
					{
						std::vector<std::pair<int,int>> new_current_path;
						for(int j=0;j<=i;++j)
							new_current_path.push_back(p[j]);
						top_explore.current_path = new_current_path;
						// top_explore.current_path.erase(p.begin()+i+1);
						top_explore.current_path_index = i;
						break;
					}
					if(use_window)
						SDL_Delay(500);
				}
				if(current_x== top_explore.current_goal[0] && current_y==top_explore.current_goal[1])
				{
					top_explore.start_quadrants.push_back(top_explore.current_start_quadrant);
            		top_explore.goal_quadrants.push_back(top_explore.current_goal_quadrant);

            		top_explore.current_start = {top_explore.current_goal[0],top_explore.current_goal[1]};
            		top_explore.current_start_quadrant = top_explore.current_goal_quadrant;
            

					int next_quadrant_index = rand() % 4;
					int random_index = -1;
					switch (next_quadrant_index)
					{
						case 0:
							random_index = rand() % (top_explore.grid->size() - 1);
							top_explore.current_goal = {0,random_index};
							break;
						case 1:
							random_index = rand() % (top_explore.grid->size() - 1);
							top_explore.current_goal = {random_index , (int)top_explore.grid->size()-1};
							break;
						case 2:
							random_index = rand() % (top_explore.grid->size() - 1) + 1;
							top_explore.current_goal = {(int)top_explore.grid->size()-1, random_index};
							break;
						case 3 : 
							random_index = rand() % (top_explore.grid->size() - 1) + 1;
							top_explore.current_goal = {random_index , 0};
							break;
						default:
							random_index = rand() % (top_explore.grid->size() - 1) + 1;
							top_explore.current_goal = {random_index , 0};
							break;
					}
					top_explore.current_goal_quadrant  = next_quadrant_index;
					
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
				frontier:
				std::cout<<"Doing Frontier Based exploration"<<std::endl;
				f_explore.findFrontiers(current_x, current_y);
				if(f_explore.frontiers.size() <=0)
				{
					t+=1000;
					break;
				}
				
				std::vector<std::pair<int, int>> path = f_explore.getPath(current_x, current_y, f_explore.frontiers[0].x, f_explore.frontiers[0].y);
				grid[f_explore.frontiers[0].x][f_explore.frontiers[0].y] = 3;

				std::cout << "Frontier ExplorationPath size = " << path.size() << std::endl;
				std::vector<std::pair<int,int>> new_current_path_copy;
				for(int i =0;i<top_explore.current_path.size();++i)
					new_current_path_copy.push_back(top_explore.current_path[i]);
					
				for (int i = 0; i < path.size(); ++i)
				{	timesteps_taken+=1;
					new_current_path_copy.push_back(path[i]);
					top_explore.current_path_index  = new_current_path_copy.size()-1;

					grid[current_x][current_y] = 1;
					current_x = path[i].first;
					current_y = path[i].second;
					grid[current_x][current_y] = 2;
					sensor_model(current_x,current_y);
					if(!use_window)
						f<< timesteps_taken << " " << ((double)total_cells_mapped/(total_free_space)) * 100<< "\n";
					if(use_window)
					{
						fw.render_screen(grid);
						SDL_Delay(500);
					}
					// }
					if(((i+1)<path.size() && grid_original[path[i+1].first][path[i+1].second]==0))
					{
						grid[path[i+1].first][path[i+1].second] = 0;
						break;
					}
				}
				top_explore.current_path = new_current_path_copy;
				// current_x = path[path.size() - 1].first;
				// current_y = path[path.size() - 1].second;
				// sensor_model(current_x, current_y);
				// fw.render_screen(grid);
				// f_explore.findFrontiers(current_x, current_y);
					
			}
			t+=1;
			epsilon = 1.0 - ((double)total_cells_mapped/(total_free_space) + pow(2.71828,0.01*t)/(total_free_space)); //pow(2.71828,-0.02*t);
			if(t>=1000)
				break;
		}
		std::cout<<"Done Exploration"<<std::endl;

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
	int total_cells_mapped=0;
	int lidar_range = 4;
	double scale = 10.0;
	bool use_window = true;
	std::string output_file_name;
	int timesteps_taken = 0;
	int SENSOR_RANGE = 8;
	int total_free_space = 0;

};

int main(int argc, char *argv[])
{
	int choice = 1;
	bool use_window = false;
	if(choice==0)
	{
		srand(time(NULL));

		Robot robot(60, 600,10.0, 20,use_window,"result4.txt","obs4.txt");
		robot.start_exploring(0, 0);

		for(int i=0;i<10;++i)
		{
			robot = Robot(60, 600,10.0, 20,use_window,"result4.txt","obs4.txt");
			robot.topological_explore_4({0,0});
			SDL_Delay(1000);
		}

	}
	else if(choice == 1)
	{
		srand(time(NULL));
		std::vector<int> sensor_ranges = {4,8,12,16,32};
		for(int i=0;i<sensor_ranges.size();++i)
		{
			for(int j=0;j<10;++j)
			{
				std::string obstacle_file_name = "obs" + std::to_string(j) + ".txt"; 
				Robot robot(60, 600,10.0, 100,use_window,"result_frontiers.txt",obstacle_file_name,sensor_ranges[i]);
				robot.start_exploring(0,0);
				//robot.start_exploring({0,0});
				SDL_Delay(1000);
			}
		}
	}
	else
	{
		;
	}
		
	if(use_window)
	{
		SDL_Event event;
		while (!(event.type == SDL_QUIT))
		{
			SDL_Delay(10);
			SDL_PollEvent(&event);
		}	
	}
	return 0;
}