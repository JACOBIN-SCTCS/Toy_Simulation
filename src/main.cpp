#include <iostream>
#include <fstream>
#include <time.h>
#include "framework.h"
#include "frontier_explore.h"
#include "topolgoical_explore.h"
#include "modified_topological_explore.h"
#include "random_walk.h"
#include <chrono>

using namespace std::chrono;

class Robot
{
public:
	Robot(int g_size, int w_size, double scale, int n_obstacles, bool window = true , std::string filename = "results.txt", std::string obs_filename="obs0.txt",int sensor_range=8,bool d_result = false,bool d_result_visualize=false, int d_result_visualize_timestep=200,std::string depth_file_prefix="obs0_") 
	: grid_size(g_size),  fw(w_size,w_size,scale), num_obstacles(n_obstacles), SENSOR_RANGE(sensor_range) , depth_result(d_result) , depth_result_visualize(d_result_visualize), depth_result_visualize_timestep(d_result_visualize_timestep), depth_result_file_prefix(depth_file_prefix)
	{
		use_window  = window;
		output_file_name = filename;
		grid.resize(grid_size, std::vector<int>(grid_size, -1));
		grid_original.resize(grid_size,std::vector<int>(grid_size,1));
		grid_original_only_boundary.resize(grid_size,std::vector<int>(grid_size,1));
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
			for(int j=0;j<square_obstacle_size;++j)
			{
				for(int k=0;k<square_obstacle_size;++k)
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

			for(int j=0;j<square_obstacle_size;++j)
			{
				for(int k=0;k<square_obstacle_size;++k)
				{
					grid_original[x+j][y+k] = 0;
					// if(j == 0 || j == 3 || k == 0 || k == 3)
					// 	grid_original_only_boundary[x+j][y+k] = 0;
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

		std::cout << "Size of the grid = " << grid_size<<"Total Free Space = "<<total_free_space<< std::endl;
		create_ground_truth_resolutions(g_size);
	}

	void sensor_model(int x, int y,bool use_tan = false)
	{
		int num_rays = 360;
		double angle = 0;
		int range = SENSOR_RANGE;

		double resolution = (2*M_PI)/num_rays;
		bool touched_an_obstacle[obstacles.size()] = {false};

		for(int i=0;i<num_rays;++i)
		{
			if(!use_tan)
			{
				for(int j=0;j<range;++j)
				{
					double new_x = x + j*cos(angle);
					double new_y =y + j*sin(angle);
					int new_int_x = static_cast<int>(new_x);
					int new_int_y = static_cast<int>(new_y);
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
			}
			else
			{
				;
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
		std::fstream depth_file;

		if(!use_window)
		{
			
			f.open(output_file_name, std::ios::app);
			f << "-\n";
		}
		if(depth_result_visualize)
		{
			depth_file.open(depth_result_file_prefix,std::ios::app);
			depth_file<<"-\n";
		}

		sensor_model(current_x, current_y);
		if(!use_window)
		{
			if(depth_result)
			{
				auto error_result = getError();
				std::stringstream ss_error;
				for(int k=0;k<error_result.size();++k)
				{
					ss_error << error_result[k] << " ";
				}
				f << timesteps_taken << " "<< ((double)total_cells_mapped/(total_free_space)) * 100<<" "<<ss_error.str()<<"\n";
				if(depth_result_visualize)
				{
					if(timesteps_taken%depth_result_visualize_timestep ==0)
					{
						for(int j=0;j<depths.size();++j)
						{
							std::vector<std::vector<float>> current_depth_result = get_current_depth_result(j);
							std::stringstream depth_ss;
							for(int k=0;k<current_depth_result.size();++k)
							{
								for(int l=0;l<current_depth_result[0].size();++l)
								{
									depth_ss << current_depth_result[k][l] <<",";
								}
							}
							depth_ss<<"\n";
							depth_file<<depth_ss.str();
						}
						depth_file<<"-\n";
					}
					
				}
			}
		}

		FrontierExplore f_explore(&grid,&obstacles_seen);
		f_explore.findFrontiers(current_x, current_y);

		while(f_explore.frontiers.size() > 0)
		{
			double stopping_percentage = ((double)total_cells_mapped/(total_free_space)) * 100;
			if(stopping_percentage>=99.0)
				break;
			
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
				{
					if(depth_result)
					{
						auto error_result = getError();
						std::stringstream ss_error;
						for(int k=0;k<error_result.size();++k)
						{
							ss_error << error_result[k] << " ";
						}
						f << timesteps_taken << " "<< ((double)total_cells_mapped/(total_free_space)) * 100<<" "<<ss_error.str()<<"\n";
						if(depth_result_visualize)
						{
							if(timesteps_taken%depth_result_visualize_timestep ==0)
							{
								for(int j=0;j<depths.size();++j)
								{
									std::vector<std::vector<float>> current_depth_result = get_current_depth_result(j);
									std::stringstream depth_ss;
									for(int k=0;k<current_depth_result.size();++k)
									{
										for(int l=0;l<current_depth_result[0].size();++l)
										{
											depth_ss << current_depth_result[k][l] <<",";
										}
									}
									depth_ss<<"\n";
									depth_file<<depth_ss.str();
								}
								depth_file<<"-\n";
							}
							
						}
					}
					else
					{
						f << timesteps_taken << " " << ((double)total_cells_mapped/(total_free_space)) * 100<<"\n";
					}
				}
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
				// if(grid[path[path.size()-1].first][path[path.size()-1].second] != -1 && grid[path[path.size()-1].first][path[path.size()-1].second] != 3)
				// 	break;
				
				if(((i+1)<path.size() && grid_original[path[i+1].first][path[i+1].second]==0)  || towards_an_obstacle)
				{
					grid[path[i+1].first][path[i+1].second] = 0;
					break;
				}
			}
			f_explore.findFrontiers(current_x, current_y);
		}
		f.close();
		depth_file.close();
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
		std::fstream depth_file;
		if(!use_window)
		{
			f.open(output_file_name, std::ios::app);
			f << "-\n";
		}
		if(depth_result_visualize)
		{
			depth_file.open(depth_result_file_prefix,std::ios::app);
			depth_file<<"-\n";
		}
    	
		
		int start_x = current_x , start_y = current_y;
		sensor_model(current_x, current_y);
		if(!use_window)
		{
			if(depth_result)
			{
				auto error_result = getError();
				std::stringstream ss_error;
				for(int k=0;k<error_result.size();++k)
				{
					ss_error << error_result[k] << " ";
				}
				f << timesteps_taken << " "<< ((double)total_cells_mapped/(total_free_space)) * 100<<" "<<ss_error.str()<<"\n";
				if(depth_result_visualize)
				{
					if(timesteps_taken%depth_result_visualize_timestep ==0)
					{
						for(int j=0;j<depths.size();++j)
						{
							std::vector<std::vector<float>> current_depth_result = get_current_depth_result(j);
							std::stringstream depth_ss;
							for(int k=0;k<current_depth_result.size();++k)
							{
								for(int l=0;l<current_depth_result[0].size();++l)
								{
									depth_ss << current_depth_result[k][l] <<",";
								}
							}
							depth_ss<<"\n";
							depth_file<<depth_ss.str();
						}
						depth_file<<"-\n";
					}
					
				}
			}
		}

		ModifiedTopolgicalExplore top_explore(&grid,&obstacles_seen,start,&grid_original,true,square_obstacle_size);
		FrontierExplore f_explore(&grid,&obstacles_seen);
		
		std::vector<std::vector<std::pair<int,int>>> already_traversed_paths;
		double epsilon = 1.0;
		int t = 0;
		int previously_chosen = 0;
		while(true)
		{
			double drawn_number = ((double)rand()/(double)RAND_MAX);
			double stopping_percentage = ((double)total_cells_mapped/(total_free_space)) * 100;
			if(stopping_percentage>=99.0)
				break;
			
			// Can comment out the below line for epsilon
			// epsilon = 1.0;
			if(drawn_number <= epsilon)
			{
				// Adopt a topolgical exploration strategy
				// auto current_path = top_explore.getNonHomologousPaths(current_x,current_y,{});
				// if(previously_chosen==1)
				// {
				// 	std::default_random_engine generator;
            	// 	std::vector<int> quadrant_weights = top_explore.get_quadrant_vector();
            	// 	std::discrete_distribution<int> distribution(quadrant_weights.begin(),quadrant_weights.end());
            	// 	int quadrant_select_index = distribution(generator);//rand() % 4;

            	// 	std::vector<std::pair<int,int>> dest_points = top_explore.get_destination_point(quadrant_select_index);
            	// 	int random_index = rand() % dest_points.size();
            	// 	top_explore.current_goal = {dest_points[random_index].first,dest_points[random_index].second};
            	// 	top_explore.current_goal_quadrant = quadrant_select_index;
				// }
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
					// t+=1;
					// epsilon = 1.0 - ((double)total_cells_mapped/(total_free_space) + pow(2.71828,0.01*t)/(total_free_space));//pow(2.71828,-0.02*t);
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
					{
						if(depth_result)
						{
							auto error_result = getError();
							std::stringstream ss_error;
							for(int k=0;k<error_result.size();++k)
							{
								ss_error << error_result[k] << " ";
							}
							f << timesteps_taken << " " << ((double)total_cells_mapped/(total_free_space)) * 100<<" "<<ss_error.str()<<"\n";
							if(depth_result_visualize)
							{
								if(timesteps_taken%depth_result_visualize_timestep ==0)
								{
									for(int j=0;j<depths.size();++j)
									{
										std::vector<std::vector<float>> current_depth_result = get_current_depth_result(j);
										std::stringstream depth_ss;
										for(int k=0;k<current_depth_result.size();++k)
										{
											for(int l=0;l<current_depth_result[0].size();++l)
											{
												depth_ss << current_depth_result[k][l] <<",";
											}
										}
										depth_ss<<"\n";
										depth_file<<depth_ss.str();
									}
									depth_file<<"-\n";
								}
							}
						}
						else
						{
							f << timesteps_taken << " " << ((double)total_cells_mapped/(total_free_space)) * 100<<"\n";
						}
					}
					if(use_window)
						fw.render_screen(grid);
					if((i+1)< p.size() && grid_original[p[i+1].first][p[i+1].second] ==0)
					{
						grid[p[i+1].first][p[i+1].second] = 0;
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
            
					std::default_random_engine generator;
            		std::vector<int> quadrant_weights = top_explore.get_quadrant_vector();
            		std::discrete_distribution<int> distribution(quadrant_weights.begin(),quadrant_weights.end());
            		int quadrant_select_index = distribution(generator);//rand() % 4;
					int next_quadrant_index = quadrant_select_index;
            		std::vector<std::pair<int,int>> dest_points = top_explore.get_destination_point(quadrant_select_index);
            		int random_index = rand() % dest_points.size();
            		top_explore.current_goal = {dest_points[random_index].first,dest_points[random_index].second};
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
				previously_chosen = 0;

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
				for(int i =0;i<top_explore.current_path_index;++i)
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
					{
						if(depth_result)
						{
							auto error_result = getError();
							std::stringstream ss_error;
							for(int k=0;k<error_result.size();++k)
							{
								ss_error << error_result[k] << " ";
							}
							f << timesteps_taken << " " << ((double)total_cells_mapped/(total_free_space)) * 100<<" "<<ss_error.str()<<"\n";
							if(depth_result_visualize)
							{
								if(timesteps_taken%depth_result_visualize_timestep ==0)
								{
									for(int j=0;j<depths.size();++j)
									{
										std::vector<std::vector<float>> current_depth_result = get_current_depth_result(j);
										std::stringstream depth_ss;
										for(int k=0;k<current_depth_result.size();++k)
										{
											for(int l=0;l<current_depth_result[0].size();++l)
											{
												depth_ss << current_depth_result[k][l] <<",";
											}
										}
										depth_ss<<"\n";
										depth_file<<depth_ss.str();
									}
									depth_file<<"-\n";
								}
							}
						}
						else
						{
							f << timesteps_taken << " " << ((double)total_cells_mapped/(total_free_space)) * 100<<"\n";
						}
					}
					if(use_window)
					{
						fw.render_screen(grid);
						SDL_Delay(500);
					}
					// if(grid[path[path.size()-1].first][path[path.size()-1].second] != -1 && grid[path[path.size()-1].first][path[path.size()-1].second] != 3)
					// 	break;
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
				previously_chosen = 1;	
			}
			t+=1;
			// Could change the decay function here.
			// epsilon = 1.0 - ((double)total_cells_mapped/(total_free_space) + pow(2.71828,0.01*t)/(total_free_space)); //pow(2.71828,-0.02*t);
			epsilon = 1.0 - ((double)total_cells_mapped/total_free_space);
			// f_explore.findFrontiers(current_x, current_y);
			// if(f_explore.frontiers.size() <=0)
			// {
			// 	break;
			// }
			if(t>=1000)
				break;
		}
		std::cout<<"Done Exploration"<<std::endl;
		f.close();

	}

	void random_walk_explore(std::vector<int> start)
	{
		int current_x = start[0] ;
		int current_y = start[1];

		if(obstacle_id_grid[current_x][current_y] == 0)
		{
			std::cout << "Robot is in obstacle" << std::endl;
			return;
		}

		std::fstream f;
		std::fstream depth_file;
		if(!use_window)
		{	
			f.open(output_file_name, std::ios::app);
			f << "-\n";
		}
		if(depth_result_visualize)
		{
			depth_file.open(depth_result_file_prefix,std::ios::app);
			depth_file<<"-\n";
		}

		sensor_model(current_x, current_y);
		if(!use_window)
		{
			if(depth_result)
			{
				auto error_result = getError();
				std::stringstream ss_error;
				for(int k=0;k<error_result.size();++k)
				{
					ss_error << error_result[k] << " ";
				}
				f << timesteps_taken << " "<< ((double)total_cells_mapped/(total_free_space)) * 100<<" "<<ss_error.str()<<"\n";
				if(depth_result_visualize)
				{
					if(timesteps_taken%depth_result_visualize_timestep ==0)
					{
						for(int j=0;j<depths.size();++j)
						{
							std::vector<std::vector<float>> current_depth_result = get_current_depth_result(j);
							std::stringstream depth_ss;
							for(int k=0;k<current_depth_result.size();++k)
							{
								for(int l=0;l<current_depth_result[0].size();++l)
								{
									depth_ss << current_depth_result[k][l] <<",";
								}
							}
							depth_ss<<"\n";
							depth_file<<depth_ss.str();
						}
						depth_file<<"-\n";
					}
					
				}
			}
		}

		RandomWalk rw(&grid,&obstacles_seen);
		// FrontierExplore f_explore(&grid,&obstacles_seen);
		// f_explore.findFrontiers(current_x, current_y);
		std::vector<int> destination = rw.getDestinationPoint();
		while( destination.size() > 0)
		{
			double stopping_percentage = ((double)total_cells_mapped/(total_free_space)) * 100;
			if(stopping_percentage>=99.0)
				break;
			
			std::vector<std::pair<int, int>> path = rw.getPath(current_x, current_y,destination[0], destination[1]);
			std::cout << "Random Walk Exploration Path Size = " << path.size() << std::endl;
			for (int i = 0; i < path.size(); ++i)
			{
				timesteps_taken +=1;
				grid[current_x][current_y] = 1;
				current_x = path[i].first;
				current_y = path[i].second;
				grid[current_x][current_y] = 2;
				sensor_model(current_x,current_y);
				if(!use_window)
				{
					if(depth_result)
					{
						auto error_result = getError();
						std::stringstream ss_error;
						for(int k=0;k<error_result.size();++k)
						{
							ss_error << error_result[k] << " ";
						}
						f << timesteps_taken << " "<< ((double)total_cells_mapped/(total_free_space)) * 100<<" "<<ss_error.str()<<"\n";
						if(depth_result_visualize)
						{
							if(timesteps_taken%depth_result_visualize_timestep ==0)
							{
								for(int j=0;j<depths.size();++j)
								{
									std::vector<std::vector<float>> current_depth_result = get_current_depth_result(j);
									std::stringstream depth_ss;
									for(int k=0;k<current_depth_result.size();++k)
									{
										for(int l=0;l<current_depth_result[0].size();++l)
										{
											depth_ss << current_depth_result[k][l] <<",";
										}
									}
									depth_ss<<"\n";
									depth_file<<depth_ss.str();
								}
								depth_file<<"-\n";
							}
							
						}
					}
					else
					{
						f << timesteps_taken << " " << ((double)total_cells_mapped/(total_free_space)) * 100<<"\n";
					}
				}
				if(use_window)
				{
					fw.render_screen(grid);
					SDL_Delay(500);
				}
				bool towards_an_obstacle = false;
			
				if(((i+1)<path.size() && grid_original[path[i+1].first][path[i+1].second]==0)  || towards_an_obstacle)
				{
					grid[path[i+1].first][path[i+1].second] = 0;
					break;
				}
			}
			destination = rw.getDestinationPoint();
			// f_explore.findFrontiers(current_x, current_y);
		}
		f.close();
		depth_file.close();
		std::cout<<"Exploration complete"<<std::endl;
	}

	void create_ground_truth_resolutions(int g_size)
	{
		for(int i=0;i<depths.size();++i)
		{
			std::vector<std::vector<int>> current_depth_ground_truth;
			int current_depth = depths[i];
			current_depth_ground_truth.resize(current_depth,std::vector<int>(current_depth,0));

			int x_increment = (int) g_size/current_depth;
			int y_increment = (int) g_size/current_depth;

			for(int j=0;j<current_depth;++j)
			{
				for(int k=0;k<current_depth;++k)
				{
					int x = j*x_increment;
					int y = k*y_increment;
					for(int l=x ; l < (j+1)*x_increment; ++l)
					{
						for(int m=y ; m < (k+1)*y_increment; ++m)
						{
							if(grid_original[l][m] == 0)
							// if(grid_original_only_boundary[l][m] == 0)
							{
								current_depth_ground_truth[j][k] = 1;
								break;
							}
						}
						if (current_depth_ground_truth[j][k]==1)
						{
							break;
						}	
					}
					
				}
			}
			ground_truth_depth_result.push_back(current_depth_ground_truth);
		}
	}
	
	std::vector<double> getError(bool get_weighted_error = true)
	{
		std::vector<double> error;
		std::vector<double> unknown_cell_count;
		if(get_weighted_error==false)
		{
			for(int i=0;i<depths.size();++i)
			{
				int current_depth = depths[i];
				int x_increment = (int) grid_size/current_depth;
				int y_increment = (int) grid_size/current_depth;
				
				double error_count = 0;

				for(int j=0;j<current_depth;++j)
				{
					for(int k=0;k<current_depth;++k)
					{
						int x = j*x_increment;
						int y = k*y_increment;
						
						if(ground_truth_depth_result[i][j][k] == 0)
							continue;
						
						bool obstacle_there = false;
						for(int l=x ; l < (j+1)*x_increment; ++l)
						{
							for(int m=y ; m < (k+1)*y_increment; ++m)
							{
								if(grid[l][m] == 0)
								{
									obstacle_there = true;
									break;
								}
							}
							if (obstacle_there)
							{
								break;
							}	
						}
						if (!obstacle_there)
						{
							error_count+=1;
						}
					}
				}
				double u_cell_count = 0;
				for(int j=0;j<current_depth;++j)
				{
					for(int k=0;k<current_depth;++k)
					{
						int x = j*x_increment;
						int y = k*y_increment;
						
						bool isunknown  = true;
						for(int l=x ; l < (j+1)*x_increment; ++l)
						{
							for(int m=y ; m < (k+1)*y_increment; ++m)
							{
								if(grid[l][m] == 0 || grid[l][m]==1)
								{
									isunknown = false;
									break;
								}
							}
							if (!isunknown)
							{
								break;
							}	
						}
						if (isunknown)
						{
							u_cell_count+=1;
						}
					}
				}

				error.push_back(error_count);
				unknown_cell_count.push_back(u_cell_count);
			}
		}
		else
		{
			for(int i=0;i<depths.size();++i)
			{
				int current_depth = depths[i];
				int x_increment = (int) grid_size/current_depth;
				int y_increment = (int) grid_size/current_depth;
				
				double error_count = 0;

				for(int j=0;j<current_depth;++j)
				{
					for(int k=0;k<current_depth;++k)
					{
						int x = j*x_increment;
						int y = k*y_increment;
						
						if(ground_truth_depth_result[i][j][k] == 0)
							continue;
						
						double ground_truth_probability = 0 ;
						double mapped_cells = 0;
						double predicted_probability = 0;
						
						for(int l=x ; l < (j+1)*x_increment; ++l)
						{
							for(int m=y ; m < (k+1)*y_increment; ++m)
							{
								if(grid[l][m] == 0)
								{
									predicted_probability+=1;
								}
								else if(grid[l][m]==1)
								{
									mapped_cells+=1;
								}
								if(grid_original[l][m] == 0)
								{
									ground_truth_probability+=1;
								}
							}
						}
						ground_truth_probability = ground_truth_probability/((double)(x_increment*y_increment));
						predicted_probability = predicted_probability/((mapped_cells+ predicted_probability));
						error_count+= abs(ground_truth_probability - std::min(2.0,predicted_probability));
					}
				}
				double u_cell_count = 0;
				for(int j=0;j<current_depth;++j)
				{
					for(int k=0;k<current_depth;++k)
					{
						int x = j*x_increment;
						int y = k*y_increment;
						
						bool isunknown  = true;
						for(int l=x ; l < (j+1)*x_increment; ++l)
						{
							for(int m=y ; m < (k+1)*y_increment; ++m)
							{
								if(grid[l][m] == 0 || grid[l][m]==1)
								{
									isunknown = false;
									break;
								}
							}
							if (!isunknown)
							{
								break;
							}	
						}
						if (isunknown)
						{
							u_cell_count+=1;
						}
					}
				}

				error.push_back(error_count);
				unknown_cell_count.push_back(u_cell_count);
			}
		}

		for(int i=0;i<unknown_cell_count.size();++i)
			error.push_back(unknown_cell_count[i]);
		return error;
	}

	std::vector<std::vector<float>> get_current_depth_result(int depth_index)
	{
		std::vector<std::vector<float>> ret_result(grid_size,std::vector<float>(grid_size,-1));

		int current_depth = depths[depth_index];
		int x_increment = (int) grid_size/current_depth;
		int y_increment = (int) grid_size/current_depth;
		
		for(int i=0;i<current_depth;++i)
		{
			for(int j=0;j<current_depth;++j)
			{
				int x = i*x_increment;
				int y = j*y_increment;
						
				bool unknown_space_only = true;
				for(int l=x ; l < (i+1)*x_increment; ++l)
				{
					for(int m=y ; m < (j+1)*y_increment; ++m)
					{
						if(grid[l][m]!=-1)
						{
							unknown_space_only = false;
							break;
						}
					}
					if (!unknown_space_only)
					{
						break;
					}	
				}
				if(unknown_space_only)
				{
					for(int l=x ; l < (i+1)*x_increment; ++l)
					{
						for(int m=y ; m < (j+1)*y_increment; ++m)
						{
							ret_result[l][m] = -1.0;
						}
					}
				}
				else
				{
					int unknown_cell_count  = 0;
					int obstacle_cell_count = 0;
					int free_cell_count = 0;
					bool obstacle_present = false;
					if(!write_probability_depth_map)
					{
						for(int l=x ; l < (i+1)*x_increment; ++l)
						{
							for(int m=y ; m < (j+1)*y_increment; ++m)
							{
								if(grid[l][m]==0)
								{
									obstacle_present=true;
									break;
								}
							}
							if (obstacle_present)
							{
								break;
							}	
						}
						for(int l=x ; l < (i+1)*x_increment; ++l)
						{
							for(int m=y ; m < (j+1)*y_increment; ++m)
							{
								if(obstacle_present==false)
								{
									ret_result[l][m] = 1;
								}
								else
								{
									ret_result[l][m] = 0;
								}
								
							}
							
						}
					}
					else
					{
						obstacle_cell_count = 0;
						free_cell_count = 0;
						unknown_cell_count = 0;
						
						for(int l=x ; l < (i+1)*x_increment; ++l)
						{
							for(int m=y ; m < (j+1)*y_increment; ++m)
							{
								if(grid[l][m]==0)
								{
									obstacle_cell_count+=1;
								}
								else if(grid[l][m]==1)
								{
									free_cell_count+=1;
								}
								else
								{
									unknown_cell_count+=1;
								}
							}	
						}
						for(int l=x ; l < (i+1)*x_increment; ++l)
						{
							for(int m=y ; m < (j+1)*y_increment; ++m)
							{
								// if(grid[l][m]==0)
								// {
								// 	ret_result[l][m] = 1;
								// }
								// else if(grid[l][m]==1)
								// {
								// 	ret_result[l][m] = 0;
								// }
								// else
								// {
								// 	ret_result[l][m] = (double)obstacle_cell_count/(obstacle_cell_count+free_cell_count);
								// }
								ret_result[l][m] = (float)obstacle_cell_count/(obstacle_cell_count+free_cell_count);
							}	
						}

					}
					
				}
			}
		}

		return ret_result;
	}

	void render_ground_truth()
	{
		while(1)
		{
			fw.render_screen(grid_original);
		}
	}
	Framework fw;
	bool write_probability_depth_map = true;


private:
	int grid_size;
	std::vector<std::vector<int>> grid;
	std::vector<std::vector<int>> grid_original;
	std::vector<std::vector<int>> grid_original_only_boundary;

	std::vector<std::vector<int>> obstacle_id_grid;
	std::vector<std::vector<int>> obstacles;
	std::vector<std::vector<int>> obstacles_seen;


	std::vector<std::vector<std::vector<int>>> ground_truth_depth_result;
	std::vector<std::vector<std::vector<int>>> ground_truth_depth_weight_result;

	std::vector<int> depths = {2,4,8,16,32,64};
	
	int num_obstacles;
	int total_cells_mapped=0;
	int lidar_range = 4;
	double scale = 10.0;
	bool use_window = true;
	std::string output_file_name;
	int timesteps_taken = 0;
	int SENSOR_RANGE = 8;
	int total_free_space = 0;
	bool depth_result = false;
	bool depth_result_visualize = false;
	int  depth_result_visualize_timestep = 200;
	std::string depth_result_file_prefix = "obs0";
	int square_obstacle_size = 15;

};

int main(int argc, char *argv[])
{
	int choice = 5;
	bool use_window = false;
	if(choice==0)
	{
		int image_snapshot_time = 500;
		int topology_num_runs = 5;
		std::string obstacle_file = "obs_256_5.txt";
		std::string result_file = "result_"+obstacle_file;
		std::string frontier_depth_file = "result_"+obstacle_file+"_frontier.txt";
		std::string topo_depth_file = "result_" + obstacle_file+"_topo.txt";
		std::string time_result_file = "result_" + obstacle_file+"_time.txt";
		srand(time(NULL));

		// Robot robot(60,600.0,10.0,25,use_window,result_file,obstacle_file,16,true,true,100,frontier_depth_file);
		// Robot robot(60,600.0,10.0,100,use_window,result_file,obstacle_file,16,true,true,100,frontier_depth_file);
		Robot robot(256, 768,3.0, 128,use_window,result_file,obstacle_file,32,true,false,image_snapshot_time,frontier_depth_file);
		auto frontier_start = high_resolution_clock::now();
		robot.start_exploring(0, 0);
		auto frontier_stop = high_resolution_clock::now();
		auto duration_frontier = duration_cast<seconds>(frontier_stop - frontier_start);
		
		int topological_seconds = 0;
		for(int i=0;i<topology_num_runs;++i)
		{
			// robot = Robot(60, 600.0,10.0,25,use_window,result_file,obstacle_file,16,true,false,100,topo_depth_file);
			// robot = Robot(60, 600.0,10.0,100,use_window,result_file,obstacle_file,16,true,false,100,topo_depth_file);
			robot = Robot(256, 768,3.0,128,use_window,result_file,obstacle_file,32,true,false,image_snapshot_time,topo_depth_file);
			auto topo_start = high_resolution_clock::now();
			robot.topological_explore_4({0,0});
			auto topo_stop = high_resolution_clock::now();
			topological_seconds += duration_cast<seconds>(topo_stop - topo_start).count();
			SDL_Delay(1000);
		}

		double result_topo = (double)topological_seconds/((double) topology_num_runs);
		// robot = Robot(60, 600.0,10.0,25,use_window,result_file,obstacle_file,16,true,true,100,topo_depth_file);
		// robot = Robot(60, 600.0,10.0,100,use_window,result_file,obstacle_file,16,true,true,100,topo_depth_file);
		// robot = Robot(256, 768,3.0,128,use_window,result_file,obstacle_file,32,true,true,image_snapshot_time,topo_depth_file);
		// robot.topological_explore_4({0,0});

		std::ofstream MyFile(time_result_file);
  		MyFile << "Time taken by Frontier Exploration: "
		 << duration_frontier.count() << " seconds" << std::endl;
		 MyFile << "Time taken by Topological Exploration: "
		 << result_topo << " seconds" << std::endl;
  		MyFile.close();
		
		std::cout << "Time taken by Frontier Exploration: "
		 << duration_frontier.count() << " seconds" << std::endl;
		std::cout << "Time taken by Topological Exploration: "
		 << result_topo << " seconds" << std::endl;
	}
	else if(choice == -1)
	{
		int image_snapshot_time = 300;
		int topology_num_runs = 5;
		int obstacle_count = 72;
		std::string obstacle_file = "multimodal_gaussian_15_16.txt";
		std::string result_file = "m_result_"+obstacle_file;
		std::string frontier_depth_file = "m_result_"+obstacle_file+"_frontier.txt";
		std::string topo_depth_file = "m_result_" + obstacle_file+"_topo.txt";
		std::string rw_depth_file = "m_result_" + obstacle_file+"_rw.txt";
		std::string time_result_file = "m_result_" + obstacle_file+"_time.txt";
		srand(time(NULL));

		// Robot robot(60,600.0,10.0,25,use_window,result_file,obstacle_file,16,true,true,100,frontier_depth_file);
		// Robot robot(60,600.0,10.0,100,use_window,result_file,obstacle_file,16,true,true,100,frontier_depth_file);
		Robot robot(256, 768,3.0, obstacle_count,use_window,result_file,obstacle_file,32,true,false,image_snapshot_time,frontier_depth_file);
		auto frontier_start = high_resolution_clock::now();
		robot.start_exploring(0, 0);
		auto frontier_stop = high_resolution_clock::now();
		auto duration_frontier = duration_cast<seconds>(frontier_stop - frontier_start);
		
		int topological_seconds = 0;
		for(int i=0;i<topology_num_runs;++i)
		{
			// robot = Robot(60, 600.0,10.0,25,use_window,result_file,obstacle_file,16,true,false,100,topo_depth_file);
			// robot = Robot(60, 600.0,10.0,100,use_window,result_file,obstacle_file,16,true,false,100,topo_depth_file);
			robot = Robot(256, 768,3.0, obstacle_count,use_window,result_file,obstacle_file,32,true,false,image_snapshot_time,topo_depth_file);
			auto topo_start = high_resolution_clock::now();
			robot.topological_explore_4({0,0});
			auto topo_stop = high_resolution_clock::now();
			topological_seconds += duration_cast<seconds>(topo_stop - topo_start).count();
			SDL_Delay(1000);
		}

		double result_topo = (double)topological_seconds/((double) topology_num_runs);
		// robot = Robot(60, 600.0,10.0,25,use_window,result_file,obstacle_file,16,true,true,100,topo_depth_file);
		// robot = Robot(60, 600.0,10.0,100,use_window,result_file,obstacle_file,16,true,true,100,topo_depth_file);
		// robot = Robot(256, 768,3.0,128,use_window,result_file,obstacle_file,32,true,true,image_snapshot_time,topo_depth_file);
		// robot = Robot(256, 768,3.0, obstacle_count,use_window,result_file,obstacle_file,32,true,true,image_snapshot_time,topo_depth_file);
		// robot.topological_explore_4({0,0});
		int random_walk_seconds = 0;
		for(int i=0;i<topology_num_runs;++i)
		{
			// robot = Robot(60, 600.0,10.0,25,use_window,result_file,obstacle_file,16,true,false,100,topo_depth_file);
			// robot = Robot(60, 600.0,10.0,100,use_window,result_file,obstacle_file,16,true,false,100,topo_depth_file);
			robot = Robot(256, 768,3.0, obstacle_count,use_window,result_file,obstacle_file,32,true,false,image_snapshot_time,rw_depth_file);
			auto rw_start = high_resolution_clock::now();
			robot.random_walk_explore({0,0});
			auto rw_stop = high_resolution_clock::now();
			random_walk_seconds += duration_cast<seconds>(rw_stop - rw_start).count();
			SDL_Delay(1000);
		}
		
		// robot = Robot(256, 768,3.0, obstacle_count,use_window,result_file,obstacle_file,32,true,true,image_snapshot_time,rw_depth_file);
		// robot.random_walk_explore({0,0});
		double result_rw = (double)random_walk_seconds/((double) topology_num_runs);

		std::ofstream MyFile(time_result_file);
  		MyFile << "Time taken by Frontier Exploration: "
		 << duration_frontier.count() << " seconds" << std::endl;
		 MyFile << "Time taken by Topological Exploration: "
		 << result_topo << " seconds" << std::endl;
		 MyFile << "Time taken by Random Walk: "
		 << result_rw << " seconds" << std::endl;
  		MyFile.close();
		
		std::cout << "Time taken by Frontier Exploration: "
		 << duration_frontier.count() << " seconds" << std::endl;
		std::cout << "Time taken by Topological Exploration: "
		 << result_topo << " seconds" << std::endl;
		std::cout << "Time taken by Random Walk Explorartion: "
		 << result_rw << " seconds" << std::endl;

	}
	else if(choice == -2)
	{
		use_window = false;
		int image_snapshot_time = 300;
		int topology_num_runs = 5;
		int obstacle_count = 60;
		std::string obstacle_file = "obs_256_15_3.txt";
		std::string result_file = "visual_result_"+obstacle_file;
		std::string frontier_depth_file = "visual_result_"+obstacle_file+"_frontier.txt";
		std::string topo_depth_file = "visual_result_" + obstacle_file+"_topo.txt";
		std::string rw_depth_file = "visual_result_" + obstacle_file+"_rw.txt";
		std::string time_result_file = "visual_result_" + obstacle_file+"_time.txt";
		srand(time(NULL));

		// Robot robot(60,600.0,10.0,25,use_window,result_file,obstacle_file,16,true,true,100,frontier_depth_file);
		// Robot robot(60,600.0,10.0,100,use_window,result_file,obstacle_file,16,true,true,100,frontier_depth_file);
		Robot robot(256, 768,3.0, obstacle_count,use_window,result_file,obstacle_file,32,true,true,image_snapshot_time,frontier_depth_file);
		robot.write_probability_depth_map = true;
		robot.start_exploring(0, 0);
		robot = Robot(256, 768,3.0, obstacle_count,use_window,result_file,obstacle_file,32,true,true,image_snapshot_time,topo_depth_file);
		robot.write_probability_depth_map = true;
		robot.topological_explore_4({0,0});
		robot = Robot(256, 768,3.0, obstacle_count,use_window,result_file,obstacle_file,32,true,true,image_snapshot_time,rw_depth_file);
		robot.write_probability_depth_map = true;
		robot.random_walk_explore({0,0});			

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
				Robot robot(60, 600,10.0, 20,use_window,"result_frontiers.txt",obstacle_file_name,sensor_ranges[i]);
				robot.start_exploring(0,0);
				//robot.start_exploring({0,0});
				SDL_Delay(1000);
			}
		}
	}
	else if(choice == 2)
	{
		srand(time(NULL));
		std::vector<int> sensor_ranges = {4,8,12,16,32};
		for(int i=0;i<sensor_ranges.size();++i)
		{
			for(int j=0;j<10;++j)
			{
				std::string obstacle_file_name = "obs" + std::to_string(j) + ".txt"; 
				for(int k =0; k < 10; ++k)
				{
					Robot robot(60, 600,10.0, 20,use_window,"result_topology.txt",obstacle_file_name,sensor_ranges[i]);
					robot.topological_explore_4({0,0});
					SDL_Delay(1000);
				}
			
			}
		}
	}
	else if(choice==3)
	{
		use_window = true;
		Robot robot(60,600,10.0,100,use_window,"result_obs0.txt","obs0.txt",16,false,false,200,"dobs0.txt");
		// Robot robot(256, 768 ,3.0, 128,use_window,"result_256_0.txt","obs_256_0.txt",64,true,false,300,"dobs0_256_0_topo.txt");
		//robot.topological_explore_4({0,0});
		robot.start_exploring(0,0);
		// ;
	}
	else if(choice == 4)
	{
		;
	}
	else if(choice==5)
	{
		use_window = true;
		int image_snapshot_time = 500;
		int topology_num_runs = 5;
		std::string obstacle_file = "maze_256_10_2";
		std::string result_file = "result_"+obstacle_file;
		std::string frontier_depth_file = "result_"+obstacle_file+"_frontier.txt";
		std::string topo_depth_file = "result_" + obstacle_file+"_topo.txt";
		std::string time_result_file = "result_" + obstacle_file+"_time.txt";
		srand(time(NULL));

		Robot robot(256, 512,2.0, 90,use_window,"result_obs_256_4.txt","multimodal_gaussian_15_18.txt",32,true,false,300,"dobs1_frontier.txt");
		robot.render_ground_truth();
		

	}
	else if(choice == 6)
	{
		use_window = true;
		Robot robot(60, 600,10.0, 25,use_window,"result_obs_256_4.txt","obs0.txt",16,true,false,100,"dobs4_frontier.txt");
		robot.random_walk_explore({0,0});
	}
	else
	{
		std::string obstacle_file = "multimodal_gaussian_1.txt";
		std::string result_file = "result_"+obstacle_file;
		std::string frontier_depth_file = "d"+obstacle_file+"_frontier.txt";
		std::string topo_depth_file = "d"+obstacle_file+"_topo.txt";
		Robot robot(256, 768,3.0, 128,use_window,result_file,obstacle_file,32,true,true,100,frontier_depth_file);
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