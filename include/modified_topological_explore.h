
#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <vector>
#include <iostream>
#include <algorithm>
#include <queue>
#include <Eigen/Dense>
#include <complex>
#include <unordered_map>
#include <set>
#include <random>
#include <map>

class ModifiedTopolgicalExplore
{
    public:
        struct AstarNode
        {
            std::complex<double> point;
            Eigen::VectorXd h_signature;
            double f;
            double g;
            double cost;
            struct AstarNode *parent;
            std::vector<std::complex<double>> edge;

            bool operator==(const AstarNode &node) const
            {
                auto difference_h = h_signature - node.h_signature;
                bool difference_result = false;
                if(difference_h.isZero(0.001))
                {
                    difference_result = true;
                }
                return (point.real() == node.point.real()) && (point.imag() == node.point.imag()) && difference_result;
            }

            AstarNode(std::complex<double> p, Eigen::VectorXd h, double f_,double g_,struct AstarNode *pa, std::vector<std::complex<double>> e) : point(p), h_signature(h), f(f_),g(g_), parent(pa), edge(e) {}
            AstarNode(std::complex<double> p, Eigen::VectorXd h, double c_,struct AstarNode *pa, std::vector<std::complex<double>> e) : point(p), h_signature(h), cost(c_), parent(pa), edge(e) {}
        };

        struct AstarKeyHash
        {
            std::size_t operator()(const AstarNode &node) const
            {
                std::size_t h_hash = 0;
                for(unsigned int i=0;i<node.h_signature.size();++i)
                {
                    h_hash = h_hash ^ std::hash<double>()(node.h_signature(i));
                }
                return std::hash<double>()(node.point.real()) ^ std::hash<double>()(node.point.imag()) ^ h_hash;
            }
        };


        ModifiedTopolgicalExplore(std::vector<std::vector<int>> *g, std::vector<std::vector<int>> *o, std::vector<int> start, std::vector<std::vector<int>> *g_o ,bool r_explored=false) : grid(g), obstacles_seen(o), start_coordinates(start),grid_original(g_o),remove_explored_obstacles(r_explored)
        {
            current_start = {start[0],start[1]};
            std::vector<std::vector<int>> &grid_ref = *grid;
            boundary_points_path = getBoundaryPaths(grid_ref.size());
            int quadrant = -1;
            for(int i=0;i<boundary_points_path.size();++i)
            {
                for(int j = 0; j < boundary_points_path[i].size();++j)
                {
                    if(current_start[0] == boundary_points_path[i][j][0].first && current_start[1] == boundary_points_path[i][j][0].second)
                    {
                        quadrant = i;
                        break;
                    }
                }
                if(quadrant != - 1)
                    break;
            }
            current_start_quadrant = quadrant;

            std::default_random_engine generator;
            std::vector<int> quadrant_weights = get_quadrant_vector();
            std::discrete_distribution<int> distribution(quadrant_weights.begin(),quadrant_weights.end());
            int quadrant_select_index = distribution(generator);//rand() % 4;

            std::vector<std::pair<int,int>> dest_points = get_destination_point(quadrant_select_index);
            
            int random_index = rand() % dest_points.size();
            current_goal = {dest_points[random_index].first,dest_points[random_index].second};
            // switch (quadrant_select_index)
            // {
            //     case 0:
            //         random_index = rand() % (grid->size() - 1);
            //         current_goal = {0,random_index};
            //         break;
            //     case 1:
            //         random_index = rand() % (grid->size() - 1);
            //         current_goal = {random_index , (int)grid->size()-1};
            //         break;
            //     case 2:
            //         random_index = rand() % (grid->size() - 1)+1;
            //         current_goal = {(int)grid->size()-1, random_index};
            //         break;
            //     case 3 : 
            //         random_index = rand() % (grid->size() - 1) + 1;
            //         current_goal = {random_index , 0};
            //         break;
            //     default:
            //         random_index = rand() % (grid->size() - 1) + 1;
            //         current_goal = {random_index , 0};
            //         break;
            // }
            current_goal_quadrant = quadrant_select_index;
            // std::cout << "-----\nQuadrant checking----\n" << std::endl;
            // std::cout << quadrant << std::endl;
            // std::cout << quadrant_select_index << "--" << current_goal[0] << "," << current_goal[1] << std::endl; 
            
            current_path_index = 0;
            traversed_paths.clear();
            current_path.clear();
            traversed_signatures.clear();
        }


        std::vector<std::vector<std::vector<std::pair<int,int>>>> getBoundaryPaths(int grid_size)
        {
            std::vector<std::vector<std::vector<std::pair<int,int>>>> v;
            for(int i=0;i<4;++i)
            {
                std::vector<std::vector<std::pair<int,int>>> current_quadrant_paths;
                if(i==0)
                {
                    for(int j=0;j<grid_size-1;++j)
                    {
                        std::vector<std::pair<int,int>> current_path;
                        current_path.push_back({0,j});
                        for(int k = j; k>=-1;k--)
                        {
                            current_path.push_back({-1,k});
                        }
                        current_path.push_back({0,0});
                        current_quadrant_paths.push_back(current_path);
                    }
                }
                else if(i==1)
                {
                    for(int j=0;j<grid_size-1; ++ j)
                    {
                        std::vector<std::pair<int,int>> current_path;
                        current_path.push_back({j,grid_size-1});
                        for(int k = j; k>=-1;k--)
                        {
                            current_path.push_back({k,grid_size});
                        }
                        for(int k = grid_size; k>=-1;k--)
                        {
                            current_path.push_back({-1,k});
                        }
                        current_path.push_back({0,0});
                        current_quadrant_paths.push_back(current_path);
                    }
                }
                else if(i==2)
                {
                    for(int j=1;j<grid_size; ++ j)
                    {
                        std::vector<std::pair<int,int>> current_path;
                        current_path.push_back({grid_size-1,j});
                        for(int k = j; k<=grid_size;k++)
                        {
                            current_path.push_back({grid_size,k});
                        }
                        for(int k=grid_size;k>=-1;k--)
                        {
                            current_path.push_back({k,grid_size});
                        }
                        for(int k = grid_size-1; k>=-1;k--)
                        {
                            current_path.push_back({-1,k});
                        }
                        current_path.push_back({0,0});
                        current_quadrant_paths.push_back(current_path);
                    }

                }
                else
                {
                    for(int j=1;j<grid_size; ++ j)
                    {
                        std::vector<std::pair<int,int>> current_path;
                        current_path.push_back({j,0});
                        
                        
                        for(int k = j; k<=grid_size;k++)
                        {
                            current_path.push_back({k,-1});
                        }
                        for(int k=-1;k<=grid_size;k++)
                        {
                            current_path.push_back({grid_size,k});
                        }
                        for(int k=grid_size;k>=-1;k--)
                        {
                            current_path.push_back({k,grid_size});
                        }
                        for(int k = grid_size-1; k>=-1;k--)
                        {
                            current_path.push_back({-1,k});
                        }

                        current_path.push_back({0,0});
                        current_quadrant_paths.push_back(current_path);
                    }

                }
                v.push_back(current_quadrant_paths);
            }
            return v;
        }

        std::vector<std::pair<int, int>> getNeighbours(int x, int y, int max_x, int max_y)
        {
            std::vector<std::pair<int, int>> neighbours;
            int x_dir[] = {0, 0, 1, -1, 1, 1, -1, -1};
            int y_dir[] = {1, -1, 0, 0, 1, -1, 1, -1};
            for (int i = 0; i < 8; ++i)
            {
                int x_new = x + x_dir[i];
                int y_new = y + y_dir[i];
                if (x_new < 0 || x_new >= max_x || y_new < 0 || y_new >= max_y)
                    continue;
                else
                    neighbours.push_back({x_new, y_new});
            }
            return neighbours;
        }

        bool getNonHomologousPaths(int x, int y, std::vector<Eigen::VectorXd> visited_h_signatures)
        {
            // struct NonHomologouspath p;

            auto customOp = [](const std::complex<double> &a, const std::complex<double> &b) -> double
            {
                double minimum_phase_difference = std::arg(b) - std::arg(a);
                for (int i = -2; i < 3; ++i)
                {
                    for (int j = -2; j < 3; ++j)
                    {
                        double phase_difference = (std::arg(b) + 2 * M_PIf64 * i) - (std::arg(a) + 2 * M_PIf64 * j);
                        if (std::abs(phase_difference) < std::abs(minimum_phase_difference))
                        {
                            minimum_phase_difference = phase_difference;
                        }
                    }
                }
                return minimum_phase_difference;
            };
            
            std::vector<std::vector<int>> &obstacles_ref = *obstacles_seen;
            std::vector<std::vector<int>> &grid_ref = *grid;
            std::vector<std::vector<int>> obstacles_to_use;

            if(remove_explored_obstacles)
            {
                std::vector<std::vector<int>> &grid_ref = *grid;
                std::vector<std::vector<int>> &grid_ref_original = *grid_original;


                for(int  i = 0; i< obstacles_ref.size();++i)
                {
                    int current_obstacle_x = obstacles_ref[i][0];
                    int current_obstacle_y = obstacles_ref[i][1];
                    int unmapped_cell_count = 0;
                    
                    if(current_obstacle_y-1 >= 0)
                    {
                        for(int j=current_obstacle_x-1;j<current_obstacle_x + 5;++j)
                        {
                            if(j<0 || j>= grid_ref.size())
                                continue;
                            if(grid_ref[j][current_obstacle_y-1] == -1 && grid_ref_original[j][current_obstacle_y-1] != 0)
                                unmapped_cell_count+=1;
                        
                        }  
                    }
                    if(current_obstacle_y + 4 < grid_ref[0].size())
                    {
                        for(int j=current_obstacle_x-1;j<current_obstacle_x + 5;++j)
                        {
                            if(j<0 || j>= grid_ref.size())
                                continue;
                            if(grid_ref[j][current_obstacle_y+4] == -1 && grid_ref_original[j][current_obstacle_y+4] != 0)
                                unmapped_cell_count+=1;
                        }  
                    }

                    if(current_obstacle_x-1 >= 0)
                    {
                        for(int j=current_obstacle_y-1;j<current_obstacle_y + 5;++j)
                        {
                            if(j<0 || j>= grid_ref[0].size())
                                continue;
                            if(grid_ref[current_obstacle_x-1][j] == -1 && grid_ref_original[current_obstacle_x-1][j] != 0)
                                unmapped_cell_count+=1;
                        }  
                    }
                    
                    if(current_obstacle_x + 4 < grid_ref.size())
                    {
                        for(int j=current_obstacle_y-1;j<current_obstacle_y + 5;++j)
                        {
                            if(j<0 || j>= grid_ref[0].size())
                                continue;
                            if(grid_ref[current_obstacle_x+4][j] == -1 && grid_ref_original[current_obstacle_x+4][j] != 0)
                                unmapped_cell_count+=1;
                        }  
                    }

                    if(unmapped_cell_count > 0)
                        obstacles_to_use.push_back(obstacles_ref[i]);
                }
            }
            else
            {
                obstacles_to_use = obstacles_ref;
            }

            if(x== current_goal[0] && y==current_goal[1])
            {
                start_quadrants.push_back(current_start_quadrant);
                goal_quadrants.push_back(current_goal_quadrant);

                current_start = {current_goal[0],current_goal[1]};
                current_start_quadrant = current_goal_quadrant;
                
                std::default_random_engine generator;
                std::vector<int> quadrant_weights = get_quadrant_vector();
                std::discrete_distribution<int> distribution(quadrant_weights.begin(),quadrant_weights.end());
                int quadrant_select_index = distribution(generator);//rand() % 4;
                int next_quadrant_index = quadrant_select_index;               

                std::vector<std::pair<int,int>> dest_points = get_destination_point(quadrant_select_index);
            
                int random_index = rand() % dest_points.size();
                current_goal = {dest_points[random_index].first,dest_points[random_index].second};
                // int next_quadrant_index = rand() % 4;
                // int random_index = -1;
                // switch (next_quadrant_index)
                // {
                //     case 0:
                //         random_index = rand() % (grid->size() - 1);
                //         current_goal = {0,random_index};
                //         break;
                //     case 1:
                //         random_index = rand() % (grid->size() - 1);
                //         current_goal = {random_index , (int)grid->size()-1};
                //         break;
                //     case 2:
                //         random_index = rand() % (grid->size() - 1) + 1;
                //         current_goal = {(int)grid->size()-1, random_index};
                //         break;
                //     case 3 : 
                //         random_index = rand() % (grid->size() - 1) + 1;
                //         current_goal = {random_index , 0};
                //         break;
                //     default:
                //         random_index = rand() % (grid->size() - 1) + 1;
                //         current_goal = {random_index , 0};
                //         break;
                // }
                current_goal_quadrant  = next_quadrant_index;
                std::vector<std::pair<int,int>> current_path_copy;
                for(int i=0;i<current_path.size();++i)
                {
                        current_path_copy.push_back({current_path[i].first,current_path[i].second});
                }
                traversed_paths.push_back(current_path_copy);
                current_path.clear();
                current_path_index = 0;
            }
            
            traversed_signatures.clear();
            
            for(int i=0;i<traversed_paths.size();++i)
            {
                Eigen::VectorXd prev_h_signature = recompute_h_signature(traversed_paths[i]);
                Eigen::VectorXd augmented_path_signature_start = Eigen::VectorXd::Zero(obstacles_to_use.size());
                Eigen::VectorXd augmented_path_signature_goal = Eigen::VectorXd::Zero(obstacles_to_use.size());
                std::cout <<"Traversed paths size = "<< traversed_paths.size()<<std::endl;
                std::cout << "Previous H signature = " << prev_h_signature.transpose() << std::endl;
                std::cout << "Traversed_path size = " << traversed_paths[i].size() << std::endl;
                std::cout << "Current index= " << i <<std::endl;
                std::cout << "Start quadrant index = " << start_quadrants[i] << std::endl;
                std::cout << "Destination quadrant index = " << goal_quadrants[i] << std::endl;
                std::cout << "Current start = " << current_start[0] << "," << current_start[1] << std::endl;    
                std::cout << "Current goal = " << current_goal[0] << "," << current_goal[1] << std::endl;
                if(traversed_paths[i].size() == 0)
                    continue;
                
                for(int j = 0 ; j < boundary_points_path[start_quadrants[i]].size() ; ++j)
                {
                    std::cout << "Traversed path" << traversed_paths[i][0].first << ", " << traversed_paths[i][0].second << std::endl;
                    if(boundary_points_path[start_quadrants[i]][j][0].first == traversed_paths[i][0].first && boundary_points_path[start_quadrants[i]][j][0].second == traversed_paths[i][0].second)
                    {
                        std::vector<std::pair<int,int>> current_path_tmp;
                        for(int k = 0; k < boundary_points_path[start_quadrants[i]][j].size() ; ++k)
                        {
                            current_path_tmp.push_back({boundary_points_path[start_quadrants[i]][j][k].first,boundary_points_path[start_quadrants[i]][j][k].second});
                        }
                        std::reverse(current_path_tmp.begin(),current_path_tmp.end());
                        augmented_path_signature_start = recompute_h_signature(current_path_tmp);
                    }
                }


                for(int j = 0 ; j < boundary_points_path[goal_quadrants[i]].size() ; ++j)
                {
                    if(boundary_points_path[goal_quadrants[i]][j][0].first == traversed_paths[i][traversed_paths[i].size()-1].first && boundary_points_path[goal_quadrants[i]][j][0].second == traversed_paths[i][traversed_paths[i].size()-1].second)
                    {
                        std::vector<std::pair<int,int>> current_path_tmp;
                        for(int k = 0; k < boundary_points_path[start_quadrants[i]][j].size() ; ++k)
                        {
                            current_path_tmp.push_back({boundary_points_path[start_quadrants[i]][j][k].first,boundary_points_path[start_quadrants[i]][j][k].second});
                        }
                        // std::reverse(current_path_tmp.begin(),current_path_tmp.end());
                        augmented_path_signature_goal = recompute_h_signature(current_path_tmp);
                    }
                }           
                prev_h_signature =  prev_h_signature + augmented_path_signature_start + augmented_path_signature_goal;
                prev_h_signature = (prev_h_signature.array().abs() < 0.001).select(0, prev_h_signature);
                traversed_signatures.push_back(prev_h_signature);
            }


            Eigen::VectorXd partial_signature = Eigen::VectorXd::Zero(obstacles_to_use.size());
            partial_signature = recompute_h_signature(current_path,current_path_index);
            // std::cout<<"Partial signature: "<<partial_signature.transpose()<<std::endl;

        
            Eigen::VectorXcd obstacle_points = Eigen::VectorXcd::Zero(obstacles_to_use.size());
            for (unsigned int i = 0; i < obstacles_to_use.size(); ++i)
                obstacle_points(i) = std::complex<double>(obstacles_to_use[i][0], obstacles_to_use[i][1]);

            std::complex<double> start_point(x, y);
            std::vector<int> goal_coords = current_goal;
            std::complex<double> goal_point(goal_coords[0], goal_coords[1]);

            std::vector<std::complex<double>> directions = {
                std::complex<double>(1.0, 0.0),
                std::complex<double>(0.0, 1.0),
                std::complex<double>(-1.0, 0.0),
                std::complex<double>(0.0, -1.0),
                std::complex<double>(1.0, 1.0),
                std::complex<double>(-1.0, 1.0),
                std::complex<double>(1.0, -1.0),
                std::complex<double>(-1.0, -1.0),
            };

            std::priority_queue<AstarNode *, std::vector<AstarNode *>, std::function<bool(AstarNode *, AstarNode *)>> pq([](AstarNode *a, AstarNode *b)
                                                                                                                { return (a->f  +  a->g) > (b->f + b->g); });      //{ return (a->f + a->g) > (b->f + b->g); });
            std::unordered_map<std::string, double> distance_count;
            std::set<std::string> visited;
            
            double f=0.0;
            // Can comment out below for loop
            for(int i=0;i<obstacles_ref.size();++i)
            {
                int current_obstacle_x = obstacles_ref[i][0];
                int current_obstacle_y = obstacles_ref[i][1];
                std::complex<double> obs_point(current_obstacle_x,current_obstacle_y);
                f = f + std::abs(start_point - obs_point);
            }

            AstarNode *node = new AstarNode(start_point, partial_signature, f ,std::abs(goal_point - start_point), NULL, {});
    
            std::stringstream ss;
            Eigen::VectorXd zeros = Eigen::VectorXd::Zero(obstacles_to_use.size());
            ss << start_point << "-\n"
            << partial_signature;
            distance_count[ss.str()] = f; //std::abs(goal_point - start_point);
            pq.push(node);

            // for (unsigned int i = 0; i < directions.size(); ++i)
            // {
            //     std::complex<double> new_point = start_point + directions[i];
            //     if (int(real(new_point)) < 0 || int(real(new_point)) >= grid_ref.size() || int(imag(new_point)) < 0 || int(imag(new_point)) >= grid_ref[0].size() || grid_ref[int(new_point.real())][int(new_point.imag())] == 0)
            //         continue;

            //     Eigen::VectorXcd s_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), start_point) - obstacle_points;
            //     Eigen::VectorXcd e_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), new_point) - obstacle_points;
            //     Eigen::VectorXd temp = s_vec.array().binaryExpr(e_vec.array(), customOp);

            //     double cell_cost = 1.0; // grid_ref[new_point.real()][new_point.imag()];
            //     if (cell_cost == -1)
            //         cell_cost = 1.0;
                
            //     double f = cell_cost;
            //     double g = std::abs(new_point - goal_point);
            //     double c = f + g;
            //     std::vector<std::complex<double>> e = {start_point, new_point};
                
            //     AstarNode *node = new AstarNode(new_point, temp + partial_signature, f,g, NULL, e);
            //     // AstarNode *node = new AstarNode(new_point, temp + partial_signature, c, NULL, e);
            //     pq.push(node);
            // }

            while (!pq.empty())
            {
                AstarNode *node = pq.top();
                pq.pop();
                if (node->point == goal_point)
                {
                    Eigen::VectorXd filtered = (1.0 / (2 * M_PIf64)) * node->h_signature;
                    if ((filtered.array() > 1.0).any() || (filtered.array() < -1.0).any())
                        continue;

                    bool is_already_seen = false;
                    auto corrected_signature = node->h_signature;

                    Eigen::VectorXd augmented_start_signature = Eigen::VectorXd::Zero(obstacles_to_use.size());
                    Eigen::VectorXd augmented_goal_signature = Eigen::VectorXd::Zero(obstacles_to_use.size());
                    for(int j = 0; j < boundary_points_path[current_start_quadrant].size();++j)
                    {
                        if(boundary_points_path[current_start_quadrant][j][0].first == current_start[0] && boundary_points_path[current_start_quadrant][j][0].second == current_start[1])
                        {
                            augmented_start_signature =  - recompute_h_signature(boundary_points_path[current_start_quadrant][j]);
                        }
                    }

                    for(int j = 0; j < boundary_points_path[current_goal_quadrant].size();++j)
                    {
                        if(boundary_points_path[current_goal_quadrant][j][0].first == current_goal[0] && boundary_points_path[current_goal_quadrant][j][0].second == current_goal[1])
                        {
                            augmented_goal_signature =   recompute_h_signature(boundary_points_path[current_goal_quadrant][j]);
                        }
                    }

                    corrected_signature = corrected_signature + augmented_start_signature + augmented_goal_signature;
                    corrected_signature  = (corrected_signature.array().abs() < 0.001).select(0, corrected_signature);

                    for (int i = 0; i < traversed_signatures.size(); ++i)
                    {
                        Eigen::VectorXd diff = traversed_signatures[i] - corrected_signature;
                        if (diff.isZero(0.0001) || 
                            diff.isApproxToConstant(2*M_PIf64, 0.0001) ||
                            diff.isApproxToConstant(-2*M_PIf64, 0.0001) || 
                            diff.isApproxToConstant(4*M_PIf64, 0.0001) ||
                            diff.isApproxToConstant(-4*M_PIf64, 0.0001) || 
                            diff.isApproxToConstant(6*M_PIf64, 0.0001) ||
                            diff.isApproxToConstant(-6*M_PIf64, 0.0001)
                        
                            )
                        {
                            is_already_seen = true;
                            break;
                        }
                    }
                    if (is_already_seen)
                        continue;

                    std::cout<<"H signature = "<< corrected_signature << std::endl;
                    std::vector<std::pair<int, int>> path;
                    AstarNode *temp = node;
                    while (temp != NULL)
                    {
                        int current_point_x = temp->point.real(), current_point_y = temp->point.imag();
                        path.push_back(std::pair<int, int>(current_point_x, current_point_y));
                        temp = temp->parent;
                    }
                    path.push_back({x,y});
                    std::reverse(path.begin(), path.end());
                
                    for(int i=0;i<path.size();++i)
                    {
                        current_path.push_back({path[i].first,path[i].second});
                    }
                    return true;
                }
                else
                {
                    for (unsigned int i = 0; i < directions.size(); ++i)
                    {
                        std::complex<double> new_point = node->point + directions[i];
                        if (int(real(new_point)) < 0 || int(real(new_point)) >= grid_ref.size() || int(imag(new_point)) < 0 || int(imag(new_point)) >= grid_ref[0].size() || grid_ref[int(real(new_point))][int(imag(new_point))] == 0)
                            continue;
                        Eigen::VectorXcd s_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), node->point) - obstacle_points;
                        Eigen::VectorXcd e_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), new_point) - obstacle_points;
                        Eigen::VectorXd t = s_vec.array().binaryExpr(e_vec.array(), customOp);
                        Eigen::VectorXd temp = node->h_signature + t;
                        Eigen::VectorXd filtered = (1.0 / (2 * M_PIf64)) * temp;
                        if ((filtered.array() > 1.0).any() || (filtered.array() < -1.0).any())
                            continue;
                        // double cell_cost = 1.0; // grid_ref[new_point.real()][new_point.imag()];
                        // if (cell_cost == -1)
                        //     cell_cost = 1.0;

                        double cell_cost=0.0;
                        // can comment out below for loop
                        for(int i=0;i<obstacles_ref.size();++i)
                        {
                            int c_x = obstacles_ref[i][0];
                            int c_y = obstacles_ref[i][1];
                            std::complex<double> obs_point(c_x,c_y);
                            cell_cost = cell_cost + std::abs(new_point - obs_point);
                        }
                        double f = cell_cost  + node->f;
                        //double f = cell_cost;
                        double g = std::abs(new_point - goal_point);
                        // double c = node->cost + f + g;

                        std::stringstream ss;
                        ss << new_point << "-\n"
                        << temp;
                        std::string key = ss.str();
                        // if (distance_count.find(key) == distance_count.end() || distance_count[key] > (f+g))
                        if (distance_count.find(key) == distance_count.end() || distance_count[key] > f)
                        {
                            // distance_count[key] = f+g;
                            distance_count[key] = f;
                            std::vector<std::complex<double>> edge = {node->point, new_point};
                            AstarNode *new_node = new AstarNode(new_point, temp, f,g, node, edge);
                            // AstarNode *new_node = new AstarNode(new_point,temp,c,node,edge);
                            pq.push(new_node);
                        }
                    }
                }
            }
            return false;
        }
        
        Eigen::VectorXd recompute_h_signature(std::vector<std::pair<int,int>> path, int index = -1)
        {
            int n = (index==-1)?(path.size()-1):index;

            auto customOp = [](const std::complex<double> &a, const std::complex<double> &b) -> double
            {
                double minimum_phase_difference = std::arg(b) - std::arg(a);
                for (int i = -2; i < 3; ++i)
                {
                    for (int j = -2; j < 3; ++j)
                    {
                        double phase_difference = (std::arg(b) + 2 * M_PIf64 * i) - (std::arg(a) + 2 * M_PIf64 * j);
                        if (std::abs(phase_difference) < std::abs(minimum_phase_difference))
                        {
                            minimum_phase_difference = phase_difference;
                        }
                    }
                }
                return minimum_phase_difference;
            };

            // std::vector<Eigen::VectorXd> h_signatures;
            std::vector<std::vector<int>> &obstacles_ref = *obstacles_seen;
            std::vector<std::vector<int>> obstacles_to_use;

            if(remove_explored_obstacles)
            {
                std::vector<std::vector<int>> &grid_ref = *grid;
                std::vector<std::vector<int>> &grid_ref_original = *grid_original;


                for(int  i = 0; i< obstacles_ref.size();++i)
                {
                    int current_obstacle_x = obstacles_ref[i][0];
                    int current_obstacle_y = obstacles_ref[i][1];
                    int unmapped_cell_count = 0;
                    
                    if(current_obstacle_y-1 >= 0)
                    {
                        for(int j=current_obstacle_x-1;j<current_obstacle_x + 5;++j)
                        {
                            if(j<0 || j>= grid_ref.size())
                                continue;
                            if(grid_ref[j][current_obstacle_y-1] == -1 && grid_ref_original[j][current_obstacle_y-1] != 0)
                                unmapped_cell_count+=1;
                        
                        }  
                    }
                    if(current_obstacle_y + 4 < grid_ref[0].size())
                    {
                        for(int j=current_obstacle_x-1;j<current_obstacle_x + 5;++j)
                        {
                            if(j<0 || j>= grid_ref.size())
                                continue;
                            if(grid_ref[j][current_obstacle_y+4] == -1 && grid_ref_original[j][current_obstacle_y+4] != 0)
                                unmapped_cell_count+=1;
                        }  
                    }

                    if(current_obstacle_x-1 >= 0)
                    {
                        for(int j=current_obstacle_y-1;j<current_obstacle_y + 5;++j)
                        {
                            if(j<0 || j>= grid_ref[0].size())
                                continue;
                            if(grid_ref[current_obstacle_x-1][j] == -1 && grid_ref_original[current_obstacle_x-1][j] != 0)
                                unmapped_cell_count+=1;
                        }  
                    }
                    
                    if(current_obstacle_x + 4 < grid_ref.size())
                    {
                        for(int j=current_obstacle_y-1;j<current_obstacle_y + 5;++j)
                        {
                            if(j<0 || j>= grid_ref[0].size())
                                continue;
                            if(grid_ref[current_obstacle_x+4][j] == -1 && grid_ref_original[current_obstacle_x+4][j] != 0)
                                unmapped_cell_count+=1;
                        }  
                    }

                    if(unmapped_cell_count > 0)
                        obstacles_to_use.push_back(obstacles_ref[i]);
                }
            }
            else
            {
                obstacles_to_use = obstacles_ref;
            }

            Eigen::VectorXd current_h_signature = Eigen::VectorXd::Zero(obstacles_to_use.size());
            
            Eigen::VectorXcd obstacle_points = Eigen::VectorXcd::Zero(obstacles_to_use.size());
            for (unsigned int i = 0; i < obstacles_to_use.size(); ++i)
                obstacle_points(i) = std::complex<double>(obstacles_to_use[i][0], obstacles_to_use[i][1]);
            
            Eigen::VectorXd sum = Eigen::VectorXd::Zero(obstacles_to_use.size());
            for(int i=1;i<=n;++i)
            {
                Eigen::VectorXcd s_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), std::complex<double>(path[i-1].first, path[i-1].second)) - obstacle_points;
                Eigen::VectorXcd e_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), std::complex<double>(path[i].first, path[i].second)) - obstacle_points;
                Eigen::VectorXd t = s_vec.array().binaryExpr(e_vec.array(), customOp);
                Eigen::VectorXd temp = t;
                sum += temp;
            }
            current_h_signature = sum;
            return current_h_signature;
        }

        std::vector<int> get_quadrant_vector()
        {
            std::vector<int> ret;
            std::vector<std::vector<int>> &grid_ref = *grid;
            
            int count=0;
            for(int j=0;j<grid_ref[0].size()-1;++j)
            {
                if(grid_ref[0][j]==-1)
                    count+=1; 
            }
            ret.push_back(1+count);

            count = 0 ;   
            for(int j=0;j<grid_ref.size()-1; ++ j)
            {
                if(grid_ref[j][grid_ref[0].size()-1]==-1)
                    count+=1;
                       
            }
            ret.push_back(1+count);
    
            count = 0;
            for(int j=1;j<grid_ref[0].size(); ++ j)
            {
                if(grid_ref[grid_ref.size()-1][j]==-1)
                    count+=1;
            
            }
            ret.push_back(1+count);

                
            count = 0;  
            for(int j=1;j<grid_ref.size(); ++ j)
            {
                if(grid_ref[j][0]==-1)
                    count+=1;
            }
            ret.push_back(1+count);

            return ret;
        }

        std::vector<std::pair<int,int>> get_destination_point(int quadrant_index)
        {
            std::vector<std::pair<int,int>> destination_points;
            std::vector<std::vector<int>> &grid_ref = *grid;
            if(quadrant_index == 0)
            {
                for(int j=0;j<grid_ref[0].size()-1;++j)
                {
                    if(grid_ref[0][j]==-1)
                        destination_points.push_back({0,j}); 
                }
                if(destination_points.size() ==0)
                {
                    int random_index = rand() % (grid->size() - 1);
                    destination_points.push_back({0,random_index});
                }
            }
            else if(quadrant_index==1)
            {
                for(int j=0;j<grid_ref.size()-1; ++ j)
                {
                    if(grid_ref[j][grid_ref[0].size()-1]==-1)
                        destination_points.push_back({j,grid_ref[0].size()-1});                       
                }
                if(destination_points.size() == 0 )
                {
                    int random_index = rand() % (grid->size() - 1);
                    destination_points.push_back({random_index , (int)grid->size()-1});
                }
            }
            else if(quadrant_index == 2)
            {
                for(int j=1;j<grid_ref[0].size(); ++ j)
                {
                    if(grid_ref[grid_ref.size()-1][j]==-1)
                        destination_points.push_back({grid_ref.size()-1,j});
                }
                if (destination_points.size() == 0)
                {
                    int random_index = rand() % (grid->size() - 1)+1;
                    destination_points.push_back({(int)grid->size()-1, random_index});
                }
            }
            else
            {
                for(int j=1;j<grid_ref.size(); ++ j)
                {
                    if(grid_ref[j][0]==-1)
                        destination_points.push_back({j,0});
                }
                if(destination_points.size() == 0)
                {
                    int random_index = rand() % (grid->size() - 1) + 1;
                    destination_points.push_back({random_index,0});
                }
            }
            return destination_points;
        }


std::vector<std::vector<int>> *grid;
std::vector<std::vector<int>> *obstacles_seen;
std::map<std::string, int> done_signatures;

std::vector<std::vector<std::vector<std::pair<int,int>>>> boundary_points_path;


std::vector<int> start_coordinates;
std::vector<int> goal_coordinates;
std::vector<std::vector<int>> goals;

std::vector<int> n_times_chosen;

std::vector<std::vector<std::pair<int,int>>> corner_point_paths;




std::vector<int> current_start;
int current_start_quadrant;
std::vector<int> current_goal;
int current_goal_quadrant;
std::vector<int> start_quadrants;
std::vector<int> goal_quadrants;
std::vector<std::vector<std::pair<int,int>>> traversed_paths;
std::vector<std::pair<int,int>> current_path;
int current_path_index;
std::vector<Eigen::VectorXd> traversed_signatures;
std::vector<std::vector<int>> *grid_original;
bool remove_explored_obstacles = false;

};