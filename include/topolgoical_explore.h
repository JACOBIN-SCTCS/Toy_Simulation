
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

class TopolgicalExplore
{
public:
    struct Frontier
    {
        int x;
        int y;
        std::vector<std::pair<int, int>> cells;
    };

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


    TopolgicalExplore(std::vector<std::vector<int>> *g, std::vector<std::vector<int>> *o, std::vector<int> start, std::vector<int> goal,std::vector<std::vector<int>> *g_original) : grid(g), obstacles_seen(o), start_coordinates(start), goal_coordinates(goal) , grid_original(g_original)
    {
        current_start = {start[0],start[1]};
        current_goal  = {goal[0],goal[1]};
        current_path_index = 0;
        traversed_paths.clear();
        current_path.clear();
        traversed_signatures.clear();
        use_four_corner_points = false;
    }

    TopolgicalExplore(std::vector<std::vector<int>> *g, std::vector<std::vector<int>> *o, std::vector<int> start, std::vector<std::vector<int>> goals,std::vector<std::vector<int>> *g_original) : grid(g), obstacles_seen(o), start_coordinates(start), goals(goals), grid_original(g_original)
    {
        current_start = {start[0],start[1]};
        int random_index = rand() % 4;
        current_goal  = {goals[random_index][0],goals[random_index][1]};
        current_path_index = 0;
        traversed_paths.clear();
        current_path.clear();
        traversed_signatures.clear();
        use_four_corner_points = true;
        corner_point_paths = create_corner_points_paths(grid->size());
        for(int i=0;i<4;++i)
        {
            n_times_chosen.push_back(0);
        }
        n_times_chosen[random_index] = 1;
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

    std::vector<std::vector<std::pair<int,int>>> create_corner_points_paths(int grid_size)
	{
		 std::cout<<"Reached here";
		std::vector<std::vector<int>> goals = {{grid_size-1,grid_size-1},{0,0},{0,grid_size-1},{grid_size-1,0}};
		std::vector<std::vector<std::pair<int,int>>> corner_points_paths;
		
		// The bottom right as the main destination point (grid_size-1,grid_size-1)
		corner_points_paths.push_back({});
		
		// From  (grid_size-1,grid_size-1) to (0,0)
		std::vector<std::pair<int,int>> path_to_top_left;
		path_to_top_left.push_back(std::make_pair(goals[0][0],goals[0][1]));
		for(int i=grid_size;i>=-1;i--)
		{
			path_to_top_left.push_back(std::make_pair(grid_size,i));
		}
		for(int i=grid_size;i>=-1;i--)
		{
			path_to_top_left.push_back(std::make_pair(i,-1));
		}
		path_to_top_left.push_back(std::make_pair(0,0));
        std::reverse(path_to_top_left.begin(),path_to_top_left.end());
		corner_points_paths.push_back(path_to_top_left);

		
        //From  (grid_size-1,grid_size-1) to (0,grid_size-1)
		std::vector<std::pair<int,int>> path_to_top_right;
		path_to_top_right.push_back(std::make_pair(goals[0][0],goals[0][1]));
		for(int i=0;i<path_to_top_left.size()-1;++i)
        {
            path_to_top_right.push_back(path_to_top_left[i]);
        }
        for(int i=-1;i<=grid_size;++i)
        {
            path_to_top_right.push_back(std::make_pair(-1,i));
        }
        path_to_top_right.push_back(std::make_pair(0,grid_size-1));
        std::reverse(path_to_top_right.begin(),path_to_top_right.end());
		corner_points_paths.push_back(path_to_top_right);


		// From (grid_size-1,grid_size-1) to (0,grid_size-1)
		std::vector<std::pair<int,int>> path_to_bottom_left;
		path_to_bottom_left.push_back(std::make_pair(goals[0][0],goals[0][1]));
        for(int i=grid_size;i>=-1;i--)
        {
            path_to_bottom_left.push_back(std::make_pair(grid_size,i));
        }
        path_to_bottom_left.push_back(std::make_pair(grid_size-1,0));
        std::reverse(path_to_bottom_left.begin(),path_to_bottom_left.end());
		corner_points_paths.push_back(path_to_bottom_left);     
        return corner_points_paths;
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
                    for(int i=current_obstacle_x-1;i<current_obstacle_x + 5;++i)
                    {
                        if(i<0 || i>= grid_ref.size())
                            continue;
                        if(grid_ref[i][current_obstacle_y-1] == -1 && grid_ref_original[i][current_obstacle_y-1] != 0)
                            unmapped_cell_count+=1;
                    
                    }  
                }
                if(current_obstacle_y + 4 < grid_ref[0].size())
                {
                    for(int i=current_obstacle_x-1;i<current_obstacle_x + 5;++i)
                    {
                        if(i<0 || i>= grid_ref.size())
                            continue;
                        if(grid_ref[i][current_obstacle_y+4] == -1 && grid_ref_original[i][current_obstacle_y+4] != 0)
                            unmapped_cell_count+=1;
                    }  
                }

                if(current_obstacle_x-1 >= 0)
                {
                    for(int i=current_obstacle_y-1;i<current_obstacle_y + 5;++i)
                    {
                        if(i<0 || i>= grid_ref[0].size())
                            continue;
                        if(grid_ref[current_obstacle_x-1][i] == -1 && grid_ref_original[current_obstacle_x-1][i] != 0)
                            unmapped_cell_count+=1;
                    }  
                }
                
                if(current_obstacle_x + 4 < grid_ref.size())
                {
                    for(int i=current_obstacle_y-1;i<current_obstacle_y + 5;++i)
                    {
                        if(i<0 || i>= grid_ref[0].size())
                            continue;
                        if(grid_ref[current_obstacle_x+4][i] == -1 && grid_ref_original[current_obstacle_x+4][i] != 0)
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
            if(current_goal[0] == start_coordinates[0] && current_goal[1] == start_coordinates[1])
            {
                current_start = {start_coordinates[0],start_coordinates[1]};
                
                if(!use_four_corner_points)
                {
                    current_goal = {goal_coordinates[0],goal_coordinates[1]};
                }
                else
                {
                    int weight_sum = 0;
                    for(int i=0;i<n_times_chosen.size();++i)
                        weight_sum += n_times_chosen[i];
                    
                    std::vector<int> tmp_weights;
                    for(int i=0;i<n_times_chosen.size();++i)
                        tmp_weights.push_back(weight_sum - n_times_chosen[i]);
                    
                    
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
                    n_times_chosen[chosen_index] += 1;
                    std::cout<<"Goal point chosen = " << goals[chosen_index][0] << " " << goals[chosen_index][1] << std::endl;
                   
                    current_goal = {goals[chosen_index][0],goals[chosen_index][1]};
                }
                std::reverse(current_path.begin(),current_path.end());
            }
            else
            {
                current_start = {current_goal[0],current_goal[1]};
                current_goal = {start_coordinates[0],start_coordinates[1]};
            }
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
            Eigen::VectorXd augmented_path_signature = Eigen::VectorXd::Zero(obstacles_to_use.size());
            if(use_four_corner_points)
            {
                for(int j=0;j<goals.size();++j)
                {
                    auto path_goal_point = traversed_paths[i][traversed_paths[i].size()-1];
                    if(path_goal_point.first==goals[j][0] && path_goal_point.second == goals[j][1])
                    {
                        augmented_path_signature = recompute_h_signature(corner_point_paths[j]);
                    }
                }
            }
            prev_h_signature =  prev_h_signature + augmented_path_signature;
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
        // std::unordered_map<AstarNode*,double> distance_count;

        
        Eigen::VectorXd zeros = Eigen::VectorXd::Zero(obstacles_to_use.size());
        AstarNode* start_node_pq = new AstarNode(start_point,partial_signature,0.0,std::abs(goal_point-start_point),NULL,{});
        std::stringstream ss;
        ss << start_point << "-\n"
           << partial_signature;
        // distance_count[start_node_pq] = 0.0; //std::abs(goal_point - start_point);
        distance_count[ss.str()] = 0.0;
        pq.push(start_node_pq);

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
                if( current_goal[0] == start_coordinates[0] && current_goal[1] == start_coordinates[1])
                {
                    corrected_signature = -node->h_signature;
                    Eigen::VectorXd h_signature_correction = Eigen::VectorXd::Zero(obstacles_to_use.size());
                    if(use_four_corner_points)
                    {
                        for(int j=0;j<corner_point_paths.size();++j)
                        {
                            if(goals[j][0]== current_start[0] && goals[j][1] == current_start[1])
                            {
                                h_signature_correction = -recompute_h_signature(corner_point_paths[j]);
                                break;
                            }
                        }
                    }
                    corrected_signature = corrected_signature + h_signature_correction;
                }
                else
                {
                    corrected_signature = node -> h_signature;
                    Eigen::VectorXd h_signature_correction = Eigen::VectorXd::Zero(obstacles_to_use.size());

                    if(use_four_corner_points)
                    {
                        for(int j=0;j<corner_point_paths.size();++j)
                        {
                            if(goals[j][0]== current_goal[0] && goals[j][1] == current_goal[1])
                            {
                                h_signature_correction = recompute_h_signature(corner_point_paths[j]);
                                break;
                            }
                        } 
                    }
                    corrected_signature = corrected_signature + h_signature_correction;
                }
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
                
                // // Added Portion checking maximum difference
                // int max_difference = obstacles_ref.size();
                // for (int i = 0; i < traversed_signatures.size(); ++i)
                // {
                //     Eigen::VectorXd diff = traversed_signatures[i] - corrected_signature;
                //     std::cout<<"Difference = " << diff<<std::endl;
                //     int current_similarities = 0;
                //     for(int j=0;j<obstacles_ref.size();++j)
                //     {
                //         Eigen::VectorXd one_element_vector(1);
                //         one_element_vector << diff(j);

                //         if(
                //             one_element_vector.isZero(0.0001)
                           
                //         )
                //         {
                //             current_similarities++;
                //         }
                //     }
                //     if((obstacles_ref.size() - current_similarities) < max_difference)
                //     {
                //         max_difference = obstacles_ref.size() - current_similarities;
                //     }
                // }
                // std::cout << "Max Difference = " << max_difference << std::endl;
                // if(!(max_difference >= (obstacles_ref.size()/2)))
                //     continue;

                // The added portion ends here.

                std::cout<<"H signature = "<< corrected_signature << std::endl;
                // visited_h_signatures.push_back(node->h_signature);
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
                // std::cout<<"Current Path Coordinates ="<<std::endl;
                // for(int i=0;i<path.size();++i)
                // {
                //     std::cout<<"("<<path[i].first<<","<<path[i].second<<") ";
                // }
                // current_path.clear();
                for(int i=0;i<path.size();++i)
                {
                    current_path.push_back({path[i].first,path[i].second});
                }
             
                return true;

                // p.path = path;
                // return p;
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
                    double cell_cost = 1.0; // grid_ref[new_point.real()][new_point.imag()];
                    if (cell_cost == -1)
                        cell_cost = 1.0;
                    double f = cell_cost  + node->f;
                    //double f = cell_cost;
                    double g = std::abs(new_point - goal_point);
                    // double c = node->cost + f + g;
                    std::vector<std::complex<double>> edge = {node->point, new_point};

                    AstarNode *new_node = new AstarNode(new_point, temp, f,g, node, edge);
                    std::stringstream ss;
                    ss << new_point << "-\n"
                    << temp;
                    std::string key = ss.str();
                    if (distance_count.find(key) == distance_count.end() || distance_count[key] > (f+g))
                    // if (distance_count.find(new_node) == distance_count.end() || distance_count[new_node] > f)
                    {
                        // distance_count[key] = f+g;
                        distance_count[key] = f;
                        // distance_count[new_node] = f;
                        // std::vector<std::complex<double>> edge = {node->point, new_point};
                        // AstarNode *new_node = new AstarNode(new_point, temp, f,g, node, edge);
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
                    for(int i=current_obstacle_x-1;i<current_obstacle_x + 5;++i)
                    {
                        if(i<0 || i>= grid_ref.size())
                            continue;
                        if(grid_ref[i][current_obstacle_y-1] == -1 && grid_ref_original[i][current_obstacle_y-1] != 0)
                            unmapped_cell_count+=1;
                    
                    }  
                }
                if(current_obstacle_y + 4 < grid_ref[0].size())
                {
                    for(int i=current_obstacle_x-1;i<current_obstacle_x + 5;++i)
                    {
                        if(i<0 || i>= grid_ref.size())
                            continue;
                        if(grid_ref[i][current_obstacle_y+4] == -1 && grid_ref_original[i][current_obstacle_y+4] != 0)
                            unmapped_cell_count+=1;
                    }  
                }

                if(current_obstacle_x-1 >= 0)
                {
                    for(int i=current_obstacle_y-1;i<current_obstacle_y + 5;++i)
                    {
                        if(i<0 || i>= grid_ref[0].size())
                            continue;
                        if(grid_ref[current_obstacle_x-1][i] == -1 && grid_ref_original[current_obstacle_x-1][i] != 0)
                            unmapped_cell_count+=1;
                    }  
                }
                
                if(current_obstacle_x + 4 < grid_ref.size())
                {
                    for(int i=current_obstacle_y-1;i<current_obstacle_y + 5;++i)
                    {
                        if(i<0 || i>= grid_ref[0].size())
                            continue;
                        if(grid_ref[current_obstacle_x+4][i] == -1 && grid_ref_original[current_obstacle_x+4][i] != 0)
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
        
        Eigen::VectorXd sum = Eigen::VectorXd::Zero(obstacle_points.size());
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



std::vector<std::vector<int>> *grid;
std::vector<std::vector<int>> *grid_original;

std::vector<std::vector<int>> *obstacles_seen;
std::vector<Frontier> frontiers;
std::map<std::string, int> done_signatures;

std::vector<int> start_coordinates;
std::vector<int> goal_coordinates;
std::vector<std::vector<int>> goals;
bool use_four_corner_points = false;

std::vector<int> n_times_chosen;

std::vector<std::vector<std::pair<int,int>>> corner_point_paths;

std::vector<int> current_start;
std::vector<int> current_goal;

std::vector<std::vector<std::pair<int,int>>> traversed_paths;
std::vector<std::pair<int,int>> current_path;
int current_path_index;
std::vector<Eigen::VectorXd> traversed_signatures;

bool remove_explored_obstacles = true;
};