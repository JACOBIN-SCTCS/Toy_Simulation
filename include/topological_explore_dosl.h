
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
#include <dosl/dosl>


class DOSLNode : public AStar::Node<DOSLNode,double>
{
    public:
        std::complex<double> state;
        Eigen::VectorXd h_signature;

        DOSLNode(std::complex<double> state, Eigen::VectorXd h_signature) : state(state), h_signature(h_signature) {}
        DOSLNode(){}

        bool operator==(const DOSLNode& n) const 
        { 
            Eigen::VectorXd diff = h_signature - n.h_signature;
            bool is_h_equal = diff.norm() < 1e-6;
            return is_h_equal && state == n.state; 
        }

        int getHashBin (void) 
        {
            std::size_t h_hash = 0;
            for(unsigned int i=0;i<h_signature.size();++i)
            {
                h_hash = h_hash ^ std::hash<double>()(h_signature(i));
            }
            return std::hash<double>()(state.real()) ^ std::hash<double>()(state.imag()) ^ h_hash;
        }  
        
        void print (std::string head="", std::string tail="") const
        { 
            _dosl_cout << _GREEN + head << " (" << this << ")" GREEN_ " x=" << state.real() << ", y=" << state.imag() << "; ";
            (g_score==std::numeric_limits<double>::max())? printf("INF") : printf("g_score = %0.8f; h = [", g_score);
            for (int a=0; a<h_signature.size(); ++a){
                printf("%lf",  h_signature(a));
                if (a!=h_signature.size()-1) printf(", ");
            } 
            printf("]\n");
            std::cout << tail << _dosl_endl;
        }
};

class searchAlgorithm : public AStar::Algorithm<searchAlgorithm,DOSLNode,double>
{

    public:
        std::vector<std::vector<int>> *grid;
        std::vector<std::vector<int>> *obstacles_to_use;

        std::complex<double> start_point;
        Eigen::VectorXd start_h_signature;
        std::complex<double> goal_point;

        std::vector<Eigen::VectorXd> visited_h_signatures;
        Eigen::VectorXd current_start_h_signature;
        Eigen::VectorXd current_goal_h_signature;
        bool reverse_path = false;

        std::function<double(const std::complex<double>&, const std::complex<double>&)> customOp = [](const std::complex<double> &a, const std::complex<double> &b) -> double
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
        
        searchAlgorithm(std::vector<std::vector<int>> *g,std::vector<std::vector<int>> *o, std::complex<double> s_point , Eigen::VectorXd start_signature, std::complex<double> g_point,
            std::vector<Eigen::VectorXd> visited_signatures,
            Eigen::VectorXd current_start_signatures,
            Eigen::VectorXd current_goal_signatures , bool r_path=false) : grid(g),obstacles_to_use(o),start_point(s_point), start_h_signature(start_signature), goal_point(g_point),
            visited_h_signatures(visited_signatures),
            current_start_h_signature(current_start_signatures),
            current_goal_h_signature(current_goal_signatures),
            reverse_path(r_path)
        {}

        void getSuccessors(DOSLNode &n, std::vector<DOSLNode>* s , std::vector<double>* c)
        {
            DOSLNode new_node;
            std::vector<std::pair<int,int>> directions = {{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};
            std::vector<std::vector<int>> &grid_ref = *grid;
            std::vector<std::vector<int>> &obstacles_to_use_ref = *obstacles_to_use;
        
            for(int i=0;i<directions.size();++i)
            {
                int new_x = n.state.real() + directions[i].first;
                int new_y = n.state.imag() + directions[i].second;

                if(new_x < 0 || new_x >= grid_ref.size() || new_y < 0 || new_y >= grid_ref[0].size() || (grid_ref[new_x][new_y] == 0))
                    continue;

                std::complex<double> new_point(new_x, new_y);

                Eigen::VectorXcd obstacle_points = Eigen::VectorXcd::Zero(obstacles_to_use_ref.size());
                for(int i=0;i<obstacles_to_use_ref.size();++i)
                {
                    obstacle_points(i) = std::complex<double>(obstacles_to_use_ref[i][0], obstacles_to_use_ref[i][1]);
                }

                Eigen::VectorXcd s_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), n.state) - obstacle_points;
                Eigen::VectorXcd e_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), new_point) - obstacle_points;
                Eigen::VectorXd temp = s_vec.array().binaryExpr(e_vec.array(), customOp);
                Eigen::VectorXd filtered = (1.0 / (2 * M_PIf64)) * (n.h_signature+temp);
                if ((filtered.array() > 1.0).any() || (filtered.array() < -1.0).any())
                    continue;
            
                s->push_back(DOSLNode(new_point, n.h_signature + temp));
                c->push_back(std::abs(new_point - n.state));
            }
        }

        std::vector<DOSLNode> getStartNodes (void) {
            std::vector<DOSLNode> startNodes;
            
            DOSLNode start_node;
            start_node.state = start_point;
            start_node.h_signature = start_h_signature;
            startNodes.push_back(start_node);
            return (startNodes);
        }

        bool stopSearch (DOSLNode &n) 
        {
            if(n.state == goal_point)
            {
                bool already_visited = false;
                Eigen::VectorXd filtered = (1.0 / (2 * M_PIf64)) * (n.h_signature);
                if ((filtered.array() > 1.0).any() || (filtered.array() < -1.0).any())
                    return false;
            
                auto corrected_signature = n.h_signature;
                if(reverse_path)
                    corrected_signature = -n.h_signature;
                corrected_signature = corrected_signature + current_start_h_signature + current_goal_h_signature;
                for(int i=0;i<visited_h_signatures.size();++i)
                {
                    if((visited_h_signatures[i] - corrected_signature).norm() < 1e-6)
                    {
                        already_visited = true;
                        break;
                    }
                }
                if(!already_visited)
                {
                    visited_h_signatures.push_back(corrected_signature);
                    return true;

                }
            }
            return false;
        }

        double getHeuristics (DOSLNode& n) {
            double dx = goal_point.real() - n.state.real();
            double dy = goal_point.imag() - n.state.imag();
            return (sqrt(dx*dx + dy*dy)); // Euclidean heuristic function
        }


};

class TopologicalExploreDOSL 
{
    public:
        
        TopologicalExploreDOSL(std::vector<std::vector<int>> *g, std::vector<std::vector<int>> *o, std::vector<int> start, std::vector<int> goal,std::vector<std::vector<int>> *g_original) : grid(g), obstacles_seen(o), start_coordinates(start), goal_coordinates(goal) , grid_original(g_original)
        {
            current_start = {start[0],start[1]};
            current_goal  = {goal[0],goal[1]};
            current_path_index = 0;
            traversed_paths.clear();
            current_path.clear();
            traversed_signatures.clear();
            use_four_corner_points = false;
        }

        TopologicalExploreDOSL(std::vector<std::vector<int>> *g, std::vector<std::vector<int>> *o, std::vector<int> start, std::vector<std::vector<int>> goals,std::vector<std::vector<int>> *g_original) : grid(g), obstacles_seen(o), start_coordinates(start), goals(goals), grid_original(g_original)
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


            // From (grid_size-1,grid_size-1) to (grid_size-1,0)
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
            obstacles_to_use = obstacles_ref;
            
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
    
            Eigen::VectorXcd obstacle_points = Eigen::VectorXcd::Zero(obstacles_to_use.size());
            for (unsigned int i = 0; i < obstacles_to_use.size(); ++i)
                obstacle_points(i) = std::complex<double>(obstacles_to_use[i][0], obstacles_to_use[i][1]);

            std::complex<double> start_point(x, y);
            std::vector<int> goal_coords = current_goal;
            std::complex<double> goal_point(goal_coords[0], goal_coords[1]);
            std::vector<std::complex<double>> traversed_points;

            Eigen::VectorXd current_start_signature,current_goal_signature;
            bool should_reverse_path = false;
            
            if(!use_four_corner_points)
            {
                current_start_signature = Eigen::VectorXd::Zero(obstacles_to_use.size());
                current_goal_signature = Eigen::VectorXd::Zero(obstacles_to_use.size());
                if(current_goal[0] == start_coordinates[0] && current_goal[1] == start_coordinates[1])
                {
                    should_reverse_path = true;
                }
            }
            else
            {
                current_start_signature = Eigen::VectorXd::Zero(obstacles_to_use.size());
                current_goal_signature = Eigen::VectorXd::Zero(obstacles_to_use.size());

                if(current_goal[0] == start_coordinates[0] && current_goal[1] == start_coordinates[1])
                {
                    for(int j=0;j<goals.size();++j)
                    {
                        auto path_goal_point = std::make_pair(current_start[0],current_start[1]);
                        if(path_goal_point.first==goals[j][0] && path_goal_point.second == goals[j][1])
                        {
                           current_goal_signature = recompute_h_signature(corner_point_paths[j]);
                        }
                    }   
                    should_reverse_path = true;
                }
                else
                {
                    for(int j=0;j<goals.size();++j)
                    {
                        auto path_goal_point = std::make_pair(current_goal[0],current_goal[1]);
                        if(path_goal_point.first==goals[j][0] && path_goal_point.second == goals[j][1])
                        {
                           current_goal_signature = recompute_h_signature(corner_point_paths[j]);
                        }
                    }  
                }
            }
          
            searchAlgorithm alg(grid,obstacles_seen,start_point,partial_signature,goal_point,traversed_signatures,current_start_signature,current_goal_signature,should_reverse_path);
            alg.search();
            
            std::vector<DOSLNode*> path = alg.reconstruct_pointer_path(DOSLNode(goal_point,alg.visited_h_signatures[alg.visited_h_signatures.size()-1]));
            if(path.size() == 0)
                return false;
            for(int i=path.size()-1;i>=0;i--)
            {
                current_path.push_back({path[i]->state.real(),path[i]->state.imag()});
            }
            return true;

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
            obstacles_to_use = obstacles_ref;
            
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

        std::vector<int> start_coordinates;
        std::vector<int> goal_coordinates;
        std::vector<std::vector<int>> goals;

        std::vector<int> n_times_chosen;
        std::vector<std::vector<std::pair<int,int>>> corner_point_paths;

        std::vector<int> current_start;
        std::vector<int> current_goal;

        std::vector<std::vector<std::pair<int,int>>> traversed_paths;
        std::vector<std::pair<int,int>> current_path;
        int current_path_index;
        std::vector<Eigen::VectorXd> traversed_signatures;

        bool remove_explored_obstacles = true;
        bool use_four_corner_points = false;

};