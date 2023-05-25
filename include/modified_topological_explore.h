
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


        ModifiedTopolgicalExplore(std::vector<std::vector<int>> *g, std::vector<std::vector<int>> *o, std::vector<int> start) : grid(g), obstacles_seen(o), start_coordinates(start)
        {
            current_start = {start[0],start[1]};
            boundary_points_path = getBoundaryPaths(grid->size());
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

            int quadrant_select_index = rand() % 4;
            int random_index = -1;
            switch (quadrant_select_index)
            {
                case 0:
                    random_index = rand() % (grid->size() - 1);
                    current_goal = {0,random_index};
                    break;
                case 1:
                    random_index = rand() % (grid->size() - 1);
                    current_goal = {random_index , (int)grid->size()-1};
                    break;
                case 2:
                    random_index = rand() % (grid->size() - 1)+1;
                    current_goal = {(int)grid->size()-1, random_index};
                    break;
                case 3 : 
                    random_index = rand() % (grid->size() - 1) + 1;
                    current_goal = {random_index , 0};
                    break;
                default:
                    random_index = rand() % (grid->size() - 1) + 1;
                    current_goal = {random_index , 0};
                    break;
            }
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
                        for(int k = grid_size-1; k>=-1;k--)
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

        if(x== current_goal[0] && y==current_goal[1])
        {
            start_quadrants.push_back(current_start_quadrant);
            goal_quadrants.push_back(current_goal_quadrant);

            current_start = {current_goal[0],current_goal[1]};
            current_start_quadrant = current_goal_quadrant;
            

            int next_quadrant_index = rand() % 4;
            int random_index = -1;
            switch (next_quadrant_index)
            {
                case 0:
                    random_index = rand() % (grid->size() - 1);
                    current_goal = {0,random_index};
                    break;
                case 1:
                    random_index = rand() % (grid->size() - 1);
                    current_goal = {random_index , (int)grid->size()-1};
                    break;
                case 2:
                    random_index = rand() % (grid->size() - 1) + 1;
                    current_goal = {(int)grid->size()-1, random_index};
                    break;
                case 3 : 
                    random_index = rand() % (grid->size() - 1) + 1;
                    current_goal = {random_index , 0};
                    break;
                default:
                    random_index = rand() % (grid->size() - 1) + 1;
                    current_goal = {random_index , 0};
                    break;
            }
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
            Eigen::VectorXd augmented_path_signature_start = Eigen::VectorXd::Zero(obstacles_ref.size());
            Eigen::VectorXd augmented_path_signature_goal = Eigen::VectorXd::Zero(obstacles_ref.size());

            for(int j = 0 ; j < boundary_points_path[start_quadrants[i]].size() ; ++j)
            {
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


        Eigen::VectorXd partial_signature = Eigen::VectorXd::Zero(obstacles_ref.size());
        partial_signature = recompute_h_signature(current_path,current_path_index);
        // std::cout<<"Partial signature: "<<partial_signature.transpose()<<std::endl;

    
        Eigen::VectorXcd obstacle_points = Eigen::VectorXcd::Zero(obstacles_ref.size());
        for (unsigned int i = 0; i < obstacles_ref.size(); ++i)
            obstacle_points(i) = std::complex<double>(obstacles_ref[i][0], obstacles_ref[i][1]);

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
        
        AstarNode *node = new AstarNode(start_point, partial_signature, 0.0 ,std::abs(goal_point - start_point), NULL, {});
  
        std::stringstream ss;
        Eigen::VectorXd zeros = Eigen::VectorXd::Zero(obstacles_ref.size());
        ss << start_point << "-\n"
           << partial_signature;
        distance_count[ss.str()] = 0.0; //std::abs(goal_point - start_point);
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

                Eigen::VectorXd augmented_start_signature = Eigen::VectorXd::Zero(obstacles_ref.size());
                Eigen::VectorXd augmented_goal_signature = Eigen::VectorXd::Zero(obstacles_ref.size());
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
                    double cell_cost = 1.0; // grid_ref[new_point.real()][new_point.imag()];
                    if (cell_cost == -1)
                        cell_cost = 1.0;
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
        Eigen::VectorXd current_h_signature = Eigen::VectorXd::Zero(obstacles_ref.size());
        
        Eigen::VectorXcd obstacle_points = Eigen::VectorXcd::Zero(obstacles_ref.size());
        for (unsigned int i = 0; i < obstacles_ref.size(); ++i)
            obstacle_points(i) = std::complex<double>(obstacles_ref[i][0], obstacles_ref[i][1]);
        
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

};