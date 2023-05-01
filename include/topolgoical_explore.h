
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
        struct AstarNode *parent;
        std::vector<std::complex<double>> edge;

        AstarNode(std::complex<double> p, Eigen::VectorXd h, double f_,double g_,struct AstarNode *pa, std::vector<std::complex<double>> e) : point(p), h_signature(h), f(f_),g(g_), parent(pa), edge(e) {}
    };


    TopolgicalExplore(std::vector<std::vector<int>> *g, std::vector<std::vector<int>> *o, std::vector<int> start, std::vector<int> goal) : grid(g), obstacles_seen(o), start_coordinates(start), goal_coordinates(goal)
    {
        current_start = {start[0],start[1]};
        current_goal  = {goal[0],goal[1]};
        current_path_index = 0;
        traversed_paths.clear();
        current_path.clear();
        traversed_signatures.clear();
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

   
 
    // struct NonHomologouspath
    // {
    //     std::vector<std::pair<int, int>> path;
    // };

    void getNonHomologousPaths(int x, int y, std::vector<Eigen::VectorXd> visited_h_signatures)
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
            if(current_goal[0] == start_coordinates[0] && current_goal[1] == start_coordinates[1])
            {
                current_start = {start_coordinates[0],start_coordinates[1]};
                current_goal = {goal_coordinates[0],goal_coordinates[1]};
                std::reverse(current_path.begin(),current_path.end());
            }
            else
            {
                current_start = {goal_coordinates[0],goal_coordinates[1]};
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
            traversed_signatures.push_back(prev_h_signature);
        }
        Eigen::VectorXd partial_signature = Eigen::VectorXd::Zero(obstacles_ref.size());
        partial_signature = recompute_h_signature(current_path,current_path_index);

    
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
                                                                                                                     { return (a->f + a->g) > (b->f + b->g); });
        std::unordered_map<std::string, double> distance_count;
        std::set<std::string> visited;
        std::stringstream ss;
        Eigen::VectorXd zeros = Eigen::VectorXd::Zero(obstacles_ref.size());

        ss << start_point << "-\n"
           << partial_signature;
        distance_count[ss.str()] = std::abs(goal_point - start_point);

        for (unsigned int i = 0; i < directions.size(); ++i)
        {
            std::complex<double> new_point = start_point + directions[i];
            if (int(real(new_point)) < 0 || int(real(new_point)) >= grid_ref.size() || int(imag(new_point)) < 0 || int(imag(new_point)) >= grid_ref[0].size() || grid_ref[int(new_point.real())][int(new_point.imag())] == 0)
                continue;

            Eigen::VectorXcd s_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), start_point) - obstacle_points;
            Eigen::VectorXcd e_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), new_point) - obstacle_points;
            Eigen::VectorXd temp = s_vec.array().binaryExpr(e_vec.array(), customOp);

            double cell_cost = 1.0; // grid_ref[new_point.real()][new_point.imag()];
            if (cell_cost == -1)
                cell_cost = 1.0;
            
            double f = cell_cost;
            double g = std::abs(new_point - goal_point);
            std::vector<std::complex<double>> e = {start_point, new_point};
            AstarNode *node = new AstarNode(new_point, temp + partial_signature, f,g, NULL, e);
            pq.push(node);
        }

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
                for (int i = 0; i < traversed_signatures.size(); ++i)
                {
                    Eigen::VectorXd diff = traversed_signatures[i] - node->h_signature;
                    if (diff.isZero(0.0001))
                    {
                        is_already_seen = true;
                        break;
                    }
                }
                if (is_already_seen)
                    continue;
                // std::cout<<"H signature = "<< node->h_signature << std::endl;
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
                current_path.clear();
                for(int i=0;i<path.size();++i)
                {
                    current_path.push_back({path[i].first,path[i].second});
                }

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
                    double f = cell_cost;
                    double g = std::abs(new_point - goal_point);

                    std::stringstream ss;
                    ss << new_point << "-\n"
                    << temp;
                    std::string key = ss.str();
                    if (distance_count.find(key) == distance_count.end() || distance_count[key] > (f+g))
                    {
                        distance_count[key] = f+g;
                        std::vector<std::complex<double>> edge = {node->point, new_point};
                        AstarNode *new_node = new AstarNode(new_point, temp, f,g, node, edge);
                        pq.push(new_node);
                    }
                }
            }
        }
    }
    
    

    Eigen::VectorXd recompute_h_signature(std::vector<std::pair<int,int>> path, int index = -1)
    {
        int n = (index==-1)?(current_path.size()-1):index;

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
std::vector<Frontier> frontiers;
std::map<std::string, int> done_signatures;

std::vector<int> start_coordinates;
std::vector<int> goal_coordinates;
std::vector<int> current_start;
std::vector<int> current_goal;

std::vector<std::vector<std::pair<int,int>>> traversed_paths;
std::vector<std::pair<int,int>> current_path;
int current_path_index;
std::vector<Eigen::VectorXd> traversed_signatures;
};