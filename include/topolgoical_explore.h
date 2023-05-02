
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

    struct PathsToFollow
    {
        std::vector<int> paths_weight;
        std::vector<std::vector<std::pair<int, int>>> paths;
    };

    TopolgicalExplore(std::vector<std::vector<int>> *g, std::vector<std::vector<int>> *o) : grid(g), obstacles_seen(o)
    {
        ;
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

    struct PathsToFollow getPaths(int x, int y, int end_x, int end_y)
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

        struct PathsToFollow paths_to_follow;

        std::vector<std::vector<int>> &obstacles_ref = *obstacles_seen;
        int limit = std::min(2, std::max((int)std::pow(2, obstacles_ref.size()), 1));
        int count = 0;
        // std::cout<<"About to find frontiers"<<std::endl;
        // findFrontiers(x,y);
        // std::vector<std::vector<int>>& obstacles_ref = *obstacles_seen;
        std::vector<std::vector<int>> &grid_ref = *grid;
        Eigen::VectorXcd obstacle_points = Eigen::VectorXcd::Zero(obstacles_ref.size());
        for (unsigned int i = 0; i < obstacles_ref.size(); ++i)
            obstacle_points(i) = std::complex<double>(obstacles_ref[i][0], obstacles_ref[i][1]);

        std::complex<double> start_point(x, y);
        // std::vector<int> goal_coords = getGoalCoordinate(x,y);
        std::vector<int> goal_coords = {end_x, end_y};
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

        std::vector<std::vector<std::pair<int, int>>> paths;
        std::priority_queue<AstarNode *, std::vector<AstarNode *>, std::function<bool(AstarNode *, AstarNode *)>> pq([](AstarNode *a, AstarNode *b)
                                                                                                                     { return (a->f+a->g) > (b->f + b->g); });
        std::unordered_map<std::string, double> distance_count;
        std::set<std::string> visited;
        std::stringstream ss;
        std::vector<Eigen::VectorXd> visited_h_signatures;

        Eigen::VectorXd zeros = Eigen::VectorXd::Zero(obstacles_ref.size());
        ss << start_point << "-\n"
           << zeros;
        distance_count[ss.str()] = std::abs(goal_point - start_point);

        for (unsigned int i = 0; i < directions.size(); ++i)
        {
            std::complex<double> new_point = start_point + directions[i];
            // unsigned int new_point_index = costmap_->getIndex((unsigned int)new_point.real(),(unsigned int)new_point.imag());

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
            AstarNode *node = new AstarNode(new_point, temp, f,g, NULL, e);
            pq.push(node);
        }

        while (!pq.empty())
        {
            AstarNode *node = pq.top();
            pq.pop();
            if (node->point == goal_point)
            {
                std::stringstream ss;
                ss << node->h_signature;
                std::string key = ss.str();
                Eigen::VectorXd filtered = (1.0 / (2 * M_PIf64)) * node->h_signature;
                if ((filtered.array() > 1.0).any() || (filtered.array() < -1.0).any())
                    continue;

                if (visited.find(key) == visited.end())
                {

                    std::cout << "Found=" << key << std::endl;
                    count += 1;
                    visited.insert(key);
                    visited_h_signatures.push_back(node->h_signature);
                    if (paths.size() > 0)
                    {
                        for (unsigned int i = 0; i < visited_h_signatures.size() - 1; ++i)
                        {
                            Eigen::VectorXd diff = visited_h_signatures[i] - node->h_signature;
                            Eigen::VectorXd filtered = (1.0 / (2 * M_PIf64)) * diff;

                            int size_ = filtered.size();
                            // std::cout<<size_<<std::endl;

                            auto boolean_filtered = filtered.unaryExpr([](double x)
                                                                       { return (x == (double)(1) || x == (double)(-1)); });
                            std::cout << "Boolean Filtered = " << boolean_filtered << std::endl;
                            // std::cout<<"Boolean filtered = "<<boolean_filtered<<std::endl;

                            // std::cout<<"filtered="<<filtered<<std::endl;
                            // auto boolean_filtered = ((filtered.array()<=-1) || (filtered.array()>=1));
                            // std::cout<<"Boolean filtered = "<<boolean_filtered<<std::endl;
                            // Eigen::Array<bool, Eigen::Dynamic, 1> boolArray = ((filtered.array() ==-1.0)|| (filtered.array() == 1.0));
                            // std::cout<<"boolArray="<<boolArray<<std::endl;
                            // Eigen::VectorXi indices = boolArray.nonZeros();
                        }
                    }
                    std::vector<std::pair<int, int>> path;
                    AstarNode *temp = node;
                    while (temp != NULL)
                    {
                        int current_point_x = temp->point.real(), current_point_y = temp->point.imag();
                        path.push_back(std::pair<int, int>(current_point_x, current_point_y));
                        temp = temp->parent;
                    }
                    std::reverse(path.begin(), path.end());
                    paths.push_back(path);
                    if (count >= limit)
                    {
                        paths_to_follow.paths = paths;
                        return paths_to_follow;
                    }
                }
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
                    double f = node->f + cell_cost;
                    double g = std::abs(new_point - goal_point);

                    std::stringstream ss;
                    ss << new_point << "-\n"
                       << temp;
                    std::string key = ss.str();
                    if (distance_count.find(key) == distance_count.end() || distance_count[key] > f)
                    {
                        distance_count[key] = f;
                        std::vector<std::complex<double>> edge = {node->point, new_point};
                        AstarNode *new_node = new AstarNode(new_point, temp, f,g, node, edge);
                        pq.push(new_node);
                    }
                }
            }
        }
        paths_to_follow.paths = paths;
        return paths_to_follow;
    }

    std::vector<std::pair<int, int>> getPath(int start_x, int start_y, int end_x, int end_y)
    {
        struct DijkstraNode
        {
            int x;
            int y;
            double f;
            double g;
            struct DijkstraNode *parent;
        };

        std::vector<std::pair<int, int>> path;
        std::vector<std::vector<int>> &grid_ref = *grid;
        int distance[grid_ref.size()][grid_ref[0].size()];

        for (int i = 0; i < grid_ref.size(); ++i)
        {
            for (int j = 0; j < grid_ref[0].size(); ++j)
            {
                distance[i][j] = INT_MAX;
            }
        }

        distance[start_x][start_y] = 0;
        std::priority_queue<DijkstraNode *, std::vector<DijkstraNode *>, std::function<bool(DijkstraNode *, DijkstraNode *)>> pq([](DijkstraNode *a, DijkstraNode *b)
                                                                                                                                 { return ((a->f + a->g) > (b->f + b->g)); });

        DijkstraNode *start_node = new DijkstraNode();
        start_node->x = start_x;
        start_node->y = start_y;
        start_node->g = sqrt((end_x - start_x) * (end_x - start_x) + (end_y - start_y) * (end_y - start_y));
        start_node->f = 0;
        start_node->parent = NULL;
        pq.push(start_node);

        while (!pq.empty())
        {
            DijkstraNode *curr = pq.top();
            // std::cout<<"("<<curr->x<<","<<curr->y<<")"<<std::endl;
            pq.pop();
            int x = curr->x;
            int y = curr->y;
            if (x == end_x && y == end_y)
            {
                std::cout << "Path found";
                DijkstraNode *ptr = curr;
                while (curr != NULL)
                {
                    std::cout << "(" << curr->x << "," << curr->y << ")" << std::endl;
                    path.push_back({curr->x, curr->y});
                    curr = curr->parent;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            std::vector<std::pair<int, int>> neighbours = getNeighbours(x, y, grid_ref.size(), grid_ref[0].size());
            for (int i = 0; i < neighbours.size(); ++i)
            {
                int x_new = neighbours[i].first;
                int y_new = neighbours[i].second;
                if (x_new == end_x && y_new == end_y)
                {
                    int g_new = sqrt((x_new - end_x) * (x_new - end_x) + (y_new - end_y) * (y_new - end_y));
                    int f_new = curr->f + 1;
                    if ((f_new + g_new) < distance[f_new][g_new])
                    {
                        distance[f_new][g_new] = f_new + g_new;
                        DijkstraNode *new_node = new DijkstraNode();
                        new_node->x = x_new;
                        new_node->y = y_new;
                        new_node->g = curr->g + sqrt((x_new - end_x) * (x_new - end_x) + (y_new - end_y) * (y_new - end_y));
                        new_node->f = curr->f + 1;
                        new_node->parent = curr;
                        pq.push(new_node);
                        continue;
                    }
                }

                if (grid_ref[x_new][y_new] == 0 || grid_ref[x_new][y_new] == -1)
                    continue;
                else
                {
                    int g_new = sqrt((x_new - end_x) * (x_new - end_x) + (y_new - end_y) * (y_new - end_y));
                    int f_new = curr->f + 1;
                    if ((f_new + g_new) < distance[f_new][g_new])
                    {
                        distance[f_new][g_new] = f_new + g_new;
                        DijkstraNode *new_node = new DijkstraNode();
                        new_node->x = x_new;
                        new_node->y = y_new;
                        new_node->g = g_new;
                        new_node->f = f_new;
                        new_node->parent = curr;
                        pq.push(new_node);
                    }
                }
            }
        }
        return path;
    }

    std::vector<int> getGoalCoordinate(int start_x, int start_y)
    {
        std::vector<std::vector<int>> &grid_ref = *grid;
        std::vector<std::vector<int>> unknown_cells;
        // std::vector<int> weights;
        // std::complex<double> start_point(start_x,start_y);

        // double total_distance = 0;

        for (int i = 0; i < grid_ref.size(); ++i)
        {
            for (int j = 0; j < grid_ref[0].size(); ++j)
            {
                if (grid_ref[i][j] == -1)
                {
                    unknown_cells.push_back({i, j});
                    std::complex<double> current_point(i, j);
                    // double current_distance = std::abs(current_point-start_point);
                    // weights.push_back(current_distance);
                    // total_distance+= current_distance;
                }
            }
        }

        // for(int i = 0; i< weights.size();++i)
        // {
        //     weights[i] = 1 - (weights[i]/total_distance);
        // }

        std::vector<int> res;
        std::random_device rd;
        std::mt19937 gen(rd());

        std::uniform_int_distribution<> dis(0, unknown_cells.size());
        // std::discrete_distribution<> dis(weights.begin(),weights.end());
        int idx = dis(gen);
        res.push_back(unknown_cells[idx][0]);
        res.push_back(unknown_cells[idx][1]);
        return res;
    }

    struct NonHomologouspath
    {
        std::vector<std::pair<int, int>> path;
        std::vector<Eigen::VectorXd> visited_non_homologous_paths;
    };

    struct NonHomologouspath getNonHomologousPaths(int x, int y, std::vector<Eigen::VectorXd> visited_h_signatures)
    {
        struct NonHomologouspath p;

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

        int count = 0;
        Eigen::VectorXcd obstacle_points = Eigen::VectorXcd::Zero(obstacles_ref.size());
        for (unsigned int i = 0; i < obstacles_ref.size(); ++i)
            obstacle_points(i) = std::complex<double>(obstacles_ref[i][0], obstacles_ref[i][1]);

        std::complex<double> start_point(x, y);
        std::vector<int> goal_coords = {59, 59};
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
           << zeros;
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
            AstarNode *node = new AstarNode(new_point, temp, f,g, NULL, e);
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
                for (int i = 0; i < visited_h_signatures.size(); ++i)
                {
                    Eigen::VectorXd diff = visited_h_signatures[i] - node->h_signature;
                    // std::cout<<"Visited = "<<visited_h_signatures[i]<<std::endl;
                    // std::cout<<"Calculated differnece = "<<diff<<std::endl;
                    if (diff.isZero(0.0001))
                    {
                        is_already_seen = true;
                        break;
                    }
                }
                if (is_already_seen)
                    continue;
                std::cout<<"H signature = "<< node->h_signature << std::endl;
                visited_h_signatures.push_back(node->h_signature);
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
                p.path = path;
                p.visited_non_homologous_paths = visited_h_signatures;
                return p;
            
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
                    if (distance_count.find(key) == distance_count.end() || distance_count[key] > f)
                    {
                        distance_count[key] = f;
                        std::vector<std::complex<double>> edge = {node->point, new_point};
                        AstarNode *new_node = new AstarNode(new_point, temp, f,g, node, edge);
                        pq.push(new_node);
                    }
                }
            }
        }
        return p;
    }
    
    

    std::vector<Eigen::VectorXd>  recompute_h_signature(std::vector<std::pair<int,int>> path)
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

        std::vector<Eigen::VectorXd> h_signatures;
        std::vector<std::vector<int>> &obstacles_ref = *obstacles_seen;
        Eigen::VectorXcd obstacle_points = Eigen::VectorXcd::Zero(obstacles_ref.size());
        for (unsigned int i = 0; i < obstacles_ref.size(); ++i)
            obstacle_points(i) = std::complex<double>(obstacles_ref[i][0], obstacles_ref[i][1]);
        Eigen::VectorXd sum = Eigen::VectorXd::Zero(obstacle_points.size());
        for(int i=1;i<path.size();++i)
        {
            Eigen::VectorXcd s_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), std::complex<double>(path[i-1].first, path[i-1].second)) - obstacle_points;
            Eigen::VectorXcd e_vec = Eigen::VectorXcd::Constant(obstacle_points.size(), std::complex<double>(path[i].first, path[i].second)) - obstacle_points;
            Eigen::VectorXd t = s_vec.array().binaryExpr(e_vec.array(), customOp);
            Eigen::VectorXd temp = t;
            sum += temp;
            h_signatures.push_back(sum);
        }
        return h_signatures;
    }



std::vector<std::vector<int>> *grid;
std::vector<std::vector<int>> *obstacles_seen;
std::vector<Frontier> frontiers;
std::map<std::string, int> done_signatures;
};