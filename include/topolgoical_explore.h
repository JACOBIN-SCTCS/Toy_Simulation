
#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include<vector>
#include<iostream>
#include <algorithm>
#include <queue>
#include <Eigen/Dense>
#include <complex>
#include <unordered_map>
#include <set>


class TopolgicalExplore
{
    public:
        
        struct Frontier 
        {
            int x;
            int y;
            std::vector<std::pair<int,int>> cells;
        };
        
        struct DijkstraNode
        {
            std::complex<double> point;
            Eigen::VectorXd h_signature;
            double cost;
            struct DijkstraNode* parent; 
            std::vector<std::complex<double>> edge;

            DijkstraNode( std::complex<double> p , Eigen::VectorXd h , double c , struct DijkstraNode* pa ,std::vector<std::complex<double>> e) : point(p) , h_signature(h),cost(c),parent(pa) , edge(e) {}
        };
        
        TopolgicalExplore(std::vector<std::vector<int>>* g , std::vector<std::vector<int>> * o) : grid(g) , obstacles_seen(o)
        {
            ;
        }
      
        std::vector<std::pair<int,int>> getNeighbours(int x ,  int y , int max_x , int max_y)
        {
            std::vector<std::pair<int,int>> neighbours;
            int x_dir[] = {0,0,1,-1,1,1,-1,-1};
            int y_dir[] = {1,-1,0,0,1,-1,1,-1};
            for(int i=0;i<8;++i)
            {
                int x_new = x + x_dir[i];
                int y_new = y + y_dir[i];
                if(x_new < 0 || x_new >= max_x || y_new < 0 || y_new >= max_y)
                    continue;
                else
                    neighbours.push_back({x_new,y_new});

            }
            return neighbours;
        }

        void findFrontiers(int x, int y)
        {
            frontiers.clear();
            std::vector<std::vector<int>>& grid_ref = *grid;
            bool visited[grid_ref.size()][grid_ref[0].size()];
            bool frontier_tag[grid_ref.size()][grid_ref[0].size()];
            std::queue<std::pair<int,int>> q;
            q.push({x,y});
            visited[x][y] = true;
            while(!q.empty())
            {
                std::pair<int,int> curr  = q.front();
                q.pop();
                int i = curr.first;
                int j = curr.second;
                std::vector<std::pair<int,int>> neighbours = getNeighbours(i,j,grid_ref.size(),grid_ref[0].size());
                for(int k=0;k<neighbours.size();++k)
                {
                    int x_new = neighbours[k].first;
                    int y_new = neighbours[k].second;
                    if(visited[x_new][y_new] == true || frontier_tag[x_new][y_new]==true)
                        continue;
                    visited[x_new][y_new] = true;
                    
                    if(grid_ref[x_new][y_new]==0)
                        continue;
                    
                    else if(grid_ref[x_new][y_new]==-1)
                    {
                        std::vector<std::pair<int,int>> cell_neighbours = getNeighbours(x_new,y_new,grid_ref.size(),grid_ref[0].size());
                        for(int j =0;j<4;++j)
                        {
                            int x_prime_new = cell_neighbours[j].first;
                            int y_prime_new = cell_neighbours[j].second;
                            if(grid_ref[x_prime_new][y_prime_new]==1)
                            {
                                frontier_tag[x_new][y_new] = true;
                                break;
                            }
                        }
                        if(frontier_tag[x_new][y_new]==true)
                        {
                            std::queue<std::pair<int,int>> new_q;
                            new_q.push({x_new,y_new});
                            std::vector<std::pair<int,int>> frontier_cells;
                            frontier_cells.push_back({x_new,y_new});
                            while(!new_q.empty())
                            {
                                std::vector<std::pair<int,int>> cell_neighbors = getNeighbours(new_q.front().first,new_q.front().second,grid_ref.size(),grid_ref[0].size());
                                new_q.pop();
                                for(int j=0;j<cell_neighbors.size();++j)
                                {
                                    int x_prime_new = cell_neighbors[j].first;
                                    int y_prime_new = cell_neighbors[j].second;
                                    if(visited[x_prime_new][y_prime_new]==true || frontier_tag[x_prime_new][y_prime_new]==true)
                                        continue;
                                    visited[x_prime_new][y_prime_new] = true;
                                
                                    if(grid_ref[x_prime_new][y_prime_new]==-1)
                                    {
                                        std::vector<std::pair<int,int>> frontier_cell_neighbours = getNeighbours(x_prime_new,y_prime_new,grid_ref.size(),grid_ref[0].size());  
                                        for(int k=0;k<4;++k)
                                        {
                                            int x_prime_prime_new = frontier_cell_neighbours[k].first;
                                            int y_prime_prime_new = frontier_cell_neighbours[k].second;
                                            if(grid_ref[x_prime_prime_new][y_prime_prime_new]==1)
                                            {
                                                frontier_tag[x_prime_new][y_prime_new] = true;
                                                break;
                                            }
                                        }
                                        if(frontier_tag[x_prime_new][y_prime_new]==true)
                                        {
                                            new_q.push({x_prime_new,y_prime_new});
                                            frontier_cells.push_back({x_prime_new,y_prime_new});
                                        }                                      
                                    }
                                }
                            } 
                            Frontier frontier;
                            frontier.x = x_new;
                            frontier.y = y_new;
                            frontier.cells = frontier_cells;
                            frontiers.push_back(frontier);
                        }
                    }
                    else
                    {
                        q.push({x_new,y_new});
                   
                    }
                }

            }
        }
        


        std::vector<std::vector<std::pair<int,int>>> getPaths(int x , int y)
        {
            auto customOp = [](const std::complex<double>& a, const std::complex<double>& b) -> double
            {
                double minimum_phase_difference = std::arg(b) - std::arg(a);
                for(int i=-2;i<3;++i)
                {
                    for(int j = -2; j<3;++j)
                    {
                        double phase_difference =  (std::arg(b) +2*M_PIf64*i) - (std::arg(a) + 2*M_PIf64*j);
                        if(std::abs(phase_difference) < std::abs(minimum_phase_difference))
                        {
                            minimum_phase_difference = phase_difference;
                        }  
                    }
                }
                return minimum_phase_difference;

            };

            std::vector<std::vector<int>>& obstacles_ref = *obstacles_seen;
            int limit = std::min(4, std::max((int)std::pow(2,obstacles_ref.size()),1));
            int count = 0;
            findFrontiers(x,y);
            //std::vector<std::vector<int>>& obstacles_ref = *obstacles_seen;
            std::vector<std::vector<int>>& grid_ref = *grid;
            Eigen::VectorXcd obstacle_points = Eigen::VectorXcd::Zero(obstacles_ref.size());
            for(unsigned int i= 0 ; i < obstacles_ref.size();++i)
                obstacle_points(i) = std::complex<double>(obstacles_ref[i][0],obstacles_ref[i][1]);

            std::complex<double> start_point(x,y);
            std::complex<double> goal_point(frontiers[0].x,frontiers[0].y);
            

            std::vector<std::complex<double>> directions = {
                std::complex<double>(1.0,0.0),
                std::complex<double>(0.0,1.0),
                std::complex<double>(-1.0,0.0),
                std::complex<double>(0.0,-1.0),
                std::complex<double>(1.0,1.0),
                std::complex<double>(-1.0,1.0),
                std::complex<double>(1.0,-1.0),
                std::complex<double>(-1.0,-1.0),
            };


            std::vector<std::vector<std::pair<int,int>>> paths;	
            std::priority_queue<DijkstraNode*, std::vector<DijkstraNode*>, std::function<bool(DijkstraNode*, DijkstraNode*)>> pq([](DijkstraNode* a, DijkstraNode* b) { return a->cost > b->cost; });
            std::unordered_map<std::string,double> distance_count;
            std::set<std::string> visited;
            std::stringstream ss;
            Eigen::VectorXd zeros = Eigen::VectorXd::Zero(obstacles_ref.size());
	        ss << start_point << "-\n"<< zeros;
	        distance_count[ss.str()] = std::abs(goal_point-start_point);

	        for(unsigned int i=0;i<directions.size();++i)
	        {
                std::complex<double> new_point = start_point + directions[i];
                //unsigned int new_point_index = costmap_->getIndex((unsigned int)new_point.real(),(unsigned int)new_point.imag());

                if(real(new_point)<0.0 || real(new_point)>grid_ref.size() || imag(new_point)<0.0 || imag(new_point)>grid_ref[0].size() || grid_ref[new_point.real()][new_point.imag()]==0)
                        continue;
                Eigen::VectorXcd s_vec = Eigen::VectorXcd::Constant(obstacle_points.size(),start_point) - obstacle_points;
                Eigen::VectorXcd e_vec = Eigen::VectorXcd::Constant(obstacle_points.size(),new_point) - obstacle_points;
                Eigen::VectorXd temp = s_vec.array().binaryExpr(e_vec.array(),customOp);
                
                double cell_cost = grid_ref[new_point.real()][new_point.imag()];
                if(cell_cost == -1)
                    cell_cost = 1.0;
                double c = cell_cost + std::abs(new_point-goal_point);
                std::vector<std::complex<double>> e = {start_point,new_point};
                DijkstraNode* node = new DijkstraNode(new_point,temp,c,NULL,e);
                pq.push(node);
	        }
	
            while(!pq.empty())
            {
                DijkstraNode* node = pq.top();
                pq.pop();
                if(node->point == goal_point)
                {
                    std::stringstream ss;
                    ss << node->h_signature;
                    std::string key = ss.str();
                    Eigen::VectorXd filtered = (1.0/(2*M_PIf64))*node->h_signature;
			        if((filtered.array()> 1.0).any() || (filtered.array() < -1.0).any())
				        continue;
			
                
                    if(visited.find(key) == visited.end())
                    {
                        std::cout<<"Found="<<key<<std::endl;
                        count +=1;
                        visited.insert(key);
                        std::vector<std::pair<int,int>> path;
                        DijkstraNode* temp = node;
                        while(temp!=NULL)
                        {
                            int current_point_x  = temp->point.real(), current_point_y = temp->point.imag();
                            path.push_back(std::pair<int,int>(current_point_x,current_point_y));
                            temp = temp->parent;
                        }
                        std::reverse(path.begin(),path.end());
                        paths.push_back(path);
                        if(count>=limit)
                        {
                            return paths;
                        }
                    }
                }
                else
                {
                    for(unsigned int i=0;i<directions.size();++i)
                    {
                        std::complex<double> new_point = node->point + directions[i];
                        if(real(new_point)<0.0 || real(new_point)> grid_ref.size() || imag(new_point)<0.0 || imag(new_point)>grid_ref[0].size() || grid_ref[new_point.real()][new_point.imag()]==0)
                            continue;
                        Eigen::VectorXcd s_vec = Eigen::VectorXcd::Constant(obstacle_points.size(),node->point) - obstacle_points;
                        Eigen::VectorXcd e_vec = Eigen::VectorXcd::Constant(obstacle_points.size(),new_point) - obstacle_points;
                        Eigen::VectorXd t =   s_vec.array().binaryExpr(e_vec.array(),customOp);
                        Eigen::VectorXd temp =  node->h_signature + t;
                        Eigen::VectorXd filtered = (1.0/(2*M_PIf64))*temp;
                        if((filtered.array()> 1.0).any() || (filtered.array() < -1.0).any())
                            continue;
                        double cell_cost = grid_ref[new_point.real()][new_point.imag()];
                        if(cell_cost == -1)
                            cell_cost = 1.0;
                        double c = node->cost + cell_cost + std::abs(new_point-goal_point);
                        
                        std::stringstream ss;
                        ss << new_point << "-\n"<< temp;
                        std::string key = ss.str();
                        if(distance_count.find(key)==distance_count.end() || distance_count[key] > c)
                        {
                            distance_count[key] = c;
                            std::vector<std::complex<double>> edge = { node->point,new_point}; 
                            DijkstraNode* new_node = new DijkstraNode(new_point,temp,c,node,edge);
                            pq.push(new_node);
                        }
                    }
                }

	        }
            return paths;
        }   

        std::vector<std::vector<int>>* grid;
        std::vector<std::vector<int>>* obstacles_seen;
        std::vector<Frontier> frontiers;
};