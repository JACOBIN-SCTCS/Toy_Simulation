#include <iostream>
#include <vector>
#include <utility>
#include <unordered_map>
#include <set>
#include <queue>
#include <functional>
#include <climits>
#include <algorithm>
#include<math.h>
#include<complex>

class FrontierExplore
{
    public:
        struct Frontier 
        {
            int x;
            int y;
            std::vector<std::pair<int,int>> cells;
        };

        struct AstarNode
        {
            int x;
            int y;
            double f;
            double g;
            double cost;
            struct AstarNode* parent;
        };

        FrontierExplore(std::vector<std::vector<int>> *grid, std::vector<std::vector<int>> *obs) : grid(grid), obstacles(obs)
        {
            ;
        }

        std::vector<std::pair<int,int>> getNeighbours(int x ,  int y , int max_x , int max_y,bool get_four_neighbours = false)
        {
            std::vector<std::pair<int,int>> neighbours;
            if(!get_four_neighbours)
            {
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
            }
            else
            {
                int x_dir[] = {0,0,1,-1};
                int y_dir[] = {1,-1,0,0};
                for(int i=0;i<4;++i)
                {
                    int x_new = x + x_dir[i];
                    int y_new = y + y_dir[i];
                    if(x_new < 0 || x_new >= max_x || y_new < 0 || y_new >= max_y)
                        continue;
                    else
                        neighbours.push_back({x_new,y_new});

                }
            }
            return neighbours;
        }

        void findFrontiers(int x, int y)
        {
            std::vector<std::vector<int>> &obstacles_ref = *obstacles;
            frontiers.clear();
            std::vector<std::vector<int>>& grid_ref = *grid;
            bool visited[grid_ref.size()][grid_ref[0].size()] = {false};
            bool frontier_tag[grid_ref.size()][grid_ref[0].size()] = {false};
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
                    // for(int i=0;i<obstacles_ref.size();++i)
                    // {
                    //     if(obstacles_ref[i][0] == x_new && obstacles_ref[i][1]==y_new)
                    //         continue;
                    // }
                    if(grid_ref[x_new][y_new]==-1)
                    {
                        std::vector<std::pair<int,int>> cell_neighbours = getNeighbours(x_new,y_new,grid_ref.size(),grid_ref[0].size(),true);
                        for(int j =0;j<cell_neighbours.size();++j)
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
                                        std::vector<std::pair<int,int>> frontier_cell_neighbours = getNeighbours(x_prime_new,y_prime_new,grid_ref.size(),grid_ref[0].size(),true);  
                                        for(int k=0;k<frontier_cell_neighbours.size();++k)
                                        {
                                            int x_prime_prime_new = frontier_cell_neighbours[k].first;
                                            int y_prime_prime_new = frontier_cell_neighbours[k].second;
                                            if(grid_ref[x_prime_prime_new][y_prime_prime_new]==1)  // some bug here
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

        std::vector<std::pair<int,int>> getPath(int start_x, int start_y , int end_x, int end_y)
        {
            std::cout<<"Source = (" << start_x <<","<<start_y<<")\n";
            std::cout<<"Destination = (" << end_x <<","<<end_y<<")\n";

            std::vector<std::vector<int>>& grid_ref = *grid;
            int distance[grid_ref.size()][grid_ref[0].size()];

            for(int i=0;i<grid_ref.size();++i)
            {
                for(int j=0;j<grid_ref[0].size();++j)
                {
                    distance[i][j] = INT_MAX;
                }
            }


            distance[start_x][start_y] = 0;
    	    std::priority_queue<AstarNode*, std::vector<AstarNode*>, std::function<bool(AstarNode*, AstarNode*)>> pq([](AstarNode* a, AstarNode* b) {
                return  ((a->f + a->g) > (b->f + b->g));
                // return ((a->cost) > (b->cost));
             });

            double cell_cost = 0.0;
            
            std::vector<std::vector<int>>& obstacles_ref = *obstacles;
            
            // // Can comment out the for loop
            // for(int i=0;i<obstacles_ref.size();++i)
            // {
            //     std::complex<double> map_point(start_x,start_y);
            //     std::complex<double> obs_point(obstacles_ref[i][0],obstacles_ref[i][1]);
            //     cell_cost = cell_cost + std::abs(map_point - obs_point);
            // }

            AstarNode* start_node = new AstarNode();
            start_node->x = start_x;
            start_node->y = start_y;
            start_node->g = sqrt((end_x-start_x)*(end_x-start_x) + (end_y-start_y)*(end_y-start_y));
            start_node->f = cell_cost;
            // start_node->cost = 0.0;
            start_node->parent = NULL;
            pq.push(start_node);

            while(!pq.empty())
            {
                AstarNode* curr = pq.top();
                // std::cout<<"("<<curr->x<<","<<curr->y<<")"<<std::endl;
                pq.pop();
                int x = curr->x;
                int y = curr->y;
                if(x==end_x && y==end_y)
                {
                    // std::cout <<"Path found";
                    std::vector<std::pair<int,int>> path;
                    AstarNode* ptr = curr;
                    while(curr!=NULL)
                    {
                        // std::cout<<"("<<curr->x<<","<<curr->y<<")"<<std::endl;
                        path.push_back({curr->x,curr->y});
                        curr = curr->parent;
                    }
                    std::reverse(path.begin(),path.end());
                    return path;
                }
                std::vector<std::pair<int,int>> neighbours = getNeighbours(x,y,grid_ref.size(),grid_ref[0].size());
                for(int i=0;i<neighbours.size();++i)
                {
                    int x_new = neighbours[i].first;
                    int y_new = neighbours[i].second;
                    if(x_new==end_x && y_new==end_y)
                    {
                        // int g_new = sqrt((x_new-end_x)*(x_new-end_x) + (y_new-end_y)*(y_new-end_y));
                        // int f_new = curr->f  + 1;
                        // double cost_new = curr->cost +  sqrt((x_new-end_x)*(x_new-end_x) + (y_new-end_y)*(y_new-end_y)) + 1;
                        // double new_cell_cost = 1.0;
                        std::complex<double> current_point_complex(x,y);
                        std::complex<double> neighbour_complex(x_new,y_new);
                        double new_cell_cost = std::abs(neighbour_complex - current_point_complex);
                        // for(int i=0;i<obstacles_ref.size();++i)
                        // {
                        //     std::complex<double> map_point(start_x,start_y);
                        //     std::complex<double> obs_point(obstacles_ref[i][0],obstacles_ref[i][1]);
                        //     new_cell_cost = cell_cost + std::abs(map_point - obs_point);
                        // }
                        
                        double f_new = curr->f +  new_cell_cost ;
                        
                        
                        double g_new = sqrt((x_new-end_x)*(x_new-end_x) + (y_new-end_y)*(y_new-end_y));
                        // if((f_new+g_new) < distance[x_new][y_new])
                        if(f_new < distance[x_new][y_new])
                        {
                            // distance[x_new][y_new] = f_new + g_new;
                            distance[x_new][y_new] = f_new;
                            AstarNode* new_node = new AstarNode();
                            new_node->x = x_new;
                            new_node->y = y_new;
                            new_node->g = g_new; //curr->g + sqrt((x_new-end_x)*(x_new-end_x) + (y_new-end_y)*(y_new-end_y));
                            new_node->f = f_new;
                            // new_node->cost = cost_new;
                            new_node->parent = curr;
                            pq.push(new_node);
                            continue;
                        }
                    }

                    if(grid_ref[x_new][y_new]==0 || grid_ref[x_new][y_new]==-1)
                        continue;
                    else
                    {
                        // int g_new = sqrt((x_new-end_x)*(x_new-end_x) + (y_new-end_y)*(y_new-end_y));
                        // int f_new = curr->f  + 1;
                        // double cost_new = curr->cost + sqrt((x_new-end_x)*(x_new-end_x) + (y_new-end_y)*(y_new-end_y)) + 1;
                        // double new_cell_cost = 1.0;
                        std::complex<double> current_point_complex(x,y);
                        std::complex<double> neighbour_complex(x_new,y_new);
                        double new_cell_cost = std::abs(neighbour_complex - current_point_complex);
                        // for(int i=0;i<obstacles_ref.size();++i)
                        // {
                        //     std::complex<double> map_point(start_x,start_y);
                        //     std::complex<double> obs_point(obstacles_ref[i][0],obstacles_ref[i][1]);
                        //     new_cell_cost = cell_cost + std::abs(map_point - obs_point);
                        // }
                        
                        double f_new = curr->f +  new_cell_cost;
                        double g_new = sqrt((x_new-end_x)*(x_new-end_x) + (y_new-end_y)*(y_new-end_y));
                        // if((f_new+g_new) < distance[x_new][y_new]) //error here
                        if(f_new < distance[x_new][y_new])
                        {
                            distance[x_new][y_new] = f_new;
                            // distance[x_new][y_new] = cost_new;
                            AstarNode* new_node = new AstarNode();
                            new_node->x = x_new;
                            new_node->y = y_new;
                            new_node->g = g_new;
                            new_node->f = f_new;
                            // new_node->cost = cost_new;
                            new_node->parent = curr;
                            pq.push(new_node);
                        }
                    }
                }

            }
            return {};
        }

        void printElement()
        {
            std::vector<std::vector<int>>& grid_ref = *grid;
            int ele = grid_ref[0][0];
            std::cout<<ele<<std::endl;
        }

        std::vector<std::vector<int>>* grid;
        std::vector<std::vector<int>> *obstacles;
        std::vector<Frontier> frontiers;
};