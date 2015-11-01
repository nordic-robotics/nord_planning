#include "ros/ros.h"
#include "ros/package.h"
#include <fstream>
#include <sstream>
#include <string>
#include "map.hpp"
#include <math.h>  

class Maps{
	public:
		int n_wall=29;int n_point=19;
	
		Maps(): pot_wall(n_wall,std::vector<long int> (n_wall)), pot_point(n_point,std::vector<long int> (n_point)){
						
			//n_wall==19
			/*pot_wall={{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},
					{100,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,100},
					{100,200,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,200,100},
					{100,200,300,400,400,400,400,400,400,400,400,400,400,400,400,400,300,200,100},
					{100,200,300,400,500,500,500,500,500,500,500,500,500,500,500,400,300,200,100},
					{100,200,300,400,500,600,600,600,600,600,600,600,600,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,700,700,700,700,700,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,800,800,800,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,900,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,1000,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,900,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,800,800,800,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,700,700,700,700,700,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,600,600,600,600,600,600,600,600,500,400,300,200,100},
					{100,200,300,400,500,500,500,500,500,500,500,500,500,500,500,400,300,200,100},
					{100,200,300,400,400,400,400,400,400,400,400,400,400,400,400,400,300,200,100},
					{100,200,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,200,100},
					{100,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,100},
					{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100}};*/
			/*pot_wall={{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},
					{100,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,100},
					{100,200,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,200,100},
					{100,200,300,400,400,400,400,400,400,400,400,400,400,400,400,400,400,400,400,400,400,400,400,400,400,400,300,200,100},
					{100,200,300,400,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,400,300,200,100},
					{100,200,300,400,500,600,600,600,600,600,600,600,600,600,600,600,600,600,600,600,600,600,600,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,900,900,900,900,900,900,900,900,900,900,900,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,1000,1100,1100,1100,1100,1100,1100,1100,1100,1100,1000,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,1000,1100,1200,1200,1200,1200,1200,1200,1200,1100,1000,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1300,1300,1300,1300,1200,1100,1000,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1400,1400,1300,1200,1100,1000,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500,1400,1300,1200,1100,1000,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1400,1400,1300,1200,1100,1000,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1300,1300,1300,1300,1200,1100,1000,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,1000,1100,1200,1200,1200,1200,1200,1200,1200,1100,1000,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,1000,1100,1100,1100,1100,1100,1100,1100,1100,1100,1000,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,900,900,900,900,900,900,900,900,900,900,900,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,600,600,600,600,600,600,600,600,600,600,600,600,600,600,600,600,600,600,500,400,300,200,100},
					{100,200,300,400,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,500,400,300,200,100},
					{100,200,300,400,400,400,400,400,400,400,400,400,400,400,400,400,400,400,400,400,400,400,400,400,400,400,300,200,100},
					{100,200,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,200,100},
					{100,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,100},
					{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100}};*/
					
			pot_wall={{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},
					{100,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,100},
					{100,300,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,300,100},
					{100,300,900,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,900,300,100},
					{100,300,900,2700,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,72900,72900,72900,72900,72900,72900,72900,72900,72900,72900,72900,72900,72900,72900,72900,72900,72900,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,72900,218700,218700,218700,218700,218700,218700,218700,218700,218700,218700,218700,218700,218700,218700,218700,72900,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,72900,218700,656100,656100,656100,656100,656100,656100,656100,656100,656100,656100,656100,656100,656100,218700,72900,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,72900,218700,656100,1968300,1968300,1968300,1968300,1968300,1968300,1968300,1968300,1968300,1968300,1968300,656100,218700,72900,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,72900,218700,656100,1968300,5904900,5904900,5904900,5904900,5904900,5904900,5904900,5904900,5904900,1968300,656100,218700,72900,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,72900,218700,656100,1968300,5904900,17714700,17714700,17714700,17714700,17714700,17714700,17714700,5904900,1968300,656100,218700,72900,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,72900,218700,656100,1968300,5904900,17714700,53144100,53144100,53144100,53144100,53144100,17714700,5904900,1968300,656100,218700,72900,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,72900,218700,656100,1968300,5904900,17714700,53144100,159432300,159432300,159432300,53144100,17714700,5904900,1968300,656100,218700,72900,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,72900,218700,656100,1968300,5904900,17714700,53144100,159432300,478296900,159432300,53144100,17714700,5904900,1968300,656100,218700,72900,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,72900,218700,656100,1968300,5904900,17714700,53144100,159432300,159432300,159432300,53144100,17714700,5904900,1968300,656100,218700,72900,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,72900,218700,656100,1968300,5904900,17714700,53144100,53144100,53144100,53144100,53144100,17714700,5904900,1968300,656100,218700,72900,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,72900,218700,656100,1968300,5904900,17714700,17714700,17714700,17714700,17714700,17714700,17714700,5904900,1968300,656100,218700,72900,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,72900,218700,656100,1968300,5904900,5904900,5904900,5904900,5904900,5904900,5904900,5904900,5904900,1968300,656100,218700,72900,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,72900,218700,656100,1968300,1968300,1968300,1968300,1968300,1968300,1968300,1968300,1968300,1968300,1968300,656100,218700,72900,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,72900,218700,656100,656100,656100,656100,656100,656100,656100,656100,656100,656100,656100,656100,656100,218700,72900,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,72900,218700,218700,218700,218700,218700,218700,218700,218700,218700,218700,218700,218700,218700,218700,218700,72900,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,72900,72900,72900,72900,72900,72900,72900,72900,72900,72900,72900,72900,72900,72900,72900,72900,72900,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,24300,8100,2700,900,300,100},
					{100,300,900,2700,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,8100,2700,900,300,100},
					{100,300,900,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,2700,900,300,100},
					{100,300,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,300,100},
					{100,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,100},
					{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100}};
				
			//n_point=5
			/*pot_point={{200,200,200,200,200},
					{200,400,400,400,200},
					{200,400,600,400,200},
					{200,400,400,400,200},
					{200,200,200,200,200}};*/
			
			/*pot_point={{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},
					{100,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,100},
					{100,200,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,200,100},
					{100,200,300,400,400,400,400,400,400,400,400,400,400,400,400,400,300,200,100},
					{100,200,300,400,500,500,500,500,500,500,500,500,500,500,500,400,300,200,100},
					{100,200,300,400,500,600,600,600,600,600,600,600,600,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,700,700,700,700,700,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,800,800,800,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,900,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,1000,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,900,900,900,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,800,800,800,800,800,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,700,700,700,700,700,700,700,600,500,400,300,200,100},
					{100,200,300,400,500,600,600,600,600,600,600,600,600,600,500,400,300,200,100},
					{100,200,300,400,500,500,500,500,500,500,500,500,500,500,500,400,300,200,100},
					{100,200,300,400,400,400,400,400,400,400,400,400,400,400,400,400,300,200,100},
					{100,200,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,200,100},
					{100,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,100},
					{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100}};	*/
			
			/*pot_point={{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
					{1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1},
					{1,2,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,2,1},
					{1,2,3,4,4,4,4,4,4,4,4,4,4,4,4,4,3,2,1},
					{1,2,3,4,5,5,5,5,5,5,5,5,5,5,5,4,3,2,1},
					{1,2,3,4,5,6,6,6,6,6,6,6,6,6,5,4,3,2,1},
					{1,2,3,4,5,6,7,7,7,7,7,7,7,6,5,4,3,2,1},
					{1,2,3,4,5,6,7,8,8,8,8,8,7,6,5,4,3,2,1},
					{1,2,3,4,5,6,7,8,9,9,9,8,7,6,5,4,3,2,1},
					{1,2,3,4,5,6,7,8,9,10,9,8,7,6,5,4,3,2,1},
					{1,2,3,4,5,6,7,8,9,9,9,8,7,6,5,4,3,2,1},
					{1,2,3,4,5,6,7,8,8,8,8,8,7,6,5,4,3,2,1},
					{1,2,3,4,5,6,7,7,7,7,7,7,7,6,5,4,3,2,1},
					{1,2,3,4,5,6,6,6,6,6,6,6,6,6,5,4,3,2,1},
					{1,2,3,4,5,5,5,5,5,5,5,5,5,5,5,4,3,2,1},
					{1,2,3,4,4,4,4,4,4,4,4,4,4,4,4,4,3,2,1},
					{1,2,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,2,1},
					{1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1},
					{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}};*/
			pot_point={{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
					{1,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,1},
					{1,3,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,3,1},
					{1,3,9,27,27,27,27,27,27,27,27,27,27,27,27,27,9,3,1},
					{1,3,9,27,81,81,81,81,81,81,81,81,81,81,81,27,9,3,1},
					{1,3,9,27,81,243,243,243,243,243,243,243,243,243,81,27,9,3,1},
					{1,3,9,27,81,243,729,729,729,729,729,729,729,243,81,27,9,3,1},
					{1,3,9,27,81,243,729,2187,2187,2187,2187,2187,729,243,81,27,9,3,1},
					{1,3,9,27,81,243,729,2187,6561,6561,6561,2187,729,243,81,27,9,3,1},
					{1,3,9,27,81,243,729,2187,6561,19683,6561,2187,729,243,81,27,9,3,1},
					{1,3,9,27,81,243,729,2187,6561,6561,6561,2187,729,243,81,27,9,3,1},
					{1,3,9,27,81,243,729,2187,2187,2187,2187,2187,729,243,81,27,9,3,1},
					{1,3,9,27,81,243,729,729,729,729,729,729,729,243,81,27,9,3,1},
					{1,3,9,27,81,243,243,243,243,243,243,243,243,243,81,27,9,3,1},
					{1,3,9,27,81,81,81,81,81,81,81,81,81,81,81,27,9,3,1},
					{1,3,9,27,27,27,27,27,27,27,27,27,27,27,27,27,9,3,1},
					{1,3,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,3,1},
					{1,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,1},
					{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}};

			map=read_map(ros::package::getPath("nord_planning") + "/data/small_maze.txt");
			/*	int i,j;
			std::cout<<int(max_x*100)<<'\n';
			for (i=0;i<int(max_x*100);i+=1){
				for(j=0;j<int(max_y*100);j+=1){
					 std::cout<<map[i][j]<<' ';
				}
				std::cout<<'\n';
			}	*/
			
			create_pointmap();
			
			create_pot_map();
		}
		
		std::vector< std::vector<int> > read_map(std::string filename){
			std::ifstream file(filename);
			std::string l;
			
			float x0, y0, x1, y1;
			while (std::getline(file, l))
			{
				std::istringstream iss(l);
				if (l[0] == '#')
					continue;

				iss >> x0 >> y0 >> x1 >> y1;
				min_x = std::min({min_x, x0, x1});
				min_y = std::min({min_y, y0, y1});
				max_x = std::max({max_x, x0, x1});
				max_y = std::max({max_y, y0, y1});
			}
			
			std::cout<<max_x<<' '<<max_y<<'\n';
			file.close();
			file.open(filename);

			std::vector< std::vector<int> > map1(int(max_x*100+1), std::vector<int> (int(max_y*100+1)));
			int mx0,mx1,my0,my1;
			int cx,cy;
			for(cx=0;cx<int(max_x*100+1);cx+=1){
				for(cy=0;cy<int(max_y*100+1);cy+=1){
					map1[cx][cy]=0;
				}
			}

		/*	int i,j;
			std::cout<<int(max_x*100)<<'\n';
			for (i=0;i<int(max_x*100);i+=1){
				for(j=0;j<int(max_y*100);j+=1){
					 std::cout<<map1[i][j]<<' ';
				}
				std::cout<<'\n';
			}	*/
						
			
			while (std::getline(file, l))
			{
				std::istringstream iss(l);
				if (l[0] == '#')
					continue;

				iss >> x0 >> y0 >> x1 >> y1;
				mx0=int(x0*100);
				mx1=int(x1*100);
				my0=int(y0*100);
				my1=int(y1*100);

				if(mx0!=mx1){
					if(my0!=my1){//diagonal lines deal with this later!!!! Do we need to deal with this?


					}else{
						cy=my0;
						if(mx0<mx1){
							for(cx=mx0;cx<=mx1;cx+=1){
								map1[cx][cy]=1;
							}
						}else{
							for(cx=mx1;cx<=mx0;cx+=1){
								map1[cx][cy]=1;
							}
						}
					}
				}else if(my0!=mx1){
					cx=mx0;
					if(my0<my1){
						for(cy=my0;cy<=my1;cy+=1){
							map1[cx][cy]=1;
						}
					}else{
						for(cy=my1;cy<=my0;cy+=1){
							map1[cx][cy]=1;
						}
					}
				}else{ 
					map1[mx0][my0]=1;
				}	
			}
			

			return map1;

		}
		
		void create_pointmap(){
			pointmap.resize(int(max_x*100+1), std::vector<int> (int(max_y*100+1)));
			int cx,cy;
			int dist_point=25;
			num_points=0;
			
			for(cx=0;cx<int(max_x*100+1);cx+=1){
				for(cy=0;cy<int(max_y*100+1);cy+=1){
					pointmap[cx][cy]=0;
				}
			}
			
			for(cx=0;cx<int(max_x*100+1);cx+=dist_point){
				for(cy=0;cy<int(max_y*100+1);cy+=dist_point){
					pointmap[cx][cy]=1;
					num_points+=1;
				}
			}
			
		}
		
		void create_pot_map(){
			pot_map.resize(int(max_x*100+1), std::vector<long int> (int(max_y*100+1)));
			
			int cx,cy;
			for(cx=0;cx<int(max_x*100+1);cx+=1){
				for(cy=0;cy<int(max_y*100+1);cy+=1){
					pot_map[cx][cy]=0;
				}
			}

			for(cx=0;cx<int(max_x*100+1);cx+=1){
				for(cy=0;cy<int(max_y*100+1);cy+=1){
					
					if(map[cx][cy]==1){
						add_pot(cx,cy,pot_wall,n_wall);
					}
					
					if(pointmap[cx][cy]==1){
						add_pot(cx,cy,pot_point,n_point);
					}
				}
			}

		}
		
		void move_points(){
			int cx,cy;
			int flag=0;
			int fx,fy;
			int step=3;
			int i=1;
			std::vector<int> vec(2,0);
			
			while(flag==0){
				flag=1;
				for(cx=0;cx<int(max_x*100+1);cx+=1){
					for(cy=0;cy<int(max_y*100+1);cy+=1){
						if(pointmap[cx][cy]>0 && pointmap[cx][cy]<=i){
							pointmap[cx][cy]=0;
							remove_pot(cx,cy,pot_point,n_point);
							
							fx=cx;
							fy=cy;
							ROS_INFO("old pos: %d,%d",fx,fy);
							vec=next_place(fx,fy,step);
							fx=vec[0];
							fy=vec[1];
							ROS_INFO("returned: %d,%d",fx,fy);
							if((fx!=cx) || (fy!=cy)){
								flag=0;
							}
							add_pot(fx,fy,pot_point,n_point);
							pointmap[fx][fy]=i+1;
						}
					}
				}
				i+=1;
				ROS_INFO("Iteration: %d",i);
			}
			for (i=0;i<int(max_x*100+1);i+=1){
				for(int j=0;j<int(max_y*100+1);j+=1){
					if(pointmap[i][j]>0){
						map[i][j]=2;
					}
				}
			}
			
		}

		void print_info(){
			int i=0;int j=0;
			std::ofstream file(ros::package::getPath("nord_planning")+"/Map.txt");
			std::ofstream file2(ros::package::getPath("nord_planning")+"/Map_pot.txt");
			std::ofstream file3(ros::package::getPath("nord_planning")+"/links.txt");
			ROS_INFO("printing_file");
			for (i=0;i<int(max_x*100+1);i+=1){
				for(j=0;j<int(max_y*100+1);j+=1){
					file<<map[i][j]<<' ';
				}
				file<<'\n';
			}	
				std::cout<<"\n\n\n";
			for (i=0;i<int(max_x*100+1);i+=1){
				for(j=0;j<int(max_y*100+1);j+=1){
					 file2<<pot_map[i][j]<<' ';
				}
				file2<<'\n';
			}
			
			for (auto& node : m.get_graph())
			{
				file3<< node.x <<' '<< node.y <<" : ";
				for (auto link : node.get_links()){
					file3<<"\t"<<link->x<<' '<<link->y<<"\n\t";
				}
				file3<<"\n\n\n";
			}
		}
		
		void create_graph(){
			int cx,cy;
			int node=3;
			int radius=36; //Look out for the dist_point when creating the point_map, 36~= sqrt(25^2+25^2)
			int i,j;
			int flag,wall_flag;
			int fx,fy;
			int mx,my;
			
				
			for(cx=0;cx<int(max_x*100+1);cx+=1){
				for(cy=0;cy<int(max_y*100+1);cy+=1){
					if(map[cx][cy]==2){
						graph.emplace_back(cx, cy);
						map[cx][cy]=node;
						node+=1;
					}
				}
			}
			m = dijkstra::map(std::move(graph));
			
			for(cx=0;cx<int(max_x*100+1);cx+=1){
				for(cy=0;cy<int(max_y*100+1);cy+=1){
					if(map[cx][cy]>2){
						node=map[cx][cy];
						for(i=cx-radius;i<=cx+radius;i+=1){
							if((i>=0) && (i<int(max_x*100+1))){
								for(j=cy-radius;j<=cy+radius;j+=1){
									if((j>=0) && (j<int(max_y*100+1))){
										mx=i-cx;
										my=j-cy;
										if(map[i][j]>node && sqrt(mx*mx+my*my)<=radius){//meter map[i][j]>node porque a ordem de escolha é igual a anterior o garante que o no anterior ja foi visitado não precisa de ir la outra vez....
											fx=cx;
											fy=cy;
											wall_flag=0;
											flag=0;
											
											while(mx!=0 || my!=0){
												if(flag==0){
													if(mx>0){
														fx+=1;
														mx-=1;
														if(map[fx][fy]==1){
															wall_flag=1;
															break;
														}
													}else if(mx<0){
														fx-=1;
														mx+=1;
														if(map[fx][fy]==1){
															wall_flag=1;
															break;
														}
													}
													flag=1;
												}else{
													if(my>0){
														fy+=1;
														my-=1;
														if(map[fx][fy]==1){
															wall_flag=1;
															break;
														}
													}else if(my<0){
														fy-=1;
														my+=1;
														if(map[fx][fy]==1){
															wall_flag=1;
															break;
														}
													}
													flag=0;
												}
											}
											
											if(wall_flag==0){
												ROS_INFO("connected: %d : %d",(map[cx][cy]-3),(map[i][j]-3));
												ROS_INFO("connected: %d %d : %d %d",cx,cy,i,j);
												 m.connect((map[cx][cy]-3),(map[i][j]-3));
											}
										}
									}
								}
							}
						}
						
					}
				}
			}

		}
	
	private:
	
		
		float min_x, min_y, max_x, max_y;
		std::vector< std::vector<long int> > pot_wall; std::vector< std::vector<long int> > pot_point; 
		std::vector< std::vector<long int> > pot_map; std::vector< std::vector<int> > map; 
		std::vector< std::vector<int> > pointmap;
		int num_points;
		dijkstra::map m;
		std::vector<dijkstra::point> graph;
				
		void add_pot(int cx, int cy,std::vector< std::vector<long int> > pot,int n){
			int i,j;
			pot_map[cx][cy]+=pot[((n-1)/2)][((n-1)/2)];
			for(i=0;i<=((n-1)/2);i+=1){
				for(j=0;j<=((n-1)/2);j+=1){
					if(j==0 && i==0) continue;
					if((cx-i)>=0 && (cy-j)>=0){
						pot_map[cx-i][cy-j]+= pot[((n-1)/2)-i][((n-1)/2)-j];
					}
					if((cx+i)< int(max_x*100+1) && (cy-j)>=0 && i!=0){
						pot_map[cx+i][cy-j]+= pot[((n-1)/2)+i][((n-1)/2)-j];
					}
					if((cx-i)>=0 && (cy+j)<int(max_y*100+1) && j!=0){
						pot_map[cx-i][cy+j]+= pot[((n-1)/2)-i][((n-1)/2)+j];
					}
					if((cx+i)< int(max_x*100+1) && (cy+j)<int(max_y*100+1) && i!=0 && j!=0){
						pot_map[cx+i][cy+j]+= pot[((n-1)/2)+i][((n-1)/2)+j];
					}
				}
			}
		}
		
		void remove_pot(int cx, int cy,std::vector< std::vector<long int> > pot,int n){
			int i,j;
			pot_map[cx][cy]-=pot[((n-1)/2)][((n-1)/2)];
			for(i=0;i<=((n-1)/2);i+=1){
				for(j=0;j<=((n-1)/2);j+=1){
					if(j==0 && i==0) continue;
					if((cx-i)>=0 && (cy-j)>=0){
						pot_map[cx-i][cy-j]-= pot[((n-1)/2)-i][((n-1)/2)-j];
					}
					if((cx+i)< int(max_x*100+1) && (cy-j)>=0 && i!=0){
						pot_map[cx+i][cy-j]-= pot[((n-1)/2)+i][((n-1)/2)-j];
					}
					if((cx-i)>=0 && (cy+j)<int(max_y*100+1) && j!=0){
						pot_map[cx-i][cy+j]-= pot[((n-1)/2)-i][((n-1)/2)+j];
					}
					if((cx+i)< int(max_x*100+1) && (cy+j)<int(max_y*100+1) && i!=0 && j!=0){
						pot_map[cx+i][cy+j]-= pot[((n-1)/2)+i][((n-1)/2)+j];
					}
				}
			}
		}
		
		std::vector<int> next_place(int fx, int fy, int step){
			int i,j;
			int min_val=-1;
			std::vector<int> vec(2,0);
			vec[0]=fx;
			vec[1]=fy;
			if (pot_map[fx][fy]!=0){
				for(i=fx-step;i<=fx+step;i+=1){
					if((i>=0) && (i<int(max_x*100+1))){
						
						for(j=fy-step;j<=fy+step;j+=1){
							if((j>=0) && (j<int(max_y*100+1))){
								
								if(min_val==-1 || min_val>pot_map[i][j]){
									min_val=pot_map[i][j];
									vec[0]=i;
									vec[1]=j;
								}
							}
						}
					}
				}
			}
			return vec;
		}
	
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "nord_planning");
    ros::NodeHandle n;
	
	Maps run;
	ROS_INFO("OLA");
	
	run.move_points();
	
	run.create_graph();
	run.print_info();
    return 0;
}
