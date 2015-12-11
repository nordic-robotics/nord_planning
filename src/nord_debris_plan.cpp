#include "ros/ros.h"
#include "ros/package.h"
#include <fstream>
#include <sstream>
#include <string>
#include "nord_messages/Debris.h"
#include "nord_messages/DebrisArray.h"
#include <math.h> 
#include "dijkstra/path.hpp"      
#include "dijkstra/map.hpp" 
#include "dijkstra/point.hpp"
#include "nord_messages/Graph.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Empty.h"

dijkstra::map graph;

std::vector< std::vector<int> >  map;
std::vector< std::vector<int> >  map_walls;
std::vector< std::vector<int> >  map_debris;
std::vector< std::vector<int> > map_aux;
ros::Publisher g_pub; 
ros::Publisher rviz_pub;
ros::Publisher abort_pub; 

visualization_msgs::Marker rviz_marker;

int n_wall=29;
int clear_debris=12;
int n_debris=2*clear_debris+1;
float min_x, min_y, max_x, max_y;
std::vector<dijkstra::point> actual_path;
unsigned int n_max=0;//number of nodes at the moment

std::vector<dijkstra::point> read_path(std::string filename)
{
    std::ifstream file(filename);
    std::string l;
    std::vector<dijkstra::point> path;
    while (std::getline(file, l))
    {
        std::istringstream iss(l);
        float x, y;
        iss >> x >> y;
        path.emplace_back(x, y);
    }
    return path;
}

void load_graph(std::string filename)
{
    std::ifstream file(filename);
    
    std::string l;

    std::vector<dijkstra::point> nodes(100000);

    // get all nodes without links
    while (std::getline(file, l))
    {
        if (l[0] != '%')
            continue;

        std::getline(file, l);
        std::istringstream iss(l);
        std::getline(iss, l, ',');
        float x = std::stod(l);
        std::getline(iss, l, ',');
        float y = std::stod(l);
        nodes[n_max]=dijkstra::point(x, y);

		n_max++;//number of nodes max
    }
	
    graph = dijkstra::map(std::move(nodes));

    std::ifstream file2(filename);
    unsigned int parent = -1;
    // get all links
    while (std::getline(file2, l))
    {
        if(l.size()==0){
            std::cout << "shit is fucked yo!" << std::endl;
            exit(1);
        }
        if (l[0] == '%')
        {
            std::getline(file2, l);
            std::istringstream iss(l);
            std::getline(iss, l, ',');
            float x = std::stod(l);
            std::getline(iss, l, ',');
            float y = std::stod(l);
            // std::cout << "looking for " << x << " " << y << std::endl;

            for (size_t i = 0; i < n_max; i++)
            { 
                if (graph.get_graph()[i].x == x && graph.get_graph()[i].y == y)
                {
                    parent = i;
                    break;
                }
            }
            std::getline(file2, l);
        }

        // std::cout << "line: " << l << std::endl;
        std::istringstream iss(l);
        std::getline(iss, l, ',');
        float x = std::stod(l);
        std::getline(iss, l, ',');
        float y = std::stod(l);

        unsigned int child = -1;
        for (size_t i = 0; i < n_max; i++)
        {
            if (graph.get_graph()[i].x == x && graph.get_graph()[i].y == y)
            {
                child = i;
                break;
            }
        }

        graph.connect(parent, child);
    }
}

void build_map_wall(int cx,int cy,int n){
	
	int i,j;
	map_walls[cx][cy]=1;
	for(i=0;i<=((n-1)/2);i+=1){
		for(j=0;j<=((n-1)/2);j+=1){
			if((j==0) && (i==0)) continue;
			if((cx-i)>=0 && (cy-j)>=0){
				map_walls[cx-i][cy-j]= 1;
			}
			if((cx+i)< int(max_x*100+1) && (cy-j)>=0 && i!=0){
				map_walls[cx+i][cy-j]= 1;
			}
			if((cx-i)>=0 && (cy+j)<int(max_y*100+1) && j!=0){
				map_walls[cx-i][cy+j]= 1;
			}
			if((cx+i)< int(max_x*100+1) && (cy+j)<int(max_y*100+1) && i!=0 && j!=0){
				map_walls[cx+i][cy+j]= 1;
			}
		}
	}
}
//Fazer como as connections isto assim mete 2 depois de ter passado uma parede
bool build_map_debris(int cx,int cy,int n){

	bool flag=false;
	int i,j;
	
	for(i=0;i<=((n-1)/2);i++){
		for(j=0;j<=((n-1)/2);j++){
			if((j==0) && (i==0) && map_debris[cx][cy]==0){
				map_debris[cx][cy]=1;
				flag=true;
				continue;
			} 
			if((cx-i)>=0 && (cy-j)>=0 && map_debris[cx-i][cy-j]==0){
				map_debris[cx-i][cy-j]=1;
				flag=true;
			}
			if((cx+i)< int(max_x*100+1) && (cy-j)>=0 && i!=0 && map_debris[cx+i][cy-j]==0){
				map_debris[cx+i][cy-j]=1;
				flag=true;
			}
			if((cx-i)>=0 && (cy+j)<int(max_y*100+1) && j!=0 && map_debris[cx-i][cy+j]==0){
				map_debris[cx-i][cy+j]=1;
				flag=true;
			}
			if((cx+i)< int(max_x*100+1) && (cy+j)<int(max_y*100+1) && i!=0 && j!=0 && map_debris[cx+i][cy+j]==0){
				map_debris[cx+i][cy+j]=1;
				flag=true;
			}
		}
	}
	return flag;
}

std::vector< std::vector<int> > read_map(std::string filename){
	std::ifstream file(filename);
	std::string l;
	
	//Read file first time to get max e min values to declare map matrix
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
	
	file.close();
	file.open(filename);

	std::vector< std::vector<int> > map1(int(max_x*100+1), std::vector<int> (int(max_y*100+1)));
	map_walls.resize(int(max_x*100+1), std::vector<int> (int(max_y*100+1)));
	int mx0,mx1,my0,my1;
	int cx,cy;
	int diffx,diffy;
	float a,b,f;
	int fx,fy;
	
	for(cx=0;cx<int(max_x*100+1);cx+=1){
		for(cy=0;cy<int(max_y*100+1);cy+=1){
			map1[cx][cy]=0;
			map_walls[cx][cy]=0;
		}
	}
	//lines will be represented by 1 in the variable "map1" which will be "map" outside this function
	while (std::getline(file, l))
	{
		std::istringstream iss(l);
		if (l[0] == '#')
			continue;

		iss >> x0 >> y0 >> x1 >> y1;
		mx0=(int)(x0*100);
		mx1=(int)(x1*100);
		my0=(int)(y0*100);
		my1=(int)(y1*100);

		if(mx0!=mx1){
			if(my0!=my1){//diagonal lines 
				
				diffx=std::abs(mx1-mx0);
				diffy=std::abs(my1-my0);
				if (diffx>=diffy){//choose the coord varying the most and create a line depending on that one
					a=(((float) my0)-((float) my1))/(((float) mx0)-((float) mx1));
					b=((float) my1)-(a*((float) mx1));
					if(mx0>mx1){
						for(fx=mx1;fx<=mx0;fx+=1){
							f=(a*fx+b);
							fy=ceil(f);
							map1[fx][fy]=1;
							build_map_wall(fx,fy,n_wall);
							fy=floor(f);
							map1[fx][fy]=1;
							build_map_wall(fx,fy,n_wall);
						}
					}else{
						for(fx=mx0;fx<=mx1;fx+=1){
							f=(a*fx+b);
							fy=ceil(f);
							map1[fx][fy]=1;
							build_map_wall(fx,fy,n_wall);
							fy=floor(f);
							map1[fx][fy]=1;
							build_map_wall(fx,fy,n_wall);
						}
					}
					
				}else{
					a=(((float) mx0)-((float) mx1))/(((float) my0)-((float) my1));
					b=((float) mx1)-(a*((float) my1));
					if(my0>my1){
						for(fy=my1;fy<=my0;fy+=1){
							f=(a*fy+b);
							fx=ceil(f);
							map1[fx][fy]=1;
							build_map_wall(fx,fy,n_wall);
							fx=floor(f);
							map1[fx][fy]=1;
							build_map_wall(fx,fy,n_wall);
							
						}
					}else{
						for(fy=my0;fy<=my1;fy+=1){
							f=(a*fy+b);
							fx=ceil(f);
							map1[fx][fy]=1;
							build_map_wall(fx,fy,n_wall);
							fx=floor(f);
							map1[fx][fy]=1;
							build_map_wall(fx,fy,n_wall);
							
						}
					}
					
				}
			}else{//line which varies in x
				cy=my0;
				if(mx0<mx1){
					for(cx=mx0;cx<=mx1;cx+=1){
						map1[cx][cy]=1;
						build_map_wall(cx,cy,n_wall);
					}
				}else{
					for(cx=mx1;cx<=mx0;cx+=1){
						map1[cx][cy]=1;
						build_map_wall(cx,cy,n_wall);
					}
				}
			}
		}else if(my0!=my1){//line which varies in y
			cx=mx0;
			if(my0<my1){
				for(cy=my0;cy<=my1;cy+=1){
					map1[cx][cy]=1;
					build_map_wall(cx,cy,n_wall);
				}
			}else{
				for(cy=my1;cy<=my0;cy+=1){
					map1[cx][cy]=1;
					build_map_wall(cx,cy,n_wall);
				}
			}
		}else{ //just a point not a line
			map1[mx0][my0]=1;
			build_map_wall(mx0,my0,n_wall);
		}	
	}
	

	return map1;

}

bool check_connection(int mx0,int my0,int mx1,int my1){

	int diffx,diffy;
	float a,b,f;
	int fx,fy;
	
	if(mx0<0){
		mx0=0;
	}else if(mx0>int(max_x*100)){
		mx0=int(max_x*100);
	}
	if(mx1<0){
		mx1=0;
	}else if(mx1>int(max_x*100)){
		mx1=int(max_x*100);
	}
	if(my1<0){
		my1=0;
	}else if(my1>int(max_y*100)){
		my1=int(max_y*100);
	}
	if(my0<0){
		my0=0;
	}else if(my0>int(max_y*100)){
		my0=int(max_y*100);
	}

	if(mx0!=mx1){
		if(my0!=my1){//diagonal lines 
			
			diffx=std::abs(mx1-mx0);
			diffy=std::abs(my1-my0);
			if (diffx>=diffy){//choose the coord varying the most and create a line depending on that one
				a=(((float) my0)-((float) my1))/(((float) mx0)-((float) mx1));
				b=((float) my1)-(a*((float) mx1));
				if(mx0>mx1){
					for(fx=mx1;fx<=mx0;fx+=1){
						f=(a*fx+b);
						fy=std::lround(f);
						if(fy>=0 && fy<int(max_y*100+1) && map_debris[fx][fy]!=0){
							//std::cout<<fx<<' '<<fy<<' '<<map_debris[fx][fy]<<std::endl;
							return true;
						}
						/*fy=floor(f);
						if(fy>=0 && fy<int(max_y*100+1) && map_debris[fx][fy]!=0){
							flag_cut=true;
							break;
						}*/
					}
				}else{
					for(fx=mx0;fx<=mx1;fx+=1){
						f=(a*fx+b);
						fy=std::lround(f);
						if(fy>=0 && fy<int(max_y*100+1) && map_debris[fx][fy]!=0){
							//std::cout<<fx<<' '<<fy<<' '<<map_debris[fx][fy]<<std::endl;
							return true;
						}
						/*fy=floor(f);
						if(fy>=0 && fy<int(max_y*100+1) && map_debris[fx][fy]!=0){
							flag_cut=true;
							break;
						}*/
					}
				}
				
			}else{
				a=(((float) mx0)-((float) mx1))/(((float) my0)-((float) my1));
				b=((float) mx1)-(a*((float) my1));
				if(my0>my1){
					for(fy=my1;fy<=my0;fy+=1){
						f=(a*fy+b);
						fx=std::lround(f);
						if(fx>=0 && fx<int(max_x*100+1) && map_debris[fx][fy]!=0){
						//	std::cout<<fx<<' '<<fy<<' '<<map_debris[fx][fy]<<std::endl;
							return true;
						}
						/*fx=floor(f);
						if(fx>=0 && fx<int(max_x*100+1) && map_debris[fx][fy]!=0){
							flag_cut=true;
							break;
						}*/
						
					}
				}else{
					for(fy=my0;fy<=my1;fy+=1){
						f=(a*fy+b);
						fx=std::lround(f);
						if(fx>=0 && fx<int(max_x*100+1) && map_debris[fx][fy]!=0){
						//	std::cout<<fx<<' '<<fy<<' '<<map_debris[fx][fy]<<std::endl;
							return true;
						}
						/*fx=floor(f);
						if(fx>=0 && fx<int(max_x*100+1) && map_debris[fx][fy]!=0){
							flag_cut=true;
							break;
						}*/
						
					}
				}
				
			}
		}else{//line which varies in x
			fy=my0;
			if(mx0<mx1){
				for(fx=mx0;fx<=mx1;fx+=1){
					if(map_debris[fx][fy]!=0){
						//std::cout<<' '<<map_debris[fx][fy]<<std::endl;
						return true;
					}
				}
			}else{
				for(fx=mx1;fx<=mx0;fx+=1){
					if(map_debris[fx][fy]!=0){
					//	std::cout<<' '<<map_debris[fx][fy]<<std::endl;
						return true;
					}
				}
			}
		}
	}else if(my0!=my1){//line which varies in y
		fx=mx0;
		if(my0<my1){
			for(fy=my0;fy<=my1;fy+=1){
				if(map_debris[fx][fy]!=0){
				//	std::cout<<' '<<map_debris[fx][fy]<<std::endl;
					return true;
				}
			}
		}else{
			for(fy=my1;fy<=my0;fy+=1){
				if(map_debris[fx][fy]!=0){
				//	std::cout<<' '<<map_debris[fx][fy]<<std::endl;
					return true;
				}
			}
		}
	}else{ //just a point not a line
		if(map_debris[mx0][my0]!=0){
			//std::cout<<' '<<map_debris[mx0][my0]<<std::endl;
			return true;
		}
	}
	return false;
	
}

bool create_new_nodes(nord_messages::Debris data){
	bool flag_new=false;
	unsigned int i;
	int mx0,my0,mx1,my1;
	int diffx,diffy;
	float a,b,f;
	int fx,fy;
	
	size_t num_nodes=n_max;
	mx0=std::lround(data.x*100);
	my0=std::lround(data.y*100);
	if(mx0<0){
		mx0=0;
	}else if(mx0>int(max_x*100)){
		mx0=int(max_x*100);
	}
	if(my0<0){
		my0=0;
	}else if(my0>int(max_y*100)){
		my0=int(max_y*100);
	}
	fx=mx0+clear_debris+1;
	fy=my0+clear_debris+1;
	if(fx < int(max_x*100+1) && fy < int(max_y*100+1) && map_debris[fx][fy]==0){
		flag_new=true;
		graph[n_max]=dijkstra::point(fx/100.0f,fy/100.0f);
		n_max++;
		//ROS_INFO("new node: %d %d",fx,fy);
	}
	fx=mx0-clear_debris-1;
	fy=my0+clear_debris+1;
	if(fx >0 && fy < int(max_y*100+1) && map_debris[fx][fy]==0){
		flag_new=true;
		graph[n_max]=dijkstra::point(fx/100.0f,fy/100.0f);
		n_max++;
		//ROS_INFO("new node: %d %d",fx,fy);
	}
	fx=mx0-clear_debris-1;
	fy=my0-clear_debris-1;
	if(fx >0 && fy > 0 && map_debris[fx][fy]==0){
		flag_new=true;
		graph[n_max]=dijkstra::point(fx/100.0f,fy/100.0f);
		n_max++;
		//ROS_INFO("new node: %d %d",fx,fy);
	}
	fx=mx0+clear_debris+1;
	fy=my0-clear_debris-1;
	if(fx < int(max_x*100+1) && fy > 0 && map_debris[fx][fy]==0){
		flag_new=true;
		graph[n_max]=dijkstra::point(fx/100.0f,fy/100.0f);
		n_max++;
		//ROS_INFO("new node: %d %d",fx,fy);
	}
	/*for(i=0;i<=data.hull.size();i++){
		//ROS_INFO("beginning");
		mx1=std::lround(data.hull[i].x*100);
		my1=std::lround(data.hull[i].y*100);
		if(mx1<0){
			mx1=0;
		}else if(mx1>int(max_x*100)){
			mx1=int(max_x*100);
		}
		if(my1<0){
			my1=0;
		}else if(my1>int(max_y*100)){
			my1=int(max_y*100);
		}
		
		if(mx0!=mx1){
			if(my0!=my1){//diagonal lines 
				
				diffx=std::abs(mx1-mx0);
				diffy=std::abs(my1-my0);
				if (diffx>=diffy){//choose the coord varying the most and create a line depending on that one
					a=(((float) my0)-((float) my1))/(((float) mx0)-((float) mx1));
					b=((float) my1)-(a*((float) mx1));
					if(mx0<mx1){
						fx=mx1-+clear_debris+1;
						f=(a*fx+b);
						fy=std::lround(f);
						-
						if(fx < int(max_x*100+1) && fy>=0 && fy < int(max_y*100+1) && map_debris[fx][fy]==0){
							flag_new=true;
							graph[n_max]=dijkstra::point(fx/100.0f,fy/100.0f);
							n_max++;
							//ROS_INFO("new node: %d %d",fx,fy);
						}
					}else{
						fx=mx1-clear_debris-1;
						f=(a*fx+b);
						fy=std::lround(f);
						if(fx>=0 && fy>=0 && fy < int(max_y*100+1) && map_debris[fx][fy]==0){
							flag_new=true;
							graph[n_max]=dijkstra::point(fx/100.0f,fy/100.0f);
							n_max++;
							//ROS_INFO("new node: %d %d",fx,fy);
						}
					}
					
				}else{
					a=(((float) mx0)-((float) mx1))/(((float) my0)-((float) my1));
					b=((float) mx1)-(a*((float) my1));
					if(my0>my1){
						fy=my1-clear_debris-1;
						f=(a*fy+b);
						fx=std::lround(f);
						if(fx>=0 && fx < int(max_x*100+1) && fy>=0 && map_debris[fx][fy]==0){
							flag_new=true;
							graph[n_max]=dijkstra::point(fx/100.0f,fy/100.0f);
							n_max++;
							//ROS_INFO("new node: %d %d",fx,fy);
						}

					}else{
						fy=my1+clear_debris+1;
						f=(a*fy+b);
						fx=std::lround(f);
						if(fx>=0 && fx < int(max_x*100+1) && fy < int(max_y*100+1) && map_debris[fx][fy]==0){
							flag_new=true;
							graph[n_max]=dijkstra::point(fx/100.0f,fy/100.0f);
							n_max++;
							//ROS_INFO("new node: %d %d",fx,fy);
						}
						
					}
					
				}
			}else{//line which varies in x
				fy=my0;
				if(mx0<mx1){
					fx=mx1+clear_debris+1;
					if(fx < int(max_x*100+1) && map_debris[fx][fy]==0){
						flag_new=true;
						graph[n_max]=dijkstra::point(fx/100.0f,fy/100.0f);
						n_max++;
						//ROS_INFO("new node: %d %d",fx,fy);
					}
					
				}else{
					fx=mx1-clear_debris-1;
					if(fx>=0 && map_debris[fx][fy]==0){
						flag_new=true;
						graph[n_max]=dijkstra::point(fx/100.0f,fy/100.0f);
						n_max++;
						//ROS_INFO("new node: %d %d",fx,fy);
					}
				}
			}
		}else if(my0!=my1){//line which varies in y
			fx=mx0;
			if(my0<my1){
				fy=my1+clear_debris+1;
				if(fy < int(max_y*100+1) && map_debris[fx][fy]==0){
					flag_new=true;
					graph[n_max]=dijkstra::point(fx/100.0f,fy/100.0f);
					n_max++;
					//ROS_INFO("new node: %d %d",fx,fy);
				}
			}else{
				fy=my1-clear_debris-1;
				if(fy >=0 && map_debris[fx][fy]==0){
					flag_new=true;
					graph[n_max]=dijkstra::point(fx/100.0f,fy/100.0f);
					n_max++;
				//	ROS_INFO("new node: %d %d",fx,fy);
				}
			}
		}
	}*/
	/*for(int fx=0;fx<=int(max_x*100+1);fx++){
		for(int fy=0;fy<=int(max_y*100+1);fy++){
			if(map_debris[fx][fy]==2){
				map_debris[fx][fy]=1;
			}
		}
	}*/
	//ROS_INFO("reaches");
	if(flag_new==true){
		while(num_nodes<n_max){
			
			for (size_t j = 0; j < n_max; j++)
			{
				if(num_nodes!=j){
					if(false==check_connection(std::lround(graph.get_graph()[num_nodes].x * 100) ,std::lround(graph.get_graph()[num_nodes].y * 100), std::lround(graph.get_graph()[j].x * 100) ,std::lround(graph.get_graph()[j].y * 100))){
						graph.connect(num_nodes, j);
					//	std::cout<<"Connecting: "<<num_nodes<<' '<<j<<std::endl;
					}
				}
			}
			num_nodes++;
		}
		
	}
	return flag_new;
}

visualization_msgs::Marker rviz_message(void){

    visualization_msgs::Marker explored;
    explored.id= 147;
    explored.header.frame_id = "/map";
    explored.ns="explore";
    explored.header.stamp = ros::Time::now();
    explored.type = visualization_msgs::Marker::CUBE_LIST;   
    explored.pose.orientation.w = 1.0f;
    explored.scale.x = 0.01f;
    explored.scale.y = 0.01f;
    explored.scale.z = 0.01f;
    explored.color.r = 0.4f;
    explored.color.g = 0.7;
    explored.color.b = 1.0f;
    explored.color.a = 0.5f;
    explored.action = visualization_msgs::Marker::ADD;
    for (int i = 0; i <=int (max_x*100); ++i)
    {
        for(int j = 0; j <=int (max_y*100); ++j){
            // ROS_INFO("cone_matrix[i][j] = %d",cone_matrix[i][j]);
            if(map_debris[i][j]==1){
                geometry_msgs::Point pos;
                pos.x = i/100.0;
                pos.y = j/100.0;
                pos.z = 0.0;
                //ROS_INFO("pos.x = %f pos.y = %f",pos.x*0.01,pos.y*0.01);
                explored.points.push_back(pos);
            }
        }
    } 
    return  explored;
}

void DebrisCallBack(const nord_messages::DebrisArray debris_array){
	ROS_INFO("TIME1");	
	map_aux=map;
	int mx0,mx1,my0,my1;
	unsigned int n=0;
	int diffx,diffy;
	float a,b,f;
	int fx,fy,ff;
	int calc;
	bool flag_neg, flag_pos;
	bool flag_neg2, flag_pos2;
	bool flag_debris,flag_cut,flag_nodes;
	flag_debris=false;
	flag_nodes=false;
	map_debris=map_walls;
	for (auto& debris : debris_array.data)
	{	
			
		flag_debris=false;
		mx0=std::lround(debris.x*100);
		my0=std::lround(debris.y*100);
		ROS_INFO("TIME2: %d %d",mx0,my0);
		if(mx0<0){
			mx0=0;
		}else if(mx0>int(max_x*100)){
			mx0=int(max_x*100);
		}
		if(my0<0){
			my0=0;
		}else if(my0>int(max_y*100)){
			my0=int(max_y*100);
		}
		if(debris.hull.size()==0){
			flag_debris=build_map_debris(mx0,my0,7);
		}else{
			flag_debris=build_map_debris(mx0,my0,n_debris);
		}
		ROS_INFO("TIME3");	
		//ROS_INFO("New debris");
/*		flag_debris=false;
		for (size_t i = 0; i <= debris.hull.size() - 1; i++)
		{
			// debris.hull[i + 1].x
			
			map_debris=map_walls;
			mx0=std::lround(debris.hull[i].x*100);
			my0=std::lround(debris.hull[i].y*100);
			if(i==(debris.hull.size()-1)){
				mx1=std::lround(debris.hull[0].x*100);
				my1=std::lround(debris.hull[0].y*100);
			}else{
				mx1=std::lround(debris.hull[i+1].x*100);
				my1=std::lround(debris.hull[i+1].y*100);
			}
			if(mx0<0){
				mx0=0;
			}else if(mx0>int(max_x*100)){
				mx0=int(max_x*100);
			}
			if(mx1<0){
				mx1=0;
			}else if(mx1>int(max_x*100)){
				mx1=int(max_x*100);
			}
			if(my1<0){
				my1=0;
			}else if(my1>int(max_y*100)){
				my1=int(max_y*100);
			}
			if(my0<0){
				my0=0;
			}else if(my0>int(max_y*100)){
				my0=int(max_y*100);
			}
				
			//ROS_INFO("%d %d %d %d",mx0,my0,mx1,my1);
			flag_debris=build_map_debris(mx0,my0,n_debris);
		
			if(build_map_debris(mx1,my1,n_debris)) flag_debris=true;

			if(mx0!=mx1){
				if(my0!=my1){//diagonal lines 
					
					diffx=std::abs(mx1-mx0);
					diffy=std::abs(my1-my0);
					if (diffx>=diffy){//choose the coord varying the most and create a line depending on that one
						a=(((float) my0)-((float) my1))/(((float) mx0)-((float) mx1));
						b=((float) my1)-(a*((float) mx1));
						if(mx0>mx1){
							for(fx=mx1;fx<=mx0;fx+=1){
								f=(a*fx+b);
								fy=ceil(f);
								if(fy>=0 && fy<int(max_y*100+1) && map_aux[fx][fy]==0){
									map_aux[fx][fy]=1;
								}
								
								//build_map_wall(fx,fy,n_wall);
								ff=floor(f);
								if(ff>=0 && ff<int(max_y*100+1) && map_aux[fx][ff]==0){
									map_aux[fx][ff]=1;
								}
								//build_map_wall(fx,fy,n_wall);
								flag_neg=false;
								flag_pos=false;
								flag_neg2=false;
								flag_pos2=false;
								for(calc=0;calc<=clear_debris;calc+=1){
									if((fy-calc>=0) && flag_neg==false){
										if(map_debris[fx][fy-calc]==0){
											map_debris[fx][fy-calc]=1;
											flag_debris=true;
										}
										if(map_aux[fx][fy-calc]!=0){
											flag_neg=true;
										}
									}
									
									if( flag_pos==false && (fy+calc<int(max_y*100+1))){
										if(map_debris[fx][fy+calc]==0){
											map_debris[fx][fy+calc]=1;
											flag_debris=true;
										}
										if(map_aux[fx][fy+calc]!=0){
											flag_pos=true;
										}
									}
									
									if((ff-calc>=0) && flag_neg2==false){
										if(map_debris[fx][ff-calc]==0){
											map_debris[fx][ff-calc]=1;
											flag_debris=true;
										}
										if(map_aux[fx][ff-calc]!=0){
											flag_neg2=true;
										}
									}
									
									if( flag_pos2==false && (ff+calc<int(max_y*100+1))){
										if(map_debris[fx][ff+calc]==0){
											map_debris[fx][ff+calc]=1;
											flag_debris=true;
										}
										if(map_aux[fx][ff+calc]!=0){
											flag_pos2=true;
										}
									}
									
									if(flag_neg && flag_pos && flag_neg2 && flag_pos2) break;
								}
							}
						}else{
							for(fx=mx0;fx<=mx1;fx+=1){
								f=(a*fx+b);
								fy=ceil(f);
								if(fy>=0 && fy<int(max_y*100+1) && map_aux[fx][fy]==0){
									map_aux[fx][fy]=1;
								}
								//build_map_wall(fx,fy,n_wall);
								ff=floor(f);
								if(ff>=0 && ff<int(max_y*100+1) && map_aux[fx][ff]==0){
									map_aux[fx][ff]=1;
								}
								//build_map_wall(fx,fy,n_wall);
								
								flag_neg=false;
								flag_pos=false;
								flag_neg2=false;
								flag_pos2=false;
								for(calc=0;calc<=clear_debris;calc+=1){
									if((fy-calc>=0) && flag_neg==false){
										if(map_debris[fx][fy-calc]==0){
											map_debris[fx][fy-calc]=1;
											flag_debris=true;
										}
										if(map_aux[fx][fy-calc]!=0){
											flag_neg=true;
										}
									}
									
									if( flag_pos==false && (fy+calc<int(max_y*100+1))){
										if(map_debris[fx][fy+calc]==0){
											map_debris[fx][fy+calc]=1;
											flag_debris=true;
										}
										if(map_aux[fx][fy+calc]!=0){
											flag_pos=true;
										}
									}
									
									if((ff-calc>=0) && flag_neg2==false){
										if(map_debris[fx][ff-calc]==0){
											map_debris[fx][ff-calc]=1;
											flag_debris=true;
										}
										if(map_aux[fx][ff-calc]!=0){
											flag_neg2=true;
										}
									}
									
									if( flag_pos2==false && (ff+calc<int(max_y*100+1))){
										if(map_debris[fx][ff+calc]==0){
											map_debris[fx][ff+calc]=1;
											flag_debris=true;
										}
										if(map_aux[fx][ff+calc]!=0){
											flag_pos2=true;
										}
									}
									
									if(flag_neg && flag_pos && flag_neg2 && flag_pos2) break;
								}
							}
						}
						
					}else{
						a=(((float) mx0)-((float) mx1))/(((float) my0)-((float) my1));
						b=((float) mx1)-(a*((float) my1));
						if(my0>my1){
							for(fy=my1;fy<=my0;fy+=1){
								f=(a*fy+b);
								fx=ceil(f);
								if(fx>=0 && fx<int(max_x*100+1) && map_aux[fx][fy]==0){
									map_aux[fx][fy]=1;
								}
								//build_map_wall(fx,fy,n_wall);
								ff=floor(f);
								if(ff>=0 && ff<int(max_x*100+1) && map_aux[ff][fy]==0){
									map_aux[ff][fy]=1;
								}
								//build_map_wall(fx,fy,n_wall);
								
								flag_neg=false;
								flag_pos=false;
								flag_neg2=false;
								flag_pos2=false;
								for(calc=0;calc<=clear_debris;calc+=1){
									if((fx-calc>=0) && flag_neg==false){
										if(map_debris[fx-calc][fy]==0){
											map_debris[fx-calc][fy]=1;
											flag_debris=true;
										}
										if(map_aux[fx-calc][fy]!=0){
											flag_neg=true;
										}
									}
									
									if( flag_pos==false && (fx+calc<int(max_x*100+1))){
										if(map_debris[fx+calc][fy]==0){
											map_debris[fx+calc][fy]=1;
											flag_debris=true;
										}
										if(map_aux[fx+calc][fy]!=0){
											flag_pos=true;
										}
									}
									
									if((ff-calc>=0) && flag_neg2==false){
										if(map_debris[ff-calc][fy]==0){
											map_debris[ff-calc][fy]=1;
											flag_debris=true;
										}
										if(map_aux[ff-calc][fy]!=0){
											flag_neg2=true;
										}
									}
									
									if( flag_pos2==false && (ff+calc<int(max_x*100+1))){
										if(map_debris[ff+calc][fy]==0){
											map_debris[ff+calc][fy]=1;
											flag_debris=true;
										}
										if(map_aux[ff+calc][fy]!=0){
											flag_pos2=true;
										}
									}
									
									if(flag_neg && flag_pos && flag_neg2 && flag_pos2) break;
								}
								
							}
						}else{
							for(fy=my0;fy<=my1;fy+=1){
								f=(a*fy+b);
								fx=ceil(f);
								if(fx>=0 && fx<int(max_x*100+1) && map_aux[fx][fy]==0){
									map_aux[fx][fy]=1;
								}
								//build_map_wall(fx,fy,n_wall);
								ff=floor(f);
								if(ff>=0 && ff<int(max_x*100+1) && map_aux[ff][fy]==0){
									map_aux[ff][fy]=1;
								}
								//build_map_wall(fx,fy,n_wall);
								
								flag_neg=false;
								flag_pos=false;
								flag_neg2=false;
								flag_pos2=false;
								for(calc=0;calc<=clear_debris;calc+=1){
									if((fx-calc>=0) && flag_neg==false){
										if(map_debris[fx-calc][fy]==0){
											map_debris[fx-calc][fy]=1;
											flag_debris=true;
										}
										if(map_aux[fx-calc][fy]!=0){
											flag_neg=true;
										}
									}
									
									if( flag_pos==false && (fx+calc<int(max_x*100+1))){
										if(map_debris[fx+calc][fy]==0){
											map_debris[fx+calc][fy]=1;
											flag_debris=true;
										}
										if(map_aux[fx+calc][fy]!=0){
											flag_pos=true;
										}
									}
									
									if((ff-calc>=0) && flag_neg2==false){
										if(map_debris[ff-calc][fy]==0){
											map_debris[ff-calc][fy]=1;
											flag_debris=true;
										}
										if(map_aux[ff-calc][fy]!=0){
											flag_neg2=true;
										}
									}
									
									if( flag_pos2==false && (ff+calc<int(max_x*100+1))){
										if(map_debris[ff+calc][fy]==0){
											map_debris[ff+calc][fy]=1;
											flag_debris=true;
										}
										if(map_aux[ff+calc][fy]!=0){
											flag_pos2=true;
										}
									}
									
									if(flag_neg && flag_pos&& flag_neg2 && flag_pos2) break;
								}
								
							}
						}
						
					}
				}else{//line which varies in x
					
					fy=my0;
					if(mx0<mx1){
						for(fx=mx0;fx<=mx1;fx+=1){
							if(map_aux[fx][fy]==0){
								map_aux[fx][fy]=1;
							}
							//build_map_wall(fx,fy,n_wall);
							if(fx>mx0+clear_debris && fx<mx1-clear_debris){
								flag_neg=false;
								flag_pos=false;
								for(calc=0;calc<=clear_debris;calc+=1){
									if((fy-calc>=0) && flag_neg==false){
										if(map_debris[fx][fy-calc]==0){
											map_debris[fx][fy-calc]=1;
											flag_debris=true;
										}
										if(map_aux[fx][fy-calc]!=0){
											flag_neg=true;
										}
									}
									
									if( flag_pos==false && (fy+calc<int(max_y*100+1))){
										if(map_debris[fx][fy+calc]==0){
											map_debris[fx][fy+calc]=1;
											flag_debris=true;
										}
										if(map_aux[fx][fy+calc]!=0){
											flag_pos=true;
										}
									}
									
									if(flag_neg && flag_pos) break;
								}
							}
						}
					}else{
						for(fx=mx1;fx<=mx0;fx+=1){
							if(map_aux[fx][fy]==0){
								map_aux[fx][fy]=1;
							}
							//build_map_wall(fx,fy,n_wall);
							if(fx>mx1+clear_debris && fx<mx0-clear_debris){
								flag_neg=false;
								flag_pos=false;
								for(calc=0;calc<=clear_debris;calc+=1){
									if((fy-calc>=0) && flag_neg==false){
										if(map_debris[fx][fy-calc]==0){
											map_debris[fx][fy-calc]=1;
											flag_debris=true;
										}
										if(map_aux[fx][fy-calc]!=0){
											flag_neg=true;
										}
									}
									
									if( flag_pos==false && (fy+calc<int(max_y*100+1))){
										if(map_debris[fx][fy+calc]==0){
											map_debris[fx][fy+calc]=1;
											flag_debris=true;
										}
										if(map_aux[fx][fy+calc]!=0){
											flag_pos=true;
										}
									}
									
									if(flag_neg && flag_pos ) break;
								}
							}
						}
					}
				
				}
			}else if(my0!=my1){//line which varies in y
				fx=mx0;
				if(my0<my1){
					for(fy=my0;fy<=my1;fy+=1){
						if(map_aux[fx][fy]==0){
							map_aux[fx][fy]=1;
						}
						//build_map_wall(fx,fy,n_wall);
						if(fy>my0+clear_debris && fy<my1-clear_debris){
							flag_neg=false;
							flag_pos=false;
							for(calc=0;calc<=clear_debris;calc+=1){
								if((fx-calc>=0) && flag_neg==false){
									if(map_debris[fx-calc][fy]==0){
										map_debris[fx-calc][fy]=1;
										flag_debris=true;
									}
									if(map_aux[fx-calc][fy]!=0){
										flag_neg=true;
									}
								}
								
								if( flag_pos==false && (fx+calc<int(max_x*100+1))){
									if(map_debris[fx+calc][fy]==0){
										map_debris[fx+calc][fy]=1;
										flag_debris=true;
									}
									if(map_aux[fx+calc][fy]!=0){
										flag_pos=true;
									}
								}
								
								if(flag_neg && flag_pos) break;
							}
						}
					}
				}else{
					for(fy=my1;fy<=my0;fy+=1){
						if(map_aux[fx][fy]==0){
							map_aux[fx][fy]=1;
						}
						//build_map_wall(fx,fy,n_wall);
						if(fy>my1+clear_debris && fy<my0-clear_debris){
							flag_neg=false;
							flag_pos=false;
							for(calc=0;calc<=clear_debris;calc+=1){
								if((fx-calc>=0) && flag_neg==false){
									if(map_debris[fx-calc][fy]==0){
										map_debris[fx-calc][fy]=1;
										flag_debris=true;
									}
									if(map_aux[fx-calc][fy]!=0){
										flag_neg=true;
									}
								}
								
								if( flag_pos==false && (fx+calc<int(max_x*100+1))){
									if(map_debris[fx+calc][fy]==0){
										map_debris[fx+calc][fy]=1;
										flag_debris=true;
									}
									if(map_aux[fx+calc][fy]!=0){
										flag_pos=true;
									}
								}
								
								if(flag_neg && flag_pos) break;
							}
						}
					}
				}
				
			}else{ //just a point not a line
				if(map_aux[mx0][my0]==0){
					map_aux[mx0][my0]=1;
				}
				//build_map_wall(fx,fy,n_wall);
			}
		
		}*/
	//	std::cout <<"flag_debris "<< flag_debris<<std::endl;

		if(flag_debris){//valid debris was placed in the map, check connections
			auto& node=graph.get_graph();
			bool flag_cut_master=false;
			while(n<n_max)
			{   ROS_INFO("TIME4");	
				ROS_INFO("n: %d / %d",n,n_max);
				mx0=std::lround(node[n].x * 100);
				my0=std::lround(node[n].y * 100);
				flag_cut=false;
				if(map_debris[mx0][my0]!=0){
					ROS_INFO("Unlink");
					graph.unlink(n);
					ROS_INFO(" finish Unlink");
					flag_cut=true;
					flag_cut_master=true;
				}
				if(flag_cut==false){
					auto& all_nodes=graph.get_graph();
					auto link=node[n].get_links();
					unsigned int z=0;
				
					while(z<link.size()){
						mx1=std::lround(link[z]->x*100);
						my1=std::lround(link[z]->y*100);
						unsigned int p;
						if(check_connection(mx0,my0,mx1,my1)){
							ROS_INFO("removing connection: %d %d %d %d",mx0,my0,mx1,my1);
							for (p=0;p<n_max;p++){
								if(all_nodes[p].x==link[z]->x && all_nodes[p].y==link[z]->y ){
									ROS_INFO("removed connection: %d %d",n,p); 
									graph.disconnect(n, p); 
									flag_cut_master=true; 
									break;
								}
							}						
							
							
							
							
						}
					 z++;
					}
				}
				n++;
			}
			if(flag_cut_master){
				create_new_nodes(debris);
				flag_nodes=true;
				ROS_INFO("Created nodes");
			}
			
			map_walls=map_debris;
			map=map_aux;
		}
	}
   	if(flag_nodes){
		nord_messages::Graph msg_graph;
		auto& all_nodes=graph.get_graph();
	
		unsigned int n2=0;
		while(n2<n_max){
			nord_messages::GraphNode new_node;
			new_node.id=n2;
			new_node.x=all_nodes[n2].x;
			new_node.y=all_nodes[n2].y;
			for(auto link:all_nodes[n2].get_links()){
				
				unsigned int p;
			
				for (p=n2;p<n_max;p++){
					if(all_nodes[p].x==link->x && all_nodes[p].y==link->y ) 
						new_node.children.push_back(p);
				}
				
			}
			msg_graph.data.push_back(new_node);
			//std::cout<<"graph "<<new_node.children.size()<<std::endl;
			n2++;
		}

		g_pub.publish(msg_graph);
		abort_pub.publish(std_msgs::Empty());
		ROS_INFO("TIME6");
		
	}
	rviz_marker=rviz_message();
	rviz_pub.publish(rviz_marker);
	//ROS_INFO("TIME2");
	
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nord_debris_plan");
    ros::NodeHandle n;
	
	ros::Subscriber deb_sub;
	rviz_pub      = n.advertise<visualization_msgs::Marker>("/nord/debris", 10);
	deb_sub=n.subscribe("/nord/estimation/debris",20,&DebrisCallBack);
	g_pub=n.advertise<nord_messages::Graph>("/nord/planning/graph",10);
	abort_pub=n.advertise<std_msgs::Empty>("/nord/houston/mission_abort",10);


	//ROS_INFO("Loads");
	load_graph(ros::package::getPath("nord_planning") + "/Lucas_links.txt");
//ROS_INFO("Loads");
	//actual_path = read_path(ros::package::getPath("nord_planning")+"/data/plan.txt");
//ROS_INFO("Loads");
	map=read_map(ros::package::getPath("nord_planning") + "/data/contest_maze.txt");

	auto& node=graph.get_graph();
	unsigned int n1=0;
	int mx0,my0,mx1,my1;
	map_debris=map_walls;
	while(n1<n_max){

		mx0=std::lround(node[n1].x * 100);
		my0=std::lround(node[n1].y * 100);
		bool flag_cut=false;
		if(map_debris[mx0][my0]!=0){
			graph.unlink(n1);
			flag_cut=true;
		}
		if(flag_cut==false){
			auto& all_nodes=graph.get_graph();
			auto link=node[n1].get_links();
			unsigned int z=0;
		
			while(z<link.size()){
				mx1=std::lround(link[z]->x*100);
				my1=std::lround(link[z]->y*100);
				unsigned int p;
				if(check_connection(mx0,my0,mx1,my1)){
					//ROS_INFO("removing connection: %d %d %d %d",mx0,my0,mx1,my1);
					for (p=0;p<n_max;p++){
						if(all_nodes[p].x==link[z]->x && all_nodes[p].y==link[z]->y ){
							//ROS_INFO("removed connection: %d %d",n1,p);
							graph.disconnect(n1, p); 
							break;
						}
					}						

				}
			 z++;
			}
		}
		n1++;
	}
	
{
	std::ofstream file3(ros::package::getPath("nord_planning")+"/" +(false ? "Lucas" : "Tobias") + "_links2.txt");
	std::ofstream file2(ros::package::getPath("nord_planning")+"/" +(true ? "Lucas" : "Tobias") + "_links2.txt");
	n1=0;
	while(n1<n_max)
		{   
			file3 << "%" << "\n";
			file3<< (node[n1].x ) <<','<< (node[n1].y);
			for (auto link : node[n1].get_links()){
				file3<<"\n\t"<<(link->x )<<','<<(link->y);
			}
			file3<<"\n";
			n1++;
		}
		if(!false)
			file3 << "%";

	n1=0;
	while(n1<n_max)
		{   
			file2 << "%" << "\n";
			file2<< (node[n1].x ) <<','<< (node[n1].y);
			for (auto link : node[n1].get_links()){
				file2<<"\n\t"<<(link->x )<<','<<(link->y );
			}
			file2<<"\n";
			n1++;
		}
		if(!true)
			file2 << "%";

}
	ROS_INFO("Starting Spin");	
	ros::spin();
    return 0;
}
