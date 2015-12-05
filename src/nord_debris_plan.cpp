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

dijkstra::map graph;

std::vector< std::vector<int> >  map;
std::vector< std::vector<int> >  map_walls;
std::vector< std::vector<int> >  map_debris;
std::vector< std::vector<int> > map_aux; 

int n_wall=29;
int clear_debris=11;
int n_debris=2*clear_debris+1;
float min_x, min_y, max_x, max_y;
std::vector<dijkstra::point> actual_path;

std::vector<dijkstra::point> read_path(std::string filename)
{
    std::ifstream file(filename);
    std::string l;
    std::vector<dijkstra::point<2>> path;
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

    std::vector<dijkstra::point> nodes;

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
        nodes.emplace_back(x, y);
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

            for (size_t i = 0; i < graph.get_graph().size(); i++)
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
        for (size_t i = 0; i < graph.get_graph().size(); i++)
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
	for(i=0;i<=((n-1)/2);i+=1){
		for(j=0;j<=((n-1)/2);j+=1){
			if((j==0) && (i==0) && map_debris[cx][cy]==0){
				map_debris[cx][cy]=2;
				flag=true;
				continue;
			} 
			if((cx-i)>=0 && (cy-j)>=0 && map_debris[cx-i][cy-j]==0){
				map_debris[cx-i][cy-j]= 2;
				flag=true;
			}
			if((cx+i)< int(max_x*100+1) && (cy-j)>=0 && i!=0 && map_debris[cx+i][cy-j]==0){
				map_debris[cx+i][cy-j]= 2;
				flag=true;
			}
			if((cx-i)>=0 && (cy+j)<int(max_y*100+1) && j!=0 && map_debris[cx-i][cy+j]==0){
				map_debris[cx-i][cy+j]= 2;
				flag=true;
			}
			if((cx+i)< int(max_x*100+1) && (cy+j)<int(max_y*100+1) && i!=0 && j!=0 && map_debris[cx+i][cy+j]==0){
				map_debris[cx+i][cy+j]= 2;
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
	int mx0,mx1,my0,my1;
	int cx,cy;
	int diffx,diffy;
	float a,b,f;
	int fx,fy;
	
	for(cx=0;cx<int(max_x*100+1);cx+=1){
		for(cy=0;cy<int(max_y*100+1);cy+=1){
			map1[cx][cy]=0;
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
						build_map_wall(fx,fy,n_wall);
					}
				}else{
					for(cx=mx1;cx<=mx0;cx+=1){
						map1[cx][cy]=1;
						build_map_wall(fx,fy,n_wall);
					}
				}
			}
		}else if(my0!=my1){//line which varies in y
			cx=mx0;
			if(my0<my1){
				for(cy=my0;cy<=my1;cy+=1){
					map1[cx][cy]=1;
					build_map_wall(fx,fy,n_wall);
				}
			}else{
				for(cy=my1;cy<=my0;cy+=1){
					map1[cx][cy]=1;
					build_map_wall(fx,fy,n_wall);
				}
			}
		}else{ //just a point not a line
			map1[mx0][my0]=1;
			build_map_wall(fx,fy,n_wall);
		}	
	}
	
	return map1;

}

bool check_connection(int mx0,int my0,int mx1,int my1){
	bool flag_cut=false;
	int diffx,diffy;
	float a,b,f;
	int fx,fy;
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
						if(map_aux[fx][fy]!=0){
							flag_cut=true;
							break;
						}
						fy=floor(f);
						if(map_aux[fx][fy]!=0){
							flag_cut=true;
							break;
						}
					}
				}else{
					for(fx=mx0;fx<=mx1;fx+=1){
						f=(a*fx+b);
						fy=ceil(f);
						if(map_aux[fx][fy]!=0){
							flag_cut=true;
							break;
						}
						fy=floor(f);
						if(map_aux[fx][fy]!=0){
							flag_cut=true;
							break;
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
						if(map_aux[fx][fy]!=0){
							flag_cut=true;
							break;
						}
						fx=floor(f);
						if(map_aux[fx][fy]!=0){
							flag_cut=true;
							break;
						}
						
					}
				}else{
					for(fy=my0;fy<=my1;fy+=1){
						f=(a*fy+b);
						fx=ceil(f);
						if(map_aux[fx][fy]!=0){
							flag_cut=true;
							break;
						}
						fx=floor(f);
						if(map_aux[fx][fy]!=0){
							flag_cut=true;
							break;
						}
						
					}
				}
				
			}
		}else{//line which varies in x
			fy=my0;
			if(mx0<mx1){
				for(fx=mx0;fx<=mx1;fx+=1){
					if(map_aux[fx][fy]!=0){
						flag_cut=true;
						break;
					}
				}
			}else{
				for(fx=mx1;fx<=mx0;fx+=1){
					if(map_aux[fx][fy]!=0){
						flag_cut=true;
						break;
					}
				}
			}
		}
	}else if(my0!=my1){//line which varies in y
		fx=mx0;
		if(my0<my1){
			for(fy=my0;fy<=my1;fy+=1){
				if(map_aux[fx][fy]!=0){
					flag_cut=true;
					break;
				}
			}
		}else{
			for(fy=my1;fy<=my0;fy+=1){
				if(map_aux[fx][fy]!=0){
					flag_cut=true;
					break;
				}
			}
		}
	}else{ //just a point not a line
		if(map_aux[mx0][my0]!=0){
			flag_cut=true;
		}
	}
	return flag_cut;
	
}

bool create_new_nodes(std::vector<nord_messages::Debris> data){
	bool flag_new=false;
	int i;
	int mx0,my0,mx1,my1;
	int diffx,diffy;
	float a,b,f;
	int fx,fy;
	
	std::vector< std::vector<int> >  map_debris2=map_debris;
	size_t num_nodes=graph.get_graph().size();
	mx0=lround(data.x*100);
	my0=lround(data.y*100);
	num_nodes=0;
	for(i=0;i<=data.hull.size();i++){
		mx1=lround(data.hull[i].x*100);
		my1=lround(data.hull[i].y*100);
		
		if(mx0!=mx1){
			if(my0!=my1){//diagonal lines 
				
				diffx=std::abs(mx1-mx0);
				diffy=std::abs(my1-my0);
				if (diffx>=diffy){//choose the coord varying the most and create a line depending on that one
					a=(((float) my0)-((float) my1))/(((float) mx0)-((float) mx1));
					b=((float) my1)-(a*((float) mx1));
					if(mx0<mx1){
						fx=mx1+clear_debris+1;
						f=(a*fx+b);
						fy=lround(f);
						if(fx < int(max_x*100+1) && fy>=0 && fy < int(max_y*100+1) && map_debris[fx][fy]==0){
							flag_new=true;
							graph.add(dijkstra::point(fx/100.0f,fy/100.0f));
						}
					}else{
						fx=mx1-clear_debris-1;
						f=(a*fx+b);
						fy=lround(f);
						if(fx>=0 && fy>=0 && fy < int(max_y*100+1) && map_debris[fx][fy]==0){
							flag_new=true;
							graph.add(dijkstra::point(fx/100.0f,fy/100.0f));
						}
					}
					
				}else{
					a=(((float) mx0)-((float) mx1))/(((float) my0)-((float) my1));
					b=((float) mx1)-(a*((float) my1));
					if(my0>my1){
						fy=my1-clear_debris-1;
						f=(a*fy+b);
						fx=lround(f);
						if(fx>=0 && fx < int(max_x*100+1) && fy>=0 && map_debris[fx][fy]==0){
							flag_new=true;
							graph.add(dijkstra::point(fx/100.0f,fy/100.0f));
						}

					}else{
						fy=my1+clear_debris+1;
						f=(a*fy+b);
						fx=lround(f);
						if(fx>=0 && fx < int(max_x*100+1) && fy < int(max_y*100+1) && map_debris[fx][fy]==0){
							flag_new=true;
							graph.add(dijkstra::point(fx/100.0f,fy/100.0f));
						}
						
					}
					
				}
			}else{//line which varies in x
				fy=my0;
				if(mx0<mx1){
					fx=mx1+clear_debris+1;
					if(fx < int(max_x*100+1) && map_debris[fx][fy]==0){
						flag_new=true;
						graph.add(dijkstra::point(fx/100.0f,fy/100.0f));
					}
					
				}else{
					fx=mx1-clear_debris-1;
					if(fx>=0 && map_debris[fx][fy]==0){
						flag_new=true;
						graph.add(dijkstra::point(fx/100.0f,fy/100.0f));
					}
				}
			}
		}else if(my0!=my1){//line which varies in y
			fx=mx0;
			if(my0<my1){
				fy=my1+clear_debris+1;
				if(fy < int(max_y*100+1) && map_debris[fx][fy]==0){
					flag_new=true;
					graph.add(dijkstra::point(fx/100.0f,fy/100.0f));
				}
			}else{
				fy=my1-clear_debris-1;
				if(fy >=0 && map_debris[fx][fy]==0){
					flag_new=true;
					graph.add(dijkstra::point(fx/100.0f,fy/100.0f));
				}
			}
		}
	}
	for(int fx=0;fx<=int(max_x*100+1);fx++){
		for(int fy=0;fy<=int(max_y*100+1);fy++){
			if(map_debris[fx][fy]==2){
				map_debris[fx][fy]=1;
			}
		}
	}
	
	if(flag_new==true){
		while(num_nodes<graph.get_graph().size()){
			
			for (size_t j = 0; j < graph.get_graph().size(); j++)
			{
				if(num_nodes!=j){
					if(check_connection(graph.get_graph()[num_nodes].x * 100 ,graph.get_graph()[num_nodes].y * 100, graph.get_graph()[j].x * 100 ,graph.get_graph()[j].y * 100)){//Verify the units 
						graph.connect(num_nodes, j);//connects both ways RIGHT???
					}
				}
			}
			num_nodes++;
		}
		
	}
	return flag_new;
}

void DebrisCallBack(const nord_messages::DebrisArray debris_array){
	map_aux=map;
	
	int mx0,mx1,my0,my1;
	int n=0;
	int cx,cy;
	int diffx,diffy;
	float a,b,f;
	int fx,fy,ff;
	int calc;
	bool flag_neg, flag_pos;
	bool flag_neg2, flag_pos2;
	bool flag_debris,flag_cut;
	
	for (auto& debris : debris_array.data)
	{
		flag_debris=false;
		for (size_t i = 0; i <= debris.hull.size() - 1; i++)
		{
			// debris.hull[i + 1].x
			
			map_debris==map_walls;
			mx0=(int)(debris.hull[i].x*100);
			my0=(int)(debris.hull[i].y*100);
			if(i==(debris.hull.size()-1)){
				mx1=(int)(debris.hull[0].x*100);
				my1=(int)(debris.hull[0].y*100);
			}else{
				mx1=(int)(debris.hull[i+1].x*100);
				my1=(int)(debris.hull[i+1].y*100);
			}
			
			if(map_aux[mx0][my0]==0){
				flag_debris=build_map_debris(mx0,my0,n_debris);
			}
			if(map_aux[mx1][my1]==0){
				flag_debris=build_map_debris(mx1,my1,n_debris);
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
								fy=ceil(f);
								if(map_aux[fx][fy]==0){
									map_aux[fx][fy]=2;
								}
								
								//build_map_wall(fx,fy,n_wall);
								ff=floor(f);
								if(map_aux[fx][fy]==0){
									map_aux[fx][fy]=2;
								}
								//build_map_wall(fx,fy,n_wall);
								flag_neg=false;
								flag_pos=false;
								flag_neg2=false;
								flag_pos2=false;
								for(calc=0;calc<=clear_debris;calc+=1){
									if((fy-calc>=0) && flag_neg==false){
										if(map_debris[fx][fy-calc]==0){
											map_debris[fx][fy-calc]=2;
											flag_debris=true;
										}
										if(map_aux[fx][fy-calc]!=0){
											flag_neg=true;
										}
									}
									
									if( flag_pos==false && (fy+calc<int(max_y*100+1))){
										if(map_debris[fx][fy+calc]==0){
											map_debris[fx][fy+calc]=2;
											flag_debris=true;
										}
										if(map_aux[fx][fy+calc]!=0){
											flag_pos=true;
										}
									}
									
									if((ff-calc>=0) && flag_neg2==false){
										if(map_debris[fx][ff-calc]==0){
											map_debris[fx][ff-calc]=2;
											flag_debris=true;
										}
										if(map_aux[fx][ff-calc]!=0){
											flag_neg2=true;
										}
									}
									
									if( flag_pos2==false && (ff+calc<int(max_y*100+1))){
										if(map_debris[fx][ff+calc]==0){
											map_debris[fx][ff+calc]=2;
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
								if(map_aux[fx][fy]==0){
									map_aux[fx][fy]=2;
								}
								//build_map_wall(fx,fy,n_wall);
								ff=floor(f);
								if(map_aux[fx][fy]==0){
									map_aux[fx][fy]=2;
								}
								//build_map_wall(fx,fy,n_wall);
								
								flag_neg=false;
								flag_pos=false;
								flag_neg2=false;
								flag_pos2=false;
								for(calc=0;calc<=clear_debris;calc+=1){
									if((fy-calc>=0) && flag_neg==false){
										if(map_debris[fx][fy-calc]==0){
											map_debris[fx][fy-calc]=2;
											flag_debris=true;
										}
										if(map_aux[fx][fy-calc]!=0){
											flag_neg==true;
										}
									}
									
									if( flag_pos==false && (fy+calc<int(max_y*100+1))){
										if(map_debris[fx][fy+calc]==0){
											map_debris[fx][fy+calc]=2;
											flag_debris=true;
										}
										if(map_aux[fx][fy+calc]!=0){
											flag_pos==true;
										}
									}
									
									if((ff-calc>=0) && flag_neg2==false){
										if(map_debris[fx][ff-calc]==0){
											map_debris[fx][ff-calc]=2;
											flag_debris=true;
										}
										if(map_aux[fx][ff-calc]!=0){
											flag_neg2==true;
										}
									}
									
									if( flag_pos2==false && (ff+calc<int(max_y*100+1))){
										if(map_debris[fx][ff+calc]==0){
											map_debris[fx][ff+calc]=2;
											flag_debris=true;
										}
										if(map_aux[fx][ff+calc]!=0){
											flag_pos2==true;
										}
									}
									
									if(flag_neg==true && flag_pos==true && flag_neg2==true && flag_pos2==true) break;
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
								if(map_aux[fx][fy]==0){
									map_aux[fx][fy]=2;
								}
								//build_map_wall(fx,fy,n_wall);
								ff=floor(f);
								if(map_aux[fx][fy]==0){
									map_aux[fx][fy]=2;
								}
								//build_map_wall(fx,fy,n_wall);
								
								flag_neg=false;
								flag_pos=false;
								flag_neg2=false;
								flag_pos2=false;
								for(calc=0;calc<=clear_debris;calc+=1){
									if((fx-calc>=0) && flag_neg==false){
										if(map_debris[fx-calc][fy]==0){
											map_debris[fx-calc][fy]=2;
											flag_debris=true;
										}
										if(map_aux[fx-calc][fy]!=0){
											flag_neg==true;
										}
									}
									
									if( flag_pos==false && (fx+calc<int(max_x*100+1))){
										if(map_debris[fx+calc][fy]==0){
											map_debris[fx+calc][fy]=2;
											flag_debris=true;
										}
										if(map_aux[fx+calc][fy]!=0){
											flag_pos==true;
										}
									}
									
									if((ff-calc>=0) && flag_neg2==false){
										if(map_debris[ff-calc][fy]==0){
											map_debris[ff-calc][fy]=2;
											flag_debris=true;
										}
										if(map_aux[ff-calc][fy]!=0){
											flag_neg2==true;
										}
									}
									
									if( flag_pos2==false && (ff+calc<int(max_x*100+1))){
										if(map_debris[ff+calc][fy]==0){
											map_debris[ff+calc][fy]=2;
											flag_debris=true;
										}
										if(map_aux[ff+calc][fy]!=0){
											flag_pos2==true;
										}
									}
									
									if(flag_neg==true && flag_pos==true && flag_neg2==true && flag_pos2==true) break;
								}
								
							}
						}else{
							for(fy=my0;fy<=my1;fy+=1){
								f=(a*fy+b);
								fx=ceil(f);
								if(map_aux[fx][fy]==0){
									map_aux[fx][fy]=2;
								}
								//build_map_wall(fx,fy,n_wall);
								ff=floor(f);
								if(map_aux[fx][fy]==0){
									map_aux[fx][fy]=2;
								}
								//build_map_wall(fx,fy,n_wall);
								
								flag_neg=false;
								flag_pos=false;
								flag_neg2=false;
								flag_pos2=false;
								for(calc=0;calc<=clear_debris;calc+=1){
									if((fx-calc>=0) && flag_neg==false){
										if(map_debris[fx-calc][fy]==0){
											map_debris[fx-calc][fy]=2;
											flag_debris=true;
										}
										if(map_aux[fx-calc][fy]!=0){
											flag_neg==true;
										}
									}
									
									if( flag_pos==false && (fx+calc<int(max_x*100+1))){
										if(map_debris[fx+calc][fy]==0){
											map_debris[fx+calc][fy]=2;
											flag_debris=true;
										}
										if(map_aux[fx+calc][fy]!=0){
											flag_pos==true;
										}
									}
									
									if((ff-calc>=0) && flag_neg2==false){
										if(map_debris[ff-calc][fy]==0){
											map_debris[ff-calc][fy]=2;
											flag_debris=true;
										}
										if(map_aux[ff-calc][fy]!=0){
											flag_neg2==true;
										}
									}
									
									if( flag_pos2==false && (ff+calc<int(max_x*100+1))){
										if(map_debris[ff+calc][fy]==0){
											map_debris[ff+calc][fy]=2;
											flag_debris=true;
										}
										if(map_aux[ff+calc][fy]!=0){
											flag_pos2==true;
										}
									}
									
									if(flag_neg==true && flag_pos==true && flag_neg2==true && flag_pos2==true) break;
								}
								
							}
						}
						
					}
				}else{//line which varies in x
					
					fy=my0;
					if(mx0<mx1){
						for(fx=mx0;fx<=mx1;fx+=1){
							if(map_aux[fx][fy]==0){
								map_aux[fx][fy]=2;
							}
							//build_map_wall(fx,fy,n_wall);
							if(fx>mx0+clear_debris && fx<mx1-clear_debris){
								flag_neg=false;
								flag_pos=false;
								for(calc=0;calc<=clear_debris;calc+=1){
									if((fy-calc>=0) && flag_neg==false){
										if(map_debris[fx][fy-calc]==0){
											map_debris[fx][fy-calc]=2;
											flag_debris=true;
										}
										if(map_aux[fx][fy-calc]!=0){
											flag_neg==true;
										}
									}
									
									if( flag_pos==false && (fy+calc<int(max_y*100+1))){
										if(map_debris[fx][fy+calc]==0){
											map_debris[fx][fy+calc]=2;
											flag_debris=true;
										}
										if(map_aux[fx][fy+calc]!=0){
											flag_pos==true;
										}
									}
									
									if(flag_neg==true && flag_pos==true ) break;
								}
							}
						}
					}else{
						for(fx=mx1;fx<=mx0;fx+=1){
							if(map_aux[fx][fy]==0){
								map_aux[fx][fy]=2;
							}
							//build_map_wall(fx,fy,n_wall);
							if(fx>mx1+clear_debris && fx<mx0-clear_debris){
								flag_neg=false;
								flag_pos=false;
								for(calc=0;calc<=clear_debris;calc+=1){
									if((fy-calc>=0) && flag_neg==false){
										if(map_debris[fx][fy-calc]==0){
											map_debris[fx][fy-calc]=2;
											flag_debris=true;
										}
										if(map_aux[fx][fy-calc]!=0){
											flag_neg==true;
										}
									}
									
									if( flag_pos==false && (fy+calc<int(max_y*100+1))){
										if(map_debris[fx][fy+calc]==0){
											map_debris[fx][fy+calc]=2;
											flag_debris=true;
										}
										if(map_aux[fx][fy+calc]!=0){
											flag_pos==true;
										}
									}
									
									if(flag_neg==true && flag_pos==true ) break;
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
							map_aux[fx][fy]=2;
						}
						//build_map_wall(fx,fy,n_wall);
						if(fy>my0+clear_debris && fy<my1-clear_debris){
							flag_neg=false;
							flag_pos=false;
							for(calc=0;calc<=clear_debris;calc+=1){
								if((fx-calc>=0) && flag_neg==false){
									if(map_debris[fx-calc][fy]==0){
										map_debris[fx-calc][fy]=2;
										flag_debris=true;
									}
									if(map_aux[fx-calc][fy]!=0){
										flag_neg==true;
									}
								}
								
								if( flag_pos==false && (fx+calc<int(max_x*100+1))){
									if(map_debris[fx+calc][fy]==0){
										map_debris[fx+calc][fy]=2;
										flag_debris=true;
									}
									if(map_aux[fx+calc][fy]!=0){
										flag_pos==true;
									}
								}
								
								if(flag_neg==true && flag_pos==true) break;
							}
						}
					}
				}else{
					for(fy=my1;fy<=my0;fy+=1){
						if(map_aux[fx][fy]==0){
							map_aux[fx][fy]=2;
						}
						//build_map_wall(fx,fy,n_wall);
						if(fy>my1+clear_debris && fy<my0-clear_debris){
							flag_neg=false;
							flag_pos=false;
							for(calc=0;calc<=clear_debris;calc+=1){
								if((fx-calc>=0) && flag_neg==false){
									if(map_debris[fx-calc][fy]==0){
										map_debris[fx-calc][fy]=2;
										flag_debris=true;
									}
									if(map_aux[fx-calc][fy]!=0){
										flag_neg==true;
									}
								}
								
								if( flag_pos==false && (fx+calc<int(max_x*100+1))){
									if(map_debris[fx+calc][fy]==0){
										map_debris[fx+calc][fy]=2;
										flag_debris=true;
									}
									if(map_aux[fx+calc][fy]!=0){
										flag_pos==true;
									}
								}
								
								if(flag_neg==true && flag_pos==true) break;
							}
						}
					}
				}
				
			}else{ //just a point not a line
				if(map_aux[mx0][my0]==0){
					map_aux[mx0][my0]=2;
				}
				//build_map_wall(fx,fy,n_wall);
			}
		
		}
		if(flag_debris==true){//valid debris was placed in the map, check connections
			for (auto& node : graph.get_graph())
			{   
				mx0=(node.x * 100);
				my0=(node.y * 100);
				flag_cut=false;
				if(map_aux[mx0][my0]!=0){
					graph.remove(n);// will this skip one node?
					n--;
					flag_cut=true;
				}
				if(flag_cut==false){
					auto& all_nodes=graph.get_graph()
					for (auto link : node.get_links()){
						mx1=link->x*100;
						my1=link->y*100;
						
						if(check_connection(mx0,my0,mx1,my1)){
							for (int p=n;p<all_nodes.size();p++){
								if(all_nodes[p].x==link->x && all_nodes[p].y==link->y ) break;
							}						
							
							graph.disconnect(n, p);
							flag_cut=true;
						}
					}
				}
				if(flag_cut==true){
					create_new_nodes(debris);
				}
				n++;
			}
			map_walls=map_debris;
			map=map_aux;
		}
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nord_planning");
    ros::NodeHandle n;
	
	ros::subscriber deb_sub;
	
	deb_sub=n.subscribe("nord_estimation/debris",&DebrisCallBack,10,this);
	
	load_graph(ros::package::getPath("nord_planning") + "/links2.txt");
	actual_path = read_path(ros::package::getPath("nord_houston")+"/data/plan.txt");
	map=read_map(ros::package::getPath("nord_planning") + "/data/contest_rehearsal_maze.txt");
	
	ros::spin();
    return 0;
}
