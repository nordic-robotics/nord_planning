#include "graph.hpp"


namespace graph
{	
	Maps::Maps(): pot_wall(n_wall,std::vector<long int> (n_wall)), pot_point(n_point,std::vector<long int> (n_point)){
		
		//Initiate wall_point potential matrix
		for(int i=0;i<=(n_wall-1)/2;i+=1){
			for(int j=0;j<=(n_wall-1)/2;j+=1){
				if(i>j){
					pot_wall[i][j]=(long int)(pow(3,j)*100);
					pot_wall[n_wall-1-i][n_wall-1-j]=(long int)(pow(3,j)*100);
					pot_wall[i][n_wall-1-j]=(long int)(pow(3,j)*100);
					pot_wall[n_wall-1-i][j]=(long int)(pow(3,j)*100);
				}else{
					pot_wall[i][j]=(long int)(pow(3,i)*100);
					pot_wall[n_wall-1-i][n_wall-1-j]=(long int)(pow(3,i)*100);
					pot_wall[i][n_wall-1-j]=(long int)(pow(3,i)*100);
					pot_wall[n_wall-1-i][j]=(long int)(pow(3,i)*100);
				}
			}
		}
		
		//Initiate node_point potential matrix
		for(int i=0;i<=(n_point-1)/2;i+=1){
			for(int j=0;j<=(n_point-1)/2;j+=1){
				if(i>j){
					pot_point[i][j]=(long int)(pow(3,j));
					pot_point[n_point-1-i][n_point-1-j]=(long int)(pow(3,j));
					pot_point[i][n_point-1-j]=(long int)(pow(3,j));
					pot_point[n_point-1-i][j]=(long int)(pow(3,j));
				}else{
					pot_point[i][j]=(long int)(pow(3,i));
					pot_point[n_point-1-i][n_point-1-j]=(long int)(pow(3,i));
					pot_point[i][n_point-1-j]=(long int)(pow(3,i));
					pot_point[n_point-1-i][j]=(long int)(pow(3,i));
				}
			}
		}
		

		map=read_map(ros::package::getPath("nord_planning") + "/data/small_maze.txt");
		
		create_pointmap();
		
		create_pot_map();
	}
	
	std::vector< std::vector<int> > Maps::read_map(std::string filename){
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
					
					diffx=mx1-mx0;
					diffy=my1-my0;
					if (diffx>=diffy){//choose the coord varying the most and create a line depending on that one
						a=(((float) my0)-((float) my1))/(((float) mx0)-((float) mx1));
						b=((float) my1)-(a*((float) mx1));
						if(mx0>mx1){
							for(fx=mx1;fx<=mx0;fx+=1){
								f=(a*fx+b);
								fy=ceil(f);
								map1[fx][fy]=1;
								fy=floor(f);
								map1[fx][fy]=1;
							}
						}else{
							for(fx=mx0;fx<=mx1;fx+=1){
								f=(a*fx+b);
								fy=ceil(f);
								map1[fx][fy]=1;
								fy=floor(f);
								map1[fx][fy]=1;
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
								fx=floor(f);
								map1[fx][fy]=1;
								
							}
						}else{
							for(fy=my0;fy<=my1;fy+=1){
								f=(a*fy+b);
								fx=ceil(f);
								map1[fx][fy]=1;
								fx=floor(f);
								map1[fx][fy]=1;
								
							}
						}
						
					}
				}else{//line which varies in x
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
			}else if(my0!=my1){//line which varies in y
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
			}else{ //just a point not a line
				map1[mx0][my0]=1;
			}	
		}
		

		return map1;

	}
	
	void Maps::create_pointmap(){
		pointmap.resize(int(max_x*100+1), std::vector<int> (int(max_y*100+1)));
		int cx,cy;
		int dist_point=25;
		num_points=0;
		//Initialize pointmap
		for(cx=0;cx<int(max_x*100+1);cx+=1){
			for(cy=0;cy<int(max_y*100+1);cy+=1){
				pointmap[cx][cy]=0;
			}
		}
		//Spread initial nodes they are represented by 1 in the variable pointmap
		for(cx=0;cx<int(max_x*100+1);cx+=dist_point){
			for(cy=0;cy<int(max_y*100+1);cy+=dist_point){
				pointmap[cx][cy]=1;
				num_points+=1;
			}
		}
		
	}
	
	void Maps::create_pot_map(){
		pot_map.resize(int(max_x*100+1), std::vector<long int> (int(max_y*100+1)));
		
		int cx,cy;
		for(cx=0;cx<int(max_x*100+1);cx+=1){
			for(cy=0;cy<int(max_y*100+1);cy+=1){
				pot_map[cx][cy]=0;
			}
		}
		
		for(cx=0;cx<int(max_x*100+1);cx+=1){
			for(cy=0;cy<int(max_y*100+1);cy+=1){
				
				if(map[cx][cy]==1){//Add potential field of wall in given coords
					add_pot(cx,cy,pot_wall,n_wall);
				}
				
				if(pointmap[cx][cy]==1){//Add potential field of node in given coords
					add_pot(cx,cy,pot_point,n_point);
				}
			}
		}

	}
	
	void Maps::move_points(){
		int cx,cy;
		int flag=0;
		int fx,fy;
		int step=3;
		int i=1;
		std::vector<int> vec(2,0);
		//move nodes to local minima
		while(flag==0){
			flag=1;
			for(cx=0;cx<int(max_x*100+1);cx+=1){
				for(cy=0;cy<int(max_y*100+1);cy+=1){
					if(pointmap[cx][cy]>0 && pointmap[cx][cy]<=i){
						pointmap[cx][cy]=0;
						remove_pot(cx,cy,pot_point,n_point);//do the analysis of local minima for a node without his potential values
						
						fx=cx;
						fy=cy;
						//ROS_INFO("old pos: %d,%d",fx,fy);
						vec=next_place(fx,fy,step); //find the local minima
						fx=vec[0];
						fy=vec[1];
						//ROS_INFO("returned: %d,%d",fx,fy);
						if((fx!=cx) || (fy!=cy)){//if one node changed iterate everything again
							flag=0;
						}
						add_pot(fx,fy,pot_point,n_point);//add the potential values again
						pointmap[fx][fy]=i+1;
					}
				}
			}
			
			ROS_INFO("Iteration: %d",i);
			i+=1;
		}
		//Write the nodes in the map matrix
		for (i=0;i<int(max_x*100+1);i+=1){
			for(int j=0;j<int(max_y*100+1);j+=1){
				if(pointmap[i][j]>0){
					map[i][j]=2;
				}
			}
		}
		
	}

	void Maps::print_info(){
		int i=0;int j=0;
		std::ofstream file(ros::package::getPath("nord_planning")+"/Map.txt");
		std::ofstream file2(ros::package::getPath("nord_planning")+"/Map_pot.txt");
		std::ofstream file3(ros::package::getPath("nord_planning")+"/links.txt");
		std::ofstream file1(ros::package::getPath("nord_planning")+"/pot_wall_point.txt");
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
		
		for(i=0;i<n_wall;i+=1){
			for(j=0;j<n_wall;j+=1){
				file1<<pot_wall[i][j]<<' ';
			}
			file1<<'\n';
		}
		file1<<"\n\n\n\n";
		for(i=0;i<n_point;i+=1){
			for(j=0;j<n_point;j+=1){
				file1<<pot_point[i][j]<<' ';
			}
			file1<<'\n';
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
	
	void Maps::create_graph(){
		int cx,cy;
		int node=3;
		int square=14;//half the size of the side of the square that goes through the line and sees if there is any wall near that path
		int i,j;
		int wall_flag;
		int fx,fy;
		int mx,my;
		float a,b;
		int calc;
		
		//place nodes in the graph	
		for(cx=0;cx<int(max_x*100+1);cx+=1){
			for(cy=0;cy<int(max_y*100+1);cy+=1){
				if(map[cx][cy]==2){
					graph.emplace_back(cx/100.0f, cy/100.0f);
					map[cx][cy]=node;//numerate the nodes
					node+=1;
				}
			}
		}
		m = dijkstra::map(std::move(graph));
		
		for(cx=0;cx<int(max_x*100+1);cx+=1){
			for(cy=0;cy<int(max_y*100+1);cy+=1){
				
				if(map[cx][cy]>2){//if the point being analysed its a node, fully analyse its connections
					
					node=map[cx][cy];
					for(i=0;i<int(max_x*100+1);i+=1){
						for(j=0;j<int(max_y*100+1);j+=1){
							
							if(map[i][j]>node){//if the point is a node with its connections not fully analysed
								mx=abs(i-cx);
								my=abs(j-cy);
								wall_flag=0;
								//do a line a analysis as before (when reading the map for diagonal lines), 
								//but this time verify also "square" number of points on each side of the line to guarantee
								//that there are no walls in the way
								if (mx>=my){
									a=(((float) cy)-((float) j))/(((float) cx)-((float) i));
									b=((float) j)-(a*((float) i));
									if(cx>i){
										for(fx=i;fx<=cx;fx+=1){
											calc=(int) (a*fx+b);
											for(fy=calc-square;fy<=calc+square;fy+=1){
												if((fy>=0) && (fy<int(max_y*100+1))){
													if(map[fx][fy]==1){
														wall_flag=1;
														break;
													}
												}
											}
											if(wall_flag==1){
												break;
											}
										}
									}else{
										for(fx=cx;fx<=i;fx+=1){
											calc=(int) (a*fx+b);
											for(fy=calc-square;fy<=calc+square;fy+=1){
												if((fy>=0) && (fy<int(max_y*100+1))){
													if(map[fx][fy]==1){
														wall_flag=1;
														break;
													}
												}
											}
											if(wall_flag==1){
												break;
											}
										}
									}
									
								}else{
									a=(((float) cx)-((float) i))/(((float) cy)-((float) j));
									b=((float) i)-(a*((float) j));
									if(cy>j){
										for(fy=j;fy<=cy;fy+=1){
											calc=(int) (a*fy+b);
											for(fx=calc-square;fx<=calc+square;fx+=1){
												if((fx>=0) && (fx<int(max_x*100+1))){
													if(map[fx][fy]==1){
														wall_flag=1;
														break;
													}
												}
											}
											if(wall_flag==1){
												break;
											}
										}
									}else{
										for(fy=cy;fy<=j;fy+=1){
											calc=(int) (a*fy+b);
											for(fx=calc-square;fx<=calc+square;fx+=1){
												if((fx>=0) && (fx<int(max_x*100+1))){
													if(map[fx][fy]==1){
														wall_flag=1;
														break;
													}
												}
											}
											if(wall_flag==1){
												break;
											}
										}
									}
									
								}
								//if there are no walls connect, note that the node number started with 3 and not zero,
								//thats way we need to take 3 of the number
								if(wall_flag==0){
									//ROS_INFO("connected: %d : %d",(map[cx][cy]-3),(map[i][j]-3));
									//ROS_INFO("connected: %d %d : %d %d",cx,cy,i,j);
									 m.connect((map[cx][cy]-3),(map[i][j]-3));
								}
							}
						}
					}
					
				}
			}
		}

	}


	
	
	//Add potential values to the pot_map
	void Maps::add_pot(int cx, int cy,std::vector< std::vector<long int> > pot,int n){
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
	//Remove potential values of the pot_map
	void Maps::remove_pot(int cx, int cy,std::vector< std::vector<long int> > pot,int n){
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
	//Search for the local minima in the given coords, and considering a max movement per call of "step"
	std::vector<int> Maps::next_place(int fx, int fy, int step){
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
	
	
}
