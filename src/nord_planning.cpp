#include "ros/ros.h"
#include "ros/package.h"
#include <fstream>
#include <sstream>
#include <string>

class Maps{
	public:
		int n_wall=19;int n_point=5;
	
		Maps(): pot_wall(n_wall,std::vector<int> (n_wall)), pot_point(n_point,std::vector<int> (n_point)){
						

			pot_wall={{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},
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
					{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100}};
		
			pot_point={{200,200,200,200,200},
					{200,400,400,400,200},
					{200,400,600,400,200},
					{200,400,400,400,200},
					{200,200,200,200,200}};

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
					if(my0!=my1){//diagonal lines deal with this later


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
			int dist_point=23;
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
			pot_map.resize(int(max_x*100+1), std::vector<int> (int(max_y*100+1)));
			
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
							ROS_INFO("Next_place");
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
			
		}

		void print_info(){
			int i=0;int j=0;
			std::ofstream file(ros::package::getPath("nord_planning")+"/Map.txt");
			std::ofstream file2(ros::package::getPath("nord_planning")+"/Map_pot.txt");
			ROS_INFO("printing_file");
			for (i=0;i<int(max_x*100+1);i+=1){
				for(j=0;j<int(max_y*100+1);j+=1){
					if(pointmap[i][j]>0){
						file<<'2'<<' ';
					}else{
					 file<<map[i][j]<<' ';
					}
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
		}
	
	private:
	
		
		float min_x, min_y, max_x, max_y;
		std::vector< std::vector<int> > pot_wall; std::vector< std::vector<int> > pot_point; 
		std::vector< std::vector<int> > pot_map; std::vector< std::vector<int> > map; 
		std::vector< std::vector<int> > pointmap;
		int num_points;
				
		void add_pot(int cx, int cy,std::vector< std::vector<int> > pot,int n){
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
		
		void remove_pot(int cx, int cy,std::vector< std::vector<int> > pot,int n){
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
	run.print_info();
    return 0;
}
