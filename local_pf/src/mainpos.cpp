#include <ros/ros.h>
#include <iostream>
#include "estimatepos.h"

using namespace std;
using namespace robot;

int main(int argc, char** argv){

  ros::init(argc, argv, "mainpos");
  estimatepos cm;
//  estimatepos::worldposition wp1;
//  estimatepos::mapposition mp1;
//  double wx[]={-4.5};
//  double wy[]={2.1};
//  int mx=9;
//  int my=9;
//  double origin_wx=-5,origin_wy=-5,origin_mx=-5,origin_my=-5;
//  double res=1;
//  int mapsize_x=10,mapsize_y=10;

//  for (int i=0;i<1;++i){
////  cout<<"wx : "<<wx[i]<<" wy : "<<wy[i]<<endl;
////  mp1=cm.to_map(wx[i],wy[i],origin_mx,origin_my,mapsize_x,mapsize_y,res);
////  cout<<COLOR_RED<<"To map : "<<mp1.mx<<
////        " , "<<mp1.my<<COLOR_NORMAL<< endl;

//  wp1=cm.to_world(mx,my,origin_wx,origin_wy,res);
//  cout<<"To World : "<<wp1.wx<<
//        " , "<<wp1.wy<<
//        endl;
//}
//int xr=10,yr=10;
//int xl=6,yl=2;
//vector<type_ii> v;
//v=cm.bresenham(xr,yr,xl,yl);
//cout<<"passed the bresenham"<<endl;
//cout<<"size 0f vector is : "<<v.size()<<" , "<<v[0].first<<" , "<<v[0].second<<endl;
//for(auto& i:v)
//  cout<<i.first<<" , "<<i.second<<endl;

//  estimatepos::lidardata ld1;
//  for (int i=0;i<10;++i){
//    ld1.cells[i]={i,i};
//    ld1.cellsvalue[i]=100;
//  }
  //  cm.update_map(ld1);
//cm.estimatepos::

//  geometry_msgs::Pose2D r_pose2d;
//  geometry_msgs::Point res;
//  r_pose2d.x=-2.00088;
//  r_pose2d.y=-0.495805;
//  r_pose2d.theta=0.0;
//  int ray_index=179;
//  float angle_inc=0.0175019,dist=1;
//  this_thread::sleep_for(chrono::seconds(5));

//    res=cm.lidarpoint_to_worldframe(std::ref(ray_index),std::ref(dist),std::ref(angle_inc),std::ref(r_pose2d));
//    cout<<"x in world frame is :"<<res.x<<"\n"
//        <<"y in world frame is :"<<res.y<<"\n"
//        <<"z in world frame is :"<<res.z<<endl;



  ros::spin();

  return(0);
}
