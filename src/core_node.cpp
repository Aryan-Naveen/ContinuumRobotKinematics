/*
 * simulator.cpp
 *
 *  Created on: Apr 1, 2017
 *      Author: haitham
 */
#include <iostream>
#include "continuum_robot/Continuum.h"

std::vector<Vector6d>  populateWaypoints(double zStart, double zFinish, int stepSize){
  std::vector<Vector6d> waypoints;
  int n = abs((int)(zFinish - zStart)/stepSize);
  std::cout << n;
  // for(int i = 0; i < n; i++){
  //   Vector6d point;
  //   point << 0, 0, zStart + i*stepSize, 0, 0, 0;
  //   waypoints.push_back(point);
  // }
  return waypoints;

}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "continuum_core");
  Continuum robot(3); // create an object robot with three sections

 // set the base Pose
 robot.setSegmentBasePose(0,tf::Vector3(0,0,0),tf::createQuaternionFromRPY(0.0, 90*PI/180,0.0));


 Vector6d thetas;
 thetas << 0, 0, 0, 0, 0, 0;
 Vector6d thinit;
 thinit << 0, 0.04, 0, 0, 0, 0;


// Assign the parameters for each section
 robot.addSegment(0,5,30,.3);  // SegID , Length, noOfSegments, radius of disk
 robot.setSegmentShape(0,0,0); // SegID , Kappa, Phi



robot.addSegment(1,5,30,.3);  // SegID , Length, noOfSegments
robot.setSegmentShape(1,0,0); // SegID , Kappa, Phi



  robot.addSegment(2,5,30,.3);  // SegID , Length, noOfSegments
 robot.setSegmentShape(2,0,0); // SegID , Kappa, Phi


 std::vector<Vector6d> waypoints;
 int zStart = 1;
 int zFinish = 0.8;
 double stepSize = 0.001;
 int n = abs((int)(zFinish - zStart)/stepSize);
 std::cout << n;
 for(int i = 0; i < n; i++){
   Vector6d point;
   point << 0, 0, zStart + i*stepSize, 0, 0, 0;
   waypoints.push_back(point);
 }
 int wayPointIndex = 0;


// Moving demonstration:
  while (ros::ok())
  { // set a pattern
    Vector6d result = robot.forwardKinematics(thetas);
    std::cout << "X est: " << result(0) << "|" << "Y est: " << result(1) << "|" << "Z: " << result(2) << "|" << "roll: " << result(3) << "|" << "pitch: " << result(4) << "|" << "yaw: " << result(5) << "|" << "\n";
    Vector6d ikresult = robot.inverseKinematics(thinit, thinit);
    result = robot.forwardKinematics(ikresult);
    std::cout << "````````````````````````````````````````````\n";
    std::cout << "X est: " << result(0) << "|" << "Y est: " << result(1) << "|" << "Z: " << result(2) << "|" << "roll: " << result(3) << "|" << "pitch: " << result(4) << "|" << "yaw: " << result(5) << "|" << "\n";
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
//  for(double i=0.25;i>=-0.25;i=i-0.01)
// {
//   robot.setSegmentShape(0,thguess(0),thguess(1)); // SegID , Kappa, Phi
//   robot.setSegmentShape(1,thguess(2),thguess(3)); // SegID , Kappa, Phi
//   robot.setSegmentShape(2,thguess(4),thguess(5)); // SegID , Kappa, Phi
//
//   robot.update();
// }
//
// for(double i=0.1;i<=.5;i=i+0.01)
// {
//   robot.setSegmentShape(0,thetas(0),thetas(1)); // SegID , Kappa, Phi
//   robot.setSegmentShape(1,thetas(2),thetas(3)); // SegID , Kappa, Phi
//   robot.setSegmentShape(2,thetas(4),thetas(5)); // SegID , Kappa, Phi
//
// robot.update();
// }
//
// for(double i=.5;i>=0.1;i=i-0.01)
// {
// 	robot.setSegmentShape(1,i,PI/2); // SegID , Kappa, Phi
// robot.update();
// }
//
// for(double i=-.5;i<=.5;i=i+0.01)
// {
// 	robot.setSegmentShape(2,i,PI/7); // SegID , Kappa, Phi
// robot.update();
// }
//
// for(double i=.5;i>=-.5;i=i-0.01)
// {
// 	robot.setSegmentShape(2,i,PI/7); // SegID , Kappa, Phi
// robot.update();
// }
  robot.update();
   }
}
