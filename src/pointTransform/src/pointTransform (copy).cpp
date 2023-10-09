#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include <sstream>

bool getTransform = false;
bool dataIsWrote  = false;

void sendCamera2ImgStartPointTransform( const std::vector<std::vector<double>> &TranslationVectors,
                                        const std::vector<std::vector<double>> &RotationVectors,
                                        const size_t i){
  assert(!TranslationVectors[i].empty());
  assert(!RotationVectors[i].empty()); 
  tf::TransformBroadcaster br;
  tf::Transform tr;
  tr.setOrigin( tf::Vector3(TranslationVectors[i][0], 
                            TranslationVectors[i][1], 
                            TranslationVectors[i][2]) );
  tf::Quaternion quaternion;
  quaternion.setRPY(RotationVectors[i][0], RotationVectors[i][1], RotationVectors[i][2]);
  tr.setRotation(quaternion);
  br.sendTransform( tf::StampedTransform(tr, ros::Time::now(), 
      "camera" + std::to_string(i), "img_start_point" + std::to_string(i)) );
  // std::cout << "\n";
  // ROS_INFO("camera -> img_start_point");
}

void sendImgStartPoint2ImgSomePointTransform( const std::vector<std::vector<double>> &WorldPoints,
                                              const size_t i,
                                              const size_t j){
  assert(!WorldPoints[j].empty());
  tf::TransformBroadcaster br;
  tf::Transform tr;
  tr.setOrigin( tf::Vector3(  WorldPoints[j][0], 
                              WorldPoints[j][1], 
                              0) );
  tf::Quaternion quaternion;
  quaternion.setRPY(0, 0, 0);
  tr.setRotation(quaternion);
  br.sendTransform(tf::StampedTransform(tr, ros::Time::now(), 
      "img_start_point" + std::to_string(i), "img_some_point_" + std::to_string(i) + "_" + std::to_string(j)));
  // ROS_INFO("img_start_point -> img_some_point");
}

void getCamera2ImgSomePointTransform( const  tf::TransformListener &listener,
                                      tf::StampedTransform  &transform,
                                      const size_t i,
                                      const size_t j){
  try{
    
    // ros::Time now = ros::Time::now();
    // listener.waitForTransform("camera" + std::to_string(i), "img_some_point_" + std::to_string(i) + "_" + std::to_string(j),
    //                           now, ros::Duration(3.0));
    // listener.lookupTransform("camera" + std::to_string(i), "img_some_point_" + std::to_string(i) + "_" + std::to_string(j),
    //                          now, transform);    

    listener.lookupTransform( "camera" + std::to_string(i), 
                              "img_some_point_" + std::to_string(i) + "_" + std::to_string(j),
                              ros::Time(0), transform);
    getTransform = true;
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    getTransform = false;
  }
}

void printCamera2ImgSomePointPose(const tf::StampedTransform &transform){
  if (!getTransform) return;
  dataIsWrote = true;
  tf::Quaternion quaternion(transform.getRotation().x(),
                            transform.getRotation().y(),
                            transform.getRotation().z(),
                            transform.getRotation().w());
  tf::Matrix3x3 m(quaternion);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  printf("x     = %f\n", transform.getOrigin().x());
  printf("y     = %f\n", transform.getOrigin().y());
  printf("z     = %f\n", transform.getOrigin().z());
  printf("roll  = %f\n", roll);
  printf("pitch = %f\n", pitch);
  printf("yaw   = %f\n", yaw);
}

void getMatFromFile(std::vector<std::vector<double>> &mat, std::string fileName){
  mat.clear();
  std::ifstream matFile(fileName);
  assert(matFile.is_open());
  std::string line = "";
  while (std::getline(matFile, line)){
    std::string word = "";
    std::stringstream x;
    x << line;
    std::vector<double> matLine;
    while (x >> word){
      matLine.push_back(std::stod(word));
    }
    // std::cout << fileName.substr(0, fileName.size() - 4) << "Line.size() = " << matLine.size() << std::endl;
    mat.push_back(matLine);
  }
  // std::cout << fileName.substr(0, fileName.size() - 4) << ".size() = " << mat.size() << std::endl;
}

// 0 <= j <= 53
// 0 <= i <= 21
void getCam2PointsZCoordinate(std::vector<double> &Cam2PointsZCoordinate,
                        const std::vector<std::vector<double>> &WorldPoints,
                        const std::vector<std::vector<double>> &TranslationVectors,
                        const std::vector<std::vector<double>> &RotationVectors,
                        const  tf::TransformListener &listener,
                        const size_t i){
  for (size_t j = 0; j < WorldPoints.size(); j++)       // для всех кл точек на кадре
  {
    ros::Rate r(100);
    tf::StampedTransform transform;
    do{
      sendCamera2ImgStartPointTransform(TranslationVectors, RotationVectors, i);
      sendImgStartPoint2ImgSomePointTransform(WorldPoints, i, j);
      getCamera2ImgSomePointTransform(listener, transform, i, j);
      r.sleep();
    } while(!getTransform);
    Cam2PointsZCoordinate.push_back(transform.getOrigin().z());
  }
}

void getFileFromVec(const std::vector<double> &vec, std::string fileName){
  assert(!vec.empty());
  std::ofstream vecFile(fileName);
  assert(vecFile.is_open());
  vecFile.clear();
  for (size_t i = 0; i < vec.size(); i++){
    vecFile << std::to_string(vec[i]) << std::endl;
  }
}

void getZCoordForAllFrames( const std::vector<std::vector<double>> &WorldPoints,
                            const std::vector<std::vector<double>> &TranslationVectors,
                            const std::vector<std::vector<double>> &RotationVectors,
                            const  tf::TransformListener &listener){

  for (size_t i = 0; i < TranslationVectors.size(); i++)  // для всех кадров
  {
    std::vector<double> Cam2PointsZCoordinate;            // Z для всех кл точек кадра
    getCam2PointsZCoordinate( Cam2PointsZCoordinate,
                              WorldPoints,
                              TranslationVectors,
                              RotationVectors,
                              listener,
                              i);
    getFileFromVec(Cam2PointsZCoordinate, "Cam2PointsZCoordinate" + std::to_string(i) + ".txt");
  }
}

void printMat(std::vector<std::vector<double>> &mat){
  assert(!mat.empty());
  for (size_t i = 0; i < mat.size(); i++)
  {
    for (auto iter = mat[i].begin(); iter != mat[i].end(); iter++)
    {
      std::cout << *iter << " ";
    }
    std::cout << "\n";
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pointTransform");
  ros::NodeHandle n;
  tf::TransformListener listener;

  std::vector<std::vector<double>>    WorldPoints;
  std::vector<std::vector<double>>    TranslationVectors;
  std::vector<std::vector<double>>    RotationVectors;
  getMatFromFile(WorldPoints,        "WorldPoints.txt");
  getMatFromFile(TranslationVectors, "TranslationVectors.txt");
  getMatFromFile(RotationVectors,    "RotationVectors.txt");

  // printMat(TranslationVectors);

  // getZCoordForAllFrames(WorldPoints, TranslationVectors, RotationVectors, listener);

  tf::TransformBroadcaster broadcaster;
  tf::StampedTransform transform;
  ros::Rate r(100);
  while(n.ok()){
    sendCamera2ImgStartPointTransform(TranslationVectors, RotationVectors, 0);
    sendImgStartPoint2ImgSomePointTransform(WorldPoints, 0, 0);
    getCamera2ImgSomePointTransform(listener, transform, 0, 0);
    printCamera2ImgSomePointPose(transform);
    r.sleep();
  }

  return 0;
}