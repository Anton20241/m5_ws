#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <math.h>

using namespace std;
using namespace cv;
using namespace cv::aruco;

const Size chessboardDimension = Size(10, 7);

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
  printf("\n");
  printf("x     = %f\n", transform.getOrigin().x());
  printf("y     = %f\n", transform.getOrigin().y());
  printf("z     = %f\n", transform.getOrigin().z());
  printf("roll  = %f\n", roll);
  printf("pitch = %f\n", pitch);
  printf("yaw   = %f\n", yaw);
}

void parseWord(const std::string &word, double &matWord){
  if(word.find(',') == std::string::npos){
    matWord = std::stod(word);
    return;
  }

  std::string integer    = word.substr(0, word.find(','));
  std::string fractional = word.substr(word.find(',') + 1, word.size());
  std::string number     = integer + '.' + fractional;
  matWord                = std::stod(number);
}

void parseLine(const std::string &line, std::vector<double> &matLine){
  std::string word = "";
  double matWord   = 0;
  for (size_t i = 0; i < line.size(); i++)
  {
    if ((line[i] >= '0' && line[i] <= '9') || line[i] == ',' || line[i] == '-'){
      word += line[i];
    } else {
      parseWord(word, matWord);
      matLine.push_back(matWord);
      word = "";
    }
  }
}

void getMatFromFile(std::vector<std::vector<double>> &mat, std::string fileName){
  mat.clear();
  std::ifstream matFile(fileName);
  assert(matFile.is_open());
  std::string line = "";
  while (std::getline(matFile, line)){
    std::vector<double> matLine;
    parseLine(line, matLine);
    mat.push_back(matLine);
  }
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
    printCamera2ImgSomePointPose(transform);
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

void getZWorldCoordForAllFrames(  const std::vector<std::vector<double>> &WorldPoints,
                                  const std::vector<std::vector<double>> &TranslationVectors,
                                  const std::vector<std::vector<double>> &RotationVectors,
                                  const  tf::TransformListener &listener){

  for (size_t i = 0; i < TranslationVectors.size(); i++)  // для всех кадров i < TranslationVectors.size()
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
  std::cout << "\n";
  for (size_t i = 0; i < mat.size(); i++)
  {
    for (auto iter = mat[i].begin(); iter != mat[i].end(); iter++)
    {
      printf("%.0f ", *iter);
    }
    std::cout << "\n";
  }
}

//извлечение из изображения обнаруженных углов шахматной доски (любые углы)
void getChessBoardCorners(cv::Mat &img, std::vector<cv::Point2f> &foundCorners, bool showResults, size_t i){

  bool found = findChessboardCorners(img, Size(chessboardDimension.width - 1,chessboardDimension.height - 1), foundCorners,
                                      CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
  std::cout << "found = " << found << std::endl;
  if (showResults) {
    drawChessboardCorners(img, Size(chessboardDimension.width - 1,chessboardDimension.height - 1), foundCorners, found);
    imshow(std::to_string(i), img);
  }
}

void printVecPoint2f(const std::vector<cv::Point2f> &vec){
  for (size_t i = 0; i < vec.size(); i++)
  {
    std::cout << vec[i].x << "\t\t" << vec[i].y << std::endl;
  }
}

void printVecDouble(const std::vector<double> &vec){
  for (size_t i = 0; i < vec.size(); i++)
  {
    std::cout << vec[i] << std::endl;
  }
}

void reverse(std::vector<cv::Point2f> &foundCorners){
  std::vector<cv::Point2f> tmpFoundCorners;

  for (size_t i = 0; i < 9; i++)
  {
    for (size_t j = 0; j < 6; j++)
    {
      tmpFoundCorners.push_back(foundCorners[j * 9 + i]);
    }
  }
  foundCorners.clear();
  foundCorners = tmpFoundCorners;

}

void getXYPixelIntersectionsCoordForAllFrames(const std::vector<std::vector<double>> &TranslationVectors,
                                              vector<vector<Point2f>> &allFoundCorners){

  for (size_t i = 0; i < 1; i++)  // для всех кадров i < TranslationVectors.size()
  {
    cv::Mat img;
    std::string imgAddr1 = "m5_calib/m_depthFrame";
    std::string imgAddr2 = std::to_string(i);
    std::string imgAddr3 = ".png";
    std::string imgAddr  = imgAddr1 + imgAddr2 + imgAddr3;
    img = cv::imread(imgAddr, cv::IMREAD_COLOR);
    assert(!img.empty());
    std::vector<cv::Point2f> foundCorners;
    getChessBoardCorners(img, foundCorners, 1, i);
    reverse(foundCorners);
    // std::cout << "foundCorners.size() = " << foundCorners.size() << std::endl;
    allFoundCorners.push_back(foundCorners);
    printVecPoint2f(foundCorners);
    waitKey(0);
  }
}

void getPopugayForOnePoint( const vector<vector<Point2f>> &allFoundCorners,
                            const std::vector<std::vector<double>> &m_depthFrame,
                            double &popugay,
                            const size_t &i,
                            const size_t &j){

  popugay = 0;
  popugay = m_depthFrame[std::round(allFoundCorners[i][j].y)][std::round(allFoundCorners[i][j].x)];

  int count = 0;
  while (popugay == 0){
    count ++;
    for (size_t k = std::round(allFoundCorners[i][j].y) - count; k < std::round(allFoundCorners[i][j].y) + count; k++)
    {
      for (size_t m = std::round(allFoundCorners[i][j].x) - count; m < std::round(allFoundCorners[i][j].x) + count; m++)
      {
        std::cout << "k = " << k << " m = " << m << std::endl;
        std::cout << "m_depthFrame[k][m] = " << m_depthFrame[k][m] << std::endl;
        popugay = m_depthFrame[k][m];
      }
    }
  }

  // allFoundCorners[i]    - вектор найденных углов на i кадре
  // allFoundCorners[i][j] - j угол на i кадре
  // allFoundCorners[i][j].x
  // allFoundCorners[i][j].y

}

void getPopugaysForOneFrame(const  vector<vector<Point2f>> &allFoundCorners, vector<double> &popugayVec, const size_t &i){ // для 1 кадра
  
  std::vector<std::vector<double>> m_depthFrame;
  std::string fileAddr1 = "m5_calib/m_depthFrame";
  std::string fileAddr2 = std::to_string(i);
  std::string fileAddr3 = ".txt";
  std::string fileAddr  = fileAddr1 + fileAddr2 + fileAddr3;

  getMatFromFile(m_depthFrame, fileAddr);
  // printf("\nm_depthFrame:\n");
  // printMat(m_depthFrame);

  // std::cout << "m_depthFrame.size() = " << m_depthFrame.size() << std::endl;
  // std::cout << "m_depthFrame[0].size() = " << m_depthFrame[0].size() << std::endl;
  
  for (size_t j = 0; j < allFoundCorners[i].size(); j++)    // для всех точек кадра
  {
    double popugay = 0;
    getPopugayForOnePoint(allFoundCorners, m_depthFrame, popugay, i, j);
    popugayVec.push_back(popugay);
  }
  std::cout << "\npopugayVec:\n";
  printVecDouble(popugayVec);

}

void getPopugaysForAllFrames(const  vector<vector<Point2f>> &allFoundCorners, 
                                    vector<vector<double>> &popugayMat){
  for (size_t i = 0; i < allFoundCorners.size(); i++)               // всего 18 векторов
  {
    vector<double> popugayVec;
    getPopugaysForOneFrame(allFoundCorners, popugayVec, i);          // получаем вектор попугаев для 1 кадра
    // std::cout << "popugayVec.size() = " << popugayVec.size() << std::endl;
    popugayMat.push_back(popugayVec);
  }
  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pointTransform");
  ros::NodeHandle n;

  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;

  std::vector<std::vector<double>>    WorldPoints;
  std::vector<std::vector<double>>    TranslationVectors;
  std::vector<std::vector<double>>    RotationVectors;
  getMatFromFile(WorldPoints,        "WorldPoints.txt");
  getMatFromFile(TranslationVectors, "TranslationVectors.txt");
  getMatFromFile(RotationVectors,    "RotationVectors.txt");

  // printf("\nWorldPoints:\n");
  // printMat(WorldPoints);
  // printf("\nTranslationVectors:\n");
  // printMat(TranslationVectors);
  // printf("\nRotationVectors:\n");
  // printMat(RotationVectors);

  // getZWorldCoordForAllFrames(WorldPoints, TranslationVectors, RotationVectors, listener);

  vector<vector<Point2f>> allFoundCorners;  // 18 х 54 углов шахматной доски
  getXYPixelIntersectionsCoordForAllFrames(TranslationVectors, allFoundCorners);
  std::cout << "allFoundCorners.size() = " << allFoundCorners.size() << std::endl;

  vector<vector<double>> popugayMat;        // 18 x 54 попугая шахматной доски
  getPopugaysForAllFrames(allFoundCorners, popugayMat);
  std::cout << "popugayMat.size() = " << popugayMat.size() << std::endl;




  return 0;
}