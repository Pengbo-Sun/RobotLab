#include <Perception/opencv.h> //always include this first! OpenCV headers define stupid macros
#include <Perception/opencvCamera.h>
#include <Geo/depth2PointCloud.h>

#include <Kin/frame.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>

//===========================================================================

void minimal_use_with_webcam()
{
  Var<byteA> _rgb; // (beyond this course: a 'Var<type>' is mutexed data, into which threads (like a cam) can write)
  Var<floatA> _depth;

#if 0 //using ros
  RosCamera cam(_rgb, _depth, "cameraRosNodeMarc", "/camera/rgb/image_raw", "/camera/depth/image_rect");
  //  RosCamera kin(_rgb, _depth, "cameraRosNodeMarc", "/kinect/rgb/image_rect_color", "/kinect/depth_registered/sw_registered/image_rect_raw", true);
#else //using a webcam
  OpencvCamera cam(_rgb);
#endif

  //looping images through opencv
  for (uint i = 0; i < 1000; i++)
  {
    cout << i << endl;
    _rgb.waitForNextRevision();
    {
      cv::Mat rgb = CV(_rgb.get());
      cv::Mat depth = CV(_depth.get());

      cout << "1" << endl;
      if (rgb.total() > 0)
        cv::imshow("OPENCV - rgb", rgb);
      if (depth.total() > 0)
        cv::imshow("OPENCV - depth", 0.5 * depth); //white=2meters
      int key = cv::waitKey(1);
      if ((key & 0xff) == 'q')
        break;
    }
  }
}

//===========================================================================
cv::Mat filter_red_pixel(const cv::Mat src)
{
  cv::Mat output(src.rows, src.cols, CV_8UC1);
  const std::vector<float> red_threshold = {200, 50, 50};
  for (unsigned int i = 0; i < src.rows; i++)
  {
    for (unsigned int j = 0; j < src.cols; j++)
    {
      //BGR
      float red_v = (float)src.at<cv::Vec3b>(i, j)[0];
      float green_v = (float)src.at<cv::Vec3b>(i, j)[1];
      float blue_v = (float)src.at<cv::Vec3b>(i, j)[2];
      //check if the pixel is red
      if (red_v >= red_threshold[0] && green_v < red_threshold[1] && blue_v < red_threshold[2])
      {
        output.at<unsigned char>(i, j) = 255;
      }
      else
      {
        output.at<unsigned char>(i, j) = 0;
      }
    }
  }
  return output;
}

auto get_contour(const cv::Mat src) -> std::pair<cv::Mat, std::vector<std::vector<cv::Point>>>
{
  cv::Mat binary = filter_red_pixel(src);
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  cv::Mat contour = src;
  for (size_t i = 0; i < contours.size(); i++)
  {
    cv::Scalar color = cv::Scalar(0, 0, 0);
    cv::drawContours(contour, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0);
  }
  return std::make_pair(contour, contours);
}

float get_meandepth(cv::Mat const depth, cv::Mat binary_img)
{
  float sum = 0;
  unsigned int number = 0;
  for (unsigned int i = 0; i < binary_img.rows; i++)
  {
    for (unsigned int j = 0; j < binary_img.cols; j++)
    {
      if (binary_img.at<unsigned char>(i, j) == 255)
      {
        sum += depth.at<float>(i, j);
        number++;
      }
    }
  }
  return sum / number;
}

void use_within_simulation()
{
  //-- basic setup
  rai::Configuration RealWorld;
  RealWorld.addFile("../../scenarios/challenge.g");

  RealWorld["obj1"]->setColor({1., 0, 0}); //set the color of one objet to red!

  rai::Simulation S(RealWorld, S._bullet, true);
  S.cameraview().addSensor("camera");

  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");
  rai::Frame *obj = C.addFrame("object");
  obj->setPosition({0, 0, 0});
  obj->setQuaternion({0, 0, 0, 1});
  obj->setShape(rai::ST_sphere, {0.1, 0.1});
  obj->setColor({1., .0, 1.});

  rai::ConfigurationViewer V;
  V.setConfiguration(C, "model world start state");

  // NEW: set the intrinsic camera parameters
  double f = 0.895;
  f *= 360.; //focal length is needed in pixels (height!), not meters!
  arr Fxypxy = {f, f, 320., 180.};

  //get a reference to the camera frame
  rai::Frame *cameraFrame = C["camera"];
  obj->setParent(cameraFrame, true);
  //  cameraFrame->setPose(d(-90 0 0 1) t(-.08 .205 .115) d(26 1 0 0) d(-1 0 1 0) d(6 0 0 1)");

  //-- the following is the simulation loop
  arr q;
  byteA _rgb;
  floatA _depth;
  arr points;
  double tau = .01; //time step

  //move the robot to become visible
  q = S.get_q();
  ;
  q(0) = -1.;
  q(1) = .5;
  S.step(q, tau, S._position);

  for (uint t = 0; t < 300; t++)
  {
    rai::wait(tau); //remove to go faster

    //grab sensor readings from the simulation
    q = S.get_q();
    if (!(t % 10))
    {
      S.getImageAndDepth(_rgb, _depth); //we don't need images with 100Hz, rendering is slow

      // NEW
      depthData2pointCloud(points, _depth, Fxypxy);
      cameraFrame->setPointCloud(points, _rgb);
      V.recopyMeshes(C); //update the model display!
      V.setConfiguration(C);

      //start working in CV, display the images there
      {
        cv::Mat rgb = CV(_rgb);
        cv::Mat depth = CV(_depth);
        //convert to binary
        cv::Mat binary = filter_red_pixel(rgb);
        //get contour
        auto contour_pair = get_contour(rgb);
        cv::Mat rgb_contour = contour_pair.first;
        std::vector<std::vector<cv::Point>> contours = contour_pair.second;
        //calculate the 2D center
        if (contours.size() != 0)
        {
          std::vector<cv::Point> contour = contours[0];
          cv::Moments mu = cv::moments(contour, true);
          float Z = get_meandepth(depth, binary);
          //create a sphere in the mode world
          float x = mu.m10 / mu.m00;
          float y = mu.m01 / mu.m00;
          float X = (x - 320) * Z / f;
          float Y = (y - 180) * Z / f;
          obj->setRelativePosition({X, Y, -Z});
          cv::Point2f center = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
          cv::Mat imgwithcenter = rgb;
          cv::circle(imgwithcenter, center, 20, cv::Scalar(0, 0, 0), 2);
          //cv::imshow("OPENCV - center", imgwithcenter);
        }

        if (rgb.total() > 0)
        {
          //cv::imshow("OPENCV - rgb", rgb);
          //cv::imshow("OPENCV - binary", binary);
          //cv::imshow("OPENCV- contour", rgb_contour);
        }
        //if (depth.total() > 0)
        //  cv::imshow("OPENCV - depth", 0.5 * depth); //white=2meters
        int key = cv::waitKey(1);
        if ((key & 0xff) == 'q')
          break;
      }
    }

    //send no controls to the simulation
    S.step({}, tau, S._none);
  }
  rai::wait();
}

//===========================================================================

void multipleCameras()
{
  rai::Configuration RealWorld;
  RealWorld.addFile("../../scenarios/challenge.g");

  //change the position of the central sensor
  rai::Frame *f = RealWorld["camera"];
  f->setPosition(f->getPosition() + arr{0., 0., .5});

  //add a frame for the additional camera
  f = RealWorld.addFrame("R_gripperCamera", "R_gripper");
  f->setRelativePosition({.0, .1, 0.});
  f->setShape(rai::ST_marker, {.5});

  rai::Simulation S(RealWorld, S._bullet, true);
  S.cameraview().addSensor("camera");                                   //camera is a pre-existing frame that specifies the intrinsic camera parameter
  S.cameraview().addSensor("Rcamera", "R_gripperCamera", 640, 360, 1.); //R_gripperCamera is a fresh frame - we have to specify intrinsic parameters explicitly

  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");

  rai::ConfigurationViewer V;
  V.setConfiguration(C, "model world start state");

  byteA _rgb;
  floatA _depth;

  for (uint k = 0; k < 5; k++)
  {
    //get images from the wrist
    S.cameraview().selectSensor("Rcamera");
    S.getImageAndDepth(_rgb, _depth);
    rai::wait();

    //get images from the main sensor
    S.cameraview().selectSensor("camera");
    S.getImageAndDepth(_rgb, _depth);
    rai::wait();
  }
}

//===========================================================================

int main(int argc, char **argv)
{
  rai::initCmdLine(argc, argv);

  //minimal_use_with_webcam();
  use_within_simulation();
  //multipleCameras();

  return 0;
}
