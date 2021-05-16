#include <Perception/opencv.h> //always include this first! OpenCV headers define stupid macros
#include <Geo/depth2PointCloud.h>

#include <Kin/feature.h>
#include <Kin/frame.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>

//===========================================================================
cv::Mat filter_red_pixel(const cv::Mat src)
{
  cv::Mat output(src.rows, src.cols, CV_8UC1);
  const std::vector<float> red_threshold = {200, 50, 50};
  for (int i = 0; i < src.rows; i++)
  {
    for (int j = 0; j < src.cols; j++)
    {
      //BGR
      double red_v = (double)src.at<cv::Vec3b>(i, j)[0];
      double green_v = (double)src.at<cv::Vec3b>(i, j)[1];
      double blue_v = (double)src.at<cv::Vec3b>(i, j)[2];
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

auto get_contour(const cv::Mat binary) -> std::vector<std::vector<cv::Point>>
{
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  return contours;
}

cv::Mat draw_contour(const cv::Mat src, const std::vector<std::vector<cv::Point>> contours)
{
  cv::Mat contour = src.clone();
  std::vector<cv::Vec4i> hierarchy;
  for (size_t i = 0; i < contours.size(); i++)
  {
    cv::Scalar color = cv::Scalar(0, 0, 0);
    cv::drawContours(contour, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0);
  }
  return contour;
}

double get_meandepth(cv::Mat const depth, cv::Mat binary_img)
{
  double sum = 0;
  double number = 0;
  for (int i = 0; i < binary_img.rows; i++)
  {
    for (int j = 0; j < binary_img.cols; j++)
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

void grasp_the_hopping_ball()
{
  //-- setup your RealWorld
  rai::Configuration RealWorld;
  RealWorld.addFile("../../scenarios/challenge.g");

  //delete frames with certain names
  for (uint o = 1; o < 30; o++)
  {
    rai::Frame *f = RealWorld[STRING("obj" << o)];
    if (f)
      delete f;
  }

  rai::Frame *realObj = RealWorld["obj0"];
  realObj->setColor({1., 0, 0}); //set the color of one objet to red!
  realObj->setShape(rai::ST_sphere, {.03});
  //  realObj->setShape(rai::ST_ssBox, {.05, .05, .2, .01});
  realObj->setPosition({0., .2, 2.});
  realObj->setContact(1);

  rai::Simulation S(RealWorld, S._bullet, 1);
  S.cameraview().addSensor("camera");

  //add an imp!!
  //  S.addImp(S._objectImpulses, {"obj0"}, {});

  //-- setup your model world
  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");

  rai::Frame *obj = C.addFrame("object");
  obj->setColor({1., 1., 0}); //set the color of one objet to red!
  obj->setShape(rai::ST_sphere, {.03});
  auto gripper_pose = C.feature(FS_position, {"R_gripperCenter"})->eval(C);
  obj->setPosition(gripper_pose.y);

  rai::ConfigurationViewer V;
  V.setConfiguration(C, "model world start state");

  //set the intrinsic camera parameters
  double f = 0.895;
  f *= 360.; //focal length is needed in pixels (height!), not meters!
  arr Fxypxy = {f, f, 320., 180.};

  //get a reference to the camera frame
  rai::Frame *cameraFrame = C["camera"];
  obj->setParent(cameraFrame, true);

  //-- the following is the simulation loop
  arr q;
  byteA _rgb;
  floatA _depth;
  arr points;
  double tau = .5; //time step

  bool gripping = false;
  bool grasped = false;

  for (uint t = 0; t < 1000; t++)
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
      cv::Mat rgb = CV(_rgb);
      cv::Mat depth = CV(_depth);

      //--<< perception pipeline
      cv::Mat binary = filter_red_pixel(rgb);
      //get contour
      std::vector<std::vector<cv::Point>> contours = get_contour(binary);
      cv::Mat rgb_contour = draw_contour(rgb, contours);
      //calculate the 2D center
      if (contours.size() != 0)
      {
        std::vector<cv::Point> contour = contours[0];
        cv::Moments mu = cv::moments(contour, true);
        double Z = get_meandepth(depth, binary);
        //create a sphere in the mode world
        double x = mu.m10 / mu.m00;
        double y = mu.m01 / mu.m00;
        double X = (x - 320) * Z / f;
        double Y = (y - 180) * Z / f;
        //set the model object to percept
        obj->setRelativePosition({X, -Y, -Z});
      }

      if (rgb.total() > 0)
      {
        //cv::imshow("OPENCV - rgb", rgb);
        cv::imshow("OPENCV - binary", binary);
        cv::imshow("OPENCV- contour", rgb_contour);
      }
    }

    C.setJointState(q); //set your robot model to match the real q
    V.setConfiguration(C);

    //some good old fashioned IK
    auto diff = C.feature(FS_positionDiff, {"R_gripperCenter", "object"})->eval(C);
    auto vecX = C.feature(FS_vectorX, {"R_gripperCenter"})->eval(C);
    auto vecZ = C.feature(FS_vectorZ, {"R_gripperCenter"})->eval(C);

    //stack them
    arr y, J;

    y.append(1e0 * diff.y); //multiply, to make faster
    J.append(1e0 * diff.J);

    y.append(vecX.y - arr{0., 1., 0}); //subtract target
    J.append(vecX.J);

    y.append(vecZ.y - arr{1. / sqrt(2.), 0., 1. / sqrt(2.)});
    J.append(vecZ.J);

    arr vel = 2. * pseudoInverse(J, NoArr, 1e-2) * (-y);

    V.setConfiguration(C, "model world start state");

    if (!gripping && length(diff.y) < .02)
    {
      S.closeGripper("R_gripper", .05, .3);
      gripping = true;
    }

    if (gripping && S.getGripperIsGrasping("R_gripper"))
    {
      cout << "GRASPED!" << endl;
      break;
    }

    //send no controls to the simulation
    S.step(vel, tau, S._velocity);
  }
  rai::wait();
}

//===========================================================================

int main(int argc, char **argv)
{
  rai::initCmdLine(argc, argv);

  grasp_the_hopping_ball();

  return 0;
}
