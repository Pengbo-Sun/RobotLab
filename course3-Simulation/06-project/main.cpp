#include <Perception/opencv.h> //always include this first! OpenCV headers define stupid macros
#include <Geo/depth2PointCloud.h>

#include <Kin/feature.h>
#include <Kin/frame.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>
#include <KOMO/komo.h>

#include <thread>     // std::thread
#include <mutex>      // std::mutex
#include <functional> //std::reference

rai::Configuration RealWorld("../../scenarios/project.g");
rai::Simulation S(RealWorld, S._bullet, 1);
rai::Configuration C;
rai::ConfigurationViewer V;
const double tau = 0.01;
std::mutex mtx;
bool StartMotionGenerator = false;
double mass = 0.2;
double g = 9.8;

arr vel = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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

cv::Mat filter_blue_pixel(const cv::Mat src)
{

  cv::Mat output(src.rows, src.cols, CV_8UC1);
  const std::vector<float> blue_threshold = {50, 50, 200};
  for (int i = 0; i < src.rows; i++)
  {
    for (int j = 0; j < src.cols; j++)
    {
      //BGR
      double red_v = (double)src.at<cv::Vec3b>(i, j)[0];
      double green_v = (double)src.at<cv::Vec3b>(i, j)[1];
      double blue_v = (double)src.at<cv::Vec3b>(i, j)[2];
      //check if the pixel is red
      if (red_v < blue_threshold[0] && green_v < blue_threshold[1] && blue_v >= blue_threshold[2])
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

void set_robotA_joint(arr &q)
{
  arr q_current = S.get_q();
  for (int i = 0; i < 7; i++)
    q.elem(i) = q_current.elem(i);
}

void visualize_velocity(arr const &v, double const &t)
{
  for (double i = 0; i <= t; i += 0.02)
  {
    std::string frame_name = "velocity" + std::to_string(i);
    arr tcp = C["R_gripperCenter"]->getPosition();
    arr position = v * i + tcp;

    position.elem(2) -= (g * i * i / 2);

    rai::Frame *velocity_frame = C.addFrame(frame_name.c_str());
    velocity_frame->setColor({1., 1., 0}); //set the color of one objet to red!
    velocity_frame->setShape(rai::ST_sphere, {.01});
    velocity_frame->setPosition(position);
  }
  V.setConfiguration(C);
}

void set_velocity(arr const &target, arr &v, double t)
{
  double g_offset = g * t * t / 2;
  cout << "g_offset " << g_offset << endl;
  arr targetgravity = target;
  targetgravity.elem(2) += g_offset;
  v = (targetgravity - RealWorld["R_gripperCenter"]->getPosition()) / t;
  cout << "target position" << targetgravity << endl;
  cout << "R_gippercenter position" << RealWorld["R_gripperCenter"]->getPosition() << endl;
  visualize_velocity(v, t);
  cout << "vel " << v << endl;
}

void generate_circular_motion(arr const center, double const radius, double const angular_velocity)
{
  mtx.lock();
  int step = 10;
  KOMO komo1;
  komo1.setModel(C, true);
  //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo1.setTiming(1., step, tau, 1);      //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
  komo1.add_qControlObjective({}, 1, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)
  //komo1.addObjective({}, FS_position, {"L_gripper"}, OT_eq, {1e3}, {0.3782, -0.0187, 0.8});//max z=1.2
  komo1.addObjective({}, FS_position, {"L_gripper"}, OT_eq, {1e3}, {0.3782, -0.0187, 1});
  komo1.addObjective({}, FS_scalarProductXZ, {"world", "boardred"}, OT_eq, {1e3}, {1});
  komo1.optimize();
  for (int i = 0; i < step; i++)
  {
    arr q = komo1.getConfiguration_qAll(i);
    S.step({}, tau);
    S.setMoveTo(q, tau);
    C.setJointState(S.get_q()); //set your working config into the optimized state
    V.setConfiguration(C);
    rai::wait(tau);
  }
  mtx.unlock();
  while (!StartMotionGenerator)
  {
    ;
  }
  float t = 0;
  arr target = center;
  rai::Frame *target_frame = C.addFrame("target_frame");
  target_frame->setShape(rai::ST_marker, {0.1, 0.1, 0.1});
  while (true)
  {
    //compute the pose on the circle in each step in y-z plane
    target(1) = center(1) + radius * cos(angular_velocity * t);
    target(2) = center(2) + radius * sin(angular_velocity * t);
    target_frame->setPose(target);

    //create a solver
    KOMO komo;
    komo.setModel(C, true);
    //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
    komo.setTiming(1., 1, tau, 1);         //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
    komo.add_qControlObjective({}, 1, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

    komo.addObjective({}, FS_positionDiff, {"boardred", "target_frame"}, OT_sos, {1e2}, {0, 0, 0});
    komo.addObjective({}, FS_scalarProductXZ, {"world", "boardred"}, OT_eq, {1e3}, {1});
    komo.optimize();
    arr q_desired = komo.getConfiguration_qAll(komo.T - 1);
    arr velB = q_desired - S.get_q();
    //set the velocity of robot B
    for (int i = 0; i < 7; i++)
    {
      vel.elem(i) = double(10) * velB.elem(i);
    }
    //move the robot
    S.step(vel, tau, S._velocity);
    //rai::wait(tau);
    C.setJointState(S.get_q());
    V.setConfiguration(C);
    t += tau;
    cout << t * 100 << endl;
    if (!(int(t * 100) % 400))
    {
      rai::wait(double(2));
    }
  }
}

void move()
{
  cout << RealWorld["L_gripper"]->getPosition() << endl;
  cout << "move" << endl;
  arr velA;
  Value diff;
  //move to the detect pose

  // define a detection frame
  rai::Frame *detection_frame = C.addFrame("detection_frame");
  detection_frame->setShape(rai::ST_marker, {0.1, 0.1, 0.1});
  detection_frame->setPosition({-1.6, 0, 1.2});
  rai::Frame *obj = C.addFrame("object");
  obj->setColor({1., 1., 0}); //set the color of one objet to red!
  obj->setShape(rai::ST_sphere, {.05});

  // define a detection frame
  rai::Frame *lift_frame = C.addFrame("lift_frame");
  lift_frame->setShape(rai::ST_marker, {0.1, 0.1, 0.1});
  lift_frame->setPosition({-1.5, 0.2, 1.2});

  rai::Frame *motion_frame = C.addFrame("motion_frame");
  motion_frame->setShape(rai::ST_marker, {0.3, 0.3, 0.3});
  for (int j = 0; j < 3; j++)
  {

    rai::wait();
    KOMO komo1;
    komo1.setModel(C, true);
    //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
    int step = 50;
    komo1.setTiming(1., step, step * tau, 2); //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
    komo1.add_qControlObjective({}, 2, 1.);   //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

    komo1.addObjective({1.0, 1.0}, FS_positionDiff, {"R_gripper", "detection_frame"}, OT_sos, {1e3}, {0, 0, 0});
    komo1.addObjective({1.0, 1.0}, FS_scalarProductZZ, {"R_gripper", "world"}, OT_eq, {1e2}, {1});
    komo1.addObjective({1.0, 1.0}, FS_scalarProductXX, {"R_gripper", "world"}, OT_eq, {1e2}, {1});
    komo1.optimize();

    arr q;
    for (int i = 0; i < step; i++)
    {
      q = komo1.getConfiguration_qAll(i);
      S.step({}, tau);
      S.setMoveTo(q, tau);
      C.setJointState(S.get_q()); //set your working config into the optimized state
      V.setConfiguration(C);
      rai::wait(tau);
    }
    rai::wait();

    bool gripping = false;
    bool grasped = false;
    bool seecylinder = false;

    //detect the cylinder
    //set the intrinsic camera parameters
    double f = 0.895;
    f *= 360.; //focal length is needed in pixels (height!), not meters!
    arr Fxypxy = {f, f, 320., 180.};

    byteA _rgb;
    floatA _depth;
    arr points;
    //detect obj

    rai::Frame *cameraFrame = C["camera"];
    S.getImageAndDepth(_rgb, _depth); //we don't need images with 100Hz, rendering is slow
                                      // NEW
    depthData2pointCloud(points, _depth, Fxypxy);
    rai::Transformation camera_pose = cameraFrame->get_X(); //this is relative to "/base_link"
    //transform to world frame
    camera_pose.applyOnPointArray(points);
    rai::Frame *worldFrame = C["world"];
    worldFrame->setPointCloud(points, _rgb);
    V.recopyMeshes(C); //update the model display!
    V.setConfiguration(C);

    cv::Mat rgb = CV(_rgb);
    cv::Mat depth = CV(_depth);

    //--<< perception pipeline
    cv::Mat binary = filter_blue_pixel(rgb);
    //get contour
    std::vector<std::vector<cv::Point>> contours = get_contour(binary);
    //calculate the 2D center
    if (contours.size() != 0)
    {
      seecylinder = true;
      std::vector<cv::Point> contour = contours[0];
      cv::Mat rgb_contour = draw_contour(rgb, contours);
      cv::imshow("contour", rgb_contour);
      int k = cv::waitKey(0);
      cv::Moments mu = cv::moments(contour, true);
      double Z = get_meandepth(depth, binary);
      //create a sphere in the mode world
      double x = mu.m10 / mu.m00;
      double y = mu.m01 / mu.m00;
      double X = (x - 320) * Z / f;
      double Y = (y - 180) * Z / f;
      arr obj_position = {X, -Y, -Z};
      cout << "obj_position " << obj_position << endl;
      rai::wait();
      //transform to world frame
      if (!camera_pose.isZero())
        camera_pose.applyOnPoint(obj_position);
      obj->setPosition(obj_position);
    }

    //move to the pre_grasp position
    KOMO komo2;
    komo2.setModel(C, true);
    //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
    komo2.setTiming(1., step, step * tau, 1); //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
    komo2.add_qControlObjective({}, 1, 1.);   //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

    komo2.addObjective({1.0, 1.0}, FS_positionDiff, {"R_gripper", "object"}, OT_sos, {1e3}, {0, -0.3, 0});
    komo2.addObjective({1.0, 1.0}, FS_scalarProductYZ, {"R_gripper", "world"}, OT_eq, {1e2}, {1});
    komo2.addObjective({1.0, 1.0}, FS_scalarProductYZ, {"world", "R_gripper"}, OT_eq, {1e2}, {-1});
    komo2.optimize();
    for (int i = 0; i < step; i++)
    {
      q = komo2.getConfiguration_qAll(i);
      S.step({}, tau);
      S.setMoveTo(q, tau);
      C.setJointState(S.get_q()); //set your working config into the optimized state
      V.setConfiguration(C);
    }

    for (uint t = 0; t < 1000; t++)
    {
      cout << "t: " << t << endl;
      //grab sensor readings from the simulation

      q = S.get_q();
      C.setJointState(q); //set your robot model to match the real q

      V.setConfiguration(C);
      //some good old fashioned IK
      //komo methode

      KOMO komo3; //create a solver
      komo3.setModel(C, true);
      //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
      komo3.setTiming(1., 1, tau, 1);         //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
      komo3.add_qControlObjective({}, 1, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

      komo3.addObjective({}, FS_positionDiff, {"object", "R_gripperCenter"}, OT_sos, {1e3}, {0, 0, 0.01});
      //komo.addObjective({}, FS_distance, {"R_gripperCenter", "object"}, OT_sos, {1e4});
      komo3.addObjective({}, FS_scalarProductYZ, {"R_gripperCenter", "world"}, OT_eq, {1e3}, {1});
      //komo3.addObjective({}, FS_scalarProductXZ, {"world", "R_gripperCenter"}, OT_eq, {1e3}, {-1});
      komo3.addObjective({}, FS_distance, {"R_finger1", "object"}, OT_ineq, {1e3}, {-0.01});
      komo3.addObjective({}, FS_distance, {"R_finger2", "object"}, OT_ineq, {1e3}, {-0.01});
      komo3.optimize();

      //get the joint vector from the optimized configuration
      arr q_target = komo3.getConfiguration_qOrg(komo3.T - 1);
      velA = q_target - S.get_q();
      diff = C.feature(FS_positionDiff, {"R_gripperCenter", "object"})->eval(C);
      if (!gripping && length(diff.y) < .05 && seecylinder)
      {
        for (int i = 7; i < 14; i++)
        {
          vel.elem(i) = 0;
        }
        rai::wait();
        ////generate small velocity
        cout << "close gripper" << endl;
        S.closeGripper("R_gripper", .03, .5);
        gripping = true;
        while (S.getGripperIsGrasping("R_gripper"))
        {
          S.step();
        }
      }

      if (gripping && S.getGripperIsGrasping("R_gripper"))
      {
        grasped = true;
        cout << "GRASPED!" << endl;
        break;
      }

      //set the velocity of robot B
      for (int i = 7; i < 14; i++)
      {
        vel.elem(i) = double(10) * velA.elem(i);
      }
      //send no controls to the simulation
      S.step(vel, tau, S._velocity);
    }
    rai::wait();

    if (grasped)
    {

      KOMO komo4; //create a solver
      komo4.setModel(C, true);
      //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
      komo4.setTiming(2., step, tau * step, 2); //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
      komo4.add_qControlObjective({}, 2, 1.);   //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

      komo4.addObjective({1.}, FS_positionDiff, {"R_gripperCenter", "object"}, OT_sos, {1e2}, {0, 0, 1});
      komo4.addObjective({1.}, FS_scalarProductYZ, {"R_gripperCenter", "world"}, OT_eq, {1e2}, {1});
      komo4.addObjective({2.}, FS_scalarProductZZ, {"R_gripperCenter", "world"}, OT_eq, {1e3}, {1});
      komo4.addObjective({2.}, FS_scalarProductYX, {"R_gripperCenter", "world"}, OT_eq, {1e3}, {1});
      komo4.addObjective({2.}, FS_positionDiff, {"R_gripperCenter", "lift_frame"}, OT_sos, {1e2});

      komo4.optimize();
      //lift the gripper to the initial pose of the ball
      for (int t = 0; t < 2 * step; t++)
      {
        rai::wait(tau);
        q = komo4.getConfiguration_qAll(t);
        S.step();
        S.setMoveTo(q, tau);
        C.setJointState(S.get_q()); //set your robot model to match the real q
        V.setConfiguration(C);
      }
    }
    //start move robotA
    //StartMotionGenerator = true;

    //detect dartboard

    S.getImageAndDepth(_rgb, _depth); //we don't need images with 100Hz, rendering is slow
                                      // NEW
    depthData2pointCloud(points, _depth, Fxypxy);
    camera_pose = cameraFrame->get_X(); //this is relative to "/base_link"
    //transform to world frame
    camera_pose.applyOnPointArray(points);
    worldFrame->setPointCloud(points, _rgb);
    V.recopyMeshes(C); //update the model display!
    V.setConfiguration(C);

    rgb = CV(_rgb);
    depth = CV(_depth);

    //--<< perception pipeline
    binary = filter_red_pixel(rgb);
    //get contour
    contours = get_contour(binary);
    bool seeboard = false;
    //calculate the 2D center
    if (contours.size() != 0)
    {
      seeboard = true;
      std::vector<cv::Point> contour = contours[0];
      cv::Mat rgb_contour = draw_contour(rgb, contours);
      cv::imshow("board", rgb_contour);
      int k = cv::waitKey(0);
      cv::Moments mu = cv::moments(contour, true);
      double Z = get_meandepth(depth, binary);
      //create a sphere in the mode world
      double x = mu.m10 / mu.m00;
      double y = mu.m01 / mu.m00;
      double X = (x - 320) * Z / f;
      double Y = (y - 180) * Z / f;
      arr obj_position = {X, -Y, -Z};
      //transform to world frame
      if (!camera_pose.isZero())
        camera_pose.applyOnPoint(obj_position);
      obj->setPosition(obj_position);
    }
    C.setJointState(S.get_q()); //set your working config into the optimized state
    V.setConfiguration(C);

    float t_fly = 0.3;
    //move to the pose in front of the object and higher than it because of gravity
    KOMO komo5; //create a solver
    komo5.setModel(C, true);
    //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
    komo5.setTiming(1., step, tau * step, 1); //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
    komo5.add_qControlObjective({}, 1, 1.);   //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)
    float z = g * t_fly * t_fly / 2;
    cout << "z " << z << endl;
    komo5.addObjective({1.}, FS_positionDiff, {"R_gripperCenter", "object"}, OT_sos, {1e2}, {-1.5, 0, z});
    komo5.addObjective({1.}, FS_scalarProductZZ, {"R_gripperCenter", "world"}, OT_eq, {1e3}, {1});
    komo5.addObjective({1.}, FS_scalarProductYX, {"R_gripperCenter", "world"}, OT_eq, {1e3}, {1});
    komo5.optimize();
    //lift the gripper to the initial pose of the ball
    for (int t = 0; t < step; t++)
    {
      rai::wait(tau);
      q = komo5.getConfiguration_qAll(t);
      S.step();
      S.setMoveTo(q, tau);
      C.setJointState(S.get_q()); //set your robot model to match the real q
      V.setConfiguration(C);
    }
    //estimate the velocity
    arr vel_cart;
    set_velocity(obj->getPosition(), vel_cart, t_fly);
    arr position = RealWorld["dart1"]->getPosition();
    motion_frame->setPosition(position);
    V.setConfiguration(C);
    arr release_pose = RealWorld["R_gripperCenter"]->getPosition();
    //compute jacobian

    //auto diff = C.feature(FS_positionDiff, {"R_gripperCenter", "object"})->eval(C);
    //  auto vecX = C.feature(FS_scalarProductXZ, {"R_gripperCenter", "world"})->eval(C);

    //stack them
    //arr y, J;
    //
    //y.append(1e0 * diff.y); //multiply, to make faster
    //J.append(1e0 * diff.J);
    //
    //y.append(vecX.y - arr{0.}); //subtract target, here scalarProduct=0
    //J.append(vecX.J);
    //
    //arr vel = 2. * pseudoInverse(J, NoArr, 1e-2) * (-y);
    diff = C.feature(FS_position, {"R_gripperCenter"})->eval(C);
    cout << "J1 " << diff.J << endl;
    cout << "J2 " << C.feature(FS_positionDiff, {"R_gripperCenter", "R_panda_link0"})->eval(C).J << endl;
    vel_cart.elem(0) += 4;
    //transform it to joint velocity
    arr vel_q = pseudoInverse(diff.J, NoArr) * vel_cart;
    //move backwars firstly
    for (int i = 7; i < 14; i++)
    {
      vel.elem(i) = -vel_q.elem(i);
    }

    for (int i = 0; i < 3; i++)
    {
      S.step(vel, tau, S._velocity);
      position = RealWorld["dart1"]->getPosition();
      motion_frame->setPosition(position);
      rai::wait();
    }
    C.setJointState(S.get_q());
    for (int i = 7; i < 14; i++)
    {
      vel.elem(i) = -vel.elem(i);
    }
    for (int i = 0; i < 10; i++)
    {
      cout << "length" << length(release_pose - RealWorld["R_gripperCenter"]->getPosition()) << std::endl;
      if (length(release_pose - RealWorld["R_gripperCenter"]->getPosition()) < 0.01)
      {
        cout << "release" << std::endl;
        S.openGripper("R_gripper", 0.15, 3);
        S.step(vel, tau, S._velocity);
        //for (int i = 7; i < 14; i++)
        //{
        //  vel.elem(i) = 0;
        //}
        break;
      }
      arr po1_q = S.get_q();
      arr po1_tcp = RealWorld["R_gripperCenter"]->getPosition();
      S.step(vel, tau, S._velocity);
      arr po2_tcp = RealWorld["R_gripperCenter"]->getPosition();
      arr po2_q = S.get_q();
      cout << "joint_vel" << (po2_q - po1_q) / tau << endl;
      cout << "joint_vel_should" << vel_q << endl;
      cout << "tcp_vel" << (po2_tcp - po1_tcp) / tau << endl;
      cout << "tcp_vel_should" << vel_cart << endl;
      position = RealWorld["dart1"]->getPosition();
      motion_frame->setPosition(position);
      rai::wait();
    }
    C.setJointState(S.get_q());
    V.setConfiguration(C);

    for (int i = 7; i < 14; i++)
    {
      vel.elem(i) = 0;
    }

    for (int i = 0; i < 20; i++)
    {
      arr po1 = RealWorld["dart1"]->getPosition();

      S.step(vel, tau, S._velocity);
      position = RealWorld["dart1"]->getPosition();
      motion_frame->setPosition(position);
      C.setJointState(S.get_q());
      V.setConfiguration(C);
      cout << "dart_vel" << (position - po1) / tau << endl;
      cout << "dart_vel_should " << vel_cart << endl;
      rai::wait();
    }
  }
  //start visual servoing
}

int main(int argc, char **argv)
{
  rai::initCmdLine(argc, argv);
  //load the world

  //setup simulation world

  S.cameraview().addSensor("camera");

  //setup model world
  C.addFile("../../scenarios/project.g");
  V.setConfiguration(C, "model");

  //motion generation for RobotA
  arr const center = {1.7, 0, 1.5, 0, 0, 1, 0};
  double const radius = 0.2;
  double const angular_velocity = 2 * M_PI / 5;
  //thread for the left robot, move to the generated motion
  std::thread l_robot(generate_circular_motion, center, radius, angular_velocity);
  std::thread r_robot(move);
  //thread for the right robot, detect the dart lift the dart, follow the dart and throw the dart
  l_robot.join();
  r_robot.join();

  return 0;
}
