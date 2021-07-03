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
bool ReadyThrow = false;
double mass = 0.2;
double g = 9.8;

//set the intrinsic camera parameters
double f = 0.895 * 360; //focal length is needed in pixels (height!), not meters!
arr Fxypxy = {f, f, 320., 180.};

arr vel = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//===========================================================================
//dart board is red, convert the dart board image to binary
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

//darts are blue, convert the image of darts into binary
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
  arr targetgravity = target;
  targetgravity.elem(2) += g_offset;
  v = (targetgravity - RealWorld["R_gripperCenter"]->getPosition()) / t;
  visualize_velocity(v, t);
}

void move_robotA(arr const center, double const radius, double const angular_velocity)
{
  cout<<"move robot A in circle"<<endl;
  mtx.lock();
  int step = 10;
  KOMO komo1;
  komo1.setModel(C, true);
  //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo1.setTiming(1., step, step*tau, 1);      //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
  komo1.add_qControlObjective({}, 1, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)
  //komo1.addObjective({}, FS_position, {"L_gripper"}, OT_eq, {1e3}, {0.3782, -0.0187, 0.8});//max z=1.2
  komo1.addObjective({}, FS_position, {"L_gripper"}, OT_eq, {1e3}, center);
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
  float t = 0;
  arr target = center;
  rai::Frame *target_frame = C.addFrame("target_frame");
  target_frame->setShape(rai::ST_marker, {0.1, 0.1, 0.1});
  while (true)
  {
    if (StartMotionGenerator)
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

      komo.addObjective({}, FS_positionDiff, {"L_gripper", "target_frame"}, OT_sos, {1e2}, {0, 0, 0});
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
      if (!(int(t * 100) % 300))
      {
        ReadyThrow = true;
        StartMotionGenerator = false;
      }
    }
  }
}

void move_robotB()
{
  cout<<"move robotB throw dart"<<endl;
  arr velB;
  Value diff;
  //move to the detect pose

  // define a detection frame
  rai::Frame *detection_frame = C.addFrame("detection_frame");
  detection_frame->setShape(rai::ST_marker, {0.1, 0.1, 0.1});
  detection_frame->setPosition({-1.7, 0, 1.2});
  rai::Frame *obj = C.addFrame("object");
  obj->setColor({1., 1., 0}); //set the color of one objet to red!
  obj->setShape(rai::ST_sphere, {.05});

  // define a detection frame
  rai::Frame *lift_frame = C.addFrame("lift_frame");
  lift_frame->setShape(rai::ST_marker, {0.1, 0.1, 0.1});
  lift_frame->setPosition({-1.5, 0.2, 1.2});

  // define motion frame to visualize the fly path of dart  
  rai::Frame *motion_frame = C.addFrame("motion_frame");
  motion_frame->setShape(rai::ST_marker, {0.3, 0.3, 0.3});

  while (true)
  {
    rai::wait();
    int step = 50;
    //===========================================================================
    cout<<"move to detection frame"<<endl;
    KOMO komo1;
    komo1.setModel(C, true);
    komo1.setTiming(1., step, step * tau, 1); 
    komo1.add_qControlObjective({}, 1, 1.);   
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
    //===========================================================================
    cout<<"detect dart"<<endl;
    bool gripping = false;
    bool grasped = false;
    bool seecylinder = false;

    //detect the cylinder
    byteA _rgb;
    floatA _depth;
    rai::Frame *cameraFrame = C["camera"];
    rai::Transformation camera_pose = cameraFrame->get_X(); //this is relative to "/base_link"
    //get the image from camera
    S.getImageAndDepth(_rgb, _depth);

/*  visualize point cloud
    arr points;
    depthData2pointCloud(points, _depth, Fxypxy);    
    //transform to world frame
    camera_pose.applyOnPointArray(points);
    rai::Frame *worldFrame = C["world"];
    worldFrame->setPointCloud(points, _rgb);
    V.recopyMeshes(C); //update the model display!
    V.setConfiguration(C);
*/

    cv::Mat rgb = CV(_rgb);
    cv::Mat depth = CV(_depth);
    cv::Mat binary = filter_blue_pixel(rgb);
    //get contour
    std::vector<std::vector<cv::Point>> contours = get_contour(binary);
    //calculate the 2D center
    if (contours.size() != 0)
    {
      seecylinder = true;
      //localize the dart
      std::vector<cv::Point> contour = contours[0];
      //cv::Mat rgb_contour = draw_contour(rgb, contours);
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

      //===========================================================================
      cout<<"move to pregrasp pose"<<endl;
      //move to the pre_grasp position
      KOMO komo2;
      komo2.setModel(C, true);
      komo2.setTiming(1., step, step * tau, 1);
      komo2.add_qControlObjective({}, 1, 1.);   
      komo2.addObjective({1.0, 1.0}, FS_positionDiff, {"R_gripper", "object"}, OT_sos, {1e3}, {0, -0.1, 0});
      komo2.addObjective({1.0, 1.0}, FS_scalarProductYZ, {"R_gripper", "world"}, OT_eq, {1e2}, {1});
      komo2.addObjective({1.0, 1.0}, FS_scalarProductYZ, {"world", "R_gripper"}, OT_eq, {1e2}, {-1});
      komo2.optimize();
      for (int i = 0; i < step; i++)
      {
        q = komo2.getConfiguration_qAll(i);
        S.step();
        rai::wait(tau);
        S.setMoveTo(q, tau);
        C.setJointState(S.get_q()); 
        V.setConfiguration(C);
      }

      //===========================================================================
      cout<<"move to dart"<<endl;
      for (uint t = 0; t < 1000; t++)
      {
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

        komo3.addObjective({}, FS_positionDiff, {"object", "R_gripperCenter"}, OT_sos, {1e3}, {0, 0, 0.003});
        //komo.addObjective({}, FS_distance, {"R_gripperCenter", "object"}, OT_sos, {1e4});
        komo3.addObjective({}, FS_scalarProductYZ, {"R_gripperCenter", "world"}, OT_eq, {1e3}, {1});
        komo3.addObjective({}, FS_distance, {"R_finger1", "object"}, OT_ineq, {1e3}, {-0.01});
        komo3.addObjective({}, FS_distance, {"R_finger2", "object"}, OT_ineq, {1e3}, {-0.01});
        komo3.optimize();

        //get the joint vector from the optimized configuration
        arr q_target = komo3.getConfiguration_qOrg(komo3.T - 1);
        velB = q_target - S.get_q();
        diff = C.feature(FS_positionDiff, {"R_gripperCenter", "object"})->eval(C);
        if (!gripping && length(diff.y) < .05 && seecylinder)
        {
          for (int i = 7; i < 14; i++)
          {
            vel.elem(i) = 0;
          }
          ////generate small velocity
          cout << "close gripper" << endl;
          S.closeGripper("R_gripper", .05, .5);
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
          if(abs(velB.elem(i))>6)
          {
            velB.elem(i)=0;
            i++;
          }
          vel.elem(i) = double(10) * velB.elem(i);
        }
        //send no controls to the simulation
        S.step(vel, tau, S._velocity);
      }

      //===========================================================================
      cout<<"lift dart"<<endl;
      if (grasped)
      {

        KOMO komo4; //create a solver
        komo4.setModel(C, true);
        komo4.setTiming(2., step, tau * step, 2); 
        komo4.add_qControlObjective({}, 2, 1.);   
        komo4.addObjective({1.}, FS_positionDiff, {"R_gripperCenter", "object"}, OT_sos, {1e2}, {0, 0, 1.2});
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
      
      //===========================================================================
      cout<<"follow dart board"<<endl;
      //start move robotA
      StartMotionGenerator = true;
      ReadyThrow = false;
      //detect dartboard
      while (!ReadyThrow)
      {
        S.getImageAndDepth(_rgb, _depth);
        camera_pose = cameraFrame->get_X();
        rgb = CV(_rgb);
        depth = CV(_depth);
        //convert to binary image
        binary = filter_red_pixel(rgb);
        //get contour
        contours = get_contour(binary);
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
          arr obj_position = {X, -Y, -Z};
          //transform to world frame
          if (!camera_pose.isZero())
            camera_pose.applyOnPoint(obj_position);
          obj->setPosition(obj_position);
        }
        else
        {
          cout<<"finish task"<<endl;
          return;
        }
        C.setJointState(S.get_q()); //set your working config into the optimized state
        V.setConfiguration(C);
        //rai::wait(tau);

        //===========================================================================
        cout<<"move to pre throw pose"<<endl;
        //move to the pose in front of the object and higher than it because of gravity
        KOMO komo5; //create a solver
        komo5.setModel(C, true);
        komo5.setTiming(1., 1, tau, 1);        
        komo5.add_qControlObjective({}, 1, 1.); 
        komo5.addObjective({1.}, FS_positionDiff, {"R_gripperCenter", "object"}, OT_sos, {1e2}, {-1.5, 0, 0});
        komo5.addObjective({1.}, FS_scalarProductZZ, {"R_gripperCenter", "world"}, OT_eq, {1e3}, {1});
        komo5.addObjective({1.}, FS_scalarProductYX, {"R_gripperCenter", "world"}, OT_eq, {1e3}, {1});
        komo5.optimize();
        arr q_desired = komo5.getConfiguration_qAll(komo5.T - 1);
        arr velB = q_desired - S.get_q();
        //set the velocity of robot A
        for (int i = 7; i < 14; i++)
        {
          if(abs(velB.elem(i))>6)
          {
            velB.elem(i)=0;
            i++;
          }
          vel.elem(i) = double(10) * velB.elem(i);
        }
        //move the robot
        S.step(vel, tau, S._velocity);
      }

      //===========================================================================
      cout<<"throw dart"<<endl;
      //set the flying time of a dart
      float t_fly = 0.3;
      //move to the pose in front of the object and higher than it because of gravity
      KOMO komo6; //create a solver
      komo6.setModel(C, true);
      komo6.setTiming(1., step, tau * step, 1); 
      komo6.add_qControlObjective({}, 1, 1.);  
      float z = g * t_fly * t_fly / 2;
      komo6.addObjective({1.}, FS_positionDiff, {"R_gripperCenter", "object"}, OT_sos, {1e2}, {-1.5, 0, z});
      komo6.addObjective({1.}, FS_scalarProductZZ, {"R_gripperCenter", "world"}, OT_eq, {1e3}, {1});
      komo6.addObjective({1.}, FS_scalarProductYX, {"R_gripperCenter", "world"}, OT_eq, {1e3}, {1});
      komo6.optimize();
      //lift the gripper to the initial pose of the ball
      for (int t = 0; t < step; t++)
      {
        rai::wait(tau);
        q = komo6.getConfiguration_qAll(t);
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
      arr motion_pose = release_pose;
      //move backwards firstly
      for (int i = 0; i < 10; i++)
      {
        //compute pose after time tau regarding the velocity vel_cart
        motion_pose -= vel_cart * tau;
        //move to the pose in front of the object and higher than it because of gravity
        KOMO komo7; //create a solver
        komo7.setModel(C, true);
        komo7.setTiming(1., 1, tau, 1);         
        komo7.add_qControlObjective({}, 1, 1.); 
        komo7.addObjective({1.}, FS_position, {"R_gripperCenter"}, OT_sos, {1e3}, motion_pose);
        komo7.addObjective({1.}, FS_scalarProductZZ, {"R_gripperCenter", "world"}, OT_eq, {1e3}, {1});
        komo7.addObjective({1.}, FS_scalarProductYX, {"R_gripperCenter", "world"}, OT_eq, {1e3}, {1});
        komo7.optimize();
        arr vel_q = (komo7.getConfiguration_qAll(0) - S.get_q()) / tau;
        for (int j = 7; j < 14; j++)
        {
          vel.elem(j) = vel_q.elem(j);
        }
        S.step(vel, tau, S._velocity);
        C.setJointState(S.get_q());
        V.setConfiguration(C);
      }

      //move forwards
      for (int i = 0; i < 50; i++)
      {
        //compute jacobian
        motion_pose += vel_cart * tau;
        KOMO komo7; //create a solver
        komo7.setModel(C, true);
        komo7.setTiming(1., 1, tau, 1);         
        komo7.add_qControlObjective({}, 1, 1.); 
        komo7.addObjective({1.}, FS_position, {"R_gripperCenter"}, OT_sos, {1e3}, motion_pose);
        komo7.addObjective({1.}, FS_scalarProductZZ, {"R_gripperCenter", "world"}, OT_eq, {1e3}, {1});
        komo7.addObjective({1.}, FS_scalarProductYX, {"R_gripperCenter", "world"}, OT_eq, {1e3}, {1});
        komo7.optimize();
        arr vel_q = (komo7.getConfiguration_qAll(0) - S.get_q()) / tau;
        for (int j = 7; j < 14; j++)
        {
          vel.elem(j) = vel_q.elem(j);
        }
        S.step(vel, tau, S._velocity);
        C.setJointState(S.get_q());
        V.setConfiguration(C);
        cout << "length" << length(release_pose - RealWorld["R_gripperCenter"]->getPosition()) << std::endl;
        if (length(release_pose - RealWorld["R_gripperCenter"]->getPosition()) < 0.01)
        {
          cout << "release" << std::endl;
          S.openGripper("R_gripper", 0.15, 5);
          i = 48;
        }
      }
      //set velocity to zero
      for (int i = 7; i < 14; i++)
      {
        vel.elem(i) = 0;
      }

      for (int i = 0; i < 20; i++)
      {
        rai::wait(tau);
        rai::wait(tau);
        S.step();
        /* visualize the fly path
        arr po1 = RealWorld["dart1"]->getPosition();
        position = RealWorld["dart1"]->getPosition();
        motion_frame->setPosition(position);
        C.setJointState(S.get_q());
        V.setConfiguration(C);
        cout << "dart_vel" << (position - po1) / tau << endl;
        cout << "dart_vel_should " << vel_cart << endl;
        rai::wait();
        */
      }
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
  arr const center = {0.3782, -0.0187, 0.85};
  double const radius = 0.05;
  double const angular_velocity = 4 * M_PI / 2;
  //thread for the left robot, move to the generated motion
  std::thread robotA(move_robotA, center, radius, angular_velocity);
  std::thread robotB(move_robotB);
  //thread for the right robot, detect the dart lift the dart, follow the dart and throw the dart
  robotA.join();
  robotB.join();

  return 0;
}
