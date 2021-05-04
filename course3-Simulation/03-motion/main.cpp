#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/feature.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>
#include <KOMO/komo.h>

//===========================================================================

void using_KOMO_for_IK()
{
  //we don't need to instantiate the real world/simluation here

  //-- MODEL WORLD configuration, this is the data structure on which you represent
  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");
  arr q0 = C.getJointState();

  rai::Frame *obj = C.addFrame("object");
  obj->setPosition({1., 0., 1.5});
  obj->setQuaternion({1., 0., 1., 0});
  obj->setShape(rai::ST_marker, {0.1, 0.1, 0.1});
  //obj->setShape(rai::ST_capsule, {.2, .02});
  //obj->setColor({1., .0, 1.});

  //pregrasp pose
  rai::Frame *pre_grasp = C.addFrame("pre_grasp");
  pre_grasp->setPosition({1., -0.1, 1.5});
  pre_grasp->setQuaternion({1., 0., 1., 0});

  //-- using the viewer, you can view configurations or paths
  C.watch(true, "model world start state");

  //-- optimize a single configuration using KOMO

  KOMO komo;              //create a solver
  komo.setModel(C, true); //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  int step = 10;
  komo.setTiming(2, step, 5., 1); //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)

  //now add objectives!

  //the default control objective:
  komo.add_qControlObjective({}, 1, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

  //task objectives:
  //komo.addObjective({}, FS_positionDiff, {"R_gripperCenter", "object"}, OT_eq, {1e2});

  // 1. there is no difference between “the object reaching for the right gripper” and the other way around
  // komo.addObjective({}, FS_positionDiff, {"object", "R_gripperCenter"}, OT_eq, {1e2});
  //test the left gripper reaching for the right gripper
  //komo.addObjective({}, FS_positionRel, {"R_gripperCenter", "L_gripperCenter"}, OT_eq, {1e2});
  //whats the difference between FS_positionDiff and FS_positionRel?

  // 2. orientation
  //komo.addObjective({}, FS_quaternionDiff, {"R_gripperCenter", "object"}, OT_eq, {1e2});

  // 3. fixed orientation
  //komo.addObjective({}, FS_scalarProductXY, {"R_gripperCenter", "object"}, OT_eq, {1e2}, {0.001});
  //komo.addObjective({}, FS_scalarProductXZ, {"R_gripperCenter", "object"}, OT_eq, {1e2}, {0.001});
  //optimize the scalarproduct between x of gripper and y of object to zero, x is prependicular to y
  //it is more flexible than quanternion

  //4.grisping
  //pre_grasp
  komo.addObjective({1.0, 1.0}, FS_positionRel, {"R_gripperCenter", "object"}, OT_eq, {1e2}, {0.2, 0, 0});
  komo.addObjective({1.0, 1.0}, FS_scalarProductXZ, {"object", "R_gripperCenter"}, OT_eq, {1e2}, {1});
  komo.addObjective({1.0, 1.0}, FS_scalarProductXY, {"R_gripperCenter", "object"}, OT_eq, {1e2}, {-1});
  //initialize the solve
  komo.optimize();
  arr q_pregrasp;
  for (int i = 0; i < step; i++)
  {
    q_pregrasp = komo.getConfiguration_qOrg(i);
    cout << i << ": " << q_pregrasp << std::endl;
    C.setJointState(q_pregrasp); //set your working config into the optimized state
    C.watch(true, "pregrasp");
  }

  //get the joint vector from the optimized configuration
  C.watch(true, "pregrasp"); //display it
  //final grasp
  komo.reset();
  arr q_finalgrasp;
  komo.addObjective({2, 2}, FS_positionRel, {"R_gripperCenter", "object"}, OT_eq, {1e2});
  komo.optimize();
  for (int i = step; i < 2 * step; i++)
  {
    q_finalgrasp = komo.getConfiguration_qOrg(i);
    cout << i << ": " << q_finalgrasp << std::endl;
    C.setJointState(q_finalgrasp); //set your working config into the optimized state
    C.watch(true, "finalgrasp");
  }

  C.watch(true, "final grasp"); //display it
  //-- redoing the optimization with the same KOMO object!
  //   (Warning: In doubt, rather create a new KOMO instance for each optimization.)

  //-- execute this in simulation
  rai::Configuration RealWorld;
  RealWorld.addFile("../../scenarios/challenge.g");
  rai::Simulation S(RealWorld, S._bullet, true);

  S.step();
  S.setMoveTo(q_pregrasp, 2.); //2 seconds to goal
  S.setMoveTo(q_finalgrasp, 0.5);
  S.setMoveTo(q_pregrasp, 2.);
  S.setMoveTo(q0, 1.); //1 second back home
  for (uint t = 0;; t++)
  {
    //cout << "time to move: " << S.getTimeToMove() << endl;
    double tau = .001; //can set anything here time...
    S.step({}, tau);
    if (t == 2500)
    {
      cout << "close gripper" << std::endl; //??
      S.closeGripper("R_gripper", 0.005, 0.29, 20);
      rai::wait(2000 * tau);
      cout << "open gripper" << std::endl;
      S.openGripper("R_gripper", 0.10, 0.29);
    }
    rai::wait(tau);
    if (S.getTimeToMove() < 0.)
      break;
  }
  rai::wait();
}

void using_KOMO_for_TwoArms()
{
  //we don't need to instantiate the real world/simluation here

  //-- MODEL WORLD configuration, this is the data structure on which you represent
  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");
  arr q0 = C.getJointState();
  rai::Frame *obj = C.addFrame("object");
  obj->setPosition({1., 0., 1.5});
  obj->setQuaternion({1., 0., 1., 0});
  obj->setShape(rai::ST_capsule, {.2, .02});
  obj->setColor({1., .0, 1.});

  std::shared_ptr<rai::Frame> R_gripper = std::shared_ptr<rai::Frame>(C.getFrame("R_gripper"));
  std::shared_ptr<rai::Frame> L_gripper = std::shared_ptr<rai::Frame>(C.getFrame("L_gripper"));
  auto R_gripper_position = R_gripper->getPosition();
  auto L_gripper_position = L_gripper->getPosition();
  auto middle_point_position = (R_gripper_position + L_gripper_position) / 2;

  std::shared_ptr<rai::Frame> middle_point = std::shared_ptr<rai::Frame>(C.addFrame("middle_point"));
  middle_point->setPosition(middle_point_position.getArr());
  middle_point->setQuaternion({1., 0., 1., 0});
  middle_point->setShape(rai::ST_marker, {0.1, 0.1, 0.1});

  //-- using the viewer, you can view configurations or paths
  C.watch(true, "model world start state");

  //-- optimize a single configuration using KOMO

  KOMO komo;              //create a solver
  komo.setModel(C, true); //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  int step = 10;
  komo.setTiming(1., step, 5., 2); //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)

  //now add objectives!

  //the default control objective:
  komo.add_qControlObjective({}, 2, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

  komo.addObjective({1.0, 1.0}, FS_positionRel, {"R_gripperCenter", "middle_point"}, OT_eq, {1}, {0, 0., 0.02});
  komo.addObjective({1.0, 1.0}, FS_positionRel, {"L_gripperCenter", "middle_point"}, OT_eq, {1}, {0, 0., -0.02});
  komo.addObjective({1.0, 1.0}, FS_scalarProductYX, {"R_gripperCenter", "middle_point"}, OT_eq, {1e2}, {-1});
  komo.addObjective({1.0, 1.0}, FS_scalarProductZZ, {"R_gripperCenter", "middle_point"}, OT_eq, {1e2}, {1});
  komo.addObjective({1.0, 1.0}, FS_scalarProductZZ, {"L_gripperCenter", "middle_point"}, OT_eq, {1e2}, {-1});
  komo.addObjective({1.0, 1.0}, FS_scalarProductYY, {"L_gripperCenter", "middle_point"}, OT_eq, {1e2}, {1});
  //initialize the solve
  komo.optimize();
  arr q_twoarm;
  for (int i = 0; i < step; i++)
  {
    q_twoarm = komo.getConfiguration_qOrg(i);
    cout << i << ": " << q_twoarm << std::endl;
    C.setJointState(q_twoarm); //set your working config into the optimized state
    C.watch(true, "twoarm");
  }
  //-- execute this in simulation
  rai::Configuration RealWorld;
  RealWorld.addFile("../../scenarios/challenge.g");
  rai::Simulation S(RealWorld, S._bullet, true);

  S.step();
  S.setMoveTo(q_twoarm, 1.); //2 seconds to goal
  S.setMoveTo(q0, 1.);       //1 second back home
  for (uint t = 0;; t++)
  {
    //cout << "time to move: " << S.getTimeToMove() << endl;
    double tau = .001; //can set anything here time...
    S.step({}, tau);
    rai::wait(tau);
    if (S.getTimeToMove() < 0.)
      break;
  }
  rai::wait();
}

//===========================================================================

void using_KOMO_for_PathPlanning()
{
  //we don't need to instantiate the real world/simluation here

  //-- MODEL WORLD configuration, this is the data structure on which you represent
  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");

  rai::Frame *obj = C.addFrame("object");
  obj->setPosition({1., 0., 1.5});
  obj->setQuaternion({1.5, 1.5, 3., 1});
  obj->setShape(rai::ST_capsule, {.2, .02});
  obj->setColor({1., .0, 1.});

  //-- using the viewer, you can view configurations or paths
  C.watch(true, "model world start state");

  //-- optimize a single configuration using KOMO

  KOMO komo;                     //create a solver
  komo.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo.setTiming(1., 40, 5., 2); //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)

  //now add objectives!

  //the default control objective:
  komo.add_qControlObjective({}, 2, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

  //task objectives:
  komo.addObjective({1.}, FS_positionDiff, {"R_gripperCenter", "object"}, OT_sos, {1e2});

  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);
  komo.addObjective({0.7, 1.}, FS_positionRel, {"R_gripperCenter", "object"}, OT_eq, {1e2}, {0, 0, 0.1}, 2);
  komo.addObjective({}, FS_distance, {"R_gripper", "object"}, OT_ineq, {1e2}, {});

  //initialize the solver
  komo.optimize();

  //get the joint vector from the optimized configuration
  arr q = komo.getConfiguration_qOrg(komo.T - 1);

  C.setJointState(q); //set your working config into the optimized state

  komo.view(true, "optimized configuration"); //display it
  komo.view_play();

  rai::wait(.1); //TODO: otherwise the opengl gets hung up?
}

//===========================================================================

void using_KOMO_for_ExploreCollision()
{
  //we don't need to instantiate the real world/simluation here

  //-- MODEL WORLD configuration, this is the data structure on which you represent
  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");

  rai::Frame *obj = C.addFrame("object");
  obj->setPosition({1., 0., 1.5});
  obj->setQuaternion({1.5, 1.5, 3., 1});
  obj->setShape(rai::ST_capsule, {.2, .02});
  obj->setColor({1., .0, 1.});

  rai::Frame *collision = C.addFrame("collision");
  collision->setPosition({0.5, 0., 1.4});
  collision->setQuaternion({1.5, 1.5, 3., 1});
  collision->setShape(rai::ST_sphere, {0.1, 0.1});
  collision->setColor({1., .0, 1.});

  //-- using the viewer, you can view configurations or paths
  C.watch(true, "model world start state");

  //-- optimize a single configuration using KOMO

  KOMO komo;                     //create a solver
  komo.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo.setTiming(1., 40, 5., 2); //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)

  //now add objectives!

  //the default control objective:
  komo.add_qControlObjective({}, 2, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

  //task objectives:
  komo.addObjective({1.}, FS_positionDiff, {"R_gripperCenter", "object"}, OT_sos, {1e2});

  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);
  komo.addObjective({0.7, 1.}, FS_positionRel, {"R_gripperCenter", "object"}, OT_eq, {1e2}, {0, 0, 0.1}, 2);
  komo.addObjective({}, FS_distance, {"R_gripper", "object"}, OT_ineq, {1e2}, {});
  komo.addObjective({}, FS_distance, {"R_gripper", "collision"}, OT_ineq, {1e2}, {-0.4});

  //initialize the solver
  komo.optimize();

  //get the joint vector from the optimized configuration
  arr q = komo.getConfiguration_qOrg(komo.T - 1);

  C.setJointState(q); //set your working config into the optimized state

  komo.view(true, "optimized configuration"); //display it
  komo.view_play();

  rai::wait(.1); //TODO: otherwise the opengl gets hung up?
}

//===========================================================================

void using_KOMO_for_EnforceTouch()
{
  //we don't need to instantiate the real world/simluation here

  //-- MODEL WORLD configuration, this is the data structure on which you represent
  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");

  rai::Frame *obj = C.addFrame("object");
  obj->setPosition({1., 0., 1.5});
  obj->setQuaternion({1.5, 1.5, 3., 1});
  obj->setShape(rai::ST_capsule, {.2, .02});
  obj->setColor({1., .0, 1.});
  /*
  rai::Frame *collision = C.addFrame("collision");
  collision->setPosition({0.5, 0., 1.4});
  collision->setQuaternion({1.5, 1.5, 3., 1});
  collision->setShape(rai::ST_sphere, {0.1, 0.1});
  collision->setColor({1., .0, 1.});
*/
  //-- using the viewer, you can view configurations or paths
  C.watch(true, "model world start state");

  //-- optimize a single configuration using KOMO

  KOMO komo;                     //create a solver
  komo.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo.setTiming(1., 40, 5., 2); //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)

  //now add objectives!

  //the default control objective:
  komo.add_qControlObjective({}, 2, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

  //task objectives:
  //komo.addObjective({1.}, FS_positionDiff, {"R_gripperCenter", "object"}, OT_sos, {1e2});

  //komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);
  //komo.addObjective({0.7, 1.}, FS_positionRel, {"R_gripperCenter", "object"}, OT_eq, {1e2}, {0, 0, 0.1}, 2);
  //komo.addObjective({}, FS_distance, {"R_gripper", "object"}, OT_ineq, {1e2}, {});
  //komo.addObjective({}, FS_distance, {"R_gripper", "collision"}, OT_ineq, {1e2}, {-0.4});
  komo.addObjective({1.}, FS_distance, {"R_finger1", "object"}, OT_eq, {1e2});
  //komo.addObjective({1.}, FS_distance, {"R_finger2", "object"}, OT_eq, {1e2});

  //initialize the solver
  komo.optimize();

  //get the joint vector from the optimized configuration
  arr q = komo.getConfiguration_qOrg(komo.T - 1);

  C.setJointState(q); //set your working config into the optimized state

  komo.view(true, "optimized configuration"); //display it
  komo.view_play();

  rai::wait(.1); //TODO: otherwise the opengl gets hung up?
}

//===========================================================================

void using_KOMO_for_RealObject()
{
  //we don't need to instantiate the real world/simluation here

  //-- MODEL WORLD configuration, this is the data structure on which you represent

  rai::Configuration RealWorld;
  RealWorld.addFile("../../scenarios/challenge.g");
  rai::Simulation S(RealWorld, S._bullet, true);
  double tau = .01; //can set anything here time...
  int steps = 1500;

  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");
  rai::Frame *myObj = C.addFrame("myobject");
  myObj->setShape(rai::ST_ssBox, {0.1, 0.1, 0.1, .1});
  //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  myObj->setColor({1., .0, 1.});

  KOMO komo; //create a solver
             //the default control objective:

  for (double t = 0; t < steps; t++)
  {
    KOMO komo; //create a solver
    //the default control objective:
    komo.setModel(C, true);
    komo.setTiming(1., 1, tau, 2);         //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
    komo.add_qControlObjective({}, 2, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)
    myObj->setPosition(RealWorld["obj1"]->getPosition());
    myObj->setQuaternion(RealWorld["obj1"]->getQuaternion());

    //-- using the viewer, you can view configurations or paths

    //task objectives:
    komo.addObjective({}, FS_distance, {"L_gripperCenter", "myobject"}, OT_eq, {1e2});
    komo.optimize();

    arr q = komo.getConfiguration_qOrg(komo.T - 1);
    C.setJointState(q); //set your working config into the optimized state
    C.watch(true, "model world start state");
    S.step({}, tau);
    //initialize the solver
  }
}

int main(int argc, char **argv)
{
  rai::initCmdLine(argc, argv);

  //using_KOMO_for_IK();
  //using_KOMO_for_TwoArms();
  //using_KOMO_for_PathPlanning();
  //using_KOMO_for_ExploreCollision();
  //using_KOMO_for_EnforceTouch();
  using_KOMO_for_RealObject();
  return 0;
}
