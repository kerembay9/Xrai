#include <Kin/F_qFeatures.h>
#include <Kin/F_collisions.h>
#include <Kin/F_pose.h>
#include <Kin/F_forces.h>
#include <Kin/forceExchange.h>
#include <iomanip>
#include <Kin/featureSymbols.h>

extern bool orsDrawWires;
extern bool rai_Kin_frame_ignoreQuatNormalizationWarning;

//===========================================================================

void testFeature() {
  rai::Configuration C;
  C.addFrame("world");
  rai::Frame *obj1 = C.addFrame("obj1", "world");
  rai::Frame *obj2 = C.addFrame("obj2", "world");
  obj1->setRelativePosition({1.,1.,1.});
  obj2->setRelativePosition({-1.,-1.,1.});
  obj1->setShape(rai::ST_ssBox, arr{1.,1.,1.,rnd.uni(.01, .3)});
  obj2->setShape(rai::ST_ssBox, arr{1.,1.,1.,rnd.uni(.01, .3)});
  obj1->setColor({.5,.8,.5,.4});
  obj2->setColor({.5,.5,.8,.4});

  obj1->setJoint(rai::JT_free);
  obj2->setJoint(rai::JT_transXYPhi);

  obj1->setMass(1.);
  obj2->setMass(1.);

  rai::ForceExchange con(*obj1, *obj2, rai::FXT_poa);

  C.setTaus(.1);

  arr q1 = C.getJointState();

  rai::Configuration pathConfig;
  pathConfig.addConfigurationCopy(C);
  pathConfig.addConfigurationCopy(C);
  pathConfig.addConfigurationCopy(C);
//  pathConfig.jacMode = rai::Configuration::JM_rowShifted;
//  pathConfig.jacMode = rai::Configuration::JM_dense;
  pathConfig.jacMode = rai::Configuration::JM_sparse;

  uint n=pathConfig.getJointStateDimension();
  arr q=pathConfig.getJointState();
  pathConfig.setJointState(q);

  arr q0 = C.getJointState();
  pathConfig.setJointStateSlice(q0, 1);

  rai::Array<std::shared_ptr<Feature>> F;
  F.append(make_shared<F_PairCollision>(F_PairCollision::_negScalar)); F(-1)->setFrameIDs({"obj1", "obj2"}, C);
  F.append(make_shared<F_PairCollision>(F_PairCollision::_normal)); F(-1)->setFrameIDs({"obj1", "obj2"}, C);
  F.append(make_shared<F_PairCollision>(F_PairCollision::_vector)); F(-1)->setFrameIDs({"obj1", "obj2"}, C);
  F.append(make_shared<F_PairCollision>(F_PairCollision::_center)); F(-1)->setFrameIDs({"obj1", "obj2"}, C);
  F.append(make_shared<F_LinAngVel>()); F(-1)->setFrameIDs({"obj1"}, C);
  F.append(make_shared<F_LinAngVel>()); F(-1)->setFrameIDs({"obj2"}, C) .setOrder(2);
  F.append(symbols2feature(FS_position, {"obj1"}, C));
  F.append(symbols2feature(FS_positionDiff, {"obj1", "obj2"}, C));
  F.append(make_shared<F_Matrix>()); F(-1)->setFrameIDs({"obj1"}, C);
  F.append(make_shared<F_MatrixDiff>()); F(-1)->setFrameIDs({"obj1", "obj2"}, C);
  F.append(symbols2feature(FS_pose, {"obj1"}, C)); F(-1)->setOrder(0);
  F.append(symbols2feature(FS_pose, {"obj2"}, C)); F(-1)->setOrder(1);
  F.append(symbols2feature(FS_pose, {"obj1"}, C)); F(-1)->setOrder(2);
  F.append(symbols2feature(FS_poseRel, {"obj1", "obj2"}, C)); F(-1)->setOrder(0);
  F.append(symbols2feature(FS_poseRel, {"obj1", "obj2"}, C)); F(-1)->setOrder(1);
  F.append(symbols2feature(FS_poseRel, {"obj1", "obj2"}, C)); F(-1)->setOrder(2);
  F.append(symbols2feature(FS_poseDiff, {"obj1", "obj2"}, C)); F(-1)->setOrder(0);
  F.append(symbols2feature(FS_poseDiff, {"obj1", "obj2"}, C)); F(-1)->setOrder(1);
  F.append(symbols2feature(FS_poseDiff, {"obj1", "obj2"}, C)); F(-1)->setOrder(2);
  F.append(symbols2feature(FS_insideBox, {"obj1", "obj2"}, C)); F(-1)->setOrder(0);
  F.append(make_shared<F_NewtonEuler>()); F(-1)->setFrameIDs({"obj1"}, C);
  F.append(make_shared<F_fex_POA>()); F(-1)->setFrameIDs({"obj1", "obj2"}, C);
  F.append(make_shared<F_fex_Force>()); F(-1)->setFrameIDs({"obj1", "obj2"}, C);
  F.append(make_shared<F_fex_ForceIsNormal>()); F(-1)->setFrameIDs({"obj1", "obj2"}, C);
  F.append(make_shared<F_fex_ForceIsPositive>()); F(-1)->setFrameIDs({"obj1", "obj2"}, C);
  F.append(make_shared<F_fex_POAzeroRelVel>()); F(-1)->setFrameIDs({"obj1", "obj2"}, C);
  F.append(make_shared<F_fex_POAContactDistances>()); F(-1)->setFrameIDs({"obj1", "obj2"}, C);
  F.append(make_shared<F_fex_POASurfaceDistance>(rai::_left)); F(-1)->setFrameIDs({"obj1", "obj2"}, C);
  F.append(make_shared<F_fex_POASurfaceDistance>(rai::_right)); F(-1)->setFrameIDs({"obj1", "obj2"}, C);
  F.append(make_shared<F_fex_POASurfaceNormal>(rai::_left)); F(-1)->setFrameIDs({"obj1", "obj2"}, C);
  F.append(make_shared<F_fex_POASurfaceNormal>(rai::_right)); F(-1)->setFrameIDs({"obj1", "obj2"}, C);

  rai_Kin_frame_ignoreQuatNormalizationWarning=true;

  rai::timerStart();
  for(uint k=0;k<100;k++){
    arr x = 5.*(rand(n)-.5);

    bool succ=true;

    for(shared_ptr<Feature>& f: F){
      cout <<k <<std::setw(30) <<f->shortTag(C) <<' ';
      succ &= checkJacobian(f->vf2(f->getFrames(pathConfig)), x, 1e-5);
    }

    arr y = F.first()->eval(F.first()->getFrames(pathConfig));

    if(!succ) pathConfig.view(true);
  }
  cout <<"*** COMPUTE TIME: " <<rai::timerRead() <<"sec" <<endl;
}

//===========================================================================

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  rnd.clockSeed();

  testFeature();

  return 0;
}
