/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "RRT_PathFinder.h"

#include "../Gui/opengl.h"
#include "../Kin/viewer.h"
#include "../KOMO/pathTools.h"

#ifdef RAI_GL
#  include <GL/glew.h>
#  include <GL/gl.h>
#endif

double corput(uint n, uint base) {
  double q = 0.;
  double bk = 1./double(base);

  while(n > 0) {
    q += (n % base)*bk;
    n /= base;
    bk /= double(base);
  }
  return q;
}

bool checkConnection(ConfigurationProblem& P,
                     const arr& start,
                     const arr& end,
                     const uint num,
                     const bool binary) {
  if(binary) {
    for(uint i=1; i<num; ++i) {
      double ind = corput(i, 2);
      arr p = start + ind * (end-start);

      // TODO: change to check feasibility properly (with path constraints)
      if(!P.query(p)->isFeasible) {
        return false;
      }
    }
  } else {
    for(uint i=1; i<num-1; ++i) {
      arr p = start + 1.0 * i / (num-1) * (end-start);

      // TODO: change to check feasibility properly (with path constraints)
      if(!P.query(p)->isFeasible) {
        return false;
      }
    }
  }
  return true;
}

//===========================================================================

RRT_SingleTree::RRT_SingleTree(const arr& q0, const shared_ptr<QueryResult>& q0_qr) {
//  if(!q0_qr->isFeasible) LOG(0) <<"rooting RRT with infeasible start configuration -- that's likely to fail: query is:\n" <<*q0_qr;
  drawMutex.lock(RAI_HERE);
  ann.append(q0);
  parent.append(0);
  queries.append(q0_qr);

  parent_cost.append(0);
  total_cost.append(0);


  disp3d.append(q0_qr->disp3d);
  disp3d.reshape(-1, 3);

  CHECK_EQ(parent.N, ann.X.d0, "");
  CHECK_EQ(queries.N, ann.X.d0, "");
  //CHECK_EQ(disp3d.d0, ann.X.d0, "");
  drawMutex.unlock();
}

uint RRT_SingleTree::add(const arr& q, uint parentID, const shared_ptr<QueryResult>& _qr) {
  drawMutex.lock(RAI_HERE);
  ann.append(q);
  parent.append(parentID);
  queries.append(_qr);
  double costFromParent = length(q - ann.X[parentID]);
  double totalNodeCost = total_cost(parentID) + costFromParent;
  parent_cost.append(costFromParent);
  total_cost.append(totalNodeCost);

  disp3d.append(_qr->disp3d);
  disp3d.reshape(-1, 3);

  CHECK_EQ(parent.N, ann.X.d0, "");
  CHECK_EQ(queries.N, ann.X.d0, "");
  //CHECK_EQ(disp3d.d0, ann.X.d0, "");
  drawMutex.unlock();
  return parent.N-1;
}

double RRT_SingleTree::getNearest(const arr& target) {
  //find NN
  nearestID = ann.getNN(target);
  return length(target - ann.X[nearestID]);
}

arr RRT_SingleTree::getProposalTowards(const arr& target, double stepsize) {
  //find NN
  nearestID = ann.getNN(target);

  //compute default step
  arr delta = target - ann.X[nearestID]; //difference vector between q and nearest neighbor
  double dist = length(delta);
  if(dist>stepsize)  delta *= stepsize/dist;

  return getNode(nearestID) + delta;
}

arr RRT_SingleTree::getNewSample(const arr& target, double stepsize, double p_sideStep, bool& isSideStep, const uint recursionDepth) {
  //find NN
  nearestID = ann.getNN(target);
  std::shared_ptr<QueryResult> qr = queries(nearestID);

  //compute default step
  arr delta = target - getNode(nearestID);
  double dist = length(delta);
  if(dist>stepsize) delta *= stepsize/dist;

  //without side stepping, we're done
  isSideStep = false;
  if(p_sideStep<=0. || recursionDepth >= 3) return getNode(nearestID) + delta;

  //check whether this is a predicted collision
  bool predictedCollision=false;
  if(qr->coll_y.N) {
    arr y = qr->coll_y + qr->coll_J * delta;
    if(min(y)<0.) predictedCollision = true;
  }

  if(predictedCollision && p_sideStep>0. && rnd.uni()<p_sideStep) {
    isSideStep=true;

    //compute new target
    arr d = qr->getSideStep();
    d *= rnd.uni(stepsize, 2.) / length(d);
    arr targ = getNode(nearestID) + d;
    bool tmp;
    return getNewSample(targ, stepsize, p_sideStep, tmp, recursionDepth + 1);
  } else {
    return getNode(nearestID) + delta;
  }

  HALT("shouldn't be here");
  return NoArr;
}

arr RRT_SingleTree::getPathFromNode(uint fromID) {
  arr path;
  uint node = fromID;
  for(;;) {
    path.append(ann.X[node]);
    if(!node) break;
    node = getParent(node);
  }
  path.reshape(-1, ann.X.d1);
  return path;
}

arr RRT_SingleTree::getNeighbors(const arr& q, double radius) {
  arr neighbors; // Initialize an empty array for the neighbors
  for (uint i = 0; i < ann.X.d0 -1; ++i) {
    arr node = ann.X[i]; // Get the position of node i

    // Compute the Euclidean distance using the length function
    if (length(node - q) <= radius) { // Check if the distance is within the radius
      neighbors.append(i); // Add the index of the neighboring node
    }
  }
  return neighbors; // Return the array of neighbor indices
}

void RRT_SingleTree::changeParent(uint nodeID, uint newParentID, double newPathCost) {
  // Validate indices
  if (nodeID >= parent.N || newParentID >= parent.N) {
    return;
  }

  parent(nodeID) = newParentID;
  parent_cost(nodeID) = newPathCost;
  total_cost(nodeID) = total_cost(newParentID) + newPathCost;
}


//===========================================================================

bool RRT_PathFinder::growTreeTowardsRandom(RRT_SingleTree& rrt) {
  const arr start = rrt.ann.X[0];
  arr t(rrt.getNode(0).N);
  rndUniform(t, -RAI_2PI, RAI_2PI, false);
  HALT("DON'T USE 2PI")

  arr q = rrt.getProposalTowards(t, stepsize);

  auto qr = P.query(q);
  if(qr->isFeasible) {
    if(subsampleChecks>0 && !checkConnection(P, start, q, subsampleChecks, true)) {
      return false;
    }

    rrt.add(q, rrt.nearestID, qr);
    return true;
  }
  return false;
}


bool RRT_PathFinder::growTreeToTree(RRT_SingleTree& rrt_A, RRT_SingleTree& rrt_B) {
  bool isSideStep, isForwardStep;
  //decide on a target: forward or random
  arr t;
  if(rnd.uni()<p_forwardStep) {
    t = rrt_B.getRandomNode();
    isForwardStep = true;
  } else {
#if 1
    t.resize(rrt_A.getNode(0).N);
    for(uint i=0; i<t.N; i++) {
      double lo=P.limits(0, i), up=P.limits(1, i);
      CHECK_GE(up-lo, 1e-3, "limits are null interval: " <<i <<' ' <<P.C.getJointNames());
      t.elem(i) = lo + rnd.uni()*(up-lo);
    }
#else
    t.resize(rrt_A.getNode(0).N);
    rndUniform(t, -RAI_2PI, RAI_2PI, false);
#endif
    isForwardStep = false;
  }

  //sample configuration towards target, possibly sideStepping
  arr q = rrt_A.getNewSample(t, stepsize, p_sideStep, isSideStep, 0);
  uint parentID = rrt_A.nearestID;

  //special rule: if parent is already in collision, isFeasible = smaller collisions
  shared_ptr<QueryResult>& pr = rrt_A.queries(parentID);
  double org_collisionTolerance = P.collisionTolerance;
  if(pr->totalCollision>P.collisionTolerance) P.collisionTolerance = pr->totalCollision + 1e-6;

  //evaluate the sample
  auto qr = P.query(q);
  if(isForwardStep) {  n_forwardStep++; if(qr->isFeasible) n_forwardStepGood++; }
  if(!isForwardStep) {  n_rndStep++; if(qr->isFeasible) n_rndStepGood++; }
  if(isSideStep) {  n_sideStep++; if(qr->isFeasible) n_sideStepGood++; }

  //if infeasible, make a backward step from the sample configuration
  if(!qr->isFeasible && p_backwardStep>0. && rnd.uni()<p_backwardStep) {
    t = q + qr->getBackwardStep();
    q = rrt_A.getNewSample(t, stepsize, p_sideStep, isSideStep, 0);
    qr = P.query(q);
    n_backStep++; if(qr->isFeasible) n_backStepGood++;
    if(isSideStep) {  n_sideStep++; if(qr->isFeasible) n_sideStepGood++; }
  };

  //checking subsamples
  if(qr->isFeasible && subsampleChecks>0) {
    const arr start = rrt_A.ann.X[parentID];
    qr->isFeasible = checkConnection(P, start, q, subsampleChecks, true);
  }

  P.collisionTolerance = org_collisionTolerance;

  //finally adding the new node to the tree
  if(qr->isFeasible){
    rrt_A.add(q, parentID, qr);
    double dist = rrt_B.getNearest(q);
    if(subsampleChecks>0) { if(dist<stepsize/subsampleChecks) return true; }
    else { if(dist<stepsize) return true; }
  }

  return false;
}

//===========================================================================

RRT_PathFinder::RRT_PathFinder(ConfigurationProblem& _P, const arr& _starts, const arr& _goals, double _stepsize, int _subsampleChecks, int _maxIters, int _verbose)
  : P(_P),
    stepsize(_stepsize),
    maxIters(_maxIters),
    verbose(_verbose),
    subsampleChecks(_subsampleChecks) {

  if(stepsize<0.) stepsize = rai::getParameter<double>("rrt/stepsize", .1);
  if(subsampleChecks<0) subsampleChecks = rai::getParameter<int>("rrt/subsamples", 4);
  if(maxIters<0) maxIters = rai::getParameter<int>("rrt/maxIters", 5000);
  if(verbose<0) verbose = rai::getParameter<int>("rrt/verbose", 0);

  arr q0 = _starts;
  arr qT = _goals;
  auto q0ret = P.query(q0);
  auto qTret = P.query(qT);
  if(!q0ret->isFeasible) { LOG(0) <<"initializing with infeasible q0:"; q0ret->writeDetails(std::cout, P); }
  if(!qTret->isFeasible) { LOG(0) <<"initializing with infeasible qT:"; qTret->writeDetails(std::cout, P); }
  rrt0 = make_shared<RRT_SingleTree>(q0, q0ret);
  rrtT = make_shared<RRT_SingleTree>(qT, qTret);

  if(verbose>2) {
    DISP.copy(P.C);
  }
}

void RRT_PathFinder::planForward(const arr& q0, const arr& qT) {
  bool success=false;

  for(uint i=0; i<100000; i++) {
    iters++;
    //let rrt0 grow
    bool added = growTreeTowardsRandom(*rrt0);
    if(added) {
      if(length(rrt0->getLast() - qT)<stepsize) success = true;
    }
    if(success) break;

    //some output
    if(verbose>2) {
      if(!(i%100)) {
        DISP.setJointState(rrt0->getLast());
        DISP.view(false);
        std::cout <<"RRT samples=" <<i <<" tree size = " <<rrt0->getNumberNodes() <<std::endl;
      }
    }
  }

  if(!success) return;

  if(verbose>0) {
    std::cout <<"SUCCESS!"
              <<"\n  tested samples=" <<P.evals
              <<"\n  #tree-size=" <<rrt0->getNumberNodes()
              << std::endl;
  }

  arr path = rrt0->getPathFromNode(rrt0->nearestID);
  revertPath(path);

  //display
  if(verbose>1) {
    std::cout << "path-length= " << path.d0 <<std::endl;
    DISP.proxies.clear();

    for(uint t=0; t<path.d0; t++) {
      DISP.setJointState(path[t]);
      //DISP.view();
      DISP.view(false);
      rai::wait(.1);
    }
  }

  path >>FILE("z.path");
}

int RRT_PathFinder::stepConnect() {
  iters++;
  if(iters>(uint)maxIters) return -1;

  bool success = growTreeToTree(*rrt0, *rrtT);
  if(!success) success = growTreeToTree(*rrtT, *rrt0);

  //animation display
  if(verbose>2) {
    if(!(iters%100)) {
      DISP.setJointState(rrt0->getLast());
      DISP.view(verbose>4, STRING("planConnect evals " <<P.evals));
    }
  }
  if(verbose>1) {
    if(!(iters%100)) {
      std::cout <<"RRT queries=" <<P.evals <<" tree sizes = " <<rrt0->getNumberNodes()  <<' ' <<rrtT->getNumberNodes() <<std::endl;
    }
  }

  //-- the rest is only on success -> extract the path, display, etc

  if(success) {
    if(verbose>0) {
      std::cout <<"  -- rrt success:";
      std::cout <<" queries:" <<P.evals <<" tree sizes: " <<rrt0->getNumberNodes()  <<' ' <<rrtT->getNumberNodes() <<std::endl;
    }

    path = rrt0->getPathFromNode(rrt0->nearestID);
    arr pathT = rrtT->getPathFromNode(rrtT->nearestID);

    revertPath(path);
    path.append(pathT);

    //display
    if(verbose>1) {
      std::cout <<"  path-length=" <<path.d0 <<std::endl;
      if(verbose>2) {
        DISP.proxies.clear();
        for(uint t=0; t<path.d0; t++) {
          DISP.setJointState(path[t]);
          DISP.view(false, STRING("rrt result "<<t));
          rai::wait(.1);
        }
        DISP.view(verbose>3);
        DISP.clear();
      }
    }

    return 1;
  }

  return 0;
}

arr RRT_PathFinder::planConnect() {
  int r=0;
  while(!r) { r = stepConnect(); }
  if(r==-1) path.clear();
  return path;
}

void revertPath(arr& path) {
  uint N = path.d0;
  arr x;
  for(uint i=0; i<N/2; i++) {
    x = path[i];
    path[i] = path[N-1-i];
    path[N-1-i] = x;
  }
}

arr RRT_PathFinder::run(double timeBudget) {
  planConnect();
  return path;
}

namespace rai {

void PathFinder::setProblem(const Configuration& C, const arr& starts, const arr& goals, double collisionTolerance) {
  if(collisionTolerance<0.) collisionTolerance = rai::getParameter<double>("rrt/collisionTolerance", 1e-4);
  problem = make_shared<ConfigurationProblem>(C, true, collisionTolerance, 1);
  problem->verbose=0;
  rrtSolver = make_shared<RRT_PathFinder>(*problem, starts, goals);
  rrtStarSolver = make_shared<RRT_Star_PathFinder>(*problem, starts, goals);
  singlerrtStarSolver = make_shared<Single_RRT_Star_PathFinder>(*problem, starts, goals);
  psbirrtSolver = make_shared<PSBI_RRT_PathFinder>(*problem, starts, goals);

}

void PathFinder::setExplicitCollisionPairs(const StringA& collisionPairs) {
  CHECK(problem, "need to set problem first");
  problem->setExplicitCollisionPairs(collisionPairs);
}

void PathFinder::helloworld() const {
    std::cout << "Hello, World!" << std::endl;
}

shared_ptr<SolverReturn> PathFinder::solve() {
  if(!ret) ret = make_shared<SolverReturn>();

  ret->time -= rai::cpuTime();
  rrtSolver->run();
  ret->time += rai::cpuTime();

  ret->done = true; //(r!=0);
  ret->feasible = rrtSolver->path.N; //(r==1);
  ret->x = rrtSolver->path;
  ret->evals = rrtSolver->iters;

  std::cout << "normal solve completed!" << std::endl;
  return ret;
}

shared_ptr<SolverReturn> PathFinder::star_solve() {
  if(!star_ret) star_ret = make_shared<SolverReturn>();

  star_ret->time -= rai::cpuTime();
  rrtStarSolver->run();
  star_ret->time += rai::cpuTime();

  star_ret->done = true; //(r!=0);
  star_ret->feasible = rrtStarSolver->path.N; //(r==1);
  star_ret->x = rrtStarSolver->path;
  star_ret->evals = rrtStarSolver->iters;
  std::cout << "Star solve completed!" << std::endl;
  return star_ret;
}

shared_ptr<SolverReturn> PathFinder::single_star_solve() {
  if(!single_star_ret) single_star_ret = make_shared<SolverReturn>();

  single_star_ret->time -= rai::cpuTime();
  singlerrtStarSolver->run();
  single_star_ret->time += rai::cpuTime();

  single_star_ret->done = true; //(r!=0);
  single_star_ret->feasible = singlerrtStarSolver->path.N; //(r==1);
  single_star_ret->x = singlerrtStarSolver->path;
  single_star_ret->evals = singlerrtStarSolver->iters;
  std::cout << "Single Star solve completed!" << std::endl;
  return single_star_ret;
}

shared_ptr<SolverReturn> PathFinder::psbi_solve() {
  if(!psbi_ret) psbi_ret = make_shared<SolverReturn>();

  psbi_ret->time -= rai::cpuTime();
  psbirrtSolver->run();
  psbi_ret->time += rai::cpuTime();

  psbi_ret->done = true; //(r!=0);
  psbi_ret->feasible = psbirrtSolver->path.N; //(r==1);
  psbi_ret->x = psbirrtSolver->path;
  psbi_ret->evals = psbirrtSolver->iters;
  std::cout << "Star solve completed!" << std::endl;
  return psbi_ret;
}


arr PathFinder::get_resampledPath(uint T){ return path_resampleLinear(ret->x, T); }

} //namespace

////////////////////////////////////////////////////////

bool RRT_Star_PathFinder::growTreeTowardsRandom(RRT_SingleTree& rrt) {
  std::cout << "entered growTreeTowardsRandom" << std::endl;
  const arr start = rrt.ann.X[0]; // Starting point, first node in the tree
  arr t(rrt.getDim()); // t is the target, dimension matches the tree's dimension
  rndUniform(t, -RAI_2PI, RAI_2PI, false); // Uniform random target within bounds

  arr q = rrt.getProposalTowards(t, stepsize); // Propose new node toward target

  auto qr = P.query(q); // Query if the new point is feasible
  if (qr->isFeasible) {
    if (subsampleChecks > 0 && !checkConnection(P, start, q, subsampleChecks, true)) {
      return false; // Connection check failed
    }

    uint parentID = rrt.nearestID; // Nearest node's index
    rrt.add(q, parentID, qr); // Add new node to the tree

    // Rewiring: search for neighbors and potentially rewire them
    double radius = stepsize; // Set a rewiring radius
    arr neighbors = rrt.getNeighbors(q, radius); // Get neighbors of the new node
    for (uint i = 0; i < neighbors.N; ++i) {
      uint neighborID = neighbors(i);
      arr neighborNode = rrt.getNode(neighborID);

      // Recompute the cost and check if rewiring is beneficial
      double currentCost = rrt.getNearest(q) + length(neighborNode - q); // Use length instead of distance
      if (currentCost < rrt.getNearest(neighborNode) + length(neighborNode - q)) {
        // If rewiring improves the path, update the parent
        rrt.add(q, neighborID, qr);
      }
    }
    return true;
  }
  return false;
}

bool RRT_Star_PathFinder::growTreeToTree(RRT_SingleTree& rrt_A, RRT_SingleTree& rrt_B) {
  bool isSideStep, isForwardStep;
  //decide on a target: forward or random
  arr t;
  if(rnd.uni()<p_forwardStep) {
    t = rrt_B.getRandomNode();
    isForwardStep = true;
  } else {
    t.resize(rrt_A.getNode(0).N);
    for(uint i=0; i<t.N; i++) {
      double lo=P.limits(0, i), up=P.limits(1, i);
      CHECK_GE(up-lo, 1e-3, "limits are null interval: " <<i <<' ' <<P.C.getJointNames());
      t.elem(i) = lo + rnd.uni()*(up-lo);
    }
    isForwardStep = false;
  }

  //sample configuration towards target, possibly sideStepping
  arr q = rrt_A.getNewSample(t, stepsize, p_sideStep, isSideStep, 0);
  uint parentID = rrt_A.nearestID;

  shared_ptr<QueryResult>& pr = rrt_A.queries(parentID);
  double org_collisionTolerance = P.collisionTolerance;
  if(pr->totalCollision>P.collisionTolerance) P.collisionTolerance = pr->totalCollision + 1e-6;

  auto qr = P.query(q);
  if(isForwardStep) {  n_forwardStep++; if(qr->isFeasible) n_forwardStepGood++; }
  if(!isForwardStep) {  n_rndStep++; if(qr->isFeasible) n_rndStepGood++; }
  if(isSideStep) {  n_sideStep++; if(qr->isFeasible) n_sideStepGood++; }

  if(!qr->isFeasible && p_backwardStep>0. && rnd.uni()<p_backwardStep) {
    t = q + qr->getBackwardStep();
    q = rrt_A.getNewSample(t, stepsize, p_sideStep, isSideStep, 0);
    qr = P.query(q);
    n_backStep++; if(qr->isFeasible) n_backStepGood++;
    if(isSideStep) {  n_sideStep++; if(qr->isFeasible) n_sideStepGood++; }
  };

  if(qr->isFeasible && subsampleChecks>0) {
    const arr start = rrt_A.ann.X[parentID];
    qr->isFeasible = checkConnection(P, start, q, subsampleChecks, true);
  }

  P.collisionTolerance = org_collisionTolerance;

  double radius = stepsize;
  if (qr->isFeasible) {
  uint newNodeID = rrt_A.add(q, parentID, qr);
  arr newNode = rrt_A.getNode(newNodeID);
  arr neighbors = rrt_A.getNeighbors(q, radius); 
  for (uint i = 0; i < neighbors.N; ++i) {
    uint neighborID = neighbors(i);
    arr neighborNode = rrt_A.getNode(neighborID);
    if (parentID == neighborID) {
    } else {
      double newPathCost = rrt_A.getCost(neighborID) + length(neighborNode - newNode);
      double currentPathCost = rrt_A.getCost(newNodeID);
      if (newPathCost < currentPathCost) {
        rrt_A.changeParent(newNodeID, neighborID, newPathCost);
      }
    }
  }
  // if possible, update neighbors' parents to new node
  for (uint i = 0; i < neighbors.N; ++i) {
    uint neighborID = neighbors(i);
    arr neighborNode = rrt_A.getNode(neighborID);
    if (parentID == neighborID) {
    } else {
      double newPathCost = rrt_A.getCost(newNodeID) + length(neighborNode - newNode);
      double currentPathCost = rrt_A.getCost(neighborID);
      if (newPathCost < currentPathCost) {
        std::cout << "parent changed " << std:: endl;
        rrt_A.changeParent(neighborID, newNodeID, newPathCost);
      }
    }
  }
  double dist = rrt_B.getNearest(q);
  if(subsampleChecks>0) { if(dist<stepsize/subsampleChecks) return true; }
  else { if(dist<stepsize) return true; }
  }
  return false;
}

RRT_Star_PathFinder::RRT_Star_PathFinder(ConfigurationProblem& _P, const arr& _starts, const arr& _goals, double _stepsize, int _subsampleChecks, int _maxIters, int _verbose)
  : P(_P),
    stepsize(_stepsize),
    maxIters(_maxIters),
    verbose(_verbose),
    subsampleChecks(_subsampleChecks) {

  std::cout << "entered RRT_Star_PathFinder" << std::endl;
  if(stepsize<0.) stepsize = rai::getParameter<double>("rrt/stepsize", .1);
  if(subsampleChecks<0) subsampleChecks = rai::getParameter<int>("rrt/subsamples", 4);
  if(maxIters<0) maxIters = rai::getParameter<int>("rrt/maxIters", 5000);
  if(verbose<0) verbose = rai::getParameter<int>("rrt/verbose", 0);

  arr q0 = _starts;
  arr qT = _goals;
  auto q0ret = P.query(q0);
  auto qTret = P.query(qT);
  if(!q0ret->isFeasible) { LOG(0) <<"initializing with infeasible q0:"; q0ret->writeDetails(std::cout, P); }
  if(!qTret->isFeasible) { LOG(0) <<"initializing with infeasible qT:"; qTret->writeDetails(std::cout, P); }
  rrt0 = make_shared<RRT_SingleTree>(q0, q0ret);
  rrtT = make_shared<RRT_SingleTree>(qT, qTret);
  rrtTcopy = make_shared<RRT_SingleTree>(qT, qTret);

  if(verbose>2) {
    DISP.copy(P.C);
  }
}

void RRT_Star_PathFinder::planForward(const arr& q0, const arr& qT) {
  bool success=false;

  for(uint i=0; i<100000; i++) {
    iters++;
    //let rrt0 grow
    bool added = growTreeTowardsRandom(*rrt0);
    if(added) {
      if(length(rrt0->getLast() - qT)<stepsize) success = true;
    }
    // if(success) {std::cout <<"success" <<std::endl;}//break;

    //some output
    if(verbose>2) {
      if(!(i%100)) {
        DISP.setJointState(rrt0->getLast());
        DISP.view(false);
        std::cout <<"RRT samples=" <<i <<" tree size = " <<rrt0->getNumberNodes() <<std::endl;
      }
    }
  }

  if(!success) return;

  if(verbose>0) {
    std::cout <<"SUCCESS!"
              <<"\n  tested samples=" <<P.evals
              <<"\n  #tree-size=" <<rrt0->getNumberNodes()
              << std::endl;
  }

  arr path = rrt0->getPathFromNode(rrt0->nearestID);
  revertPath(path);

  //display
  if(verbose>1) {
    std::cout << "path-length= " << path.d0 <<std::endl;
    DISP.proxies.clear();

    for(uint t=0; t<path.d0; t++) {
      DISP.setJointState(path[t]);
      //DISP.view();
      DISP.view(false);
      rai::wait(.1);
    }
  }

  path >>FILE("z.path");
}

int RRT_Star_PathFinder::stepConnect() {
  iters++;
  if(iters>(uint)maxIters) return foundPath; //-1

  bool success = growTreeToTree(*rrt0, *rrtT);
  if(!success) success = growTreeToTree(*rrtT, *rrt0);

  if(success) {
    p_forwardStep = 0.0;

    path = rrt0->getPathFromNode(rrt0->nearestID);
    arr pathT = rrtT->getPathFromNode(rrtT->nearestID);

    revertPath(path);
    path.append(pathT);


    auto rootQueryResult = std::make_shared<QueryResult>();
    rootQueryResult->disp3d = path[0]; // Use the first node's position for display data

    // Create the tree with the root node
    auto tree = std::make_shared<RRT_SingleTree>(path[0], rootQueryResult);

    // Add the rest of the nodes to the tree
    for (uint i = 1; i < path.d0; ++i) {
      arr currentNode = path[i];
      arr parentNode = path[i - 1];

      uint parentID = i-1;

      auto queryResult = std::make_shared<QueryResult>();
      queryResult->disp3d = currentNode;

      tree->add(currentNode, parentID, queryResult);
    }
    uint finalID = path.d0 -1; 

    while(iters<=(uint)maxIters) { 
      iters++;
      growTreeToTree(*tree,*rrtTcopy);
    }

    path = tree->getPathFromNode(finalID);
    revertPath(path);
    foundPath = 1;
    return 1; //0 to continue
  }

  return 0;
}

arr RRT_Star_PathFinder::planConnect() {
  int r=0;
  while(!r) { r = stepConnect(); }
  if(r==1) { std::cout <<"done and now I will add stuff"<<std::endl;}
  if(r==-1) path.clear();
  return path;
}

arr RRT_Star_PathFinder::run(double timeBudget) {
  planConnect();
  return path;
}

void RRT_Star_PathFinder::revertNodes() {
  // Lock the tree's drawMutex to ensure thread safety
  rrtT->drawMutex.lock(RAI_HERE);

  uint N = rrtT->getNumberNodes(); // Total number of nodes
  for (uint i = 0; i < N / 2; i++) {
    // Swap nodes in the ANN (point cloud)
    auto tempNode = rrtT->ann.X[i];
    rrtT->ann.X[i] = rrtT->ann.X[N - 1 - i];
    rrtT->ann.X[N - 1 - i] = tempNode;

    // Swap parent indices
    auto tempParent = rrtT->parent(i);
    rrtT->parent(i) = rrtT->parent(N - 1 - i);
    rrtT->parent(N - 1 - i) = tempParent;

    // Swap parent costs
    auto tempParentCost = rrtT->parent_cost(i);
    rrtT->parent_cost(i) = rrtT->parent_cost(N - 1 - i);
    rrtT->parent_cost(N - 1 - i) = tempParentCost;

    // Swap total costs
    auto tempTotalCost = rrtT->total_cost(i);
    rrtT->total_cost(i) = rrtT->total_cost(N - 1 - i);
    rrtT->total_cost(N - 1 - i) = tempTotalCost;

    // Swap query results
    auto tempQuery = rrtT->queries(i);
    rrtT->queries(i) = rrtT->queries(N - 1 - i);
    rrtT->queries(N - 1 - i) = tempQuery;

    // Swap display data if needed
    arr tempDisp = rrtT->disp3d[i];
    rrtT->disp3d[i] = rrtT->disp3d[N - 1 - i];
    rrtT->disp3d[N - 1 - i] = tempDisp;
  }

  // Unlock the tree's drawMutex after operation
  rrtT->drawMutex.unlock();
}

void RRT_Star_PathFinder::mergeTrees() {
  rrt0->drawMutex.lock(RAI_HERE);
  rrtT->drawMutex.lock(RAI_HERE);

  /////// revert
  uint N = rrtT->getNumberNodes(); // Total number of nodes
  for (uint i = 0; i < N / 2; i++) {
    // Swap nodes in the ANN (point cloud)
    auto tempNode = rrtT->ann.X[i];
    rrtT->ann.X[i] = rrtT->ann.X[N - 1 - i];
    rrtT->ann.X[N - 1 - i] = tempNode;

    // Swap parent indices
    auto tempParent = rrtT->parent(i);
    rrtT->parent(i) = rrtT->parent(N - 1 - i);
    rrtT->parent(N - 1 - i) = tempParent;

    // Swap parent costs
    auto tempParentCost = rrtT->parent_cost(i);
    rrtT->parent_cost(i) = rrtT->parent_cost(N - 1 - i);
    rrtT->parent_cost(N - 1 - i) = tempParentCost;

    // Swap total costs
    auto tempTotalCost = rrtT->total_cost(i);
    rrtT->total_cost(i) = rrtT->total_cost(N - 1 - i);
    rrtT->total_cost(N - 1 - i) = tempTotalCost;

    // Swap query results
    auto tempQuery = rrtT->queries(i);
    rrtT->queries(i) = rrtT->queries(N - 1 - i);
    rrtT->queries(N - 1 - i) = tempQuery;

    // Swap display data if needed
    arr tempDisp = rrtT->disp3d[i];
    rrtT->disp3d[i] = rrtT->disp3d[N - 1 - i];
    rrtT->disp3d[N - 1 - i] = tempDisp;
  }
  ///////


  //TODO; Take nearest node and change its parent to rrt0 parent first, then add all the other stuff
  // there may be two ways to do this: either start from i = rrtT->getNumberNodes() go to 0 and add accordingly
  // or revert as above and continue as is.
  std::cout << "entering adding part" << std::endl;
  rrt0->drawMutex.unlock();
  rrtT->drawMutex.unlock();
  rrt0->add(rrtT->ann.X[0],rrt0->nearestID, P.query(rrtT->ann.X[0]));
  for (uint i = 1; i < rrtT->getNumberNodes(); ++i) {
    std::cout << "adding " << i  << std::endl;
    std::cout << "vals are :" << rrtT->ann.X[i] << rrtT->parent(i) << P.query(rrtT->ann.X[i]) << std::endl;
    rrt0->add(rrtT->ann.X[i],rrtT->parent(i), P.query(rrtT->ann.X[i]));
    
  }

  
}


////////////////////////////////////////////////////////

bool Single_RRT_Star_PathFinder::growTreeTowardsRandom(RRT_SingleTree& rrt) {
  std::cout << "entered growTreeTowardsRandom" << std::endl;
  const arr start = rrt.ann.X[0]; // Starting point, first node in the tree
  arr t(rrt.getDim()); // t is the target, dimension matches the tree's dimension
  rndUniform(t, -RAI_2PI, RAI_2PI, false); // Uniform random target within bounds

  arr q = rrt.getProposalTowards(t, stepsize); // Propose new node toward target

  auto qr = P.query(q); // Query if the new point is feasible
  if (qr->isFeasible) {
    if (subsampleChecks > 0 && !checkConnection(P, start, q, subsampleChecks, true)) {
      return false; // Connection check failed
    }

    uint parentID = rrt.nearestID; // Nearest node's index
    rrt.add(q, parentID, qr); // Add new node to the tree

    // Rewiring: search for neighbors and potentially rewire them
    double radius = stepsize; // Set a rewiring radius
    arr neighbors = rrt.getNeighbors(q, radius); // Get neighbors of the new node
    for (uint i = 0; i < neighbors.N; ++i) {
      uint neighborID = neighbors(i);
      arr neighborNode = rrt.getNode(neighborID);

      // Recompute the cost and check if rewiring is beneficial
      double currentCost = rrt.getNearest(q) + length(neighborNode - q); // Use length instead of distance
      if (currentCost < rrt.getNearest(neighborNode) + length(neighborNode - q)) {
        // If rewiring improves the path, update the parent
        rrt.add(q, neighborID, qr);
      }
    }
    return true;
  }
  return false;
}

bool Single_RRT_Star_PathFinder::growTreeToTree(RRT_SingleTree& rrt_A, RRT_SingleTree& rrt_B) {
  bool isSideStep, isForwardStep;
  //decide on a target: forward or random
  arr t;
  // std::cout << "entered growTreeToTree" << std::endl;
  if(rnd.uni()<p_forwardStep) {
    t = rrt_B.getRandomNode();
    isForwardStep = true;
  } else {
    t.resize(rrt_A.getNode(0).N);
    for(uint i=0; i<t.N; i++) {
      double lo=P.limits(0, i), up=P.limits(1, i);
      CHECK_GE(up-lo, 1e-3, "limits are null interval: " <<i <<' ' <<P.C.getJointNames());
      t.elem(i) = lo + rnd.uni()*(up-lo);
    }
    isForwardStep = false;
  }

  //sample configuration towards target, possibly sideStepping
  arr q = rrt_A.getNewSample(t, stepsize, p_sideStep, isSideStep, 0);
  uint parentID = rrt_A.nearestID;

  //special rule: if parent is already in collision, isFeasible = smaller collisions
  shared_ptr<QueryResult>& pr = rrt_A.queries(parentID);
  double org_collisionTolerance = P.collisionTolerance;
  if(pr->totalCollision>P.collisionTolerance) P.collisionTolerance = pr->totalCollision + 1e-6;

  //evaluate the sample
  auto qr = P.query(q);
  if(isForwardStep) {  n_forwardStep++; if(qr->isFeasible) n_forwardStepGood++; }
  if(!isForwardStep) {  n_rndStep++; if(qr->isFeasible) n_rndStepGood++; }
  if(isSideStep) {  n_sideStep++; if(qr->isFeasible) n_sideStepGood++; }

  //if infeasible, make a backward step from the sample configuration
  if(!qr->isFeasible && p_backwardStep>0. && rnd.uni()<p_backwardStep) {
    t = q + qr->getBackwardStep();
    q = rrt_A.getNewSample(t, stepsize, p_sideStep, isSideStep, 0);
    qr = P.query(q);
    n_backStep++; if(qr->isFeasible) n_backStepGood++;
    if(isSideStep) {  n_sideStep++; if(qr->isFeasible) n_sideStepGood++; }
  };

  //checking subsamples
  if(qr->isFeasible && subsampleChecks>0) {
    const arr start = rrt_A.ann.X[parentID];
    qr->isFeasible = checkConnection(P, start, q, subsampleChecks, true);
  }

  P.collisionTolerance = org_collisionTolerance;

  //finally adding the new node to the tree
  double radius = stepsize;
  if (qr->isFeasible) {
  uint newNodeID = rrt_A.add(q, parentID, qr); // Add the new node with its initial parent
  arr newNode = rrt_A.getNode(newNodeID);
  // std::cout << "adding " << newNodeID << std::endl;
  arr neighbors = rrt_A.getNeighbors(q, radius); // Get neighbors within the specified radius
  // std::cout << "neighbors are " << neighbors << std::endl;
  // find the parent from neighbors
  for (uint i = 0; i < neighbors.N; ++i) {
    uint neighborID = neighbors(i);
    arr neighborNode = rrt_A.getNode(neighborID);
    // std::cout << "neighborID " << neighborID << " neighborNode " << neighborNode << std::endl;
    if (parentID == neighborID) {
      // std::cout << "Invalid parentID: " << parentID << std::endl;
    } else {
      // arr parent = rrt_A.getNode(parentID);
      // std::cout << "parentID: " << parentID << std::endl;
    //   // Compute the cost of the current path and the potential rewired path
      double newPathCost = rrt_A.getCost(neighborID) + length(neighborNode - newNode);
      double currentPathCost = rrt_A.getCost(newNodeID);
      // std::cout << "newPathCost is " << newPathCost << "currentPathCost is " << currentPathCost << std::endl;
      if (newPathCost < currentPathCost) {
        // std::cout << "found shorter path" << std::endl;
        // Rewire the neighbor if the new path is better
        rrt_A.changeParent(newNodeID, neighborID, newPathCost);
      }
    }
  }
  // if possible, update neighbors' parents to new node
  for (uint i = 0; i < neighbors.N; ++i) {
    uint neighborID = neighbors(i);
    arr neighborNode = rrt_A.getNode(neighborID);
    // std::cout << "neighborID " << neighborID << " neighborNode " << neighborNode << std::endl;
    if (parentID == neighborID) {
      // std::cout << "Invalid parentID: " << parentID << std::endl;
    } else {
      // arr parent = rrt_A.getNode(parentID);
      // std::cout << "parentID: " << parentID << std::endl;
    //   // Compute the cost of the current path and the potential rewired path
      double newPathCost = rrt_A.getCost(newNodeID) + length(neighborNode - newNode);
      double currentPathCost = rrt_A.getCost(neighborID);
      // std::cout << "newPathCost is " << newPathCost << "currentPathCost is " << currentPathCost << std::endl;
      if (newPathCost < currentPathCost) {
        // std::cout << "found shorter path" << std::endl;
        // Rewire the neighbor if the new path is better
        rrt_A.changeParent(neighborID, newNodeID, newPathCost);
      }
    }
  }
  double dist = rrt_B.getNearest(q);
  // std::cout << "dist: " << dist << " stepsize/subsampleChecks: " << stepsize/subsampleChecks << std::endl;
  if(subsampleChecks>0) { if(dist<stepsize/subsampleChecks) return true; }
  else { if(dist<stepsize) return true; }
  }
  return false;
}

Single_RRT_Star_PathFinder::Single_RRT_Star_PathFinder(ConfigurationProblem& _P, const arr& _starts, const arr& _goals, double _stepsize, int _subsampleChecks, int _maxIters, int _verbose)
  : P(_P),
    stepsize(_stepsize),
    maxIters(_maxIters),
    verbose(_verbose),
    subsampleChecks(_subsampleChecks) {

  std::cout << "entered Single_RRT_Star_PathFinder" << std::endl;
  if(stepsize<0.) stepsize = rai::getParameter<double>("rrt/stepsize", .1);
  if(subsampleChecks<0) subsampleChecks = rai::getParameter<int>("rrt/subsamples", 4);
  if(maxIters<0) maxIters = rai::getParameter<int>("rrt/maxIters", 5000);
  if(verbose<0) verbose = rai::getParameter<int>("rrt/verbose", 0);

  arr q0 = _starts;
  arr qT = _goals;
  auto q0ret = P.query(q0);
  auto qTret = P.query(qT);
  if(!q0ret->isFeasible) { LOG(0) <<"initializing with infeasible q0:"; q0ret->writeDetails(std::cout, P); }
  if(!qTret->isFeasible) { LOG(0) <<"initializing with infeasible qT:"; qTret->writeDetails(std::cout, P); }
  rrt0 = make_shared<RRT_SingleTree>(q0, q0ret);
  rrtT = make_shared<RRT_SingleTree>(qT, qTret);

  if(verbose>2) {
    DISP.copy(P.C);
  }
}

void Single_RRT_Star_PathFinder::planForward(const arr& q0, const arr& qT) {
  // std::cout << "entered planForward" << std::endl;
  bool success=false;

  for(uint i=0; i<100000; i++) {
    iters++;
    //let rrt0 grow
    bool added = growTreeTowardsRandom(*rrt0);
    if(added) {
      if(length(rrt0->getLast() - qT)<stepsize) success = true;
    }
    // if(success) {std::cout <<"success" <<std::endl;}//break;

    //some output
    if(verbose>2) {
      if(!(i%100)) {
        DISP.setJointState(rrt0->getLast());
        DISP.view(false);
        std::cout <<"RRT samples=" <<i <<" tree size = " <<rrt0->getNumberNodes() <<std::endl;
      }
    }
  }

  if(!success) return;

  if(verbose>0) {
    std::cout <<"SUCCESS!"
              <<"\n  tested samples=" <<P.evals
              <<"\n  #tree-size=" <<rrt0->getNumberNodes()
              << std::endl;
  }

  arr path = rrt0->getPathFromNode(rrt0->nearestID);
  revertPath(path);

  //display
  if(verbose>1) {
    std::cout << "path-length= " << path.d0 <<std::endl;
    DISP.proxies.clear();

    for(uint t=0; t<path.d0; t++) {
      DISP.setJointState(path[t]);
      //DISP.view();
      DISP.view(false);
      rai::wait(.1);
    }
  }

  path >>FILE("z.path");
}

int Single_RRT_Star_PathFinder::stepConnect() {
  iters++;
  if(iters>(uint)maxIters) return foundPath; //-1

  bool success = growTreeToTree(*rrt0, *rrtT);

  if(success) {
    p_forwardStep = 0.0;
    path = rrt0->getPathFromNode(rrt0->nearestID);
    arr pathT = rrtT->getPathFromNode(rrtT->nearestID);
  
    revertPath(path);
    path.append(pathT);
    foundPath = 1;
    return 0; //1 //0 to continue normally
  }

  return 0;
}

arr Single_RRT_Star_PathFinder::planConnect() {
  int r=0;
  while(!r) { r = stepConnect(); }
  if(r==1) { std::cout <<"done and now I will add stuff"<<std::endl;}
  if(r==-1) path.clear();
  return path;
}

arr Single_RRT_Star_PathFinder::run(double timeBudget) {
  planConnect();
  return path;
}


////////////////////////////////////////////////////////

bool PSBI_RRT_PathFinder::growTreeTowardsRandom(RRT_SingleTree& rrt) {
  std::cout << "entered growTreeTowardsRandom" << std::endl;
  const arr start = rrt.ann.X[0]; // Starting point, first node in the tree
  arr t(rrt.getDim()); // t is the target, dimension matches the tree's dimension
  rndUniform(t, -RAI_2PI, RAI_2PI, false); // Uniform random target within bounds

  arr q = rrt.getProposalTowards(t, stepsize); // Propose new node toward target

  auto qr = P.query(q); // Query if the new point is feasible
  if (qr->isFeasible) {
    if (subsampleChecks > 0 && !checkConnection(P, start, q, subsampleChecks, true)) {
      return false; // Connection check failed
    }

    uint parentID = rrt.nearestID; // Nearest node's index
    rrt.add(q, parentID, qr); // Add new node to the tree

    // Rewiring: search for neighbors and potentially rewire them
    double radius = stepsize; // Set a rewiring radius
    arr neighbors = rrt.getNeighbors(q, radius); // Get neighbors of the new node
    for (uint i = 0; i < neighbors.N; ++i) {
      uint neighborID = neighbors(i);
      arr neighborNode = rrt.getNode(neighborID);

      // Recompute the cost and check if rewiring is beneficial
      double currentCost = rrt.getNearest(q) + length(neighborNode - q); // Use length instead of distance
      if (currentCost < rrt.getNearest(neighborNode) + length(neighborNode - q)) {
        // If rewiring improves the path, update the parent
        rrt.add(q, neighborID, qr);
      }
    }
    return true;
  }
  return false;
}

bool PSBI_RRT_PathFinder::growTreeToTree(RRT_SingleTree& rrt_A, RRT_SingleTree& rrt_B) {
  bool isSideStep, isForwardStep;
  //decide on a target: forward or random
  arr t;
  // std::cout << "entered growTreeToTree" << std::endl;
  if(rnd.uni()<p_forwardStep) {
    t = rrt_B.getRandomNode();
    isForwardStep = true;
  } else {
    t.resize(rrt_A.getNode(0).N);
    for(uint i=0; i<t.N; i++) {
      double lo=P.limits(0, i), up=P.limits(1, i);
      CHECK_GE(up-lo, 1e-3, "limits are null interval: " <<i <<' ' <<P.C.getJointNames());
      t.elem(i) = lo + rnd.uni()*(up-lo);
    }
    isForwardStep = false;
  }

  //sample configuration towards target, possibly sideStepping
  arr q = rrt_A.getNewSample(t, stepsize, p_sideStep, isSideStep, 0);
  uint parentID = rrt_A.nearestID;

  //special rule: if parent is already in collision, isFeasible = smaller collisions
  shared_ptr<QueryResult>& pr = rrt_A.queries(parentID);
  double org_collisionTolerance = P.collisionTolerance;
  if(pr->totalCollision>P.collisionTolerance) P.collisionTolerance = pr->totalCollision + 1e-6;

  //evaluate the sample
  auto qr = P.query(q);
  if(isForwardStep) {  n_forwardStep++; if(qr->isFeasible) n_forwardStepGood++; }
  if(!isForwardStep) {  n_rndStep++; if(qr->isFeasible) n_rndStepGood++; }
  if(isSideStep) {  n_sideStep++; if(qr->isFeasible) n_sideStepGood++; }

  //if infeasible, make a backward step from the sample configuration
  if(!qr->isFeasible && p_backwardStep>0. && rnd.uni()<p_backwardStep) {
    t = q + qr->getBackwardStep();
    q = rrt_A.getNewSample(t, stepsize, p_sideStep, isSideStep, 0);
    qr = P.query(q);
    n_backStep++; if(qr->isFeasible) n_backStepGood++;
    if(isSideStep) {  n_sideStep++; if(qr->isFeasible) n_sideStepGood++; }
  };

  //checking subsamples
  if(qr->isFeasible && subsampleChecks>0) {
    const arr start = rrt_A.ann.X[parentID];
    qr->isFeasible = checkConnection(P, start, q, subsampleChecks, true);
  }

  P.collisionTolerance = org_collisionTolerance;

  //finally adding the new node to the tree
  double radius = stepsize;
  if (qr->isFeasible) {
  uint newNodeID = rrt_A.add(q, parentID, qr); // Add the new node with its initial parent
  arr newNode = rrt_A.getNode(newNodeID);
  // std::cout << "adding " << newNodeID << std::endl;
  arr neighbors = rrt_A.getNeighbors(q, radius); // Get neighbors within the specified radius
  // std::cout << "neighbors are " << neighbors << std::endl;
  // find the parent from neighbors
  for (uint i = 0; i < neighbors.N; ++i) {
    uint neighborID = neighbors(i);
    arr neighborNode = rrt_A.getNode(neighborID);
    // std::cout << "neighborID " << neighborID << " neighborNode " << neighborNode << std::endl;
    if (parentID == neighborID) {
      // std::cout << "Invalid parentID: " << parentID << std::endl;
    } else {
      // arr parent = rrt_A.getNode(parentID);
      // std::cout << "parentID: " << parentID << std::endl;
    //   // Compute the cost of the current path and the potential rewired path
      double newPathCost = rrt_A.getCost(neighborID) + length(neighborNode - newNode);
      double currentPathCost = rrt_A.getCost(newNodeID);
      // std::cout << "newPathCost is " << newPathCost << "currentPathCost is " << currentPathCost << std::endl;
      if (newPathCost < currentPathCost) {
        // std::cout << "found shorter path" << std::endl;
        // Rewire the neighbor if the new path is better
        rrt_A.changeParent(newNodeID, neighborID, newPathCost);
      }
    }
  }
  // if possible, update neighbors' parents to new node
  for (uint i = 0; i < neighbors.N; ++i) {
    uint neighborID = neighbors(i);
    arr neighborNode = rrt_A.getNode(neighborID);
    // std::cout << "neighborID " << neighborID << " neighborNode " << neighborNode << std::endl;
    if (parentID == neighborID) {
      // std::cout << "Invalid parentID: " << parentID << std::endl;
    } else {
      // arr parent = rrt_A.getNode(parentID);
      // std::cout << "parentID: " << parentID << std::endl;
    //   // Compute the cost of the current path and the potential rewired path
      double newPathCost = rrt_A.getCost(newNodeID) + length(neighborNode - newNode);
      double currentPathCost = rrt_A.getCost(neighborID);
      // std::cout << "newPathCost is " << newPathCost << "currentPathCost is " << currentPathCost << std::endl;
      if (newPathCost < currentPathCost) {
        // std::cout << "found shorter path" << std::endl;
        // Rewire the neighbor if the new path is better
        rrt_A.changeParent(neighborID, newNodeID, newPathCost);
      }
    }
  }
  double dist = rrt_B.getNearest(q);
  // std::cout << "dist: " << dist << " stepsize/subsampleChecks: " << stepsize/subsampleChecks << std::endl;
  if(subsampleChecks>0) { if(dist<stepsize/subsampleChecks) return true; }
  else { if(dist<stepsize) return true; }
  }
  return false;
}

PSBI_RRT_PathFinder::PSBI_RRT_PathFinder(ConfigurationProblem& _P, const arr& _starts, const arr& _goals, double _stepsize, int _subsampleChecks, int _maxIters, int _verbose)
  : P(_P),
    stepsize(_stepsize),
    maxIters(_maxIters),
    verbose(_verbose),
    subsampleChecks(_subsampleChecks) {

  std::cout << "entered PSBI_RRT_PathFinder" << std::endl;
  if(stepsize<0.) stepsize = rai::getParameter<double>("rrt/stepsize", .1);
  if(subsampleChecks<0) subsampleChecks = rai::getParameter<int>("rrt/subsamples", 4);
  if(maxIters<0) maxIters = rai::getParameter<int>("rrt/maxIters", 5000);
  if(verbose<0) verbose = rai::getParameter<int>("rrt/verbose", 0);

  arr q0 = _starts;
  arr qT = _goals;
  auto q0ret = P.query(q0);
  auto qTret = P.query(qT);
  if(!q0ret->isFeasible) { LOG(0) <<"initializing with infeasible q0:"; q0ret->writeDetails(std::cout, P); }
  if(!qTret->isFeasible) { LOG(0) <<"initializing with infeasible qT:"; qTret->writeDetails(std::cout, P); }
  rrt0 = make_shared<RRT_SingleTree>(q0, q0ret);
  rrtT = make_shared<RRT_SingleTree>(qT, qTret);

  if(verbose>2) {
    DISP.copy(P.C);
  }
}

void PSBI_RRT_PathFinder::planForward(const arr& q0, const arr& qT) {
  // std::cout << "entered planForward" << std::endl;
  bool success=false;

  for(uint i=0; i<100000; i++) {
    iters++;
    //let rrt0 grow
    bool added = growTreeTowardsRandom(*rrt0);
    if(added) {
      if(length(rrt0->getLast() - qT)<stepsize) success = true;
    }
    // if(success) {std::cout <<"success" <<std::endl;}//break;

    //some output
    if(verbose>2) {
      if(!(i%100)) {
        DISP.setJointState(rrt0->getLast());
        DISP.view(false);
        std::cout <<"RRT samples=" <<i <<" tree size = " <<rrt0->getNumberNodes() <<std::endl;
      }
    }
  }

  if(!success) return;

  if(verbose>0) {
    std::cout <<"SUCCESS!"
              <<"\n  tested samples=" <<P.evals
              <<"\n  #tree-size=" <<rrt0->getNumberNodes()
              << std::endl;
  }

  arr path = rrt0->getPathFromNode(rrt0->nearestID);
  revertPath(path);

  //display
  if(verbose>1) {
    std::cout << "path-length= " << path.d0 <<std::endl;
    DISP.proxies.clear();

    for(uint t=0; t<path.d0; t++) {
      DISP.setJointState(path[t]);
      //DISP.view();
      DISP.view(false);
      rai::wait(.1);
    }
  }

  path >>FILE("z.path");
}

int PSBI_RRT_PathFinder::stepConnect() {
  iters++;
  if(iters>(uint)maxIters) return foundPath; //-1

  bool success = growTreeToTree(*rrt0, *rrtT);

  if(success) {
    p_forwardStep = 0.0;
    std::cout << "execution completed" << std::endl;
    if(verbose>0) {
      std::cout <<"  -- rrt success:";
      std::cout <<" queries:" <<P.evals <<" tree sizes: " <<rrt0->getNumberNodes()  <<' ' <<rrtT->getNumberNodes() <<std::endl;
    }
    std::cout << "before path" << std::endl;
    path = rrt0->getPathFromNode(rrt0->nearestID);
    std::cout << "after path" << std::endl;
    arr pathT = rrtT->getPathFromNode(rrtT->nearestID);
    std::cout << "after pathT" << std::endl;

    revertPath(path);
    path.append(pathT);

    //display
    if(verbose>1) {
      std::cout <<"  path-length=" <<path.d0 <<std::endl;
      if(verbose>2) {
        DISP.proxies.clear();
        for(uint t=0; t<path.d0; t++) {
          DISP.setJointState(path[t]);
          DISP.view(false, STRING("rrt result "<<t));
          rai::wait(.1);
        }
        DISP.view(verbose>3);
        DISP.clear();
      }
    }
    std::cout << "viable path found" << std::endl;
    foundPath = 1;
    return 0; //1 //0 to continue normally
  }

  return 0;
}

arr PSBI_RRT_PathFinder::planConnect() {
  std::cout << "entered planConnect" << std::endl;
  int r=0;
  while(!r) { r = stepConnect(); }
  if(r==1) { std::cout <<"done and now I will add stuff"<<std::endl;}
  if(r==-1) path.clear();
  return path;
}

arr PSBI_RRT_PathFinder::run(double timeBudget) {
  std::cout << "entered run" << std::endl;
  planConnect();
  return path;
}

