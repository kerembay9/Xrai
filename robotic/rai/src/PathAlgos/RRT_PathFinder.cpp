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
  std::cout << "entered getPathFromNode" << std::endl;
  std::cout << "parent" << parent << " fromID " << fromID << std::endl;
  for(;;) {
    path.append(ann.X[node]);
    std::cout << "path append" << ann.X[node] << "node" << node << std::endl;
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
    std::cerr << "Error: Invalid nodeID or newParentID. nodeID=" << nodeID 
              << ", newParentID=" << newParentID 
              << ", parent size=" << parent.N << std::endl;
    return;
  }

  // Debug output
  std::cout << "Changing parent of node " << nodeID 
            << " to " << newParentID 
            << " with path cost " << newPathCost << std::endl;
  std::cout << "parent was " << parent << std::endl; 
  // Update parent and costs
  parent(nodeID) = newParentID;
  parent_cost(nodeID) = newPathCost;
  total_cost(nodeID) = total_cost(newParentID) + newPathCost;
  std::cout << "parent is " << parent << std::endl; 

  // Debug output to confirm changes
  std::cout << "Updated parent array: ";
  // for (size_t i = 0; i < parent.N; ++i) {
  //   std::cout << parent(i) << " ";
  // }
  std::cout << std::endl;
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
//      std::cout <<"  forwardSteps: " <<(100.*n_forwardStepGood/n_forwardStep) <<"%/" <<n_forwardStep;
//      std::cout <<"  backSteps: " <<(100.*n_backStepGood/n_backStep) <<"%/" <<n_backStep;
//      std::cout <<"  rndSteps: " <<(100.*n_rndStepGood/n_rndStep) <<"%/" <<n_rndStep;
//      std::cout <<"  sideSteps: " <<(100.*n_sideStepGood/n_sideStep) <<"%/" <<n_sideStep;
//      std::cout <<std::endl;
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
  std::cout << "entered growTreeToTree" << std::endl;
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
  double radius = 0.05;
  if (qr->isFeasible) {
  uint newNodeID = rrt_A.add(q, parentID, qr); // Add the new node with its initial parent
  arr newNode = rrt_A.getNode(newNodeID);
  std::cout << "adding " << newNodeID << std::endl;
  arr neighbors = rrt_A.getNeighbors(q, radius); // Get neighbors within the specified radius
  std::cout << "neighbors are " << neighbors << std::endl;
  // find the parent from neighbors
  for (uint i = 0; i < neighbors.N; ++i) {
    uint neighborID = neighbors(i);
    arr neighborNode = rrt_A.getNode(neighborID);
    std::cout << "neighborID " << neighborID << " neighborNode " << neighborNode << std::endl;
    if (parentID == neighborID) {
      std::cout << "Invalid parentID: " << parentID << std::endl;
    } else {
      // arr parent = rrt_A.getNode(parentID);
      std::cout << "parentID: " << parentID << std::endl;
    //   // Compute the cost of the current path and the potential rewired path
      double newPathCost = rrt_A.getCost(neighborID) + length(neighborNode - newNode);
      double currentPathCost = rrt_A.getCost(newNodeID);
      std::cout << "newPathCost is " << newPathCost << "currentPathCost is " << currentPathCost << std::endl;
      if (newPathCost < currentPathCost) {
        std::cout << "found shorter path" << std::endl;
        // Rewire the neighbor if the new path is better
        rrt_A.changeParent(newNodeID, neighborID, newPathCost);
      }
    }
  }
  // if possible, update neighbors' parents to new node
  for (uint i = 0; i < neighbors.N; ++i) {
    uint neighborID = neighbors(i);
    arr neighborNode = rrt_A.getNode(neighborID);
    std::cout << "neighborID " << neighborID << " neighborNode " << neighborNode << std::endl;
    if (parentID == neighborID) {
      std::cout << "Invalid parentID: " << parentID << std::endl;
    } else {
      // arr parent = rrt_A.getNode(parentID);
      std::cout << "parentID: " << parentID << std::endl;
    //   // Compute the cost of the current path and the potential rewired path
      double newPathCost = rrt_A.getCost(newNodeID) + length(neighborNode - newNode);
      double currentPathCost = rrt_A.getCost(neighborID);
      std::cout << "newPathCost is " << newPathCost << "currentPathCost is " << currentPathCost << std::endl;
      if (newPathCost < currentPathCost) {
        std::cout << "found shorter path" << std::endl;
        // Rewire the neighbor if the new path is better
        rrt_A.changeParent(neighborID, newNodeID, newPathCost);
      }
    }
  }
  double dist = rrt_B.getNearest(q);
  std::cout << "dist: " << dist << " stepsize/subsampleChecks: " << stepsize/subsampleChecks << std::endl;
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

  if(verbose>2) {
    DISP.copy(P.C);
  }
}

void RRT_Star_PathFinder::planForward(const arr& q0, const arr& qT) {
  std::cout << "entered planForward" << std::endl;
  bool success=false;

  for(uint i=0; i<100000; i++) {
    iters++;
    //let rrt0 grow
    bool added = growTreeTowardsRandom(*rrt0);
    if(added) {
      if(length(rrt0->getLast() - qT)<stepsize) success = true;
    }
    if(success) {std::cout <<"success" <<std::endl;}//break;

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
  // std::cout << "entered stepConnect" << std::endl;
  iters++;
  if(iters>(uint)maxIters) return foundPath; //-1

  bool success = growTreeToTree(*rrt0, *rrtT);
  // std::cout << "success line 1: " << success << std::endl;
  if(!success) success = growTreeToTree(*rrtT, *rrt0);


  //animation display
  // if(verbose>2) {
  //   if(!(iters%100)) {
  //     DISP.setJointState(rrt0->getLast());
  //     DISP.view(verbose>4, STRING("planConnect evals " <<P.evals));
  //   }
  // }
  // if(verbose>1) {
  //   if(!(iters%100)) {
  //     std::cout <<"RRT queries=" <<P.evals <<" tree sizes = " <<rrt0->getNumberNodes()  <<' ' <<rrtT->getNumberNodes() <<std::endl;
  //   }
  // }

  //-- the rest is only on success -> extract the path, display, etc

  if(success) {
    std::cout << "execution completed" << std::endl;
    if(verbose>0) {
      std::cout <<"  -- rrt success:";
      std::cout <<" queries:" <<P.evals <<" tree sizes: " <<rrt0->getNumberNodes()  <<' ' <<rrtT->getNumberNodes() <<std::endl;
//      std::cout <<"  forwardSteps: " <<(100.*n_forwardStepGood/n_forwardStep) <<"%/" <<n_forwardStep;
//      std::cout <<"  backSteps: " <<(100.*n_backStepGood/n_backStep) <<"%/" <<n_backStep;
//      std::cout <<"  rndSteps: " <<(100.*n_rndStepGood/n_rndStep) <<"%/" <<n_rndStep;
//      std::cout <<"  sideSteps: " <<(100.*n_sideStepGood/n_sideStep) <<"%/" <<n_sideStep;
//      std::cout <<std::endl;
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
    return 1; //1 //0 to continue normally
  }

  return 0;
}

arr RRT_Star_PathFinder::planConnect() {
  std::cout << "entered planConnect" << std::endl;
  int r=0;
  while(!r) { r = stepConnect(); }
  if(r==1) { std::cout <<"done and now I will add stuff";}
  if(r==-1) path.clear();
  return path;
}

arr RRT_Star_PathFinder::run(double timeBudget) {
  std::cout << "entered run" << std::endl;
  planConnect();
  return path;
}