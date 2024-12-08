import robotic as ry
import time
ry.params_clear()
ry.params_add({'rrt/stepsize':.1, 'rrt/verbose': 3}) #verbose=3 makes it very slow, and displays result, and verbose=4 waits keypress..

rrt = ry.PathFinder()
rrt.helloworld()

# Create a new configuration
# Create a new configuration
C = ry.Config()

C.addFile("/home/monke/Xrai/maze.g")

# Add the moving "ego" frame with constraints on its motion
C.addFrame("ego", "floorwalls") \
    .setJoint(ry.JT.transXYPhi, [-1., 1., -1., 1., -3., 3.]) \
    .setRelativePosition([0.2, 0.2, 0.4]) \
    .setShape(ry.ST.ssBox, size=[0.05, 0.3, 0.05, 0.01]) \
    .setColor([0, 1., 1.]) \
    .setContact(1)

C.addFrame("goal1") \
    .setPosition(C.getFrame('ego').getPosition()+[0, -3, 0]) \
    .setShape(ry.ST.ssBox, size=[0.05, 0.3, 0.05, 0.01]) \
    .setColor([1, 0, 0]) \

C.addFrame("goal2") \
    .setPosition(C.getFrame('goal1').getPosition()+[4, 0, 0]) \
    .setShape(ry.ST.ssBox, size=[0.05, 0.3, 0.05, 0.01]) \
    .setColor([1, 0, 0]) \

# C.view()
q0 = [0.2, 0.2, 0.4]
qT = C.getFrame('goal1').getPosition()

ry.params_clear()
ry.params_add({'rrt/stepsize':.1, 'rrt/verbose': 0,'rrt/maxIters':500}) #verbose=3 makes it very slow, and displays result, and verbose=4 waits keypress..

rrt = ry.PathFinder()
rrt.setProblem(C, [q0], [qT])
ret = rrt.star_solve()
print(ret)
path = ret.x

# Visualize the solution path
if ret.feasible:
    for i in range(len(path)):
        C.addFrame(f"path_{i}").setPosition(path[i]).setShape(ry.ST.sphere, [0.02])  # Add a sphere at each path point
        C.view()
else:
    print("No valid path found.")