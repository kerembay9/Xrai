import robotic as ry
import json

C = ry.Config()

C.addFile("/home/monke/Xrai/maze.g")

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

q0 = [0.2, 0.2, 0.4]
qT = C.getFrame('goal1').getPosition()

ry.params_clear()
ry.params_add({'rrt/stepsize':0.1, 'rrt/verbose': 0,'rrt/maxIters':100}) #verbose=3 makes it very slow, and displays result, and verbose=4 waits keypress..

rrt = ry.PathFinder()
rrt.setProblem(C, [q0], [qT])
# ret = rrt.solve()
# single_ret = rrt.single_star_solve()
star_ret = rrt.psbi_solve()
# psbi_ret = rrt.psbi_solve()
# print(ret)
# print(single_ret)
print(star_ret)
# print(psbi_ret)
# path = ret.x
# print(path)
# with open('path_data.json', 'w') as f:
#     json.dump(path.tolist(), f)  # Convert numpy array to list for JSON serialization
print("Path saved to 'path_data.json'.")
# # Visualize the solution path
# if ret.feasible:
#     for i in range(len(path)):
#         C.addFrame(f"path_{i}").setPosition(path[i]).setShape(ry.ST.sphere, [0.02])  # Add a sphere at each path point
#         C.view()
# else:
#     print("No valid path found.")