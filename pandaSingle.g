world: {}

### box as base

box(world): {
 shape: ssBox, Q: "t(0 0 0.2)", size: [0.4, 0.5, 0.4, 0.02], color: [0.6, 0.3, 0.1],
 contact: 1, logical: { },
 friction: 0.1
}

### one panda

Prefix: "l_"
#Include: <panda_fixRobotiq.g>
Include: <panda_fixGripper.g>

Prefix: False

Edit l_panda_base (box): { Q: "t(0 .08 0.2) d(90 0 0 1)" joint:rigid }

Edit l_panda_joint2: { q: -.5 }
Edit l_panda_joint4: { q: -2 }
Edit l_panda_joint7: { q: -.5 }

### camera

cameraTop(world): {
 Q: "t(-0.01 -.2 1.8) d(-150 1 0 0)",
 shape: marker, size: [.1],
 focalLength: 0.895, width: 640, height: 360, zRange: [.5, 100]
}

cameraWrist(l_panda_joint7): {
 Q: [0.0566288, -0.0138618, 0.158583, 0.371288, -0.0124238, 0.0272688, -0.928034],
 shape: camera, size: [.1],
 focalLength: 0.895, width: 640, height: 360, zRange: [.1, 10]
}

panda_collCameraWrist(cameraWrist): {
 Q: "d(90 0 1 0) t(-.02 0 0)",
 shape: capsule, color: [1., 1., 1., 0.2], size: [.05, .03], contact: -3
}
