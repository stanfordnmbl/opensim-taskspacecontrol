from opensim import Model, Body, PlanarJoint, PinJoint, Vec3, \
        DisplayGeometry, Transform, CoordinateActuator

pi = 3.14159

linkage = Model()
linkage.setName('threelink')


# Bodies.
# -------
link1 = Body()
link1.setName('link1')
link1.setMass(1.0)
link1.setMassCenter(Vec3(1.0, 0, 0))

link2 = Body()
link2.setName('link2')
link2.setMass(1.0)
link2.setMassCenter(Vec3(1.0, 0, 0))

link3 = Body()
link3.setName('link3')
link3.setMass(1.0)
link3.setMassCenter(Vec3(1.0, 0, 0))


# Joints.
# -------
# 3-DOF joint: 1 rotation and 2 translations.
joint1 = PlanarJoint('joint1',
        linkage.getGroundBody(), Vec3(0, 0, 0), Vec3(-0.5 * pi, 0, 0),
        link1, Vec3(0, 0, 0), Vec3(0.5 * pi, 0, 0))
cset1 = joint1.upd_CoordinateSet()
cset1.get(0).setName('q1')
cset1.get(1).setName('q2')
cset1.get(2).setName('q3')

joint2 = PinJoint('joint2',
        link1, Vec3(1.0, 0, 0), Vec3(0, 0, 0),
        link2, Vec3(0, 0, 0), Vec3(0, 0, 0))
cset2 = joint2.upd_CoordinateSet()
cset2.get(0).setName('q4')

joint3 = PinJoint('joint3',
        link2, Vec3(1.0, 0, 0), Vec3(0, 0, 0),
        link3, Vec3(0, 0, 0), Vec3(0, 0, 0))
cset3 = joint3.upd_CoordinateSet()
cset3.get(0).setName('q5')


# Display geometry.
# -----------------
dispground = DisplayGeometry("box.vtp")
dispground.setScaleFactors(Vec3(1.0, 0.01, 1.0))
linkage.getGroundBody().updDisplayer().updGeometrySet().cloneAndAppend(dispground)
displink= DisplayGeometry("box.vtp")
displink.setScaleFactors(Vec3(1, 0.1, 0.1))

link1.updDisplayer().updGeometrySet().cloneAndAppend(displink)
link2.updDisplayer().updGeometrySet().cloneAndAppend(displink)
link3.updDisplayer().updGeometrySet().cloneAndAppend(displink)

linkage.addBody(link1)
linkage.addBody(link2)
linkage.addBody(link3)


# Actuation.
# ----------
act1 = CoordinateActuator('q1')
act1.setName('q1')
act2 = CoordinateActuator('q2')
act2.setName('q2')
act3 = CoordinateActuator('q3')
act3.setName('q3')
act4 = CoordinateActuator('q4')
act4.setName('q4')
act5 = CoordinateActuator('q5')
act5.setName('q5')

linkage.addForce(act1)
linkage.addForce(act2)
linkage.addForce(act3)
linkage.addForce(act4)
linkage.addForce(act5)

linkage.printToXML('threelinks.osim')
