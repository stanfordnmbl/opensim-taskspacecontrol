from opensim import Model, Body, PlanarJoint, PinJoint, Vec3, \
        DisplayGeometry, Transform, CoordinateActuator

from numpy import pi

linkage = Model()
linkage.setName('fourlinks')


# Bodies.
# -------
link1 = Body()
link1.setName('link1')
link1.setMass(1.0)
link1.setMassCenter(Vec3(1.0, 0, 0))

link2 = Body()
link2.setName('link2')
link2.setMass(1.0)
link2.setMassCenter(Vec3(0.5, 0, 0))

link3 = Body()
link3.setName('link3')
link3.setMass(1.0)
link3.setMassCenter(Vec3(0.5, 0, 0))

link4 = Body()
link4.setName('link4')
link4.setMass(1.0)
link4.setMassCenter(Vec3(0.2, 0, 0))


# Joints.
# -------
joint1 = PinJoint('joint1',
        linkage.getGroundBody(), Vec3(0, 0, 0), Vec3(-0.5 * pi, 0, 0),
        link1, Vec3(0, 0, 0), Vec3(-0.5 * pi, 0, 0))
cset1 = joint1.upd_CoordinateSet()
cset1.get(0).setName('q1')

joint2 = PinJoint('joint2',
        link1, Vec3(0, 1.0, 0), Vec3(0, 0, 0),
        link2, Vec3(0, 0, 0), Vec3(0, 0, 0))
cset2 = joint2.upd_CoordinateSet()
cset2.get(0).setName('q2')

joint3 = PinJoint('joint3',
        link2, Vec3(0.5, 0, 0), Vec3(0, 0, 0),
        link3, Vec3(0, 0, 0), Vec3(0, 0, 0))
cset3 = joint3.upd_CoordinateSet()
cset3.get(0).setName('q3')

joint4 = PinJoint('joint4',
        link3, Vec3(0.5, 0, 0), Vec3(0, 0, 0),
        link4, Vec3(0, 0, 0), Vec3(0, 0, 0))
cset4 = joint4.upd_CoordinateSet()
cset4.get(0).setName('q4')


# Display geometry.
# -----------------
dispground = DisplayGeometry("box.vtp")
dispground.setScaleFactors(Vec3(1.0, 0.01, 1.0))
linkage.getGroundBody().updDisplayer().updGeometrySet().cloneAndAppend(
        dispground)

displink1 = DisplayGeometry("box.vtp")
displink1.setScaleFactors(Vec3(0.1, 1.0, 0.1))
displink1.setTransform(Transform(Vec3(0, 0.5, 0)))

link1.updDisplayer().updGeometrySet().cloneAndAppend(displink1)

displink2 = DisplayGeometry("box.vtp")
displink2.setScaleFactors(Vec3(0.5, 0.1, 0.1))
displink2.setTransform(Transform(Vec3(0.25, 0, 0)))
displink2.setColor(Vec3(1.0, 0.0, 0.0))

link2.updDisplayer().updGeometrySet().cloneAndAppend(displink2)

displink3 = DisplayGeometry("box.vtp")
displink3.setScaleFactors(Vec3(0.5, 0.1, 0.1))
displink3.setTransform(Transform(Vec3(0.25, 0, 0)))
displink3.setColor(Vec3(0.0, 1.0, 0.0))

link3.updDisplayer().updGeometrySet().cloneAndAppend(displink3)

displink4 = DisplayGeometry("box.vtp")
displink4.setScaleFactors(Vec3(0.2, 0.1, 0.1))
displink4.setTransform(Transform(Vec3(0.1, 0, 0)))
displink4.setColor(Vec3(0.0, 0.0, 1.0))

link4.updDisplayer().updGeometrySet().cloneAndAppend(displink4)

linkage.addBody(link1)
linkage.addBody(link2)
linkage.addBody(link3)
linkage.addBody(link4)


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

linkage.addForce(act1)
linkage.addForce(act2)
linkage.addForce(act3)
linkage.addForce(act4)

linkage.printToXML('fourlinks.osim')
