import pybullet as p

physicsClient = p.connect(p.GUI)
p.setRealTimeSimulation(1, physicsClient)

p.setAdditionalSearchPath('./models/robotiq_85')
robotiq = p.loadURDF('robotiq_85.urdf')

# disable default control
for i in range(p.getNumJoints(robotiq)):
    p.setJointMotorControl2(robotiq,
                            jointIndex=i,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=0,
                            force=0
                            )

FINGER_JOINT = 4  # TODO: get value from the file itself.

#apply gear constraints idx -> mult
gear_multipliers = {6: -1, 8: 1, 9: 1, 11: -1, 13: 1}
for joint_id, mult in gear_multipliers.items():
    c = p.createConstraint(
        parentBodyUniqueId=robotiq,
        parentLinkIndex=FINGER_JOINT,
        childBodyUniqueId=robotiq,
        childLinkIndex=joint_id,
        jointType=p.JOINT_GEAR,
        jointAxis=[0, 1, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0]
    )
    p.changeConstraint(c, gearRatio=-mult, maxForce=100, erp=1)


if __name__ == '__main__':
    import time
    grasp = [0, 1]
    idx = 0
    while True:
        idx ^= 1
        p.setJointMotorControl2(
            robotiq, FINGER_JOINT, p.POSITION_CONTROL,
            targetPosition=float(grasp[idx]), force=0.1
        )
        time.sleep(1)
