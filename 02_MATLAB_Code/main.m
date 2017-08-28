%%
robot = ReConBot
robot.topic = '/reconbot_trajectory'
robot.jointNames = {'joint_4' 'joint_5' 'joint_3' 'joint_1' 'joint_2' 'joint_6' 'mode'}
robot.pos =data
robot.trajPoints = robot.transform()
robot.tj = robot.buildTrajectory()