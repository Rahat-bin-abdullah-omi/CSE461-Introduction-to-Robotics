"""line_follower controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motor_list = ['front_right_wheel', 'rear_right_wheel',
              'front_left_wheel', 'rear_left_wheel']
sensor_list = ['ir_right', 'ir_left', 'ir_ext_right', 'ir_ext_left',
               'ir_right_turn', 'ir_wall']

motor = dict()
sensor = dict()

for m in motor_list:
    motor[m] = robot.getDevice(m)
    motor[m].setPosition(float('inf'))
    motor[m].setVelocity(0.0)

for s in sensor_list:
    sensor[s] = robot.getDevice(s)
    sensor[s].enable(timestep)
    
def SetMotorSpeed(r_speed, l_speed):
    motor['front_right_wheel'].setVelocity(r_speed)
    motor['rear_right_wheel'].setVelocity(r_speed)
    motor['front_left_wheel'].setVelocity(l_speed)
    motor['rear_left_wheel'].setVelocity(l_speed)

while robot.step(timestep) != -1:
    right_value = sensor['ir_right'].getValue()
    left_value = sensor['ir_left'].getValue()
    right_ext_value = sensor['ir_ext_right'].getValue()
    left_ext_value = sensor['ir_ext_left'].getValue()
    right_turn_value = sensor['ir_right_turn'].getValue()
    wall_value = sensor['ir_wall'].getValue()

    print("ir_right \n", right_value)
    print("ir_left \n", left_value)
    print("ir_ext_right \n", right_ext_value)
    print("ir_ext_left \n", left_ext_value)
    print("ir_right_turn \n", right_turn_value)
    print("ir_wall \n", wall_value)
    
    if right_value < 900 and left_value > 900:
        SetMotorSpeed(5, 0)
    elif right_value > 900 and left_value < 900:
        SetMotorSpeed(0, 5)
    elif right_turn_value > 900:
        SetMotorSpeed(0, 5)
    elif 900 < wall_value < 990:
        SetMotorSpeed(0, 5)
    elif right_value < 850 and left_value < 900 and wall_value == 1000:
        SetMotorSpeed(0, 0)
    else:
        SetMotorSpeed(5, 5)
    pass

# Enter here exit cleanup code.
