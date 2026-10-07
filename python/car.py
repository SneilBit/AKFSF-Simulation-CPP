import copy
import math
from collections import deque

from display import Vector2, transformPoints, offsetPoints
from utils import wrapAngle


def _signbit(value):
    return math.copysign(1.0, value) < 0


class VehicleState:
    def __init__(self, setX=0.0, setY=0.0, setPsi=0.0, setV=0.0, setPsiDot=0.0, setSteering=0.0):
        self.x = setX
        self.y = setY
        self.psi = setPsi
        self.V = setV
        self.yaw_rate = setPsiDot
        self.steering = setSteering


class MotionCommandBase:
    def __init__(self):
        self.m_velocity_command = 0.0
        self.m_steering_command = 0.0
        self.m_start_time = 0.0
        self.m_start_state = VehicleState()

    def startCommand(self, time, state):
        self.m_start_time = time
        self.m_start_state = copy.copy(state)

    def endCommand(self, time, dt, state):
        pass

    def update(self, time, dt, state):
        return False

    def getVelocityCommand(self):
        return self.m_velocity_command

    def getSteeringCommand(self):
        return self.m_steering_command


class MotionCommandStraight(MotionCommandBase):
    def __init__(self, command_time, command_velocity):
        super().__init__()
        self.m_command_time = command_time
        self.m_command_velocity = command_velocity

    def update(self, time, dt, state):
        self.m_velocity_command = self.m_command_velocity
        self.m_steering_command = 0.0
        return time > (self.m_start_time + self.m_command_time)


class MotionCommandTurnTo(MotionCommandBase):
    def __init__(self, command_heading, command_velocity):
        super().__init__()
        self.m_command_heading = command_heading
        self.m_command_velocity = command_velocity

    def update(self, time, dt, state):
        self.m_velocity_command = self.m_command_velocity
        angle_error = wrapAngle(self.m_command_heading - state.psi)
        self.m_steering_command = angle_error * (-1.0 if _signbit(state.V) else 1.0)
        return math.fabs(angle_error) < 0.001


class MotionCommandMoveTo(MotionCommandBase):
    def __init__(self, command_x, command_y, command_velocity):
        super().__init__()
        self.m_command_x = command_x
        self.m_command_y = command_y
        self.m_command_velocity = command_velocity

    def update(self, time, dt, state):
        self.m_velocity_command = self.m_command_velocity
        delta_x = self.m_command_x - state.x
        delta_y = self.m_command_y - state.y
        range = math.sqrt(delta_x * delta_x + delta_y * delta_y)
        angle_command = math.atan2(delta_y, delta_x)
        psi = wrapAngle(state.psi - (math.pi if _signbit(state.V) else 0.0))
        angle_error = wrapAngle(angle_command - psi)
        self.m_steering_command = angle_error * (-1.0 if _signbit(state.V) else 1.0)
        return (range < 5.0)


class BicycleMotion:

    def __init__(self, x0=0.0, y0=0.0, psi0=0.0, V0=0.0):
        self.m_initial_state = VehicleState(x0, y0, psi0, V0)
        self.m_wheel_base = 4.0
        self.m_max_velocity = 28.0
        self.m_max_acceleration = 2.0
        self.m_max_steering = 0.8
        self.reset()

    def reset(self, state=None):
        if state is not None:
            self.m_initial_state = copy.copy(state)
        self.m_current_state = copy.copy(self.m_initial_state)
        self.m_steering_command = self.m_initial_state.steering
        self.m_velocity_command = self.m_initial_state.V

    def update(self, dt):
        cosPsi = math.cos(self.m_current_state.psi)
        sinPsi = math.sin(self.m_current_state.psi)
        x = self.m_current_state.x + self.m_current_state.V * cosPsi * dt
        y = self.m_current_state.y + self.m_current_state.V * sinPsi * dt

        accel = self.m_velocity_command - self.m_current_state.V
        if accel > self.m_max_acceleration:
            accel = self.m_max_acceleration
        if accel < -self.m_max_acceleration:
            accel = -self.m_max_acceleration

        steer = self.m_steering_command
        if steer > self.m_max_steering:
            steer = self.m_max_steering
        if steer < -self.m_max_steering:
            steer = -self.m_max_steering

        vel = self.m_current_state.V + accel * dt
        if vel > self.m_max_velocity:
            vel = self.m_max_velocity
        if vel < -self.m_max_velocity:
            vel = -self.m_max_velocity

        psi_dot = self.m_current_state.V * steer / self.m_wheel_base
        psi = wrapAngle(self.m_current_state.psi + psi_dot * dt)
        self.m_current_state = VehicleState(x, y, psi, vel, psi_dot, steer)

    def setSteeringCmd(self, steer):
        self.m_steering_command = steer

    def setVelocityCmd(self, accel):
        self.m_velocity_command = accel

    def getVehicleState(self):
        return copy.copy(self.m_current_state)


class Car:

    def __init__(self):
        self.m_vehicle_model = BicycleMotion()
        self.m_current_command = None
        self.m_vehicle_commands = deque()

        # Create Display Geometry
        self.m_car_lines_body = [Vector2(2, -1), Vector2(2, 1), Vector2(-2, 1), Vector2(-2, -1), Vector2(2, -1)]
        self.m_marker_lines = [[Vector2(0.5, 0.5), Vector2(-0.5, -0.5)], [Vector2(0.5, -0.5), Vector2(-0.5, 0.5)], [Vector2(0, 0), Vector2(3.5, 0)]]
        self.m_wheel_lines = [Vector2(-0.6, 0.3), Vector2(0.6, 0.3), Vector2(0.6, -0.3), Vector2(-0.6, -0.3), Vector2(-0.6, 0.3)]
        self.m_wheel_fl_offset = Vector2(2, -1.6)
        self.m_wheel_fr_offset = Vector2(2, 1.6)
        self.m_wheel_rl_offset = Vector2(-2, -1.6)
        self.m_wheel_rr_offset = Vector2(-2, 1.6)

    def reset(self, x0, y0, psi0, V0):
        self.m_vehicle_model.reset(VehicleState(x0, y0, psi0, V0))
        self.m_vehicle_commands.clear()
        self.m_current_command = None

    def addVehicleCommand(self, cmd):
        if cmd is not None:
            self.m_vehicle_commands.append(cmd)

    def getVehicleState(self):
        return self.m_vehicle_model.getVehicleState()

    def update(self, time, dt):
        # Update Command
        if self.m_current_command is None and len(self.m_vehicle_commands) > 0:
            self.m_current_command = self.m_vehicle_commands.popleft()
            self.m_current_command.startCommand(time, self.m_vehicle_model.getVehicleState())

        # Run Command
        if self.m_current_command is not None:
            cmd_complete = self.m_current_command.update(time, dt, self.m_vehicle_model.getVehicleState())
            self.m_vehicle_model.setSteeringCmd(self.m_current_command.getSteeringCommand())
            self.m_vehicle_model.setVelocityCmd(self.m_current_command.getVelocityCommand())
            if cmd_complete:
                self.m_current_command = None
        else:
            self.m_vehicle_model.setSteeringCmd(0.0)
            self.m_vehicle_model.setVelocityCmd(0.0)

        # Update Vehicle
        self.m_vehicle_model.update(dt)

        return True

    def render(self, disp):
        steeringPsi = self.m_vehicle_model.getVehicleState().steering
        carPsiOffset = self.m_vehicle_model.getVehicleState().psi
        carPosOffset = Vector2(self.m_vehicle_model.getVehicleState().x, self.m_vehicle_model.getVehicleState().y)

        disp.setDrawColour(0, 255, 0)
        disp.drawLines(transformPoints(self.m_car_lines_body, carPosOffset, carPsiOffset))
        disp.drawLines(transformPoints(self.m_marker_lines, carPosOffset, carPsiOffset))

        disp.setDrawColour(0, 201, 0)
        disp.drawLines(transformPoints(transformPoints(self.m_wheel_lines, self.m_wheel_fl_offset, steeringPsi), carPosOffset, carPsiOffset))
        disp.drawLines(transformPoints(transformPoints(self.m_wheel_lines, self.m_wheel_fr_offset, steeringPsi), carPosOffset, carPsiOffset))
        disp.drawLines(transformPoints(offsetPoints(self.m_wheel_lines, self.m_wheel_rl_offset), carPosOffset, carPsiOffset))
        disp.drawLines(transformPoints(offsetPoints(self.m_wheel_lines, self.m_wheel_rr_offset), carPosOffset, carPsiOffset))
