import math

from kalmanfilter import KalmanFilter
from display import Vector2, string_format, offsetPoints
from car import Car, VehicleState
from beacons import BeaconMap
from sensors import GyroSensor, GPSSensor, LidarSensor
from utils import wrapAngle, calculateRMSE, generateEllipse, generateCircle


def _cout_double(value):
    # Default std::cout formatting of a double (precision 6, %g style)
    return "%g" % value


class SimulationParams:
    def __init__(self):
        self.profile_name = ""
        self.time_step = 0.1
        self.end_time = 120

        self.gps_enabled = True
        self.gps_update_rate = 1.0
        self.gps_position_noise_std = 3
        self.gps_error_probability = 0.0
        self.gps_denied_x = 0.0
        self.gps_denied_y = 0.0
        self.gps_denied_range = -1.0

        self.lidar_enabled = False
        self.lidar_id_enabled = True
        self.lidar_update_rate = 10.0
        self.lidar_range_noise_std = 3
        self.lidar_theta_noise_std = 0.02

        self.gyro_enabled = True
        self.gyro_update_rate = 10.0
        self.gyro_noise_std = 0.001
        self.gyro_bias = 0.0

        self.car_initial_x = 0.0
        self.car_initial_y = 0.0
        self.car_initial_psi = 0.0
        self.car_initial_velocity = 5.0

        self.car_commands = []


class Simulation:

    def __init__(self):
        self.m_sim_parameters = SimulationParams()
        self.m_kalman_filter = KalmanFilter()
        self.m_car = Car()
        self.m_beacons = BeaconMap()
        self.m_gyro_sensor = GyroSensor()
        self.m_gps_sensor = GPSSensor()
        self.m_lidar_sensor = LidarSensor()

        self.m_is_paused = False
        self.m_is_running = False
        self.m_time_multiplier = 1
        self.m_view_size = 100

        self.m_time = 0.0
        self.m_time_till_gyro_measurement = 0.0
        self.m_time_till_gps_measurement = 0.0
        self.m_time_till_lidar_measurement = 0.0

        self.m_gps_measurement_history = []
        self.m_lidar_measurement_history = []

        self.m_vehicle_position_history = []
        self.m_filter_position_history = []

        self.m_filter_error_x_position_history = []
        self.m_filter_error_y_position_history = []
        self.m_filter_error_heading_history = []
        self.m_filter_error_velocity_history = []

    def reset(self, sim_params=None):
        if sim_params is not None:
            self.m_sim_parameters = sim_params

        # Reset Simulation
        self.m_time = 0.0
        self.m_time_till_gyro_measurement = 0.0
        self.m_time_till_gps_measurement = 0.0
        self.m_time_till_lidar_measurement = 0.0

        self.m_is_running = True
        self.m_is_paused = False

        self.m_kalman_filter.reset()

        self.m_gps_sensor.reset()
        self.m_gps_sensor.setGPSNoiseStd(self.m_sim_parameters.gps_position_noise_std)
        self.m_gps_sensor.setGPSErrorProb(self.m_sim_parameters.gps_error_probability)
        self.m_gps_sensor.setGPSDeniedZone(self.m_sim_parameters.gps_denied_x, self.m_sim_parameters.gps_denied_y, self.m_sim_parameters.gps_denied_range)

        self.m_gyro_sensor.reset()
        self.m_gyro_sensor.setGyroNoiseStd(self.m_sim_parameters.gyro_noise_std)
        self.m_gyro_sensor.setGyroBias(self.m_sim_parameters.gyro_bias)

        self.m_lidar_sensor.reset()
        self.m_lidar_sensor.setLidarNoiseStd(self.m_sim_parameters.lidar_range_noise_std, self.m_sim_parameters.lidar_theta_noise_std)
        self.m_lidar_sensor.setLidarDAEnabled(self.m_sim_parameters.lidar_id_enabled)

        self.m_car.reset(self.m_sim_parameters.car_initial_x, self.m_sim_parameters.car_initial_y, self.m_sim_parameters.car_initial_psi, self.m_sim_parameters.car_initial_velocity)

        for cmd in self.m_sim_parameters.car_commands:
            self.m_car.addVehicleCommand(cmd)

        # Plotting Variables
        self.m_gps_measurement_history.clear()
        self.m_lidar_measurement_history.clear()
        self.m_vehicle_position_history.clear()
        self.m_filter_position_history.clear()

        # Stats Variables
        self.m_filter_error_x_position_history.clear()
        self.m_filter_error_y_position_history.clear()
        self.m_filter_error_heading_history.clear()
        self.m_filter_error_velocity_history.clear()

        print("Simulation: Reset")

    def update(self):
        if self.m_is_running and not self.m_is_paused:
            # Time Multiplier
            for i in range(self.m_time_multiplier):
                # Check for End Time
                if self.m_time >= self.m_sim_parameters.end_time:
                    self.m_is_running = False
                    print("Simulation: Reached End of Simulation Time (" + _cout_double(self.m_time) + ")")
                    return

                # Update Motion
                self.m_car.update(self.m_time, self.m_sim_parameters.time_step)
                self.m_vehicle_position_history.append(Vector2(self.m_car.getVehicleState().x, self.m_car.getVehicleState().y))

                # Gyro Measurement / Prediction Step
                if self.m_sim_parameters.gyro_enabled:
                    if self.m_time_till_gyro_measurement <= 0:
                        meas = self.m_gyro_sensor.generateGyroMeasurement(self.m_car.getVehicleState().yaw_rate)
                        self.m_kalman_filter.predictionStep(meas, self.m_sim_parameters.time_step)
                        self.m_time_till_gyro_measurement += 1.0 / self.m_sim_parameters.gyro_update_rate
                    self.m_time_till_gyro_measurement -= self.m_sim_parameters.time_step

                # GPS Measurement
                if self.m_sim_parameters.gps_enabled:
                    if self.m_time_till_gps_measurement <= 0:
                        gps_meas = self.m_gps_sensor.generateGPSMeasurement(self.m_car.getVehicleState().x, self.m_car.getVehicleState().y)
                        self.m_kalman_filter.handleGPSMeasurement(gps_meas)
                        self.m_gps_measurement_history.append(gps_meas)
                        self.m_time_till_gps_measurement += 1.0 / self.m_sim_parameters.gps_update_rate
                    self.m_time_till_gps_measurement -= self.m_sim_parameters.time_step

                # Lidar Measurement
                if self.m_sim_parameters.lidar_enabled:
                    if self.m_time_till_lidar_measurement <= 0:
                        lidar_measurements = self.m_lidar_sensor.generateLidarMeasurements(self.m_car.getVehicleState().x, self.m_car.getVehicleState().y, self.m_car.getVehicleState().psi, self.m_beacons)
                        self.m_kalman_filter.handleLidarMeasurements(lidar_measurements, self.m_beacons)
                        self.m_lidar_measurement_history = lidar_measurements
                        self.m_time_till_lidar_measurement += 1.0 / self.m_sim_parameters.lidar_update_rate
                    self.m_time_till_lidar_measurement -= self.m_sim_parameters.time_step

                # Save Filter History and Calculate Stats
                if self.m_kalman_filter.isInitialised():
                    vehicle_state = self.m_car.getVehicleState()
                    filter_state = self.m_kalman_filter.getVehicleState()
                    self.m_filter_position_history.append(Vector2(filter_state.x, filter_state.y))
                    self.m_filter_error_x_position_history.append(filter_state.x - vehicle_state.x)
                    self.m_filter_error_y_position_history.append(filter_state.y - vehicle_state.y)
                    self.m_filter_error_heading_history.append(wrapAngle(filter_state.psi - vehicle_state.psi))
                    self.m_filter_error_velocity_history.append(filter_state.V - vehicle_state.V)

                # Update Time
                self.m_time += self.m_sim_parameters.time_step

    def render(self, disp):
        marker_lines1 = [Vector2(0.5, 0.5), Vector2(-0.5, -0.5)]
        marker_lines2 = [Vector2(0.5, -0.5), Vector2(-0.5, 0.5)]

        disp.setView(self.m_view_size * disp.getScreenAspectRatio(), self.m_view_size, self.m_car.getVehicleState().x, self.m_car.getVehicleState().y)

        self.m_car.render(disp)
        self.m_beacons.render(disp)

        disp.setDrawColour(0, 100, 0)
        disp.drawLines(self.m_vehicle_position_history)

        disp.setDrawColour(100, 0, 0)
        disp.drawLines(self.m_filter_position_history)

        if self.m_kalman_filter.isInitialised():
            filter_state = self.m_kalman_filter.getVehicleState()
            cov = self.m_kalman_filter.getVehicleStatePositionCovariance()

            x = filter_state.x
            y = filter_state.y
            sigma_xx = cov[0, 0]
            sigma_yy = cov[1, 1]
            sigma_xy = cov[0, 1]

            marker_lines1_world = offsetPoints(marker_lines1, Vector2(x, y))
            marker_lines2_world = offsetPoints(marker_lines2, Vector2(x, y))
            disp.setDrawColour(255, 0, 0)
            disp.drawLines(marker_lines1_world)
            disp.drawLines(marker_lines2_world)

            cov_world = generateEllipse(x, y, sigma_xx, sigma_yy, sigma_xy)
            disp.setDrawColour(255, 0, 0)
            disp.drawLines(cov_world)

        # Render GPS Measurements
        m_gps_marker = [[Vector2(0.5, 0.5), Vector2(-0.5, -0.5)], [Vector2(0.5, -0.5), Vector2(-0.5, 0.5)]]
        disp.setDrawColour(255, 255, 255)
        for meas in self.m_gps_measurement_history:
            disp.drawLines(offsetPoints(m_gps_marker, Vector2(meas.x, meas.y)))

        # Render GPS Denied Zone
        if self.m_sim_parameters.gps_denied_range > 0:
            zone_lines = generateCircle(self.m_sim_parameters.gps_denied_x, self.m_sim_parameters.gps_denied_y, self.m_sim_parameters.gps_denied_range)
            disp.setDrawColour(255, 150, 0)
            disp.drawLines(zone_lines)

        # Render Lidar Measurements
        for meas in self.m_lidar_measurement_history:
            x0 = self.m_car.getVehicleState().x
            y0 = self.m_car.getVehicleState().y
            delta_x = meas.range * math.cos(meas.theta + self.m_car.getVehicleState().psi)
            delta_y = meas.range * math.sin(meas.theta + self.m_car.getVehicleState().psi)
            disp.setDrawColour(201, 201, 0)
            disp.drawLine(Vector2(x0, y0), Vector2(x0 + delta_x, y0 + delta_y))

        stride = 20
        # Simulation Status / Parameters
        x_offset = 10
        y_offset = 30
        time_string = string_format("Time: %0.2f (x%d)", self.m_time, self.m_time_multiplier)
        profile_string = string_format("Profile: %s", self.m_sim_parameters.profile_name)
        gps_string = string_format("GPS: %s (%0.1f Hz)", ("ON" if self.m_sim_parameters.gps_enabled else "OFF"), self.m_sim_parameters.gps_update_rate)
        lidar_string = string_format("LIDAR: %s (%0.1f Hz)", ("ON" if self.m_sim_parameters.lidar_enabled else "OFF"), self.m_sim_parameters.lidar_update_rate)
        gyro_string = string_format("GYRO: %s (%0.1f Hz)", ("ON" if self.m_sim_parameters.gyro_enabled else "OFF"), self.m_sim_parameters.gyro_update_rate)
        disp.drawText_MainFont(profile_string, Vector2(x_offset, y_offset + stride * -1), 1.0, (255, 255, 255))
        disp.drawText_MainFont(time_string, Vector2(x_offset, y_offset + stride * 0), 1.0, (255, 255, 255))
        disp.drawText_MainFont(gps_string, Vector2(x_offset, y_offset + stride * 1), 1.0, (255, 255, 255))
        disp.drawText_MainFont(lidar_string, Vector2(x_offset, y_offset + stride * 2), 1.0, (255, 255, 255))
        disp.drawText_MainFont(gyro_string, Vector2(x_offset, y_offset + stride * 3), 1.0, (255, 255, 255))
        if self.m_is_paused:
            disp.drawText_MainFont("PAUSED", Vector2(x_offset, y_offset + stride * 4), 1.0, (255, 0, 0))
        if not self.m_is_running:
            disp.drawText_MainFont("FINISHED", Vector2(x_offset, y_offset + stride * 5), 1.0, (255, 0, 0))

        # Vehicle State
        x_offset = 800
        y_offset = 10
        velocity_string = string_format("    Velocity: %0.2f m/s", self.m_car.getVehicleState().V)
        yaw_string = string_format("   Heading: %0.2f deg", self.m_car.getVehicleState().psi * 180.0 / math.pi)
        xpos = string_format("X Position: %0.2f m", self.m_car.getVehicleState().x)
        ypos = string_format("Y Position: %0.2f m", self.m_car.getVehicleState().y)
        disp.drawText_MainFont("Vehicle State", Vector2(x_offset - 5, y_offset + stride * 0), 1.0, (255, 255, 255))
        disp.drawText_MainFont(velocity_string, Vector2(x_offset, y_offset + stride * 1), 1.0, (255, 255, 255))
        disp.drawText_MainFont(yaw_string, Vector2(x_offset, y_offset + stride * 2), 1.0, (255, 255, 255))
        disp.drawText_MainFont(xpos, Vector2(x_offset, y_offset + stride * 3), 1.0, (255, 255, 255))
        disp.drawText_MainFont(ypos, Vector2(x_offset, y_offset + stride * 4), 1.0, (255, 255, 255))

        kf_velocity_string = string_format("    Velocity: %0.2f m/s", self.m_kalman_filter.getVehicleState().V)
        kf_yaw_string = string_format("   Heading: %0.2f deg", self.m_kalman_filter.getVehicleState().psi * 180.0 / math.pi)
        kf_xpos = string_format("X Position: %0.2f m", self.m_kalman_filter.getVehicleState().x)
        kf_ypos = string_format("Y Position: %0.2f m", self.m_kalman_filter.getVehicleState().y)
        disp.drawText_MainFont("Filter State", Vector2(x_offset, y_offset + stride * 6), 1.0, (255, 255, 255))
        disp.drawText_MainFont(kf_velocity_string, Vector2(x_offset, y_offset + stride * 7), 1.0, (255, 255, 255))
        disp.drawText_MainFont(kf_yaw_string, Vector2(x_offset, y_offset + stride * 8), 1.0, (255, 255, 255))
        disp.drawText_MainFont(kf_xpos, Vector2(x_offset, y_offset + stride * 9), 1.0, (255, 255, 255))
        disp.drawText_MainFont(kf_ypos, Vector2(x_offset, y_offset + stride * 10), 1.0, (255, 255, 255))

        # Keyboard Input
        x_offset = 10
        y_offset = 650
        disp.drawText_MainFont("Reset Key: r", Vector2(x_offset, y_offset + stride * 0), 1.0, (255, 255, 255))
        disp.drawText_MainFont("Pause Key: [space bar]", Vector2(x_offset, y_offset + stride * 1), 1.0, (255, 255, 255))
        disp.drawText_MainFont("Speed Multiplier (+/-) Key: [ / ] ", Vector2(x_offset, y_offset + stride * 2), 1.0, (255, 255, 255))
        disp.drawText_MainFont("Zoom (+/-) Key: + / - (keypad)", Vector2(x_offset, y_offset + stride * 3), 1.0, (255, 255, 255))
        disp.drawText_MainFont("Motion Profile Key: 1 - 9,0", Vector2(x_offset, y_offset + stride * 4), 1.0, (255, 255, 255))

        # Filter Error State
        x_offset = 750
        y_offset = 650
        xpos_error_string = string_format("X Position RMSE: %0.2f m", calculateRMSE(self.m_filter_error_x_position_history))
        ypos_error_string = string_format("Y Position RMSE: %0.2f m", calculateRMSE(self.m_filter_error_y_position_history))
        heading_error_string = string_format("   Heading RMSE: %0.2f deg", 180.0 / math.pi * calculateRMSE(self.m_filter_error_heading_history))
        velocity_error_string = string_format("    Velocity RMSE: %0.2f m/s", calculateRMSE(self.m_filter_error_velocity_history))
        disp.drawText_MainFont(xpos_error_string, Vector2(x_offset, y_offset + stride * 0), 1.0, (255, 255, 255))
        disp.drawText_MainFont(ypos_error_string, Vector2(x_offset, y_offset + stride * 1), 1.0, (255, 255, 255))
        disp.drawText_MainFont(heading_error_string, Vector2(x_offset, y_offset + stride * 2), 1.0, (255, 255, 255))
        disp.drawText_MainFont(velocity_error_string, Vector2(x_offset, y_offset + stride * 3), 1.0, (255, 255, 255))

    def increaseTimeMultiplier(self):
        self.m_time_multiplier += 1
        print("Simulation: Time Multiplier Increased (x" + str(self.m_time_multiplier) + ")")

    def decreaseTimeMultiplier(self):
        if self.m_time_multiplier > 1:
            self.m_time_multiplier -= 1
            print("Simulation: Time Multiplier Decreased (x" + str(self.m_time_multiplier) + ")")

    def setTimeMultiplier(self, multiplier):
        self.m_time_multiplier = int(multiplier)

    def increaseZoom(self):
        if self.m_view_size > 25:
            self.m_view_size -= 25
        print("Simulation: Zoom Increased (" + _cout_double(self.m_view_size) + "m)")

    def decreaseZoom(self):
        if self.m_view_size < 400:
            self.m_view_size += 25
        print("Simulation: Zoom Decreased (" + _cout_double(self.m_view_size) + "m)")

    def togglePauseSimulation(self):
        self.m_is_paused = not self.m_is_paused
        print("Simulation: Paused (" + ("True" if self.m_is_paused else "False") + ")")

    def isPaused(self):
        return self.m_is_paused

    def isRunning(self):
        return self.m_is_running
