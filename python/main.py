# System Includes
import math
import sys

import pygame

from simulation import Simulation, SimulationParams
from car import MotionCommandStraight, MotionCommandTurnTo, MotionCommandMoveTo
from display import Display, Vector2

# Screen dimension constants
SCREEN_WIDTH = 1024
SCREEN_HEIGHT = 768
GRID_SIZE = 500
GRID_SPACEING = 25


# Main Loop
def main():
    mDisplay = Display()
    mSimulation = Simulation()

    # Start Graphics
    try:
        pygame.display.init()
    except pygame.error as e:
        print("SDL could not initialize! SDL_Error: " + str(e))
        return -1
    try:
        pygame.font.init()
    except pygame.error as e:
        print("SDL_ttf could not initialize! SDL_ttf Error: " + str(e))
        return -1

    # Create Display
    if not mDisplay.createRenderer("AKFSF Simulations", SCREEN_WIDTH, SCREEN_HEIGHT):
        return False

    # Main Simulation Loop
    mSimulation.reset(loadSimulation1Parameters())
    # mSimulation.setTimeMultiplier(10)
    mRunning = True
    while mRunning:
        # Update Simulation
        mSimulation.update()

        # Update Display
        mDisplay.clearScreen()

        # Draw Background Grid
        mDisplay.setDrawColour(101, 101, 101)
        for x in range(-GRID_SIZE, GRID_SIZE + 1, GRID_SPACEING):
            mDisplay.drawLine(Vector2(x, -GRID_SIZE), Vector2(x, GRID_SIZE))
        for y in range(-GRID_SIZE, GRID_SIZE + 1, GRID_SPACEING):
            mDisplay.drawLine(Vector2(-GRID_SIZE, y), Vector2(GRID_SIZE, y))

        # Draw Simulation
        mSimulation.render(mDisplay)

        mDisplay.showScreen()

        # Handle Events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                mRunning = False
            elif event.type == pygame.KEYDOWN:
                key = event.key
                if key == pygame.K_SPACE: mSimulation.togglePauseSimulation()
                elif key == pygame.K_ESCAPE: mRunning = False
                elif key == pygame.K_KP_PLUS: mSimulation.increaseZoom()
                elif key == pygame.K_KP_MINUS: mSimulation.decreaseZoom()
                elif key == pygame.K_RIGHTBRACKET: mSimulation.increaseTimeMultiplier()
                elif key == pygame.K_LEFTBRACKET: mSimulation.decreaseTimeMultiplier()
                elif key == pygame.K_r: mSimulation.reset()
                elif key == pygame.K_1: mSimulation.reset(loadSimulation1Parameters())
                elif key == pygame.K_2: mSimulation.reset(loadSimulation2Parameters())
                elif key == pygame.K_3: mSimulation.reset(loadSimulation3Parameters())
                elif key == pygame.K_4: mSimulation.reset(loadSimulation4Parameters())
                elif key == pygame.K_5: mSimulation.reset(loadSimulation5Parameters())
                elif key == pygame.K_6: mSimulation.reset(loadSimulation6Parameters())
                elif key == pygame.K_7: mSimulation.reset(loadSimulation7Parameters())
                elif key == pygame.K_8: mSimulation.reset(loadSimulation8Parameters())
                elif key == pygame.K_9: mSimulation.reset(loadSimulation9Parameters())
                elif key == pygame.K_0: mSimulation.reset(loadSimulation0Parameters())

    # Destroy Renderer
    mDisplay.destroyRenderer()

    # Unload SDL
    pygame.font.quit()
    pygame.quit()

    return 0


def loadSimulation1Parameters():
    sim_params = SimulationParams()
    sim_params.profile_name = "1 - Constant Velocity + GPS + GYRO + Zero Initial Conditions"
    sim_params.car_initial_velocity = 5
    sim_params.car_initial_psi = math.pi / 180.0 * 45.0
    sim_params.car_commands.append(MotionCommandMoveTo(500, 500, 5))
    return sim_params


def loadSimulation2Parameters():
    sim_params = SimulationParams()
    sim_params.profile_name = "2 - Constant Velocity + GPS + GYRO + Non-zero Initial Conditions"
    sim_params.car_initial_x = 500
    sim_params.car_initial_y = 500
    sim_params.car_initial_velocity = 5
    sim_params.car_initial_psi = math.pi / 180.0 * -135.0
    sim_params.car_commands.append(MotionCommandMoveTo(0, 0, 5))
    return sim_params


def loadSimulation3Parameters():
    sim_params = SimulationParams()
    sim_params.profile_name = "3 - Constant Speed Profile + GPS + GYRO"
    sim_params.car_initial_velocity = 5
    sim_params.car_initial_psi = math.pi / 180.0 * 45.0
    sim_params.car_commands.append(MotionCommandMoveTo(100, 100, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(100, -100, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(0, 100, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(0, 0, 5))
    return sim_params


def loadSimulation4Parameters():
    sim_params = SimulationParams()
    sim_params.profile_name = "4 - Variable Speed Profile + GPS + GYRO"
    sim_params.end_time = 200
    sim_params.car_initial_velocity = 0
    sim_params.car_initial_psi = math.pi / 180.0 * 45.0
    sim_params.car_commands.append(MotionCommandMoveTo(100, 100, 2))
    sim_params.car_commands.append(MotionCommandMoveTo(100, -100, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(0, 100, 7))
    sim_params.car_commands.append(MotionCommandMoveTo(0, 0, 2))
    return sim_params


def loadSimulation5Parameters():
    sim_params = loadSimulation1Parameters()
    sim_params.profile_name = "5 - Constant Velocity + GPS + GYRO + LIDAR+ Zero Initial Conditions"
    sim_params.lidar_enabled = True
    return sim_params


def loadSimulation6Parameters():
    sim_params = loadSimulation2Parameters()
    sim_params.profile_name = "6 - Constant Velocity + GPS + GYRO + LIDAR + Non-zero Initial Conditions"
    sim_params.lidar_enabled = True
    return sim_params


def loadSimulation7Parameters():
    sim_params = loadSimulation3Parameters()
    sim_params.profile_name = "7 - Constant Speed Profile + GPS + GYRO + LIDAR"
    sim_params.lidar_enabled = True
    return sim_params


def loadSimulation8Parameters():
    sim_params = loadSimulation4Parameters()
    sim_params.profile_name = "8 - Variable Speed Profile + GPS + GYRO + LIDAR"
    sim_params.lidar_enabled = True
    return sim_params


def loadSimulation9Parameters():
    sim_params = SimulationParams()
    sim_params.profile_name = "9 - CAPSTONE"
    sim_params.gyro_enabled = True
    sim_params.lidar_enabled = True
    sim_params.end_time = 500
    sim_params.car_initial_x = 400
    sim_params.car_initial_y = -400
    sim_params.car_initial_velocity = 0
    sim_params.car_initial_psi = math.pi / 180.0 * -90.0
    sim_params.gps_error_probability = 0.05
    sim_params.gps_denied_x = 250.0
    sim_params.gps_denied_y = -250.0
    sim_params.gps_denied_range = 100.0
    sim_params.gyro_bias = -3.1 / 180.0 * math.pi
    sim_params.car_commands.append(MotionCommandStraight(3, -2))
    sim_params.car_commands.append(MotionCommandTurnTo(math.pi / 180.0 * 90.0, -2))
    sim_params.car_commands.append(MotionCommandMoveTo(400, -300, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(350, -300, 2))
    sim_params.car_commands.append(MotionCommandMoveTo(300, -250, 7))
    sim_params.car_commands.append(MotionCommandMoveTo(300, -300, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(250, -250, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(250, -300, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(200, -250, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(200, -300, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(200, -150, 2))
    sim_params.car_commands.append(MotionCommandMoveTo(100, -100, -2))
    sim_params.car_commands.append(MotionCommandMoveTo(200, 0, 7))
    sim_params.car_commands.append(MotionCommandMoveTo(300, -100, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(300, -300, 7))
    sim_params.car_commands.append(MotionCommandMoveTo(400, -300, 3))
    sim_params.car_commands.append(MotionCommandMoveTo(400, -400, 1))
    return sim_params


def loadSimulation0Parameters():
    sim_params = loadSimulation9Parameters()
    sim_params.profile_name = "0 - CAPSTONE BONUS (with No Lidar Data Association)"
    sim_params.lidar_id_enabled = False
    return sim_params


if __name__ == "__main__":
    sys.exit(main())
