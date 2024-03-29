# import the pygame module, so you can use it
import pygame
import math
import numpy as np

WHEEL_RADIUS = 1.625
WIDTH = 14
ZOOM_FACTOR = 5
ROBOT_CONSTANTS = (0.5, 0.1527, 0.143)

def rk_four(f, x, u, T):
    """
    Perform fourth-order Runge-Kutta numerical integration.

    The function to integrate is f(x, u, params), where the state variables are
    collected in the variable x, we assume a constant input vector u over time
    interval T > 0, and params is an array of the system's parameters.
    """
    k_1 = f(x, u)
    k_2 = f(x + T * k_1 / 2.0, u)
    k_3 = f(x + T * k_2 / 2.0, u)
    k_4 = f(x + T * k_3, u)
    x_new = x + T / 6.0 * (k_1 + 2.0 * k_2 + 2.0 * k_3 + k_4)
    return x_new

#x is the state vector with v_wheels appended to the end. u is the input vector with left and right voltage
def continuous_update(x, u):
    #create the state to put all the rates of change is
    x_dot = [x[3],x[4],x[5],0,0,0,0,0]
    left_accel = (u[0] - (np.sign(x[6]) * ROBOT_CONSTANTS[0] + x[6] * ROBOT_CONSTANTS[1])) / ROBOT_CONSTANTS[2]
    right_accel = (u[1] - (np.sign(x[7]) * ROBOT_CONSTANTS[0] + x[7] * ROBOT_CONSTANTS[1])) / ROBOT_CONSTANTS[2]
    #add accel to the wheels
    x_dot[6] = left_accel
    x_dot[7] = right_accel
    #calc accels
    robot_accel = (left_accel + right_accel) * WHEEL_RADIUS / 2
    robot_alpha = (right_accel - left_accel) * WHEEL_RADIUS / WIDTH
    #calc accel in global refernece frame from wheels
    x_dot[3] = (robot_accel * -math.sin(x[2]))
    x_dot[4] = (robot_accel * math.cos(x[2]))
    x_dot[5] = robot_alpha
    #accel in global refernce frame from preventing slipping added on
    slipping_mag = x[3] * math.cos(x[2]) + x[4] * math.sin(x[2])
    if np.abs(slipping_mag) > 0.2:
        #the force will be -kf * <cos(theta), sin(theta)> - 39.3701 is m/s^2 to in/s^2
        x_dot[3] -= 9.8 * 39.3701 * math.cos(x[2]) * np.sign(slipping_mag)
        x_dot[4] -= 9.8 * 39.3701 * math.sin(x[2]) * np.sign(slipping_mag)
    return np.array(x_dot)

def draw_rotated_rectangle(screen, color, center, size, angle):
    # Create a surface with dimensions of the rectangle
    surf = pygame.Surface(size, pygame.SRCALPHA)
    # Draw the rectangle on the surface
    pygame.draw.rect(surf, color, (0, 0, size[0], size[1]))
    # Rotate the surface
    rotated_surf = pygame.transform.rotate(surf, angle)
    # Get the rotated rectangle's dimensions and position
    rotated_rect = rotated_surf.get_rect(center=center)
    # Blit the rotated rectangle onto the screen
    screen.blit(rotated_surf, rotated_rect.topleft)

# define a main function
def main():
     
    # initialize the pygame module
    pygame.init()
     
    # create a surface on screen that has the size of 240 x 180
    screen = pygame.display.set_mode((800,600))
     
    # define a variable to control the main loop
    running = True
    
    #create a robot
    robot_state = np.array([-30,30,0,0,0,0])
    robot_wheels = np.array([0,0])
    robot_voltage = np.array([0,0])
    
    last_time = pygame.time.get_ticks()
     
    # main loop
    while running:
        #get current time
        current_time = pygame.time.get_ticks()
        elapsed = (current_time - last_time) / 1000
        if elapsed < 0.01:
            continue
        # event handling, gets all event from the event queue
        for event in pygame.event.get():
            # only do something if the event is of type QUIT
            if event.type == pygame.QUIT:
                # change the value to False, to exit the main loop
                running = False
        keys = pygame.key.get_pressed()
        robot_voltage = [0, 0]
        if keys[pygame.K_w]:
            robot_voltage = [6, 6]
        if keys[pygame.K_s]:
            robot_voltage = [-6, -6]
        if keys[pygame.K_a]:
            robot_voltage[0] -= 6
            robot_voltage[1] += 6
        if keys[pygame.K_d]:
            robot_voltage[0] += 6
            robot_voltage[1] -= 6
          
          
          
        
        screen.fill("white")        
        draw_rotated_rectangle(screen, "red", (-robot_state[0] * ZOOM_FACTOR, robot_state[1] * ZOOM_FACTOR), (14 * ZOOM_FACTOR, 15 * ZOOM_FACTOR), robot_state[2] * 180 / math.pi)
        
        
        
        # #calculate acceleration based on voltage
        # # print(robot_wheels)
        # left_accel = ((robot_voltage[0] - (np.sign(robot_wheels[0]) * ROBOT_CONSTANTS[0] + robot_wheels[0] * ROBOT_CONSTANTS[1])) / ROBOT_CONSTANTS[2])
        # right_accel = ((robot_voltage[1] - (np.sign(robot_wheels[1]) * ROBOT_CONSTANTS[0] + robot_wheels[1] * ROBOT_CONSTANTS[1])) / ROBOT_CONSTANTS[2])
        # #add accel to the wheels
        # robot_wheels[0] += left_accel * elapsed
        # robot_wheels[1] += right_accel * elapsed
        # #calc accels
        # robot_accel = (left_accel + right_accel) * WHEEL_RADIUS / 2
        # robot_alpha = (right_accel - left_accel) * WHEEL_RADIUS / WIDTH
        # #calculate state rate of change based on wheel accel
        # robot_state[3] += (robot_accel * -math.sin(robot_state[2])) * elapsed
        # robot_state[4] += (robot_accel * math.cos(robot_state[2])) * elapsed
        # robot_state[5] += robot_alpha * elapsed
        # #calculate state rate of change based on kinetic friction
        # #comes from dot product of v and perpendicular unit vector
        # slipping_mag = robot_state[3] * math.cos(robot_state[2]) + robot_state[4] * math.sin(robot_state[2])
        # if np.abs(slipping_mag) > 0.2:
        #   #the force will be -kf * <cos(theta), sin(theta)> - 39.3701 is m/s^2 to in/s^2
        #   robot_state[3] -= 9.8 * 39.3701 * math.cos(robot_state[2]) * elapsed * np.sign(slipping_mag)
        #   robot_state[4] -= 9.8 * 39.3701 * math.sin(robot_state[2]) * elapsed * np.sign(slipping_mag)
        # #update robot position
        # robot_state[0] += robot_state[3] * elapsed
        # robot_state[1] += robot_state[4] * elapsed
        # robot_state[2] += robot_state[5] * elapsed
        vec_for_integration = np.append(robot_state, robot_wheels)
        new_vec = rk_four(continuous_update, vec_for_integration, robot_voltage, elapsed)
        robot_state = new_vec[:6]
        robot_wheels = new_vec[6:]
        print(robot_state)
        
        pygame.display.flip()
        last_time = current_time
            
     
     
# run the main function only if this module is executed as the main script
# (if you import this as a module then nothing is executed)
if __name__=="__main__":
    # call the main function
    main()
    
    