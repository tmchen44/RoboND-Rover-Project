import numpy as np
from perception import rock_thresh

# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Variables to be used in making decisions
    # stopping distance conditional var
    within_stop_dist = Rover.mean_dist <= Rover.stop_forward
    # navigable terrain conditional var
    navigable_terr = len(Rover.nav_dists) > 0
    # conditional vars, if obstacle is in immediate front of Rover
    front_bumper = Rover.vision_image[142:148, 155:164, 0]
    obstacle_in_way = np.count_nonzero(front_bumper) >= 10
    front_left = Rover.vision_image[138:148, 152:159, 0]
    obstacle_left = np.count_nonzero(front_bumper) > 1
    front_right = Rover.vision_image[138:148, 160:167, 0]
    obstacle_right = np.count_nonzero(front_bumper) > 1
    # conditional var, if obstacle is up ahead and to the right
    extended_front_far = Rover.vision_image[130:142, 159:170, 0]
    extended_front_near = Rover.vision_image[142:147, 159:165, 0]
    front_clear = (np.count_nonzero(extended_front_far) == 0 and
                    np.count_nonzero(extended_front_near) == 0)
    # conditional var, if Rover has a good angle to come out of stop mode
    good_angle = Rover.mean_ang < 0.2 and Rover.mean_ang > -0.2

    # Use warped top-down map to detect rock
    rock_nearby = False
    rock_aligned = False
    rock_map = Rover.vision_image[:,:,1]
    if np.count_nonzero(rock_map) > 3:
        row_dists = np.nonzero(rock_map)[0] - 159
        col_dists = np.nonzero(rock_map)[1] - 159.5
        dists = np.sqrt(row_dists**2 + col_dists**2)
        avg_x = np.mean(row_dists[dists <= 35])
        avg_y = np.mean(col_dists[dists <= 35])
        if avg_x < 0 and avg_y >= -5:
            rock_nearby = True
    if Rover.picking_up != 0:
        rock_nearby = False
    # Use camera image to align Rover to rock after rock detection
    rock_cam = rock_thresh(Rover.img)
    rock_center = np.mean(np.nonzero(rock_cam)[1])
    if rock_center >= 158 and rock_center <= 161:
        rock_aligned = True

    # if no vision data, turn to failsafe mode
    if len(Rover.nav_angles) == 0:
        Rover.mode == 'failsafe'
    # Check for Rover.mode status
    if Rover.mode == 'forward':
        if rock_nearby:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
            Rover.mode = 'init_rock_stop'
        # Check the extent of navigable terrain
        elif (not within_stop_dist and navigable_terr
            and not obstacle_in_way):
            # If mode is forward, navigable terrain looks good
            # and velocity is below max, then throttle
            Rover.brake = 0
            if Rover.vel < Rover.max_vel:
                # Set throttle value to throttle setting
                Rover.throttle = Rover.throttle_set
            else: # Else coast
                Rover.throttle = 0
            # Set steering to average angle clipped to the range +/- 15
            if obstacle_left or obstacle_right:
                Rover.steer = np.clip(Rover.local_mean_ang * 180/np.pi, -15, 15)
                if Rover.vel > 0.5:
                    Rover.brake = 1
                else:
                    Rover.brake = 0
                    Rover.throttle = 0.1
            elif front_clear and Rover.mean_ang >= -0.1:
                Rover.steer = -8
            else:
                Rover.steer = np.clip(Rover.mean_ang * 180/np.pi, -15, 15)
        # If there's a lack of navigable terrain pixels then go to 'stop' mode
        else:
            # Set mode to "stop" and hit the brakes!
            Rover.throttle = 0
            # Set brake to stored brake value
            Rover.brake = Rover.brake_set
            Rover.steer = 0
            Rover.mode = 'stop'
    # If we're already in "stop" mode, then make different decisions
    elif Rover.mode == 'stop':
        # If we're in stop mode but still moving keep braking
        if Rover.near_sample:
            Rover.brake = 1
        elif Rover.vel > 0.0:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
        # If we're not moving (vel < 0.2) then do something else
        else:
            # Now we're stopped and we have vision data to see if there's a path forward
            if (Rover.mean_dist < Rover.go_forward or not navigable_terr
                or obstacle_in_way or not good_angle):
                Rover.throttle = 0
                # Release the brake to allow turning
                Rover.brake = 0
                # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                Rover.steer = 7 # Could be more clever here about which way to turn
            # If we're stopped but see sufficient navigable terrain in front then go!
            else:
                # Set throttle back to stored value
                Rover.throttle = Rover.throttle_set
                # Release the brake
                Rover.brake = 0
                # Set steer to mean angle
                Rover.steer = np.clip(Rover.mean_ang * 180/np.pi, -10, 10)
                Rover.mode = 'forward'
    # Attempt to recover rock
    elif Rover.mode == 'init_rock_stop':
        if Rover.vel > 0.0:
            Rover.throttle = 0
            Rover.brake = 1
        elif rock_center == None:
            Rover.brake = 0
            Rover.steer = -2
            Rover.throttle = 0
        elif not rock_aligned:
            Rover.brake = 0
            if rock_center <= 159:
                Rover.steer = 2
            elif rock_center >= 160:
                Rover.steer = -2
        else:
            Rover.steer = 0
            Rover.brake = 0
            Rover.mode = 'rock_crawl'
    elif Rover.mode == 'rock_crawl':
        if rock_center == []:
            Rover.steer = 0
            Rover.brake = 1
            Rover.mode = 'init_rock_stop'
        elif Rover.near_sample:
            Rover.throttle = 0
            Rover.brake = 1
            Rover.mode = 'stop'
        elif Rover.vel > 0.5:
            Rover.brake = 0
            Rover.throttle = 0
            if rock_aligned:
                Rover.steer = 0
            elif rock_center <= 159:
                Rover.steer = 2
            elif rock_center >= 160:
                Rover.steer = -2
        elif Rover.vel <= 0.5:
            Rover.brake = 0
            Rover.throttle = 0.5
            if rock_aligned:
                Rover.steer = 0
            elif rock_center <= 159:
                Rover.steer = 2
            elif rock_center >= 160:
                Rover.steer = -2
    # Just to make the rover do something
    # even if no modifications have been made to the code
    elif Rover.mode == 'failsafe':
        if Rover.vel > 0.0:
            Rover.brake = Rover.brake_set
        elif Rover.vel == 0:
            Rover.throttle = 0
            Rover.steer = 5
            Rover.brake = 0
            # exit failsafe if vision data is present
            if len(Rover.nav_angles) > 0:
                Rover.mode == 'stop'

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover
