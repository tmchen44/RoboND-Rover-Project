import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Opposite of color_thresh, identifies terrain that cannot be navigated,
# i.e. obstacles
def obs_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    obs_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be equal to or below any three threshold
    # values in RGB below_thresh will now contain a boolean array with
    # "True" where threshold was not met
    below_thresh = ((img[:,:,0] <= rgb_thresh[0]) \
                | (img[:,:,1] <= rgb_thresh[1]) \
                | (img[:,:,2] <= rgb_thresh[2])) \
                & ((img[:,:,0] > 0) | (img[:,:,1] > 0) \
                | (img[:,:,2] > 0))
    # Index the array of zeros with the boolean array and set to 1
    obs_select[below_thresh] = 1
    # Return the binary image
    return obs_select

def rock_thresh(img):
    # Convert RGB to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # define range of yellow color in HSV
    lower = np.array([22, 150, 150])
    upper = np.array([28, 255, 255])

    # Threshold the HSV image to get only yellow colors
    thresh = cv2.inRange(hsv, lower, upper)
    mask = thresh > 0
    thresh[mask] = 1
    return thresh

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))

    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image

    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO:
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    image = Rover.img
    dst_size = 5
    bottom_offset = 9
    source = np.float32([[15, 140], [301 ,140],[200, 96], [119, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset - 1],
                  [image.shape[1]/2 + dst_size - 1, image.shape[0] - bottom_offset - 1],
                  [image.shape[1]/2 + dst_size - 1, image.shape[0] - 2*dst_size - bottom_offset],
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    warped = perspect_transform(image, source, destination)
    warped[152:,:,:] = 0
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    nav_binary = color_thresh(warped)
    obs_binary = obs_thresh(warped)
    rock_binary = rock_thresh(warped)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obs_binary * 255
    Rover.vision_image[:,:,1] = rock_binary * 255
    Rover.vision_image[:,:,2] = nav_binary * 255
    # 5) Convert map image pixel values to rover-centric coords
    nav_x_rov, nav_y_rov = rover_coords(nav_binary)
    obs_x_rov, obs_y_rov = rover_coords(obs_binary)
    rock_x_rov, rock_y_rov = rover_coords(rock_binary)
    # 6) Convert rover-centric pixel values to world coordinates
    nav_x_world, nav_y_world = pix_to_world(nav_x_rov, nav_y_rov, Rover.pos[0],
                                            Rover.pos[1], Rover.yaw, 200, 10)
    obs_x_world, obs_y_world = pix_to_world(obs_x_rov, obs_y_rov, Rover.pos[0],
                                            Rover.pos[1], Rover.yaw, 200, 10)
    rock_x_world, rock_y_world = pix_to_world(rock_x_rov, rock_y_rov, Rover.pos[0],
                                            Rover.pos[1], Rover.yaw, 200, 10)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    roll = Rover.roll
    pitch = Rover.pitch
    if (roll < 0.5 or roll > 359.5) and (pitch < 0.5 or pitch > 359.5):
        Rover.worldmap[obs_y_world, obs_x_world, 0] += 1
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[nav_y_world, nav_x_world, 2] += 1
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    dists, angles = to_polar_coords(nav_x_rov, nav_y_rov)
    Rover.nav_dists, Rover.nav_angles = dists, angles
    Rover.local_mean_ang = np.mean(angles[dists < 20])
    Rover.mean_dist, Rover.mean_ang = np.mean(dists), np.mean(angles)

    return Rover
