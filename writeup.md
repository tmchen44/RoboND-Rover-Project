# Project: Search and Sample Return

---

**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (metric defined in report) job of navigating and mapping.  

[//]: # (References)

[image01]: ./output/warped_nav.jpg
[image02]: ./output/warped_obs.jpg
[image03]: ./output/original_rock.jpg
[image04]: ./output/rock_iso.jpg
[image05]: ./output/warped_rock.jpg
[image06]: ./output/obstacle_in_air.jpg
[image07]: ./output/fork.jpg
[image08]: ./output/rock_in_middle.jpg
[notebook]: ./code/Rover_Project_Test_Notebook.ipynb
[movie]: ./output/test_mapping.mp4

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
#### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

Link here to my Jupyter Notebook: [./code/Rover\_Project\_Test\_Notebook.ipynb](./code/Rover\_Project\_Test\_Notebook.ipynb)

* The function for identifying navigable terrain is called `color_thresh` and can be found in the *Color Thresholding* section of the notebook. `color_thresh` identifies parts of an image that are *above* a specified color threshold. **Figure 1** below shows the resulting warped image (`example_grid1.jpg` in the `calibration_images` folder) of navigable terrain.
* The obstacle detection function `obs_thresh` can be found immediately below `color_thresh`. It is very similar to `color_thresh` and identifies parts of the image that are *below* the RGB threshold. `obs_thresh` also ignores the black "empty" parts of the warped perspective (which are not obstacles, but the space beyond the edges of the camera). **Figure 2** below shows the resulting warped image (`example_grid1.jpg` in the `calibration_images` folder) of obstacles.
* The rock detection function `rock thresh` can be found immediately below `obs_thresh`. It uses the HSV color system to identify yellow rock samples in the rover camera images. A couple of OpenCV functions are used to convert from RGB to HSV and to threshold the image for a range of HSV values. **Figure 3** shows the original rock sample image (last image of my own test dataset, located at `test_dataset_tc/IMG`). **Figure 4** shows the color thresholded image with the original camera perspective, and **Figure 5** shows the thresholded image with the warped perspective.

Figure 1:
![alt text][image01]

Figure 2:
![alt text][image02]

Figure 3:
![alt text][image03]

Figure 4:
![alt text][image04]

Figure 5:
![alt text][image05]

#### 2. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

1. To generate a top-down map for image analysis as well as a world map, I first defined the source and destination points for the perspective transform.
2. Then I apply the `perspect_transform` function to the camera image from the rover to get a warped, top-down view.
3. I apply the three color thresholds to identify navigable terrain, obstacles, and rock samples. These are stored as three binary images.
4. I then take these three images and convert them to rover-centric coordinates using the `rover_coords` function, which is defined in the *Coordinate Transformations* section of the notebook.
5. These rover-centric coordinates are converted to world coordinates using the `pix_to_world` function, which is also defined in the *Coordinate Transformations* section of the notebook.
6. If the Rover roll and pitch are within a certain range, then the worldmap is updated with the coordinates of the navigable terrain, obstacles, and rock samples.
7. A mosaic of several image series are stitched together to create a movie (link below). Starting from the top left and continuing clockwise:
   * The original camera image
   * The warped (top-down) perspective
   * The warped perspective with color thresholding applied (navigable terrain in blue, obstacles in red, rock samples in yellow)
   * The generated world map with the ground truth shaded back in green

Link to video:
[./output/test_mapping.mp4](./output/test_mapping.mp4)

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

The `perception_step()` function is organized very similarly to the `process_image()` function (see  the above explanation for more details). However, instead of stitching the resulting images into a movie, `perception_step()` passes these images as instance variables of the Rover class instance. `Rover.vision_image` contains the warped perspective camera image with color thresholding applied (red for obstacles, blue for navigable terrain, and yellow for rock samples). `Rover.worldmap` is updated with the generated worldmap. Also, variables containing the rover-centric polar distances and angles, with their respective means, are passed to the Rover class instance as well.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: For running simulations, I used a resolution of 960 x 720 on the 'Fastest' graphics quality setting. The frames per second output on the terminal was 15-16 fps.**

In order to navigate through the entire map and attempt to recover most of the rock samples, I took a wall-hugging approach for my autonomous mode.

   * While moving forward, the rover will lean right if the terrain ahead and slightly to the right is clear of obstacles (see lines 86-87 and 26-30 of `decision.py`).
   * If there is an obstacle (or wall) in this forward detection zone, then the rover just steers towards the mean angle of navigable terrain (see lines 91-92 and 21-23 of `decision.py`).
   * I also added an additional conditional clause to guide the rover rightwards in case of obstacles on the left (see lines 88-90 of `decision.py`).

For the most part, this strategy does an excellent job of going through most of the map. However, where there is a small branch off the main path, the current version of the strategy may not necessarily go down the small branch. This is a trade-off that I needed to make, due to limitations in obstacle detection.

In particular, there are large rocks in the middle of the map that have parts that jut into the air. These rocks cause the rover to become stuck when it tries to pass underneath. Why does the rover try to pass underneath? The perception techniques are only able to accurately range obstacles that go all the way to the ground. Because these obstacles are "above ground" per se, my perception functions cannot accurately show these obstacles on the map (see **Figure 6**). Hence, the rover thinks the path is clear when, in reality, the path is not clear. The rock usually then catches onto the rover arm and causes the rover to become stuck.

![alt text][image06]

Figure 6

In order to avoid becoming stuck against a rock, I extended the forward detection zone mentioned above (line 27) so as to help the rover avoid these larger rocks that stick out in the air. Now, the rover is able to avoid these large rocks; at the same time, however, instead of following small rightward branches off the main path, it tends to skip these small branches and continue on the main path (see **Figure 7**).

![alt text][image07]

Figure 7

If I were to work on this project further, I would definitely take the time to fine-tune the decision-making code to, on the one hand, avoid obstacles that are "above ground", and on the other hand, still take all legitimate paths or branches in the map.

The rover does have the ability to locate and pickup rock samples (see lines 129-169 of `decision.py`). If a rock sample is on the rover's right side and within a certain range, the rover will attempt to recover the sample. However, if the sample is not against a wall but in the middle of a large path (see **Figure 8**), the rover will skip it. This is due to the rover needing to stay on the right side of a path. If it were to attempt to pick up a rock on its left side, it would potentially turn back and skip the rest of the path, leaving that portion of the path unmapped. If I were to work more on this project, this issue would be alleviated by adding a left-side pickup routine, which would cause the rover to continue down its previous path after picking up a rock on its left side.

![alt text][image08]

Figure 8

After detecting a rock on the warped map (lines 43-55), the rover uses the camera image with the rock threshold applied (lines 57-60) to steer the rover in the correct direction.

After mapping over 95% of the map, when the rover passes within a certain distance of its starting point, it will return there and stop (lines 170-210, 34-40). Because some rocks are spawned in difficult to reach areas or are too difficult to detect, I decided to stop the rover after one complete pass through the map.

