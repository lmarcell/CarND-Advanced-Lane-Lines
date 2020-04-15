## Writeup Template

### You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./examples/undistort_output.jpg "Undistorted"
[image2]: ./test_images/test1.jpg "Road Transformed"
[image3]: ./examples/binary_combo_example.jpg "Binary Example"
[image4]: ./examples/warped_straight_lines.jpg "Warp Example"
[image5]: ./examples/color_fit_lines.png "Fit Visual"
[image6]: ./examples/example_output.jpg "Output"
[video1]: ./project_video_output.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The related code is located in `solution.py`. 
The whole chain is called from the function `ProcessImage`.

This file contains a class called Calibrator. The purpose of this class is to do the calibration, based on the given pictures under `camera_cal` directory.

The function `PrepareObjectPoints` is collecting the object points from a 9*6 sized standard chess board.

The function `PrepareImagePoints` is loading all images from the given directory and calling the function `FindImagePoints` which is searching for the corners on a calibration image and in case of successfuly detection it is adding the point to the list called `imgpoints` and addigng the point from the original chess board to the list `objpoints`

The `Calibrate` function is doing the exact camera calibration and storing the results of that.

All the previously mentioned functions are called only once during initialization.

Later on you can use the `Undistort` function to correct the distortions on any image.

![alt text][image1]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one:
![alt text][image2]

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

As next step I applied filters on the picture. It is all implemented in the class called `FilterApplicator` which provides functions to apply different filters (sobel, filter on multi-channeled or on gray pictures).
After a lot of tries I decided to use the following filters:
-Sobel filter on x axle, using tresholds of 20 and 100.
-Filter on S channel of the HLS implementation of the picture, with tresholds of 170 and 255.
-Filter on R (red) channel of the RGB implementation of the picture with tresholds of 220 and 255. This one was helpful at the detection of the yellow lane boundary.

![alt text][image3]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The perspective transform of the pictures is done in function 'PerspectiveTransform'.
For the persective transform I used the following source and destination points:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 577, 463      | 250, 0        | 
| 706, 463      | 900, 0        |
| 1066, 687     | 900, 720      |
| 242, 687      | 250, 720      |

I also implemented a function to in order to transform back the pictures, it is called 'PerspectiveTransformInverse'.

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image4]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

The identification of lane pixels and fitting them to a polynomial is done in the class `LaneDetector`.

The whole chain is running in function `FindLane`.

The first step is to collect the lane pixels.
If the lane detection on the previous frame was succesful it is searching only around the previous polynomial with a margin of 40 pixels. It is implemented in function `FindLanePixelsAroundPoly`.
If there was no succesful detection for the previous frame (or this is the first frame) the code invokes the function `FindLanePixels`.
It creates first a histogram. Then splits the points into two sections based on the midpoint of the histogram. After that it is searching for lane pixels in windows nine iteration long. The margins of the windows are 100 pixel. It recenters the windows if it detected more than 50 pixels in the actual window. Recenterization is done seperately on the two lane boundaries.
At this point we have the lane pixels of both the left and right lane boundaries. If we missed to detect any lane pixels for any of the two lane boundaries we are not generating a new polynomial, but keeping the previous one.
We are fitting a polynomial of degree 2 to the points. I the fitting of the polynomial was not succesful we are using the polynomial from the previous frame.
The polynomial fitting is implemented in function `FindPolynomial`.

![alt text][image5]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

The measurement of the curvature is done in function `MeasureCurvaturePixels`, which is updating the found lane pixels in order to convert them back to real world coordinates. For this converion I'm using the following ratios:
ym_per_pix = 3/85
xm_per_pix = 3.7/400

I'm measuring the curvature in the bottom of the picture.

To identify the ego position in the lane I implemented the function `MeasureEgoPositionInLane`. It is calculating the position of left and right lane in the bottom of the image and calculates the mid point
of the lane based on their average. It counts the vehicle position as the middle of the picture. The relative position is the difference of the two applying the previously mentioned factor to convert the pixels to meters.

#### 6. Filter of results

The generated polynomials are filtered in function `FilterResults`.
This functions filters based on two property:
1. Lane width
It calculates the distances of point 0 of the two polynomials (so the furthest point from the car). It stores the last 20 calculated value.
If the current value and the average of the last 20 value has a difference greater than 10% the trajectory are encounted as invalid.

2. Ratio of lane curvatures
If the ratio of the curvature of left and right lane boundary has a difference more than 5 times the trajectories are encounted as invalid.

If any of these two filter are failing or the polynomial fitting was not done succesfuly, he results of the previous frame will be reused. And for the next frame the histogram will be calculated again.
In case of 10 invalid result in chain the next result counts as valid regardless of the output of the filters.

#### 7. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

The original image is decorated by marking of the line and writing on the curvature and relative position in function `DrawLanes`. It produces the following result:

![alt text][image6]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./project_video.mp4)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Maybe the biggest surprise for me was the fact that the filters which looked visually well for me were not working as good with the histograms and the ones which were noise in my view.
I think my current solution is providing an acceptable output, however it could be improved by the following points:
1. Improve the used filters based on pictures taken on different roads and different light conditions
2. It would fail in case of splitting or merging lanes due the current implementation of lane width filter, it could be improved
3. Additional filters could be introduced which are checking the change of the position or the polynomials
4. Currently the curvature values have a trend to jump, it could be avoided by averaging to last n curvature values or introducing a Kalman-filter
