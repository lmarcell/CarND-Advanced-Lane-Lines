#!/usr/bin/env python3
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import glob
from moviepy.editor import VideoFileClip
from IPython.display import HTML
#matplotlib inline

# Helper classes

class Calibrator:
    #This function is able to calibrate the pictures
    def __init__(self, calibration_images_path = "camera_cal/*.jpg", chess_board_size = (9,6)):        
        self.calibration_images_path = calibration_images_path
        self.chess_board_size = chess_board_size
        self.objpoints = []
        self.imgpoints = []
        self.PrepareObjectPoints()
        self.PrepareImagePoints()
        self.Calibrate()
       
    def PrepareObjectPoints(self):
        # Preparation of chessboard points
        self.objp = np.zeros((self.chess_board_size[1]*self.chess_board_size[0],3), np.float32)
        self.objp[:,:2] = np.mgrid[0:self.chess_board_size[0], 0:self.chess_board_size[1]].T.reshape(-1,2)    

    def PrepareImagePoints(self):
        # Load images in camera_cal directory
        images = glob.glob(self.calibration_images_path)
        if (len(images) == 0):
            raise AttributeError("No images on calibration path")
        for fname in images:
            img = cv2.imread(fname)
            self.FindImagePoints(img)
                
    def FindImagePoints(self, image):
        # Find points on calibration images
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.shape = gray.shape[::-1]
        ret, corners = cv2.findChessboardCorners(gray, (self.chess_board_size[0], self.chess_board_size[1]), None)
        
        if ret:
            self.imgpoints.append(corners)
            self.objpoints.append(self.objp)
            
    def Calibrate(self):
        # Calculate camera calibration values
        self.calibration_succesful, self.mtx, self.dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, self.shape, None, None)
        
    def Undistort(self, image):
        # Undistorion of images
        if (self.calibration_succesful == None):
            return(None)
        
        return cv2.undistort(image, self.mtx, self.dist, None, self.mtx)

# Values used for perspective transform
src = np.float32([[577,463],[706,463],[1066,687], [242,687]])
dst = np.float32([[250,0],[900,0],[900,720],[250,720]])

def PerspectiveTransform(image):        
    # Transform to bird eye view
    M = cv2.getPerspectiveTransform(src, dst)
    h, w = image.shape
    return cv2.warpPerspective(image, M, (w, h), flags=cv2.INTER_LINEAR)

def PerspectiveTransformInverse(image):
    # Transform back from bird eye view
    M = cv2.getPerspectiveTransform(dst, src)
    h, w, _ = image.shape
    return cv2.warpPerspective(image, M, (w, h), flags=cv2.INTER_LINEAR)

class FilterApplicator():
    # Class which supports color filtering
    def __init__(self, image):
        self.image = image
        
    def GenerateBinaryOutput(self, input_matrix, thresh_min, thresh_max):
        # Apply treshold of input matrix
        binary_output = np.zeros_like(input_matrix)
        binary_output[(input_matrix >= thresh_min) & (input_matrix <= thresh_max)] = 1        
        return binary_output
        
    
    def SobelFilter(self, orient, thresh_min, thresh_max, sobel_kernel = 3):
        # Function to support x, y, abosult and direction (gradient) sobel filter
        gray = cv2.cvtColor(self.image, cv2.COLOR_RGB2GRAY)
        if (orient == 'x'):
            abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel))            
            scale_base = 255*abs_sobel
            scale_factor = np.max(abs_sobel)                        
        elif (orient == 'y'):            
            abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel))
            scale_base = 255*abs_sobel
            scale_factor = np.max(abs_sobel)
        elif (orient == 'absolute'): # calculate absolute magnitude
            abs_sobelx = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel))
            abs_sobely = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel))
            magnitude = np.sqrt(abs_sobelx**2 + abs_sobely**2)
            scale_base = 255*magnitude
            scale_factor = np.max(magnitude)
        elif (orient == 'direction'): # calculate of direction gradient
            abs_sobelx = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel))
            abs_sobely = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel))
            scale_base = np.arctan2(abs_sobely, abs_sobelx)
            scale_factor = 1
        else:
            return None
        result = np.uint8(scale_base/scale_factor)
        
        return self.GenerateBinaryOutput(result, thresh_min, thresh_max)
        
    def ColorFilter(self, thresh_min, thresh_max, filtered_channel, conversion_function = None):
        # Function to support color filtering
        if (conversion_function is None):
            converted_image = self.image
        else:
            converted_image = cv2.cvtColor(self.image, conversion_function)            
                
        channel_to_filter = converted_image[:,:,filtered_channel]
        
        return self.GenerateBinaryOutput(channel_to_filter, thresh_min, thresh_max)
    
    def GrayFilter(self, thresh_min, thresh_max, conversion_function = None):                
        # Function for gray filter
        if (conversion_function is None):
            converted_image = self.image
        else:
            converted_image = cv2.cvtColor(self.image, conversion_function)
        return self.GenerateBinaryOutput(converted_image, thresh_min, thresh_max)
    
    def CombineFilters(self, filters):
        # Combination of different filters
        if (len(filters) == 0):
            return None
        combined_binary = np.zeros_like(filters[0])
        filterexpression = 0
        for current_filter in filters:
            filterexpression = filterexpression | current_filter
                
        self.filterexpression = filterexpression
        combined_binary[filterexpression == 1] = 1
        return combined_binary
        
class LaneDetector:
    # Class to detect lanes
    
    def __init__(self, number_of_windows=9, window_margin=100, minimum_number_of_pixels_to_recenter=50, polynomial_margin = 40):
                
        self.number_of_windows = number_of_windows
        self.window_margin = window_margin
        self.minimum_number_of_pixels_to_recenter = minimum_number_of_pixels_to_recenter 
        self.polynomial_margin = polynomial_margin # Margin used by searching around the previous polynomial
        
        self.last_polyomial_succesful = False
        self.last_round_succesful = False
        
        # Ration of real life coordinates
        self.ym_per_pix = 3/85
        self.xm_per_pix = 3.7/400
        
        # If there was no valid polynomial at all yet
        self.no_valid_values_yet = True
        # Store the last n values
        self.distances_of_lanes_in_point_0 = []
        # Count of invalid rounds
        self.invalid_couter = 0
            
    def FindLane(self, image):
        # Main function of lane detection
        self.image = image                        

        nonzero = image.nonzero()
        self.nonzeroy = np.array(nonzero[0])
        self.nonzerox = np.array(nonzero[1])
                
        self.left_lane_indexes = []
        self.right_lane_indexes = []
                
        if (self.last_round_succesful):
            # If last round was succesful, search only around the previous polynomial
            self.FindLanePixelsAroundPoly()
        else:
            self.FindLanePixels()
            
        self.FitPolynomial()
        self.MeasureCurvaturePixels()
        self.MeasureEgoPositionInLane()
        self.FilterResults()
    
    def FilterResults(self):
        # Filter based on the ration of left and right curvature and the distances of the two
        # polynomial points at point 0
        ratio_of_curvatures = (self.left_curverad/self.right_curverad)
        curvature_valid = (ratio_of_curvatures > 0.2 and ratio_of_curvatures < 5.0)
        distance_of_lanes_in_point_0 = np.polyval(self.right_fit, 0) - np.polyval(self.left_fit, 0)
        self.distances_of_lanes_in_point_0.append(distance_of_lanes_in_point_0)
        if (len(self.distances_of_lanes_in_point_0) > 20):
            self.distances_of_lanes_in_point_0 = self.distances_of_lanes_in_point_0[-20:-1]
        distance_of_lanes_in_point_0_ratio = np.mean(self.distances_of_lanes_in_point_0)/distance_of_lanes_in_point_0
        distance_of_lanes_in_point_0_valid =  (distance_of_lanes_in_point_0_ratio > 0.9 and distance_of_lanes_in_point_0_ratio < 1.1)
        
        # After 10 invalid frames, use measurements anyway               
        if ((self.last_polyomial_succesful and curvature_valid and distance_of_lanes_in_point_0_valid) or self.invalid_couter > 10):
            self.final_left_fitx = self.left_fitx
            self.final_right_fitx = self.right_fitx
            self.final_ploty = self.ploty
            self.final_ploty = self.ploty
            self.final_left_curverad = self.left_curverad
            self.final_right_curverad = self.right_curverad
            self.final_position_in_lane = self.position_in_lane
            self.no_valid_values_yet = False
            self.last_round_succesful = True
            self.invalid_couter = 0
        else:
            self.invalid_couter += 1 
    
    def CreateHistogram(self):
        # Create histogram
        bottom_half = self.image[self.image.shape[0]//2:,:]
        self.histogram = np.sum(bottom_half, axis=0)

    def FindLanePixels(self):
        # Find lane pixels by histogram and windows
        
        self.CreateHistogram()        
        midpoint = np.int(self.histogram.shape[0]//2)
        self.leftx_current = np.argmax(self.histogram[:midpoint])
        self.rightx_current = np.argmax(self.histogram[midpoint:]) + midpoint
        
        window_height = np.int(self.image.shape[0]//self.number_of_windows)       

        # Step through the windows one by one
        for window in range(self.number_of_windows):
            win_y_low = self.image.shape[0] - (window+1)*window_height
            win_y_high = self.image.shape[0] - window*window_height

            win_xleft_low = self.leftx_current - self.window_margin
            win_xleft_high = self.leftx_current + self.window_margin
            win_xright_low = self.rightx_current - self.window_margin
            win_xright_high = self.rightx_current + self.window_margin
            
            # Draw the windows on the visualization image
            cv2.rectangle(self.image,(win_xleft_low,win_y_low),
                          (win_xleft_high,win_y_high),(0,255,0), 2) 
            cv2.rectangle(self.image,(win_xright_low,win_y_low),
                          (win_xright_high,win_y_high),(0,255,0), 2)
                     
            # Identify the nonzero pixels in x and y within the window
            good_left_indexes = ((self.nonzeroy >= win_y_low) & (self.nonzeroy < win_y_high) & 
                                 (self.nonzerox >= win_xleft_low) &  (self.nonzerox < win_xleft_high)).nonzero()[0]
            good_right_indexes = ((self.nonzeroy >= win_y_low) & (self.nonzeroy < win_y_high) & 
                                  (self.nonzerox >= win_xright_low) &  (self.nonzerox < win_xright_high)).nonzero()[0]

            # Append these indices to the lists
            self.left_lane_indexes.append(good_left_indexes)
            self.right_lane_indexes.append(good_right_indexes)
        
            # Recenter next window if needed
            if len(good_left_indexes) > self.minimum_number_of_pixels_to_recenter:
                self.leftx_current = np.int(np.mean(self.nonzerox[good_left_indexes]))
            if len(good_right_indexes) > self.minimum_number_of_pixels_to_recenter:        
                self.rightx_current = np.int(np.mean(self.nonzerox[good_right_indexes]))
    
        # Concatenate the arrays of indices (previously was a list of lists of pixels)
        self.left_lane_indexes = np.concatenate(self.left_lane_indexes)
        self.right_lane_indexes = np.concatenate(self.right_lane_indexes)

        # Extract left and right line pixel positions
        self.leftx = self.nonzerox[self.left_lane_indexes]
        self.lefty = self.nonzeroy[self.left_lane_indexes] 
        self.rightx = self.nonzerox[self.right_lane_indexes]
        self.righty = self.nonzeroy[self.right_lane_indexes]
        
    def FindLanePixelsAroundPoly(self):        
        # Search lane pixels around the polynomial
        left_lane_inds = ((self.nonzerox > (self.left_fit[0]*(self.nonzeroy**2) + self.left_fit[1]*self.nonzeroy +
                                       self.left_fit[2] - self.polynomial_margin)) &
                          (self.nonzerox < (self.left_fit[0]*(self.nonzeroy**2) +
                                       self.left_fit[1]*self.nonzeroy + self.left_fit[2] + self.polynomial_margin)))
        right_lane_inds = ((self.nonzerox > (self.right_fit[0]*(self.nonzeroy**2) + self.right_fit[1]*self.nonzeroy +
                                        self.right_fit[2] - self.polynomial_margin)) &
                           (self.nonzerox < (self.right_fit[0]*(self.nonzeroy**2) +
                                        self.right_fit[1]*self.nonzeroy + self.right_fit[2] + self.polynomial_margin)))
   
        # Again, extract left and right line pixel positions
        self.leftx = self.nonzerox[left_lane_inds]
        self.lefty = self.nonzeroy[left_lane_inds] 
        self.rightx = self.nonzerox[right_lane_inds]
        self.righty = self.nonzeroy[right_lane_inds]

    def FitPolynomial(self):
        # Fit polynomials to the lane points
        
        # Return if there are no valid points
        if ((len(self.lefty) == 0) or (len(self.leftx) == 0)
            or (len(self.righty) == 0) or (len(self.rightx) == 0)):            
            self.last_polyomial_succesful = False
            return
        
        # Fit a second order polynomial to each lane
        self.left_fit = np.polyfit(self.lefty, self.leftx, 2)
        self.right_fit = np.polyfit(self.righty, self.rightx, 2)
        
        # Generate x and y values for plotting
        self.ploty = np.linspace(0, self.image.shape[0]-1, self.image.shape[0] )
        
        try:
            self.left_fitx = self.left_fit[0]*self.ploty**2 + self.left_fit[1]*self.ploty + self.left_fit[2]
            self.right_fitx = self.right_fit[0]*self.ploty**2 + self.right_fit[1]*self.ploty + self.right_fit[2]
            self.last_polyomial_succesful = True
        except TypeError:
            # Avoids an error if `left` and `right_fit` are still none or incorrect            
            self.left_fitx = 1*self.ploty**2 + 1*self.ploty
            self.right_fitx = 1*self.ploty**2 + 1*self.ploty            
            self.last_polyomial_succesful = False
              
    def MeasureCurvaturePixels(self):
        # Return if there are no valid points
        if ((len(self.lefty) == 0) or (len(self.leftx) == 0)
            or (len(self.righty) == 0) or (len(self.rightx) == 0)):            
            self.last_polyomial_succesful = False
            return
        
        # Calculate in real world coordinates
        left_fit_real = np.polyfit(self.lefty*self.ym_per_pix, self.leftx*self.xm_per_pix, 2)
        right_fit_real = np.polyfit(self.righty*self.ym_per_pix, self.rightx*self.xm_per_pix, 2)
        
        y_eval = np.max(self.ploty) # Measure curvature at the bottom
            
        self.left_curverad = ((1+(2*left_fit_real[0]*y_eval+left_fit_real[1])**2)**(3/2))/np.absolute(2*left_fit_real[0])
        self.right_curverad = ((1+(2*right_fit_real[0]*y_eval+right_fit_real[1])**2)**(3/2))/np.absolute(2*right_fit_real[0])
        
    
    def MeasureEgoPositionInLane(self):
        # Measurement of ego position in lane in real coordinates
        left_value = np.polyval(self.left_fit, self.image.shape[0]-1)
        right_value = np.polyval(self.right_fit, self.image.shape[0]-1)
        mid_value = (left_value + right_value)/2
        car_position = self.image.shape[1] / 2
        self.position_in_lane = (car_position-mid_value)*self.xm_per_pix        
        
    def DrawLanes(self, input_image, original_image):
        # Function for drawing lanes and text on the original picture
        self.FindLane(input_image)
        
        # Return if polynomial fitting was not succesful
        if (self.no_valid_values_yet):
            print("Return by original image")
            return original_image
        
        # Create an image to draw the lines on
        warp_zero = np.zeros_like(self.image).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([self.final_left_fitx, self.final_ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([self.final_right_fitx, self.final_ploty])))])
        pts = np.hstack((pts_left, pts_right))
                        
        # Draw the lane onto the warped blank image
        cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))
        
        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = PerspectiveTransformInverse(color_warp)
        # Combine the result with the original image
        result = cv2.addWeighted(original_image, 1, newwarp, 0.3, 0)
        
        # Write curvature and position
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(result,"Curvature: " + str(np.round(self.final_left_curverad, 2)) + ", " + str(np.round(self.final_right_curverad,2)), (10,70), font, 2, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(result,"Position: " + str(np.round(self.final_position_in_lane,2)), (10,140), font, 2, (255, 0, 0), 2, cv2.LINE_AA)
        
        return result
    
calibrator = Calibrator()
lane_detector = LaneDetector()

def ProcessImage(image):
    # Process one video frame
    
    # Undistortion on the image
    undistorted_img = calibrator.Undistort(image)
    
    # Generate binary image based on gradients and color transforms
    # Use filter for sobelx, S channel of HLS and R channel of RGB
    filter_applicator = FilterApplicator(undistorted_img)    
    filtered = filter_applicator.CombineFilters(
            [filter_applicator.SobelFilter('x', 20, 100),
             filter_applicator.ColorFilter(170,255, 2, cv2.COLOR_BGR2HLS),
             filter_applicator.ColorFilter(220,255, 0, cv2.COLOR_BGR2RGB)])
        
    # Perspective transform to reach bird-eye view
    bird_eye_image = PerspectiveTransform(filtered)    
                      
    # Get lane polynomials
    return lane_detector.DrawLanes(bird_eye_image, undistorted_img)

white_output = 'project_video_output.mp4'
clip1 = VideoFileClip("project_video.mp4")

white_clip = clip1.fl_image(ProcessImage)
cwhite_clip.write_videofile(white_output, audio=False)
