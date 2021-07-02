import sys
import copy
import time

import cv2
import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt


# Input is binary_warped image
def find_lane_pixels(binary_warped):

    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)

    # Create an output image to draw on and visualize the result
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))

    # These will be the starting point for the left and right lanes
    # midpoint    = np.int32(histogram.shape[0]//2)
    centerx_base = np.argmax(histogram[:])

    # HYPERPARAMETERS

    # Choose the number of sliding windows
    nwindows = 12

    # Set the width of the windows +/- margin
    margin = 200

    # Set minimum number of pixels found to recenter window
    minpix = 40

    # Set height of windows - based on nwindows above and image shape
    window_height = np.int32(binary_warped.shape[0]//nwindows)

    # Identify the x and y positions of all nonzero pixels in the image
    nonzero  = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    # Current positions to be updated later for each window in nwindows
    centerx_current = centerx_base

    # Create empty lists to receive center lane pixel indices
    center_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):

        # Identify window boundaries in x and y (and right and left)
        win_y_low         = binary_warped.shape[0] - (window+1)*window_height
        win_y_high        = binary_warped.shape[0] - window*window_height

        win_xcenter_low   = centerx_current - margin
        win_xcenter_high  = centerx_current + margin
        
        # Draw the windows on the visualization image
        cv2.rectangle(out_img, (win_xcenter_low, win_y_low),
                      (win_xcenter_high, win_y_high), (0,255,0), 2) 

        # Identify the nonzero pixels in x and y within the window
        good_center_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                          (nonzerox >= win_xcenter_low) & (nonzerox < win_xcenter_high)).nonzero()[0]
        
        # Append these indices to the lists
        center_lane_inds.append(good_center_inds)
        
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_center_inds) > minpix:
            centerx_current = np.int32(np.mean(nonzerox[good_center_inds]))

    # Concatenate the arrays of indices (previously was a list of lists of pixels)
    try:
        center_lane_inds = np.concatenate(center_lane_inds)
    except ValueError:
        # Avoids an error if the above is not implemented fully
        pass

    # Extract left and right line pixel positions
    centerx  = nonzerox[center_lane_inds]
    centery  = nonzeroy[center_lane_inds] 

    return centerx, centery, out_img


# input: undistorted warped bird view binary image
# ouput: params for lanes' curvature calculation
# 2nd order
def fit_poly_curvature(binary_warped):
    
    # Find our lane pixels first
    centerx, centery, out_img = find_lane_pixels(binary_warped)

    # Fit a second order polynomial to each using `np.polyfit`
    center_fit = np.polyfit(centery, centerx, 2)

    # Generate x and y values for plotting
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
    
    try:
        center_fitx = center_fit[0]*ploty**2 + center_fit[1]*ploty + center_fit[2]
    except TypeError:
        # Avoids an error if `left` and `right_fit` are still none or incorrect
        print('The function failed to fit a line!')
        center_fitx = 1*ploty**2 + 1*ploty
 
    image_shape = binary_warped.shape

    return image_shape, ploty, center_fit, center_fitx


def measure_curvature_real(image_shape, ploty, center_fit, center_fitx):

    '''
    Calculates the curvature of polynomial functions in meters.
    '''
    img_height, img_width = image_shape[0], image_shape[1]
    
    # Define y-value where we want radius of curvature
    # We'll choose the maximum y-value, corresponding to the bottom of the image
    y_eval = np.max(ploty)
    
    # Define conversions in x and y from pixels space to meters
    ym_per_pix = 1/113  # meters per pixel in y dimension
    xm_per_pix = 1/233  # 3/703  # meters per pixel in x dimension
    
    center_curverad = ((1 + (2*center_fit[0]*y_eval + center_fit[1])**2)**1.5) / np.absolute(2*center_fit[0])
    
    center_fit_cr = np.polyfit(ploty * ym_per_pix, center_fitx * xm_per_pix, 2)    
    
    # Calculation of R_curve (radius of curvature)
    center_curverad = ((1 + (2*center_fit_cr[0]*y_eval*ym_per_pix + center_fit_cr[1])**2)**1.5) / np.absolute(2*center_fit_cr[0])
    
    car_center  = xm_per_pix*(img_width/2)
    lane_center = xm_per_pix*(center_fitx[-1])
    offset      = lane_center - car_center
      
    return center_curverad, offset


def drawing(undist, warped, ploty, center_fitx, M_inv, ld_point_idx):
    
    img_x = center_fitx.astype(np.int32)
    img_y = ploty.astype(np.int32)

    # Create an image to draw the lines on
    warp_zero = np.zeros_like(warped).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    
    pts = np.vstack((img_x, img_y)).T
    pts = pts.reshape((-1, 1, 2))
    isClosed = False
    thickness = 30
    
    cv2.polylines(color_warp, [pts], isClosed, (0, 255, 0), thickness)
    
    ld_pt_x = img_x[ld_point_idx]
    ld_pt_y = img_y[ld_point_idx]  
    
    # print(ld_pt_x, ld_pt_y)
    
    cv2.circle(color_warp, (ld_pt_x, ld_pt_y), radius=30, color=(0, 0, 255), thickness=-1)
    
    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    newwarp = cv2.warpPerspective(color_warp, M_inv, (warped.shape[1], warped.shape[0])) 
    
    # Combine the result with the original image
    result = cv2.addWeighted(undist, 1, newwarp, 0.5, 0)
    
    return result
    

# # 2nd order
# def fit_polynomial(binary_warped):
    
#     # Find our lane pixels first
#     centerx, centery, out_img = find_lane_pixels(binary_warped)

#     # Fit a second order polynomial to each using `np.polyfit`
#     center_fit  = np.polyfit(centery, centerx, 2)
    
#     print("The center fit is: " + str(list(center_fit)))

#     # Generate x and y values for plotting
#     ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])

#     try:
#         center_fitx  = center_fit[0]*ploty**2 + center_fit[1]*ploty + center_fit[2]
#     except TypeError:
#         # Avoids an error if `left` and `right_fit` are still none or incorrect
#         print('The function failed to fit a line!')
#         center_fitx  = 1*ploty**2 + 1*ploty

#     ## Visualization ##
#     # Colors in the left and right lane regions
#     out_img[centery, centerx]  = [255, 0, 0] # red pixels

#     # Plots the left and right polynomials on the lane lines
#     plt.figure(figsize=(20, 10))
#     plt.plot(center_fitx, ploty, color='yellow')

#     return out_img





