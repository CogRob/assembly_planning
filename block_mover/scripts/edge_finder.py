from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

import math

min_angle = math.pi/6
max_angle = 5*math.pi/6

import cv2

from scipy import ndimage as ndi
from skimage import feature

import pickle

print(cv2.__version__)

REAL = True
USE_HSV = True

if(REAL):
    BLU_LOW_HUE     = 105
    BLU_HIGH_HUE    = 120
    BLU_LOW_SAT     = 0
    BLU_HIGH_SAT    = 255
    BLU_LOW_VAL     = 0
    BLU_HIGH_VAL    = 100

    GRN_LOW_HUE     = 50
    GRN_HIGH_HUE    = 80
    GRN_LOW_SAT     = 100
    GRN_HIGH_SAT    = 255
    GRN_LOW_VAL     = 0
    GRN_HIGH_VAL    = 255


    TEAL_LOW_HUE     = 80
    TEAL_HIGH_HUE    = 105
    TEAL_LOW_SAT     = 0
    TEAL_HIGH_SAT    = 255
    TEAL_LOW_VAL     = 0
    TEAL_HIGH_VAL    = 100

    RED_LOW_HUE_1     = 0
    RED_HIGH_HUE_1    = 10
    RED_LOW_HUE_2     = 175
    RED_HIGH_HUE_2    = 180
    RED_LOW_SAT     = 0
    RED_HIGH_SAT    = 255
    RED_LOW_VAL     = 0
    RED_HIGH_VAL    = 100

    YEL_LOW_HUE     = 14
    YEL_HIGH_HUE    = 24
    YEL_LOW_SAT     = 0
    YEL_HIGH_SAT    = 255
    YEL_LOW_VAL     = 0
    YEL_HIGH_VAL    = 255

colors = {
        "red":      {   "low_h": [RED_LOW_HUE_1, RED_LOW_HUE_2],   "high_h": [RED_HIGH_HUE_1, RED_HIGH_HUE_2],
                        "low_s": RED_LOW_SAT,   "high_s": RED_HIGH_SAT,
                        "low_v": RED_LOW_VAL,   "high_v": RED_HIGH_VAL  },
        "yellow":   {   "low_h": YEL_LOW_HUE,   "high_h": YEL_HIGH_HUE,
                        "low_s": YEL_LOW_SAT,   "high_s": YEL_HIGH_SAT,
                        "low_v": YEL_LOW_VAL,   "high_v": YEL_HIGH_VAL  },
        "green":    {   "low_h": GRN_LOW_HUE,   "high_h": GRN_HIGH_HUE,
                        "low_s": GRN_LOW_SAT,   "high_s": GRN_HIGH_SAT,
                        "low_v": GRN_LOW_VAL,   "high_v": GRN_HIGH_VAL  },
        "blue":     {   "low_h": BLU_LOW_HUE,   "high_h": BLU_HIGH_HUE,
                        "low_s": BLU_LOW_SAT,   "high_s": BLU_HIGH_SAT,
                        "low_v": BLU_LOW_VAL,   "high_v": BLU_HIGH_VAL  },
        "teal":     {   "low_h": TEAL_LOW_HUE,  "high_h": TEAL_HIGH_HUE,
                        "low_s": TEAL_LOW_SAT,  "high_s": TEAL_HIGH_SAT,
                        "low_v": TEAL_LOW_VAL,  "high_v": TEAL_HIGH_VAL  }
}

color_vals = {
    "red":    (0, 0, 255),
    "green":  (0, 255, 0),
    "blue":   (255, 0, 0),
    "yellow": (0, 255, 255),
    "teal":   (255, 60, 0)
}

def find_main_lines(orig_img, curr_img, angle_rad_major, angle_rad_minor):
    # Hough Transform
    hough_img = orig_img.copy()
    hough_pixel_acc = 1
    hough_angle_acc = np.pi / 360
    hough_min_vote = 10
    
    major_angle_thresh = math.pi/16
    minor_angle_thresh = math.pi/16

    # Standard Hough
    lines = cv2.HoughLines(curr_img, hough_pixel_acc, hough_angle_acc, hough_min_vote)
    
    angles = []
    num_top = 20
    major_lines = []
    minor_lines = []
    if(lines is not None):
        for rho, theta in lines[0]:
            
            angles.append(theta)
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho

            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))

            if(math.fabs(theta - angle_rad_major) < major_angle_thresh and len(major_lines) < num_top):
                cv2.line(hough_img,(x1,y1),(x2,y2),(0,0,0),1)
                major_lines.append(((x1,y1), (x2, y2), theta))
            
            if(math.fabs(theta - angle_rad_minor) < minor_angle_thresh and len(minor_lines) < num_top):
                cv2.line(hough_img,(x1,y1),(x2,y2),(0,0,255),1)
                minor_lines.append(((x1,y1), (x2, y2), theta))

            if(len(major_lines) == num_top and len(minor_lines) == num_top):
                break

    
    print("Of the top ", num_top, " lines, ", len(major_lines), " were aligned with major axis and ", len(minor_lines), " were aligned with minor axis.")

    #sorted_angles = sorted(angles)
    #print(sorted_angles)

    return hough_img, major_lines, minor_lines



# Lines in ((x1, y1), (x2, y2)) form
# Corners in list of (x, y) points
# TODO: Add in the boundaries of bounding box because lines can't intersect outside of bounding box...
def find_jp(major_lines, minor_lines, corners, jp_img):

    corners = np.array(np.flip(corners, axis=0))

    angle_thresh = math.pi / 8

    distance_to_corner = np.zeros((len(major_lines), len(minor_lines)))
    closest_corner = np.zeros((len(major_lines), len(minor_lines), 2))
    intersections = np.zeros((len(major_lines), len(minor_lines), 2))
    min_distance = 10000000

    for i in range(len(major_lines)):
        for j in range(len(minor_lines)):
            major_line = major_lines[i]
            minor_line = minor_lines[j]
            # Check if line 1 and 2 are within angle threshold of eachother

            angle1 = major_line[2]
            angle2 = minor_line[2]

            if(math.fabs(angle1 - angle2) < angle_thresh):
                print("Angles are not within ", angle_thresh, " of oneanother.")
                continue
            
            
            # Find intersection 
            intersection = find_intersection(np.array(major_line[0]), np.array(major_line[1]), np.array(minor_line[0]), np.array(minor_line[1]))

            # Plot for sanity check
            #cv2.line(jp_img,major_line[0],major_line[1],(0,0,0),1)
            #cv2.line(jp_img,minor_line[0], minor_line[1],(0,0,255),1)


            #cv2.circle(jp_img, tuple(intersection), 10, (0,0, 0), 1)
            
            #plt.imshow(jp_img)
            #plt.show()

            # TODO: Intersection should be within the current bounding box
            #print("Corners: ", corners)
            #print("Intersection: ", intersection)
            #print("Corners - intersection: ", corners - intersection)
            #print("Distance to corner: ", np.linalg.norm(corners - intersection, axis=0))
            distance_to_corner[i][j] = np.min(np.linalg.norm(corners - intersection, axis=0))
            
            #print(np.argmin(np.fabs(corners - intersection)))

            closest_corner_idx = np.argmin(np.linalg.norm(corners - intersection, axis=0))

            closest_corner[i][j] = corners[:, closest_corner_idx]
            print("CLOSEST CORNER: ", corners[:, closest_corner_idx], " With index ", closest_corner_idx)
            intersection = intersection.flatten()
            intersections[i][j] = intersection
            
            distance_to_corner[i][j] = np.linalg.norm(closest_corner[i][j] - intersection)

            if(distance_to_corner[i][j] < min_distance):
                min_distance = distance_to_corner[i][j]
                print("New min_distance: ", min_distance)



    #print(distance_to_corner)
    best_jp = np.unravel_index(np.argmin(distance_to_corner), distance_to_corner.shape)
    best_intersection = intersections[best_jp].astype(int)
    best_corner = closest_corner[best_jp].astype(int)

    print("The minimum distance to a corner is ", distance_to_corner[best_jp])
    print("The minimum distance to a corner is ", np.min(distance_to_corner))
    print("The best intersection is ", intersections[best_jp])
    print("The closest corner is ", closest_corner[best_jp])

    print("Best junction point: ", best_jp)
    
    
    major_line = major_lines[best_jp[0]]
    minor_line = minor_lines[best_jp[1]]

    print("MAJOR LINE: ", major_line)
    print("MINOR LINE: ", minor_line)

    cv2.line(jp_img, major_line[0], major_line[1], (0, 0, 255), 1)
    cv2.line(jp_img, minor_line[0], minor_line[1], (0, 0, 0), 1)

    cv2.circle(jp_img, tuple(best_intersection), 10, (0, 0, 0))
    cv2.circle(jp_img, tuple(best_corner), 10, (0, 0, 255))

    return jp_img

# From https://stackoverflow.com/questions/3252194/numpy-and-line-intersections
def perp(a): 
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b


    # From https://stackoverflow.com/questions/3252194/numpy-and-line-intersections
def find_intersection(a1, a2, b1, b2):
    da = a2 - a1
    db = b2 - b1
    dp = a1 - b1
    dap = perp(da)

    denom = np.dot(dap, db)
    num = np.dot(dap, dp)
    intersection =  (num / denom.astype(float))*db + b1
    
    return np.reshape(intersection, (2,1))


# Loading Image from pickle
#images = []
#images = pickle.load(open("images.p", "rb"))

for i in range(0, 10):
    # For reading real images
    img = cv2.imread("../images/green_block_" + str(i) + ".jpg", cv2.IMREAD_COLOR)
    #img = frame

    # For reading in simulated images
    #img = images[i]


    #aspect_ratio = img.shape[0] / img.shape[1]
    newsize = (1280, 800)
    #newsize = (int(img.shape[1]*0.3), int(img.shape[0]*0.3))

    print("size = ", newsize)
    img = cv2.resize(img, newsize)
    orig_img = img.copy()

    print("Scaled down image resolution: ", newsize)

    if(USE_HSV):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        #for color in colors:
        # For now, just test with color green
        color = "green"
        
        low_h = colors[color]["low_h"]
        high_h = colors[color]["high_h"]
        low_s = colors[color]["low_s"]
        high_s = colors[color]["high_s"]
        low_v = colors[color]["low_v"]
        high_v = colors[color]["high_v"]

        #Converting image to HSV format
        if color == "red":
            hsv_mask_1 = cv2.inRange(hsv_img, np.array([low_h[0], low_s, low_v]), np.array([high_h[0], high_s, high_v]))
            hsv_mask_2 = cv2.inRange(hsv_img, np.array([low_h[1], low_s, low_v]), np.array([high_h[1], high_s, high_v]))

            hsv_mask = hsv_mask_1 | hsv_mask_2


        else:
            hsv_mask = cv2.inRange(hsv_img, np.array([low_h, low_s, low_v]), np.array([high_h, high_s, high_v]))
        
        # Apply mask to original image
        masked_img = cv2.bitwise_and(img, img, mask=hsv_mask)

        #Morphological opening (remove small objects from the foreground)
        erode_1 = cv2.erode(hsv_mask, np.ones((5,5), np.uint8), iterations=1)
        dilate_1 = cv2.dilate(erode_1, np.ones((5,5), np.uint8), iterations=1)

        #Morphological closing (fill small holes in the foreground)
        dilate_2 = cv2.dilate(dilate_1, np.ones((10,10), np.uint8), iterations=1)
        erode_2 = cv2.erode(dilate_2, np.ones((10,10), np.uint8), iterations=1)

        ret, thresh = cv2.threshold(erode_1,157,255,0)

        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

        #Draw the countours.
        #cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)
        large_contours = []

        rect_img = img.copy()
        min_area_mask = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        min_area_mask.fill(0)
        
        contour_img = orig_img.copy()
        cv2.drawContours(contour_img, contours, -1, (0,255,0), 3)

        print("There are ", len(contours), " contours.")
        for c in contours:
            area = cv2.contourArea(c)

            if(area > 2000):
                large_contours.append(c)

                # Min Area Rectangle
                rect = cv2.minAreaRect(c)
                box = cv2.cv.BoxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(min_area_mask, [box], 0, (255,255,255) , -1)
                print("Min area: ", min_area_mask.shape)
                print("Image: ", img.shape)
                masked_img = cv2.bitwise_and(img, img, mask=min_area_mask)

                # Bounding Rectangle
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(rect_img, (x - 5, y-5), (x+ w + 5, y + h + 5), color_vals[color], 2)
                #cv2.rectangle(rect_img, (x, y), (x+ w, y + h), color_vals[color], 2)

                img = img[y-5:y+h+5, x-5:x+w+5]
                erode_cropped = erode_1[y-5:y+h+5, x-5:x+w+5]
                
                #img = img[y:y+h, x:x+w]
                #erode_cropped = erode_1[y:y+h, x:x+w]

                break

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Canny Edge Detection
    """
    canny_min_thresh = 10
    canny_max_thresh = 300
    canny_img = cv2.Canny(gray, canny_min_thresh, canny_max_thresh, apertureSize = 5)
    canny_img = 255 - canny_img
    #canny_img = np.logical_not(canny_img)
    """
    gray_blur = cv2.medianBlur(gray, 3)
    canny_img = feature.canny(gray_blur, sigma=1)
    canny_img = canny_img.astype(np.uint8)


    #canny_img = 255 - 255*canny_img

    """
    contours = cv2.findContours(canny_img, type(canny_img))

    for c in contours:
        aspect_ratio =
    """


    #canny_img = cv2.dilate(canny_img, np.ones((2,2), np.uint8), iterations=1)
    #canny_img = cv2.erode(canny_img, np.ones((2,2), np.uint8), iterations=1)
   
   
    blur_img = cv2.GaussianBlur(gray, (1, 1), 0)
    lap_img = cv2.Laplacian(blur_img, cv2.CV_64F)

    h, w = lap_img.shape
    edge_thresh = 20
    for col in range(w):
        for row in range(h):
            #print(lap_img[row, col])

            if math.fabs(lap_img[row, col]) > edge_thresh:
                lap_img[row, col] = 1
            else:
                lap_img[row,col] = 0

    # PCA
    pca_img = img.copy()
    h, w = gray.shape

    print("Height: ", h, " Width: ", w)

    #From a matrix of pixels to a matrix of coordinates of non-black points.
    #(note: mind the col/row order, pixels are accessed as [row, col]
    #but when we draw, it's (x, y), so have to swap here or there)
    mat = []
    for col in range(w):
        for row in range(h):
            if erode_cropped[row, col] > 0:
                #print(img[row,col])
                mat.append([col, row])

    mat = np.array(mat).astype(np.float32) #have to convert type for PCA

    #mean (e. g. the geometrical center)
    #and eigenvectors (e. g. directions of principal components)
    m, e = cv2.PCACompute(mat, mean = None)

    #now to draw: let's scale our primary axis by 100,
    #and the secondary by 50
    center = tuple(m[0])
    endpoint1 = tuple(m[0] + e[0]*100)
    endpoint2 = tuple(m[0] + e[1]*50)

    cv2.circle(pca_img, center, 5, 255)
    # Major Axis
    cv2.line(pca_img, center, endpoint1, 255, 4)

    # Minor Axis
    cv2.line(pca_img, center, endpoint2, 255, 4)


    # Calculate angle of major axis
    major_x = endpoint1[0] - center[0]
    major_y = endpoint1[1] - center[1]
    minor_x = endpoint2[0] - center[0]
    minor_y = endpoint2[1] - center[1]

    angle_rad_major = math.atan2(major_y, major_x)

    angle_rad_minor = math.atan2(minor_y, minor_x)

    angle = 180 - (angle_rad_major * 180 / np.pi) % 180

    # Corner detection
    harris_img = gray.copy()
    harris_block_size = 5
    harris_ksize = 3
    harris_k = 0.04
    
    # Find Corners
    dst = cv2.cornerHarris(harris_img,harris_block_size,harris_ksize,harris_k)

    #result is dilated for marking the corners, not important
    dst = cv2.dilate(dst,None)

    # Threshold for an optimal value, it may vary depending on the image.
    harris_img[dst>0.001*dst.max()] = 2
    corners = np.where(harris_img == 2)



    canny_hough_img, major_lines, minor_lines = find_main_lines(img, canny_img, angle_rad_major, angle_rad_minor)

    jp_img_1 = find_jp(major_lines, minor_lines, corners, img.copy())
    
    lap_img = lap_img.astype(np.uint8)

    lap_hough_img, major_lines, minor_lines = find_main_lines(img, lap_img, angle_rad_major, angle_rad_minor)

    jp_img_2 = find_jp(major_lines, minor_lines, corners, img.copy())
    # Probabilistic Hough
    minLineLength = 20
    maxLineGap = 1
    hough_p_pixel_acc = 1
    hough_p_angle_acc = np.pi/360

    lines = cv2.HoughLinesP(canny_img, hough_p_pixel_acc, hough_p_angle_acc, 10, maxLineGap, minLineLength)
    hough_p_img = img.copy()
    if(lines is not None):
        for x1,y1,x2,y2 in lines[0]:
            cv2.line(hough_p_img,(x1,y1),(x2,y2),(0,0,0),1)
            
    brisk = cv2.BRISK()
    kp_brisk = brisk.detect(gray)
    brisk_img = img.copy()

    brisk_img = cv2.drawKeypoints(img, kp_brisk, brisk_img)


  
    plt.subplot(3, 4, 1)
    plt.imshow(hsv_img,cmap='gray')
    plt.title("HSV Masked")
    plt.subplot(3, 4, 2)
    plt.imshow(rect_img,cmap='gray')
    plt.title("Bounding Box")
    plt.subplot(3, 4, 3)
    plt.imshow(canny_img,cmap='gray')
    plt.title("Canny")
    plt.subplot(3, 4, 4)
    plt.imshow(harris_img,cmap='gray')
    plt.title("Harris")
    plt.subplot(3, 4, 5)
    plt.imshow(canny_hough_img,cmap='gray')
    plt.title("Canny Hough")
    plt.subplot(3, 4, 6)
    plt.imshow(hough_p_img,cmap='gray')
    plt.title("Probabilistic Hough")
    plt.subplot(3, 4, 7)
    plt.imshow(brisk_img,cmap='gray')
    plt.title("Brisk")
    plt.subplot(3, 4, 8)
    plt.imshow(lap_img,cmap='gray')
    plt.title("Laplacian")
    plt.subplot(3, 4, 9)
    plt.imshow(pca_img,cmap='gray')
    plt.title("PCA")
    plt.subplot(3, 4, 10)
    plt.imshow(lap_hough_img,cmap='gray')
    plt.title("Laplacian Hough")
    plt.subplot(3, 4, 11)
    plt.imshow(jp_img_1,cmap='gray')
    plt.title("Junction Point (Canny)")
    plt.subplot(3, 4, 12)
    plt.imshow(jp_img_2,cmap='gray')
    plt.title("Junction Point (Laplacian)")
    plt.suptitle("Image " + str(i))
    plt.show()
    
    
    """
    cv2.namedWindow("Axes", cv2.WND_PROP_FULLSCREEN)          
    cv2.setWindowProperty("Axes", cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)
      
    cv2.imshow("Axes", lap_img)

    cv2.waitKey(1)
    """


    """
    #plt.subplot(6,10,i + 1)
    plt.subplot(6,1,1)
    plt.imshow(orig_img, cmap='gray')
    plt.title(str(angle))


    #plt.subplot(6, 10, i+11)
    plt.subplot(6, 1, 2)
    plt.imshow(erode_cropped, cmap='gray')
    plt.title(str(angle))

    #plt.subplot(6, 10, i+21)
    plt.subplot(6, 1, 3)
    plt.imshow(canny_img, cmap='gray')
    plt.title(str(angle))

    #plt.subplot(6, 10, i+31)
    plt.subplot(6, 1, 4)
    plt.imshow(pca_img, cmap='gray')
    title = r'${:.0f}\degree%'.format(angle)
    plt.title(title)
    
    #plt.subplot(6, 10, i+41)
    plt.subplot(6, 1, 5)
    plt.imshow(hough_img, cmap='gray')
    plt.title(str(angle))

    plt.show()
    """


def find_viewpoint():
    # First move along bisector until change in angle is less than threshold
    motion = MOTION_ALONG_BISECTOR
    speed = POSITIVE_SPEED
    delta_w = DELTA_W_THRESH + 1
    prev_w = calculate_major_angle(img)

    while(delta_w >= DELTA_W_THRESH):
        # Translate camera (parallel to image plane)
        translate_camera(speed, motion, prev_w)

        # First find center of blob
        center = find_block_center(img)

        # Then fixate on it
        fixate_camera(center)

        # Determine angle that major axis makes with image x axis (horizontal)
        w = calculate_major_angle(img)

        delta_w = prev_w - w

        if(delta_w > 0):
            speed = NEGATIVE_SPEED

    motion = MOTION_ACROSS_BISECTOR
    speed = POSITIVE_SPEED
    delta_w = DELTA_W_THRESH + 1

    prev_w = calculate_minor_angle(img)

    while(delta_w >= DELTA_W_THRESH):
        # Translate camera (parallel to image plane)
        translate_camera(speed, motion, prev_w)

        # First find center of blob
        center = find_block_center(img)

        # Then fixate on it
        fixate_camera(center)

        # Determine angle that major axis makes with image x axis (horizontal)
        w = calculate_major_angle(img)

        delta_w = prev_w - w

        if(delta_w < 0):
            speed = POSITIVE_SPEED

    # Final angle should be ~90 degrees

    return w


"""
def find_spherical_poses(n_obs, radius, center_point, theta, phi):
    pts = np.zeros((n_obs+1,3))
    print(pts.shape)

    for i in range(n_obs + 1):
        angle = min_angle + i * (max_angle - min_angle) / n_obs

        pts[i, 0] = radius * math.cos(angle)
        pts[i, 1] = radius * math.sin(angle)

    print(pts)

    # First rotate up to XZ plane
    R = np.array(  [[1, 0, 0],
                    [0 ,0, 1],
                    [0, 1, 0]]
                )


    pts_rot_xz = np.dot(pts, R)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(pts_rot_xz[:,0], pts_rot_xz[:,1], pts_rot_xz[:,2])

    ax.plot(pts[:,0], pts[:,1], pts[:,2])

    plt.show()





def main():
    n_obs = 10
    radius = 0.25 # meters
    center_point = np.array([0, 0, 0])
    theta = 0.0
    phi = 0.0

    spherical_poses = find_spherical_poses(n_obs, radius, center_point, theta, phi)







if __name__ == '__main__':
    main()
"""
