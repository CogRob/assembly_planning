from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

import math

min_angle = math.pi/6
max_angle = 5*math.pi/6

import cv2

from scipy import ndimage as ndi
from skimage import feature

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

# Loading Image
images = []


cap = cv2.VideoCapture('../images/block_video.MOV')
current_frame = 0

while(True):
    ret, frame = cap.read()

    current_frame += 1

    if(not current_frame%4 == 1):
        continue

#for i in range(0, 10):
    #img_1 = cv2.imread("../images/green_block_" + str(i) + ".jpg", cv2.IMREAD_COLOR)
    img = frame

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
                cv2.rectangle(rect_img, (x, y), (x+w, y+h), color_vals[color], 2)

                img = img[y:y+h, x:x+w]
                erode_cropped = erode_1[y:y+h, x:x+w]

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
    gray_blur = cv2.medianBlur(gray, 7)
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


    # PCA
    pca_img = img.copy()
    h, w = gray.shape

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


    # Hough Transform
    """
    hough_img = img.copy()

    # Standard Hough
    lines = cv2.HoughLines(canny_img,2,np.pi/720,20)
    angles = []

    if(lines is not None):
        print("There are " + str(len(lines[0])) + " lines")
        for rho, theta in lines[0][:100]:
            
            angles.append(theta)
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            #print("Image ", i, " has theta ", theta, " and rho ", rho)
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            if(math.fabs(theta - angle_rad_major) < math.pi/132):
                cv2.line(hough_img,(x1,y1),(x2,y2),(0,0,0),1)
            
            if(math.fabs(theta - angle_rad_minor) < math.pi/132):
                cv2.line(hough_img,(x1,y1),(x2,y2),(0,0,255),1)

    sorted_angles = sorted(angles)
    print(sorted_angles)
    """
    

    # Probabilistic Hough
    """
    minLineLength = 20
    maxLineGap = 2
    lines = cv2.HoughLinesP(canny_img,1,np.pi/180,10,maxLineGap,minLineLength)
    hough_p_img = img.copy()
    if(lines is not None):
        for x1,y1,x2,y2 in lines[0]:
            cv2.line(hough_p_img,(x1,y1),(x2,y2),(0,0,0),1)
    """

    """
    harris_img = gray.copy()
    # Find Corners
    dst = cv2.cornerHarris(harris_img,2,3,0.04)

    #result is dilated for marking the corners, not important
    dst = cv2.dilate(dst,None)

    # Threshold for an optimal value, it may vary depending on the image.
    harris_img[dst>0.001*dst.max()]=0

    brisk = cv2.BRISK()
    kp_brisk = brisk.detect(gray)
    brisk_img = img.copy()

    brisk_img = cv2.drawKeypoints(img, kp_brisk, brisk_img)
    """



    
    """
    plt.subplot("231")
    plt.imshow(rect_img,cmap='gray')
    plt.title("HSV Thresholded Image")
    plt.subplot("232")
    plt.imshow(canny_img,cmap='gray')
    plt.title("Canny Image")
    plt.subplot("233")
    plt.imshow(harris_img,cmap='gray')
    plt.title("Harris Image")
    plt.subplot("234")
    plt.imshow(hough_img,cmap='gray'
    plt.title("Hough Image")
    plt.subplot("235")
    plt.imshow(hough_p_img,cmap='gray')
    plt.title("Probabilistic Hough Image")
    plt.subplot("236")
    plt.imshow(brisk_img,cmap='gray')
    plt.title("Brisk Image")
    plt.suptitle("Image " + str(i))
    plt.show()
    """
    cv2.namedWindow("Axes", cv2.WND_PROP_FULLSCREEN)          
    cv2.setWindowProperty("Axes", cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)
       
    cv2.imshow("Axes", pca_img)

    cv2.waitKey(1)


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
