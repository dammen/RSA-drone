'''
The Sherlock Holmes of pattern detection
Ben Soh
'''


import numpy as np
import cv2
import cv2.cv as cv
import hsv_ranges as hsv
import patterns
import pattern_circle as pc
import math_util
import pattern as p
import math

class SherlockHolmes(object):
    # constants
    BLUR_AMOUNT = 21 # amount to blur each frame by
    PARAM_1 = 50 # parameter for circle detection (idk what it does lol)
    PARAM_2 = 38 # parameter for circle detection (the higher it is, the less
                 # circles detected)
    CIRCLE_COLOUR = (0, 255, 0) # colour for circles to be drawn
    CIRCLE_WIDTH = 2 # width of circles drawn
    CENTER_RADIUS = 3 # radius of center circle for colour checking

    # colours
    MAGENTA_LOWER = np.array(hsv.MAGENTA_LOWER)
    MAGENTA_UPPER = np.array(hsv.MAGENTA_UPPER)
    LIME_LOWER = np.array(hsv.LIME_LOWER)
    LIME_UPPER = np.array(hsv.LIME_UPPER)

    def __init__(self, draw=False):
        self.image = None
        self.draw = draw

    def detect_pattern(self):
        if self.image is None: return
        # blur the frame to aid with circle detection and convert to greyscale
        # (using gaussian blur since median blur is too slow)
        image_blur = cv2.GaussianBlur(self.image, (self.BLUR_AMOUNT, self.BLUR_AMOUNT), 0)
        image_grey = cv2.cvtColor(image_blur, cv2.COLOR_BGR2GRAY)

        # find any circles (the black backgrounds of the pattern)
        circles = cv2.HoughCircles(image_grey, cv.CV_HOUGH_GRADIENT, 1, 20,
        param1=self.PARAM_1, param2=self.PARAM_2, minRadius=0, maxRadius=0)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            # only take into account the largest circle in case of multiple
            largest_circle = sorted(circles[0, :], key=lambda c: c[2])[-1]
            # crop out that circle
            pattern = self.__crop_circle(self.image, largest_circle[0], largest_circle[1], largest_circle[2])
            # draw it for debugging
            for c in circles[0, :]:
                cv2.circle(self.image, (c[0], c[1]), c[2], (0, 255, 0), 2)
            if self.draw:
                cv2.circle(self.image, (largest_circle[0], largest_circle[1]), largest_circle[2], (255, 0, 255), 2)
            # identify the pattern and return
            return self.__identify_pattern(pattern)
        else:
            print('Failed to detect the black circle - try adjusting PARAM_2 and BLUR_AMOUNT')
            return None

    def __crop_circle(self, image, x, y, radius):
        h, w, _ = image.shape
        # use a mask to crop out a circle shape
        mask = np.zeros((h, w), np.uint8)
        cv2.circle(mask, (x, y), radius, 1, -1)
        return cv2.bitwise_and(image, image, mask=mask)

    def __detect_colours(self, pattern):
        # convert to HSV
        pattern_hsv = cv2.cvtColor(pattern, cv2.COLOR_BGR2HSV)

        # create masks for each colour
        magenta_mask = cv2.inRange(pattern_hsv, self.MAGENTA_LOWER, self.MAGENTA_UPPER)
        green_mask = cv2.inRange(pattern_hsv, self.LIME_LOWER, self.LIME_UPPER)

        # combine them
        mask = cv2.bitwise_or(magenta_mask, green_mask)

        # open then close
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # finally blur it so the edges aren't too sharp
        mask = cv2.GaussianBlur(mask, (self.BLUR_AMOUNT, self.BLUR_AMOUNT), 0)

        # return just the colours
        return cv2.bitwise_and(pattern, pattern, mask=mask)

    def __identify_circles(self, pattern):
        # first get the threshhold
        pattern_grey = cv2.cvtColor(pattern, cv2.COLOR_BGR2GRAY)
        pattern_grey = cv2.GaussianBlur(pattern_grey, (self.BLUR_AMOUNT, self.BLUR_AMOUNT), 0)
        threshold = cv2.threshold(pattern_grey, 60, 255, cv2.THRESH_BINARY)[1]

        # find contours of the threshold
        contours = cv2.findContours(threshold.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[0]

        # convert the contours to circles with colours/positions
        circles = []
        for c in contours:
            # get the center of the contour
            moments = cv2.moments(c)
            center_x = int(moments['m10'] / moments['m00'])
            center_y = int(moments['m01'] / moments['m00'])

            # find the colour from the center of the circle
            center = self.__crop_circle(pattern, center_x, center_y, self.CENTER_RADIUS)
            if self.__is_green(center):
                circles.append(pc.PatternCircle(center_x, center_y, patterns.GREEN))
                if self.draw:
                    cv2.putText(self.image, 'green', (center_x, center_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            elif self.__is_magenta(center):
                circles.append(pc.PatternCircle(center_x, center_y, patterns.PINK))
                if self.draw:
                    cv2.putText(self.image, 'pink', (center_x, center_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # draw
            if self.draw:
                cv2.drawContours(self.image, [c], -1, (255, 0, 0), 2)
                cv2.circle(self.image, (center_x, center_y), 7, (255, 255, 255), -1)
        return circles

    # orient the circles so that they are row-wise
    def __orient_circles(self, circles):
        oriented = sorted(circles, key=lambda c: c.x)
        left = sorted(oriented[:2], key=lambda c: c.y)
        right = sorted(oriented[2:], key=lambda c: c.y)
        return [left[0]] + [right[0]] + [left[1]] + [right[1]]

    def __rotate_pattern(self, pattern):
        # get the circles into a list firstly
        circles = self.__identify_circles(pattern)

        # calculate the shortest distance between two circles
        min_dist = None
        line = None
        if circles and len(circles) is 4:
            for c1 in circles:
                for c2 in circles:
                    if c1 is c2: continue
                    if not min_dist or c1.distance(c2) < min_dist:
                        min_dist = c1.distance(c2)
                        line = (c1, c2)
        
        if line and len(circles) is 4:
            c1, c2 = line # c1 and c2 are the min distance line
            c3, c4 = filter(lambda c: c is not c1 and c is not c2, circles)
            
            if self.draw:
                cv2.line(self.image, (c1.x, c1.y), (c2.x, c2.y), (0, 255, 0), 2)

            # find the angle the minimum line makes with the x-axis
            angle = math_util.line_angle((c1.x, c1.y), (c2.x, c2.y))

            # get the center of the pattern
            pattern_x = sum(map(lambda c: c.x, circles)) / len(circles)
            pattern_y = sum(map(lambda c: c.y, circles)) / len(circles)

            # rotate all circle co-ordinates by that angle
            for c in circles:
                c.x, c.y = math_util.rotate_point((c.x, c.y), -angle)

            # check if it is flipped by simply checking the y values
            if c3.y > c1.y:
                for c in circles:
                    c.x, c.y = -c.x, -c.y

            # return a pattern object
            return p.Pattern(pattern_x, pattern_y, -1, int(math.degrees(angle)), self.__orient_circles(circles))
        else:
            return None

    def __identify_pattern(self, pattern):
        # obtain a pattern object
        ptrn = self.__rotate_pattern(self.__detect_colours(pattern))
        if ptrn:
            # update the pattern ID if we find a match for the orientation
            # of colours, otherwise leave it as -1
            pattern_matrix = ((ptrn.circles[0].colour, ptrn.circles[1].colour),
                (ptrn.circles[2].colour, ptrn.circles[3].colour))
            if pattern_matrix in patterns.PATTERNS:
                ptrn.ID = patterns.PATTERNS[pattern_matrix]
            return ptrn
        else:
            return None

    def __is_magenta(self, image):
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        magenta_mask = cv2.inRange(image_hsv, self.MAGENTA_LOWER, self.MAGENTA_UPPER)
        return np.array_equal(cv2.bitwise_and(image, image, mask=magenta_mask), image)

    def __is_green(self, image):
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(image_hsv, self.LIME_LOWER, self.LIME_UPPER)
        return np.array_equal(cv2.bitwise_and(image, image, mask=green_mask), image)

