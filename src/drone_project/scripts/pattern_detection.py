import sherlock_holmes as holmes
import cv2
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--image', help='look at an image')
parser.add_argument('--video', help='look at a video')
args = parser.parse_args()

file = cv2.VideoCapture(args.video) if args.video else cv2.imread(args.image)

# frame counter
counter = 0

# pattern detector (detective)
sherlock = holmes.SherlockHolmes(draw=True)

# every x frames, try and detect a pattern
INTERVAL = 10

while True:
    # capture a frame from the video
    if args.video:
        ret, sherlock.image = file.read()
        if not ret:
            print('Video complete')
            break
    elif args.image:
        sherlock.image = file
    
    counter += 1
    if counter % INTERVAL == 0 or args.image:
        pattern = sherlock.detect_pattern()
        if pattern:
            print('Pattern {0} detected at ({1}, {2}) with a rotation of {3} degrees'.format(pattern.ID, pattern.x, pattern.y, pattern.rotation))
        else:
            print('Failed to detect a pattern')

    # display the frame
    cv2.imshow('Drone', sherlock.image)

    # close if we press escape
    wait_length = 1 if args.video else 0
    k = cv2.waitKey(wait_length) & 0xFF
    if k is 27 or args.image: break

# end
if args.video:
    file.release()
cv2.destroyAllWindows()