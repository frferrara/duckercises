# Import modules
import cv2
import duckietown_utils as du
import numpy            as np
from sensor_msgs.msg import CompressedImage


# Implement modules
class Instagram(object):
    # Constructor
    def __init__(self, filters = ""):
        # Define functions dictionary
        self.fil_fun = {
            "flip-vertical":    self.flipVer,
            "flip-horizontal":  self.flipHor,
            "grayscale":        self.grayFromBgr,
            "sepia":            self.sepiaFromBgr
        }

        # Set the filters (if available)
        sep = ":"
        self.filters = [f for f in filters.split(sep) if f in self.fil_fun]

    # Flip an image
    def flipImage(self, im, direction):
        if direction == "horizontal":
            axis = 1
        elif direction == "vertical":
            axis = 0
        else:
            raise ValueError("Direction must be either horizontal or vertical!")

        return cv2.flip(im, axis)


    # Flip horizontally
    def flipHor(self, im):
        return self.flipImage(im, "horizontal")


    # Flip vertically
    def flipVer(self, im):
        return self.flipImage(im, "vertical")


    # Convert to grayscale
    def grayFromBgr(self, bgr):
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)


    # Apply a sepia filter
    def sepiaFromBgr(self, bgr):
        # Filter matrix
        K = np.matrix([[0.272, 0.534, 0.131], \
                    [0.349, 0.686, 0.168], \
                    [0.393, 0.769, 0.189]])

        return cv2.transform(bgr, K)


    # Apply the filter function
    def applyFilFun(self, fil, im_in):
        try:
            im_out = self.fil_fun[fil](im_in)
        except:
            im_out = im_in
            print("Filter ", fil, " could not be applied!")

        return im_out


    # Apply the filters
    def applyFilters(self, im_bgr, filters):
        # Loop through the filter array
        im_mod = im_bgr
        for fil in self.filters:
            im_mod = self.applyFilFun(fil, im_mod)

        return im_mod


    # Process a message
    def processMsg(self, msg_in):
        # Get the image from the message
        im_rgb = du.rgb_from_ros(msg_in)
        im_bgr = cv2.cvtColor(im_rgb, cv2.COLOR_RGB2BGR)

        # Apply filters
        im_mod_bgr = self.applyFilters(im_bgr, self.filters)

        # Convert to message
        msg_out = msg_in
        msg_out.data = du.d8_compressed_image_from_cv_image(im_mod_bgr, msg_in).data

        return msg_out


### --- Testing --- #
if __name__ == "__main__":
    # Load
    im = cv2.imread("sunset.jpg", cv2.IMREAD_COLOR)
    msg_in = CompressedImage()

    # Not manipulated
    msg_in.data = du.d8_compressed_image_from_cv_image(im, msg_in).data
    inst = Instagram("")
    msg_out = inst.processMsg(msg_in)
    im_out_rgb = du.rgb_from_ros(msg_out)
    im_out_bgr = cv2.cvtColor(im_out_rgb, cv2.COLOR_RGB2BGR)
    cv2.imwrite("untouched.jpg", im_out_bgr)

    # Flip vertically
    msg_in.data = du.d8_compressed_image_from_cv_image(im, msg_in).data
    inst = Instagram("flip-vertical")
    msg_out = inst.processMsg(msg_in)
    im_out_rgb = du.rgb_from_ros(msg_out)
    im_out_bgr = cv2.cvtColor(im_out_rgb, cv2.COLOR_RGB2BGR)
    cv2.imwrite("vflip.jpg", im_out_bgr)

    # Flip horizontally
    msg_in.data = du.d8_compressed_image_from_cv_image(im, msg_in).data
    inst = Instagram("flip-horizontal")
    msg_out = inst.processMsg(msg_in)
    im_out_rgb = du.rgb_from_ros(msg_out)
    im_out_bgr = cv2.cvtColor(im_out_rgb, cv2.COLOR_RGB2BGR)
    cv2.imwrite("hflip.jpg", im_out_bgr)

    # Grayscale
    msg_in.data = du.d8_compressed_image_from_cv_image(im, msg_in).data
    inst = Instagram("grayscale")
    msg_out = inst.processMsg(msg_in)
    im_out_rgb = du.rgb_from_ros(msg_out)
    cv2.imwrite("grayscale.jpg", im_out_rgb)

    # Sepia
    msg_in.data = du.d8_compressed_image_from_cv_image(im, msg_in).data
    inst = Instagram("sepia")
    msg_out = inst.processMsg(msg_in)
    im_out_rgb = du.rgb_from_ros(msg_out)
    im_out_bgr = cv2.cvtColor(im_out_rgb, cv2.COLOR_RGB2BGR)
    cv2.imwrite("sepia.jpg", im_out_bgr)

    # Full flip and Grayscale
    msg_in.data = du.d8_compressed_image_from_cv_image(im, msg_in).data
    inst = Instagram("flip-vertical:flip-horizontal:grayscale")
    msg_out = inst.processMsg(msg_in)
    im_out_rgb = du.rgb_from_ros(msg_out)
    cv2.imwrite("./fflip_grayscale.jpg", im_out_rgb)

    # Full flip and Sepia
    msg_in.data = du.d8_compressed_image_from_cv_image(im, msg_in).data
    inst = Instagram("flip-horizontal:flip-vertical:sepia")
    msg_out = inst.processMsg(msg_in)
    im_out_rgb = du.rgb_from_ros(msg_out)
    im_out_bgr = cv2.cvtColor(im_out_rgb, cv2.COLOR_RGB2BGR)
    cv2.imwrite("./fflip_sepia.jpg", im_out_bgr)
