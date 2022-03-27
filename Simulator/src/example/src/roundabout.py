import numpy as np
import cv2

class Roundabout:

    def __init__(self):
        self.angle = 0.0

    def mask_image(self, image):
        polygons = np.array([
             [(self.width, self.height * 4 / 5),(self.width * 5 / 6, self.height * 3 / 5), (self.width / 6, self.height * 3 / 5), 
             (0, self.height * 4 / 5)]
        ])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, np.int32([polygons]), 255)
        MaskedImage = cv2.bitwise_and(image, mask)
        return MaskedImage