import numpy as np
import cv2

class ColorFilter:
    """
        discord: @kialli
        github: @kchan5071cv2.CAP_DSHOW
        
        This class filters out a specific color from an image and returns the image with only the color in it.
        It also returns the average position of the color in the image.
    """
    
    def __init__(self):
        self.sensitivity        = 10
        self.color_target       = [255, 0, 0]
        self.amount_in_image    = 5000
        self.alpha_threshold    = 10
        self.iterations         = 10
        self.sensitivity_cap    = 255

    def set_color_target(self, color):
        """
            sets the color that the filter will target
        """
        self.color_target = color

    def get_image(self, image):
        """
            from an image, returns an image with only the target color in it

            IMPORTANT: will automatically adjust the sensitivity of the filter based on the amount of the target color in the image

            input:
                image: the image to filter

            output:
                the image with only the target color in it
                returns None if the image is None
        """
        if image is None:
            return None
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        lower_color = np.array([self.color_target[0] - self.sensitivity, 
                                self.color_target[1] - self.sensitivity, 
                                self.color_target[2] - self.sensitivity])
        upper_color = np.array([self.color_target[0] + self.sensitivity,
                                self.color_target[1] + self.sensitivity,
                                self.color_target[2] + self.sensitivity])
        mask = cv2.inRange(image, lower_color, upper_color)
        result = cv2.bitwise_and(image, image, mask=mask)

        result = self.dilate_image(result)
        result = self.erode_image(result)

        return self.remove_values_below_threshold(result)
    
    def dilate_image(self, image):
        """
        dilates the image

        image dialation is a process that expands the boundaries of the image

        input:
            image: the image to dilate

        output:
            the dilated image
            returns None if the image is None
    """
        if image is None:
            return None
        element = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
        return cv2.dilate(image, element, self.iterations)
    
    def erode_image(self, image):
        """
            erodes the image

            image erosion is a process that shrinks the boundaries of the image

            input:
                image: the image to erode

            output:
                the eroded image
                returns None if the image is None
        """
        if image is None:
            return None
        element = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
        return cv2.erode(image, element, self.iterations)
              
    def remove_values_below_threshold(self, image):
        """
            removes values below a certain threshold (alpha_threshold) from the image
            and sets them to black

            input:
                image: the image to remove values from

            output:
                the image with values below the threshold removed
                returns None if the image is None
        """
        if image is None:
            return None
        image[image < self.alpha_threshold] = 0
        return image
    
    def get_average_position(self, image):
        """
            gets the average position of the target color in the image

            input:
                image: the image to get the average position from

            output:
                the average position of the target color in the image
                returns None if the image is None
        """
        if image is None:
            return None
        
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.ADAPTIVE_THRESH_MEAN_C)
        valid_pixels = np.where(thresh > 0)

        if len(valid_pixels[0]) < self.amount_in_image\
            and self.sensitivity < self.sensitivity_cap:
            self.sensitivity += 1

        elif len(valid_pixels[0]) > self.amount_in_image:
            self.sensitivity -= 1

        if len(valid_pixels[0]) == 0:
            return None
        
        x = np.mean(valid_pixels[1])
        y = np.mean(valid_pixels[0])

        if np.isnan(x) or np.isnan(y):
            return None
        return (x, y)

    def auto_average_position(self, image):
        """
            automatically gets the average position of the target color in the image

            calls get_image and get_average_position

            input:
                image: the image to get the average poscv2.CAP_DSHOWcv2.CAP_DSHOWition from

            output:
                the average position of the target color in the image as a tuple
                returns None if the image is None

                image: modified image
        """
        image = self.get_image(image)
        box = self.get_average_position(image)
        if box is not None:
            image = cv2.circle(img=image, center=(int(box[0]), int(box[1])), radius= 10, color=(255, 0, 0), thickness=2)
        return image, box

if __name__ == '__main__':
    _, cap = cv2.VideoCapture(5)
    filter = ColorFilter()
    while True:
        image = cap.read()
        box = filter.auto_average_position(image)
        if box is not None:
            image = cv2.circle(img=image, center=(int(box[0]), int(box[1])), radius= 10, color=(255, 0, 0), thickness=2)
        print(box)
        cv2.imshow('image', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
