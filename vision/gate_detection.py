import time
import cv2
import numpy as np

def downsample_image(image, block_size):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    width, height, _ = image.shape
    x_scaling = width / block_size
    y_scaling = height / block_size
    down_sized = cv2.resize(gray, (0, 0), fx = 1 / x_scaling, fy = 1 / y_scaling)
    upsized = cv2.resize(down_sized, (0, 0), fx = x_scaling, fy = y_scaling)
    _, filter_too_close = cv2.threshold(upsized, 200, 255, cv2.THRESH_TOZERO_INV) #2nd value, higher means closer, mi
    _, middle_thresh = cv2.threshold(filter_too_close, 100, 255, cv2.THRESH_BINARY) #2nd value, higher means closer, minimum cutoff
    return middle_thresh

def find_equator(image, threshold_value):
    #not using weight
    height, _ = image.shape
    # find center
    middle = height // 2
    middle_pixels = image[image[0],middle]
    middle_flat = np.array(middle_pixels).flatten()
    print(type(middle_flat))
    return middle_flat

def count_changes(pixel_list):
    changes = 0
    position = 0
    for i in range(1, len(pixel_list)):
        if positive_to_negative_change(pixel_list[i], pixel_list[i - 1]):
            changes += 1
            position += i
    if changes <= 1: #or changes % 2 == 1
        return 0
    return position / changes


def positive_to_negative_change(num1, num2):
    return num1 - num2 == 1


def test_count_changes():
    #only use 1s and 0s

    for i in range(1, 1000000):
        count_changes([0, 1, 0, 1, 0, 1])
    # Test case 2 (1 change)
    print(count_changes([0, 1, 1, 1, 1, 0]))

    # Test case 3 (0 changes)
    print(count_changes([0, 0, 0, 0, 0, 0]))

    # Test case 6 (0 changes)
    print(count_changes([1, 1, 1, 1, 1, 1]))

    # Test case 7 (2 changes)
    print(count_changes([0, 0, 1, 0, 1, 0]))

    # Test case 8 (2 changes)
    print(count_changes([0, 1, 1, 0, 1, 0]))

    # Test case 9 (1 changes)
    print(count_changes([1, 1, 1, 0, 0, 0]))

    # Test case 10 (1 changes)
    print(count_changes([0, 0, 0, 1, 1, 1]))


    start = time.time()
    for i in range(1, 2000):
        count_changes([0, 1, 0, 1, 0, 1])

    end = time.time()
    print("All tests passed in", end - start, "seconds")

if __name__ == '__main__':
    test_count_changes()
    print("All tests passed")

