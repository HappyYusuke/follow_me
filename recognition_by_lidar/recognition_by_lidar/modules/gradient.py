import cv2
import numpy as np


def gradation_2d_color(start, stop, size, is_horizontal):
    if is_horizontal:
        return np.tile(np.linspace(start, stop, size[1]), (size[0], 1))
    else:
        return np.tile(np.linspace(start, stop, size[0]), (size[1], 1)).T


def gradation_3d_img(start_list, stop_list, size, is_horizontal_list):
    result = np.zeros((size[0], size[1], len(start_list)), dtype=np.float16)

    for i, (start, stop, is_horizontal) in enumerate(zip(start_list, stop_list, is_horizontal_list)):
        result[:, :, i] = gradation_2d_color(start, stop, size, is_horizontal)

    return result


if __name__ == '__main__':
    size = [1,100] #height, width
    img = gradation_3d_img([0,0,255], [255,0,0], size, [True,True,True])
    print(img)
    
    img = img.astype(np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    cv2.imwrite('out.png', img)
