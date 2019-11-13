import numpy as np
from transformer import Transformer
import math
import time
import rospy


class Camera(object):

    def __init__(self):
        '''https://codeyarns.com/2015/09/08/how-to-compute-intrinsic-camera-matrix-for-a-camera/'''
        self.cx = 320.0
        self.cy = 240.0
        self.cdiag = 400.0
        self.ax = 57.5
        self.ay = 45.0
        self.adiag = 69.0
        self.fx = self.cx / math.tan(self.ax * math.pi / 180.0 / 2.0)
        self.fy = self.cy / math.tan(self.ay * math.pi / 180.0 / 2.0)
        self.fdiag = self.cdiag / math.tan(self.adiag * math.pi / 180.0 / 2.0)
        self.scale_factor = 51.59

        self.head = np.array([0.107, 0.0206, 1.038, 1])
        self.cube = np.array([0.8, 0.1, 0.72, 1])
        # self.cube = np.array([0.757, 0.5, 0.8, 1])

        tf = Transformer()
        self.base_to_head = tf.transform_matrix_of_frames(
            'base_link', 'head_camera_depth_optical_frame')
        self.head_to_base = tf.transform_matrix_of_frames(
            'head_camera_depth_optical_frame', 'base_link')

        self.b2r = tf.transform_matrix_of_frames(
            'base_link', 'head_camera_rgb_optical_frame')
        self.r2b = tf.transform_matrix_of_frames(
            'head_camera_rgb_optical_frame', 'base_link')

        self.b2c = tf.transform_matrix_of_frames(
            'base_link', 'head_camera_link')
        self.c2b = tf.transform_matrix_of_frames(
            'head_camera_link', 'base_link')

        self.d2b = tf.transform_matrix_of_frames(
            'head_camera_depth_frame', 'base_link')

        self.cube_h = np.dot(self.base_to_head, np.reshape(self.cube, (4, 1)))

        # The depth camera intrinsics
        self.camera_matrix = np.zeros((3, 3))
        self.camera_matrix[0, 0] = self.fx
        self.camera_matrix[1, 1] = self.fy
        self.camera_matrix[2, 2] = 1
        self.camera_matrix[0, 2] = self.cx
        self.camera_matrix[1, 2] = self.cy

    # def calc_3d_position(self, depth, u, v):
    def calc_3d_position(self, y, x, z):
        '''Return the position in base_link coordinate
        '''
        # Bacause the opencv coordinate is the opposite of image coordinate
        # (u, v) = (v, u)
        # z = depth[u, v] / self.scale_factor
        # if z == 0:
        #     return None, None, None
        #
        # x = (u - self.cx) * z / self.fx
        # y = (v - self.cy) * z / self.fy

        self.position_h = [x, y, z]
        # print 'position_h => ', self.position_h
        self.position_c = [z, -1 * x, -1 * y]

        position = np.array((x, y, z))
        position = np.dot(
            self.head_to_base[0:3, 0:3],
            position
        ) + self.head_to_base[0:3, 3]
        return position

    def convert_head_to_base(self, array, is_point=True):
        ret = np.dot(self.head_to_base[0:3, 0:3], array)  # Rotate
        if is_point:
            ret = ret + self.head_to_base[0:3, 3]
        return ret

    def convert_base_to_head(self, array, is_point=True):
        ret = np.dot(self.base_to_head[0:3, 0:3], array)  # Rotate
        if is_point:
            ret = ret + self.base_to_head[0:3, 3]
        return ret

    def convert_point_to_base(self, array):
        ret = np.dot(self.r2b[0:3, 0:3], array)  # Rotate
        ret = ret + self.r2b[0:3, 3]
        return ret


if __name__ == '__main__':
    rospy.init_node('camera_position_transformer')
    print time.strftime('%H-%M-%S')
    print('============================')
    cm = Camera()
    print(time.strftime('%H-%M-%S'))
    print('============================')
    print(cm.calc_3d_position(-0.0367245, -0.180973, 1.01207) - [0, 0, 0.16])
    print(cm.calc_3d_position(-0.0296898, -0.395445, 1.03899) - [0, 0, 0.16])
    print(cm.calc_3d_position(-0.0107672, 0.144967, 0.993605) - [0, 0, 0.16])
    print(cm.calc_3d_position(0.137383, -0.0579488, 0.647615) - [0, 0, 0.16])
    print(cm.calc_3d_position(0.137334, -0.353826, 0.672351) - [0, 0, 0.16])
    print(cm.calc_3d_position(0.131229, 0.264412, 0.675649) - [0, 0, 0.16])
