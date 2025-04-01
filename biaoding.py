#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os
import threading

# 标定板参数
BOARD_SIZE = (8, 6)        # 内部角点数量 (cols, rows)
SQUARE_SIZE = 0.028        # 棋盘格物理边长 (米)

# 亚像素优化参数
SUB_PIX_WIN_SIZE = (10, 10)
SUB_PIX_MAX_ITER = 30
SUB_PIX_EPSILON = 0.001

# ROS参数
IMAGE_TOPIC = "/image_raw"
QUEUE_SIZE = 10

class CameraCalibrator(Node):
    def __init__(self):
        super().__init__("camera_calibrator")
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, IMAGE_TOPIC, self.image_callback, QUEUE_SIZE
        )
        
        # 标定数据存储
        self.obj_points = []
        self.img_points = []
        self.img_size = None
        
        # 线程锁
        self.lock = threading.Lock()
        
        # 创建标定图像保存目录
        self.save_dir = "calib_images"
        os.makedirs(self.save_dir, exist_ok=True)
        self.frame_count = 0
        
        print("等待图像数据... (按空格键保存当前帧，'c'开始标定)")

    def enhance_contrast(self, img):
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        l_clahe = clahe.apply(l)
        merged = cv2.merge((l_clahe, a, b))
        return cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)

    def find_corners(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        enhanced = self.enhance_contrast(img)
        
        ret, corners = cv2.findChessboardCorners(
            enhanced, BOARD_SIZE,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH +
                  cv2.CALIB_CB_FAST_CHECK +
                  cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        
        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 
                      SUB_PIX_MAX_ITER, SUB_PIX_EPSILON)
            cv2.cornerSubPix(gray, corners, SUB_PIX_WIN_SIZE, (-1,-1), criteria)
            
            # 可视化角点
            cv2.drawChessboardCorners(img, BOARD_SIZE, corners, ret)
        
        return ret, corners, img

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {str(e)}")
            return
        
        if self.img_size is None:
            self.img_size = (cv_image.shape[1], cv_image.shape[0])
        
        # 检测角点
        ret, corners, disp_img = self.find_corners(cv_image)
        
        # 显示实时画面
        cv2.imshow("Calibration View", disp_img)
        key = cv2.waitKey(1)
        
        # 空格键保存有效帧
        if key == 32 and ret:
            with self.lock:
                self.frame_count += 1
                save_path = os.path.join(self.save_dir, f"frame_{self.frame_count:04d}.jpg")
                cv2.imwrite(save_path, cv_image)
                print(f"已保存标定帧: {save_path}")
                
                # 添加标定数据
                objp = np.zeros((BOARD_SIZE[0]*BOARD_SIZE[1], 3), np.float32)
                objp[:,:2] = np.mgrid[0:BOARD_SIZE[0], 0:BOARD_SIZE[1]].T.reshape(-1,2) * SQUARE_SIZE
                self.obj_points.append(objp)
                self.img_points.append(corners)
        
        # 'c'键开始标定
        elif key == ord('c'):
            self.perform_calibration()

    def perform_calibration(self):
        if len(self.obj_points) < 20:
            print(f"需要至少20组数据，当前: {len(self.obj_points)}")
            return
        
        # 初始焦距估算
        sensor_width_mm = 5.7  # 1/2.5"传感器
        fx = (self.img_size[0] * 12) / sensor_width_mm
        
        camera_matrix = np.array([
            [fx, 0, self.img_size[0]/2],
            [0, fx, self.img_size[1]/2],
            [0, 0, 1]
        ], dtype=np.float64)
        
        # 标定参数
        flags = (cv2.CALIB_USE_INTRINSIC_GUESS + 
                cv2.CALIB_RATIONAL_MODEL +
                cv2.CALIB_FIX_ASPECT_RATIO)
        
        try:
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                self.obj_points, self.img_points, self.img_size,
                camera_matrix, None, flags=flags
            )
        except Exception as e:
            print(f"标定失败: {str(e)}")
            return
        
        # 保存结果
        self.save_yaml(mtx, dist, ret)
        print("标定完成! 按Ctrl+C退出")
        cv2.destroyAllWindows()

    def save_yaml(self, mtx, dist, error):
        data = {
            "image_width": self.img_size[0],
            "image_height": self.img_size[1],
            "camera_name": "camera",
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": mtx.flatten().tolist()
            },
            "distortion_model": "rational_polynomial",
            "distortion_coefficients": {
                "rows": 1,
                "cols": 8,
                "data": dist.flatten().tolist()
            },
            "reprojection_error": float(error)
        }
        
        with open("ost.yaml", 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        print("结果已保存至ost.yaml")

def main(args=None):
    rclpy.init(args=args)
    calibrator = CameraCalibrator()
    
    # 使用多线程执行器处理ROS回调
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(calibrator)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
