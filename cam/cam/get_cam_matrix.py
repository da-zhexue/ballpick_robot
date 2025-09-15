import numpy as np
import cv2
import glob
import os

def calibrate_camera(images_path, chessboard_size, square_size, output_file=None):
    """
    使用棋盘格图像进行相机标定
    
    参数:
    - images_path: 包含标定图像的路径，支持通配符 (如: 'calibration_images/*.jpg')
    - chessboard_size: 棋盘格内角点数量 (width, height)
    - square_size: 每个棋盘格方格的物理尺寸 (单位: 米)
    - output_file: 可选，保存标定结果的文件路径
    
    返回:
    - camera_matrix: 相机内参矩阵
    - dist_coeffs: 畸变系数
    - rvecs: 旋转向量列表
    - tvecs: 平移向量列表
    """
    
    # 定义标定板的3D点 (对象点)
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size  # 乘以实际尺寸
    
    # 存储所有图像的对象点和图像点
    objpoints = []  # 3D点在世界坐标系中
    imgpoints = []  # 2D点在图像平面中
    
    # 获取所有标定图像
    images = glob.glob(images_path)
    
    if not images:
        print(f"在路径 {images_path} 中没有找到图像")
        return None, None, None, None
    
    print(f"找到 {len(images)} 张图像")
    
    # 遍历所有图像
    for i, fname in enumerate(images):
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # 查找棋盘格角点
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        
        # 如果找到，添加对象点和图像点
        if ret:
            objpoints.append(objp)
            
            # 提高角点检测精度
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            
            # 绘制并显示角点
            img = cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
            cv2.imshow('Chessboard', img)
            cv2.waitKey(500)  # 显示0.5秒
            
            print(f"图像 {i+1}/{len(images)}: 成功检测到角点")
        else:
            print(f"图像 {i+1}/{len(images)}: 未检测到角点")
    
    cv2.destroyAllWindows()
    
    if not objpoints:
        print("没有在任何图像中检测到角点，请检查棋盘格大小和图像质量")
        return None, None, None, None
    
    print(f"成功处理了 {len(objpoints)} 张图像，正在进行标定...")
    
    # 执行相机标定
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)
    
    if not ret:
        print("相机标定失败")
        return None, None, None, None
    
    # 打印标定结果
    print("\n相机标定完成!")
    print(f"重投影误差: {ret}")
    print("\n相机内参矩阵:")
    print(camera_matrix)
    print("\n畸变系数:")
    print(dist_coeffs)
    
    # 保存标定结果到文件
    if output_file:
        np.savez(output_file, 
                camera_matrix=camera_matrix, 
                dist_coeffs=dist_coeffs,
                rvecs=rvecs,
                tvecs=tvecs)
        print(f"\n标定结果已保存到 {output_file}")
    
    return camera_matrix, dist_coeffs, rvecs, tvecs

def load_calibration_results(calibration_file):
    """
    从文件加载相机标定结果
    
    参数:
    - calibration_file: 包含标定结果的.npz文件路径
    
    返回:
    - camera_matrix: 相机内参矩阵
    - dist_coeffs: 畸变系数
    """
    if not os.path.exists(calibration_file):
        print(f"标定文件 {calibration_file} 不存在")
        return None, None
    
    data = np.load(calibration_file)
    camera_matrix = data['camera_matrix']
    dist_coeffs = data['dist_coeffs']
    
    print("已加载相机标定结果:")
    print("相机内参矩阵:")
    print(camera_matrix)
    print("畸变系数:")
    print(dist_coeffs)
    
    return camera_matrix, dist_coeffs

def test_calibration(images_path, camera_matrix, dist_coeffs):
    """
    测试标定结果，显示校正后的图像
    
    参数:
    - images_path: 测试图像的路径，支持通配符
    - camera_matrix: 相机内参矩阵
    - dist_coeffs: 畸变系数
    """
    images = glob.glob(images_path)
    
    if not images:
        print(f"在路径 {images_path} 中没有找到测试图像")
        return
    
    for i, fname in enumerate(images):
        img = cv2.imread(fname)
        h, w = img.shape[:2]
        
        # 校正图像
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            camera_matrix, dist_coeffs, (w, h), 1, (w, h))
        undistorted = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)
        
        # 裁剪图像
        x, y, w, h = roi
        undistorted = undistorted[y:y+h, x:x+w]
        
        # 显示原始和校正后的图像
        combined = np.hstack((img, undistorted))
        cv2.imshow('原始图像 (左) vs 校正后图像 (右)', combined)
        cv2.waitKey(1000)
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # 设置参数
    images_path = "calibration_images/*.jpg"  # 标定图像路径
    chessboard_size = (9, 6)  # 棋盘格内角点数量 (width, height)
    square_size = 0.025  # 每个棋盘格方格的物理尺寸 (单位: 米，这里是2.5厘米)
    calibration_file = "camera_calibration.npz"  # 保存标定结果的文件
    
    # 执行相机标定
    camera_matrix, dist_coeffs, rvecs, tvecs = calibrate_camera(
        images_path, chessboard_size, square_size, calibration_file)
    
    if camera_matrix is not None:
        # 测试标定结果
        test_images_path = "test_images/*.jpg"  # 测试图像路径
        test_calibration(test_images_path, camera_matrix, dist_coeffs)
        
        # 演示如何加载标定结果
        print("\n演示加载标定结果:")
        loaded_camera_matrix, loaded_dist_coeffs = load_calibration_results(calibration_file)