import cv2
from uvc_camera import UVCCamera
import stag
import numpy as np
import json
import time
import os
from scipy.linalg import svd
from pymycobot import *
from marker_utils import *
import shutil
import glob
from pymycobot.tool_coords import CvtRotationMatrixToEulerAngle, CvtEulerAngleToRotationMatrix


ports = glob.glob('/dev/ttyTHS1') + glob.glob('/dev/ttyACM*')
print(ports)
if ports:
    arm_port = ports[0]
else:
    raise Exception("No MyCobot device found")


mc = MyCobot280(arm_port, 1000000)  # è®¾ç½®ç«¯å£
            
np.set_printoptions(suppress=True, formatter={'float_kind': '{:.2f}'.format})



class camera_detect:
    #Camera parameter initialize
    def __init__(self, camera_id, marker_size, mtx, dist):
        self.camera_id = camera_id
        self.mtx = mtx
        self.dist = dist
        self.marker_size = marker_size
        self.camera = UVCCamera(self.camera_id, self.mtx, self.dist)
        self.camera_open()

        self.origin_mycbot_horizontal = [0,60,-60,0,0,0]
        self.origin_mycbot_level = [0, 0, -90, 0, 0, 0]
   
        # Initialize EyesInHand_matrix to None or load from a document if available
        self.EyesInHand_matrix = None
        file_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.matrix_file_path = os.path.join(file_dir, "config","EyesInHand_matrix.json")
    
        self.load_matrix()

    def save_matrix(self, filename="EyesInHand_matrix.json"):
        # Save the EyesInHand_matrix to a JSON file
        if self.EyesInHand_matrix is not None:
            with open(filename, 'w') as f:
                json.dump(self.EyesInHand_matrix.tolist(), f)
                
            try:
                # å¤å¶æä»¶å°ç®æ è·¯å¾
                shutil.copy(filename, self.matrix_file_path)
                print(f"File copied to {self.matrix_file_path}")
            except IOError as e:
                print(f"Failed to copy file: {e}")
    
    def load_matrix(self, filename="EyesInHand_matrix.json"):
        # Load the EyesInHand_matrix from a JSON file, if it exists
        try:
            with open(filename, 'r') as f:
                self.EyesInHand_matrix = np.array(json.load(f))
        except FileNotFoundError:
            print("Matrix file not found. EyesInHand_matrix will be initialized later.")

    def wait(self):
        time.sleep(0.5)
        while mc.is_moving():
            time.sleep(0.2)
    
    def camera_open(self):
        self.camera.capture()  # æå¼€æåå¤´

    # è·åç©ä½åæ (ç¸æºç³»)
    def calc_markers_base_position(self, corners, ids):
        if len(corners) == 0:
            return []
        rvecs, tvecs = solve_marker_pnp(corners, self.marker_size, self.mtx, self.dist)  # é€è¿äºç»´ç è§ç¹è·åç©ä½æè½¬åéåå¹³ç§»åé
        for i, tvec, rvec in zip(ids, tvecs, rvecs):
            tvec = tvec.squeeze().tolist()
            rvec = rvec.squeeze().tolist()
            rotvector = np.array([[rvec[0], rvec[1], rvec[2]]])
            Rotation = cv2.Rodrigues(rotvector)[0]  # å°æè½¬åéè½¬ä¸ºæè½¬ç©éµ
            Euler = CvtRotationMatrixToEulerAngle(Rotation)  # å°æè½¬ç©éµè½¬ä¸ºæ¬§æè§
            target_coords = np.array([tvec[0], tvec[1], tvec[2], Euler[0], Euler[1], Euler[2]])  # ç©ä½åæ (ç¸æºç³»)
        return target_coords

    
    def eyes_in_hand_calculate(self, pose, tbe1, Mc1, tbe2, Mc2, tbe3, Mc3, Mr):

        tbe1, Mc1, tbe2, Mc2, tbe3, Mc3, Mr = map(np.array, [tbe1, Mc1, tbe2, Mc2, tbe3, Mc3, Mr])
        # Convert pose from degrees to radians
        euler = np.array(pose) * np.pi / 180
        Rbe = CvtEulerAngleToRotationMatrix(euler)
        print("Rbe", Rbe)
        Reb = Rbe.T
        
        A = np.hstack([(Mc2 - Mc1).reshape(-1, 1), 
                    (Mc3 - Mc1).reshape(-1, 1), 
                    (Mc3 - Mc2).reshape(-1, 1)])
        
        b = Reb @ np.hstack([(tbe1 - tbe2).reshape(-1, 1), 
                            (tbe1 - tbe3).reshape(-1, 1), 
                            (tbe2 - tbe3).reshape(-1, 1)])
        
        print("A = ", A)
        print("B = ", b)
        U, S, Vt = svd(A @ b.T)
        Rce = Vt.T @ U.T
        
        tce = Reb @ (Mr - (1/3)*(tbe1 + tbe2 + tbe3) - (1/3)*(Rbe @ Rce @ (Mc1 + Mc2 + Mc3)))
        
        eyes_in_hand_matrix = np.vstack([np.hstack([Rce, tce.reshape(-1, 1)]), np.array([0, 0, 0, 1])])
        
        return eyes_in_hand_matrix

    # è¯»åCameraåæ ï¼åæ¬¡ï¼
    def stag_identify(self):
        self.camera.update_frame()  # å·æ°ç¸æºçé¢
        frame = self.camera.color_frame()  # è·åå½åå¸§
        (corners, ids, rejected_corners) = stag.detectMarkers(frame, 11)  # è·åç»é¢ä¸­äºç»´ç çè§åº¦åid
        # ç»å¶æ£€æµå°çæ è®°åå¶ID
        stag.drawDetectedMarkers(frame, corners, ids)
        # ç»å¶è¢«æç»çå€é€åºåï¼é¢è²è®¾ä¸ºçº¢è²
        stag.drawDetectedMarkers(frame, rejected_corners, border_color=(255, 0, 0))
        marker_pos_pack = self.calc_markers_base_position(corners, ids)  # è·åç©çåæ (ç¸æºç³»)
        if(len(marker_pos_pack) == 0):
            marker_pos_pack = self.stag_identify()
        # print("Camera coords = ", marker_pos_pack)
        # cv2.imshow("rrrr", frame)
        # cv2.waitKey(1)
        return marker_pos_pack

    def Eyes_in_hand_calibration(self, ml):
        ml.send_angles(self.origin_mycbot_level, 50)  # ç§»å¨å°è§æµç¹
        time.sleep(5)
        print("movement complete")
     #   self.wait()
        input("make sure camera can observe the stag, enter any key quit")
        coords = ml.get_coords()
        while coords is None or isinstance(coords, int) or len(coords) < 6:
            print("Waiting for robot coords...")
            time.sleep(0.5)
            coords = ml.get_coords()
        pose = coords[3:6]
        print(pose)
        # self.camera_open_loop()
        Mc1,tbe1 = self.reg_get(ml)
        ml.send_coord(1, coords[0] + 30, 30)
        time.sleep(5)
     #   self.wait()
        Mc2,tbe2 = self.reg_get(ml)
        ml.send_coord(1, coords[0] - 10, 30)
        time.sleep(5)
        #self.wait()
        ml.send_coord(3, coords[2] + 20, 30)
        time.sleep(5)
      #  self.wait()
        Mc3,tbe3 = self.reg_get(ml)

        input("Move the end of the robot arm to the calibration point, press any key to release servo")
        ml.release_all_servos()
        input("focus servo and get current coords")
        ml.power_on()
        time.sleep(1)
        coords = ml.get_coords()
        while len(coords) == 0:
            coords = ml.get_coords()
        Mr = coords[0:3]
        print(Mr)

        self.EyesInHand_matrix = self.eyes_in_hand_calculate(pose, tbe1, Mc1, tbe2, Mc2, tbe3, Mc3, Mr)
        print("EyesInHand_matrix = ", self.EyesInHand_matrix)
        self.save_matrix()  # Save the matrix to a file after calculating it
        print("save successe")
    
    def reg_get(self, ml):
        for i in range(50):
            Mc_all = self.stag_identify()
        tbe_all = ml.get_coords() # è·åæºæ¢°èå½ååæ 
        while (tbe_all is None):
            tbe_all = ml.get_coords()

        tbe = tbe_all[0:3]
        Mc = Mc_all[0:3]
        print("tbe = ", tbe)
        print("Mc = ", Mc)
        return Mc,tbe


if __name__ == "__main__":
    if mc.is_power_on()==0:
        mc.power_on()
    camera_params = np.load("camera_params.npz")  # ç¸æºéç½®æä»¶
    mtx, dist = camera_params["mtx"], camera_params["dist"]
    m = camera_detect(0, 32, mtx, dist)
    tool_len = 20
  #  mc.set_tool_reference([0, 0, tool_len, 0, 0, 0])
 #   mc.set_end_type(1)

    m.Eyes_in_hand_calibration(mc)  #æç¼æ å®
    
