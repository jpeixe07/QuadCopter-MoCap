import cv2
import numpy as np
from pseyepy import Camera, Display

#TODO: Put the camera arguments into a .yaml
class PS3Calib:
    def __init__(self, fps=60, resolution=Camera.RES_LARGE, gain=10 , auto_wb=True, rgb=True, display=False):
        self.cam = Camera(fps=fps, resolution=resolution, colour=rgb, gain=gain, auto_whitebalance=auto_wb)
        if display:
            # Another way to configure the camera pipeline
            self.d = Display(self.cam)
            
        self.CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)    
        # Chessboard configuration
        self.squareSize = 0.040  # Square size in meters (4 cm)
        self.chessCorners = (10, 9)
        # Prepare object points
        self.objp = np.zeros((self.chessCorners[0] * self.chessCorners[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.chessCorners[0], 0:self.chessCorners[1]].T.reshape(-1, 2) * self.squareSize

        # Lists to store object points and image points
        self.objPoints = []
        self.imgPoints = []

        self.requiredFrames = 0 # Define how many valid calibration frames you need
        self.frameCount = 0

    def capture_data(self, required_frames=15):
        """
        Register the frames that have the chessboard corners for further analysis with cv2.imwrite()
        Args:
        (int) required_frames: Number of frames to save

        Output:
        None
        """
        self.requiredFrames = required_frames
        while self.frameCount < self.requiredFrames:
            image = self.get_frame(isRotated=True)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            self.imgShape = gray.shape[::-1]
            found, corners = cv2.findChessboardCorners(gray, self.chessCorners, None)

            if found:
                #Visualize the corners
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.CRITERIA)
                
                cv2.drawChessboardCorners(image, self.chessCorners, corners2, found)

                
                print("Press 's' to save, 'n' to skip or 'ESC' to exit.")

                while True:
                    key = cv2.waitKey(0) & 0xFF
                    if key == ord('s'):  # Salvar o frame
                        self.save_image(image)
                        self.frameCount += 1
                        print(f"Saved Frame ({self.frameCount}/{self.requiredFrames})")
                        
                        self.objPoints.append(self.objp)
                        self.imgPoints.append(corners2)

                        break


                    elif key == ord('n'):  # Ignorar o frame e continuar
                        print("Frame ignorado.")
                        break

                    elif key == 27:  # Pressionar 'ESC' para sair
                        print("Exiting...")
                        cv2.destroyAllWindows()
                        return  # Sai completamente da função
                    
            cv2.imshow('Chessboard Detection', image)
            cv2.waitKey(1)
        print("Done")
        cv2.destroyAllWindows()

    def calibrate(self, alphaScaling=0):
        """
        Perform the camera calibration using the data captured by capture_data()
        Args:
        float alphaScaling: Scaling factor for the new camera matrix ranging from 0 to 1. 

        Output:
        (tuple) mtx, dist, rvecs, tvecs: Camera matrix, distortion coefficients, 
                                        rotation vectors and translation vectors
        """        
        
        print("\nPerforming Camera Calibration...")
        ret, mtx, dist, rvecs, tvecs =  cv2.calibrateCamera(self.objPoints, self.imgPoints, self.imgShape, None, None)

        print("Camera Matrix:\n", mtx)
        print("Distortion Coefficients:\n", dist)
        if ret:
            frame = self.get_frame(isRotated=True)

            h, w = frame.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), alphaScaling, (w, h))
            undistorted = cv2.undistort(frame, mtx, dist, None, newcameramtx)

            x, y, w, h = roi
            undistorted = undistorted[y:y+h, x:x+w]

            cv2.imshow("Undistorted Image", undistorted)
            cv2.imwrite("calibresult.png", undistorted)
            print("New Camera Matrix:\n", newcameramtx)

            cv2.waitKey(0)
            cv2.destroyAllWindows()

            return mtx, dist, rvecs, tvecs
    
    # def calibrate(self):
    #     """
    #     Perform the camera calibration using already captured data, 
    #     the function presumes that the images are stored in the same directory as the script. 
    #     The images should be named as frame_XX.png where XX is the frame number starting from 00.
    #     Args:
    #     None
    #     """           

    def get_frame(self, isRotated=False):
        image = self.cam.read()
        frame = cv2.cvtColor(image[0], cv2.COLOR_RGB2BGR)
        if isRotated:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        return frame
    
    def save_image(self, image):
        """
        Salva a imagem com um nome formatado e incrementa o contador de frames.

        Args:
            image (numpy.ndarray): A imagem a ser salva.

        Output:
            None
        """
        filename = f"frame_{self.frameCount:02d}.png"
        cv2.imwrite(filename, image)
        print(f"Frame salvo: {filename}")
    
    def save_result(self, mtx, dist):
        """
        Salva a matriz da câmera e os coeficientes de distorção em um arquivo .npz

        Args:
            mtx (numpy.ndarray): Matriz da câmera.
            dist (numpy.ndarray): Coeficientes de distorção.

        Output:
            None
        """
        np.savez("ps3-eye_calibration.npz", mtx=mtx, dist=dist)
        print("Calibração salva com sucesso!")

    def eval_reprojection(self, mtx, dist, rvecs, tvecs):
        """
        Evaluate the reprojection error of the calibration
        Args:
        None
        """
        mean_error = 0
        for i in range(len(self.objPoints)):
            imgpoints2, _ = cv2.projectPoints(self.objPoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(self.imgPoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error
        print("\nTotal error: {}".format(mean_error / len(self.objPoints)))



if __name__ == "__main__":
    calib = PS3Calib()
    calib.capture_data(required_frames=15)
    mtx, dist, rvecs, tvecs = calib.calibrate()
    calib.eval_reprojection(mtx, dist, rvecs, tvecs)
    calib.save_result(mtx,dist)