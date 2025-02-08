#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Obtener el diccionario de ArUco
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.__getattribute__("DICT_4X4_50"))

# Crear los parámetros del detector
arucoParams = cv2.aruco.DetectorParameters()

# Crear el detector de ArUco
detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

# Crear el objeto CvBridge
bridge = CvBridge()

def aruco_display(corners, ids, rejected, image):
    if len(corners) > 0:
        ids = ids.flatten()
        
        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
            
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            
            cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
            print("[Inference] ArUco marker ID: {}".format(markerID))
            
    return image

def image_callback(msg):
    try:
        # Convertir la imagen de ROS a OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Detectar los marcadores ArUco
        corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, arucoDict, parameters=arucoParams)
        
        # Mostrar los marcadores detectados
        detected_markers = aruco_display(corners, ids, rejected, cv_image)
        
        # Mostrar la imagen con los marcadores detectados
        cv2.imshow("ArUco Detection", detected_markers)
        cv2.waitKey(1)
        
    except Exception as e:
        rospy.logerr("Error al procesar la imagen: %s", str(e))

def main():
    rospy.init_node('aruco_subscriber', anonymous=True)
    rospy.loginfo("Iniciando el suscriptor de ArUco...")
    
    # Suscribirse al tópico de la cámara
    rospy.Subscriber('/camera/image_raw', Image, image_callback)
    
    # Mantener el nodo en ejecución
    rospy.spin()

    # Cerrar todas las ventanas de OpenCV al salir
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass