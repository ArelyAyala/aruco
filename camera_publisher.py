#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node('camera_publisher', anonymous=True)
    rospy.loginfo("Iniciando el publicador de cámara...")  # Mensaje para verificar que el nodo se inicializó

    # Crear el publicador
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    rospy.loginfo("Publicador de imagen creado")

    # Crear el objeto CvBridge
    bridge = CvBridge()
    rospy.loginfo("CvBridge creado")

    # Abrir la cámara (0 es la cámara por defecto)
    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        rospy.logerr("No se pudo abrir la cámara")
        return

    rospy.loginfo("Cámara abierta correctamente")

    rate = rospy.Rate(30)  # Frecuencia de 30 Hz

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("No se pudo capturar la imagen de la cámara")
            break

        try:
            # Convertir la imagen de OpenCV a un mensaje ROS
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            image_pub.publish(ros_image)
            rospy.loginfo("Imagen publicada")  # Mensaje para indicar que se publicó la imagen

        except Exception as e:
            rospy.logerr("Error al convertir la imagen: %s", str(e))

        rate.sleep()

    # Liberar la cámara cuando se termine
    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
