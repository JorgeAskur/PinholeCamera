#!/usr/bin/env python3
# -- coding: utf-8 --

from cgitb import enable
from distutils.ccompiler import new_compiler
import math
from pickle import FALSE
from re import X
import rospy
from geometry_msgs.msg import Pose, Point, Vector3
from visualization_msgs.msg import Marker
from vision.msg import obj_detected_list, poses, poseList
import actionlib
from image_geometry import PinholeCameraModel, StereoCameraModel
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge
import math

#Modelo de la camara frontal
camera = PinholeCameraModel()

stereo = StereoCameraModel()

setG = False
setR = False
setL = False
obj_id = 0

def infoCamRightCB(msg):
    global stereo, setR
    if not setR:
        stereo.right.fromCameraInfo(msg)
        setR = True

def infoCamLeftCB(msg):
    global stereo, setL
    if not setL:
        stereo.left.fromCameraInfo(msg)
        setL = True

# #Callback que nos da la posicion actual del uuv
# def ins_pose_callback(pose):
#     global currentPose
#     currentPose.x = pose.position.x
#     currentPose.y = pose.position.y
#     currentPose.z = pose.position.z

def infoCamCB(msg):
    global camera, setG
    if not setG:
        camera.fromCameraInfo(msg)
        setG = True

def imageCB(msg):
    global imagen
    imagen = CvBridge().imgmsg_to_cv2(msg,"bgr8")
    
def pixelto3D(u, v, d):
    #Obtener el rayo 3D de la camara al pixel indicado
    ray = camera.projectPixelTo3dRay((u,v))

    #Multiplicar por la distancia para obtener la coordenada 
    #(se cambia el orden de las x, y y z debido a que hay una rotacion en el frame de referencia de la imagen)
    (y, z, x) = [el * (d) for el in ray]
    
    return (x, y, z)


def detected_objects_callback(msg):
    global currentPose, obj_id
    objects = poseList()
    pub = rospy.Publisher("/uuv_perception/objects", poseList, queue_size=10)
    pointPub = rospy.Publisher("objectDetected", Marker, queue_size=10)
    #Regresar lista con coordenadas de cada uno
    obj_id = 0
    class_name = "Gun"
    msgMarker = Marker()

    color = ColorRGBA()
    
    color.r = 0
    color.g = 0
    color.b = 1
    color.a = 1
    
    colorList = []
    pointList = []
    movingAverage = [[]]

    for object in msg.objects:
        item = poses()

        x, y, z = pixelto3D(object.X, object.Y, object.Depth)

        item.location.x = x 
        item.location.y = y 
        item.location.z = z 
        item.name = object.clase

        if object.clase == class_name:
            p = Point()

            p.x = x
            p.y = y
            p.z = z
            
            if not math.isnan(p.x) and not str(p.x) == "-inf":
                pointList.append(p)
                colorList.append(color)      

        objects.targets.append(item)
        objects.len += 1

    # movingAverage.append(pointList)

    # if len(movingAverage) > 5:
    #     movingAverage.pop(0)
    
    # for pList in movingAverage:
        
    msgMarker.header.stamp = rospy.get_rostime()
    msgMarker.header.frame_id = "zed2i_base_link"

    msgMarker.ns = class_name
    msgMarker.id = 0
    
    msgMarker.type = 7
    msgMarker.action = 0
    msgMarker.pose.orientation.w = 1.0
    msgMarker.scale.x = 0.1
    msgMarker.scale.y = 0.1
    msgMarker.scale.z = 0.1

    msgMarker.color.r = 0.0
    msgMarker.color.g = 1.0
    msgMarker.color.b = 0.0
    msgMarker.color.a = 1.0

    msgMarker.points = pointList
    msgMarker.colors = colorList

    if len(pointList) > 0:
        pointPub.publish(msgMarker)
        print(pointList)

    pub.publish(objects)
    
        
#Current pose
# rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, ins_pose_callback)
        
#Object detector
rospy.Subscriber('/yolov7/objects_detected', obj_detected_list, detected_objects_callback, queue_size=10)
rospy.Subscriber('/zed2i/zed_node/right_raw/camera_info', CameraInfo, infoCamRightCB, queue_size=10)
rospy.Subscriber('/zed2i/zed_node/left_raw/camera_info', CameraInfo, infoCamLeftCB, queue_size=10)
rospy.Subscriber('/zed2i/zed_node/rgb/camera_info', CameraInfo, infoCamCB)
rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color", Image, imageCB)


if __name__ == "__main__":
    try:
        rospy.init_node("pose_estimation", anonymous=False)
        start_time = rospy.get_time()
        rospy.spin()

    except rospy.ROSInterruptException:     
        pass