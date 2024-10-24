#!/usr/bin/env python

import rospy
import cv2
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool  # Importa il tipo di messaggio Bool


class DroneImageSaver:
    def __init__(self):
        # Inizializza il nodo ROS
        rospy.init_node('drone_image_saver', anonymous=True)

        # Directory dove salvare immagini e video
        self.image_dir = rospy.get_param('~image_dir', '/tmp/drone_images')
        self.video_dir = rospy.get_param('~video_dir', '/tmp/drone_videos')

        # Frame rate per il salvataggio video
        self.frame_rate = rospy.get_param('~frame_rate', 20)
        self.record_duration = rospy.get_param('~record_duration', 10)
        self.bridge = CvBridge()
        self.video_writer = None
        self.image_counter = 0

        # Crea le directory per immagini e video, se non esistono
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
        if not os.path.exists(self.video_dir):
            os.makedirs(self.video_dir)

        # Sottoscrizione al topic della RGB Camera
        rospy.Subscriber('/carla/flying_sensor/rgb_down/image', Image, self.image_callback)

        # Sottoscrizione al topic per il controllo della registrazione
        rospy.Subscriber('/toggle_recording', Bool, self.toggle_video_recording)

        # Variabili per la gestione del video
        self.recording = False
        self.start_time = None

        rospy.loginfo("DroneImageSaver initialized.")

    def image_callback(self, msg):
        # Converti l'immagine ROS in OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Errore nella conversione dell'immagine: {e}")
            return

        # Salva l'immagine
        image_file = os.path.join(self.image_dir, f'image_{self.image_counter:04d}.jpg')
        cv2.imwrite(image_file, cv_image)
        rospy.loginfo(f"Immagine salvata: {image_file}")
        self.image_counter += 1

        # Gestione del video
        if self.recording:
            if self.video_writer is None:
                self.start_video(cv_image)

            self.video_writer.write(cv_image)

            # Log del tempo trascorso
            rospy.loginfo(f"Tempo trascorso: {rospy.get_time() - self.start_time} secondi")

            # Fermare la registrazione dopo 10 secondi (di default)
            if rospy.get_time() - self.start_time > self.record_duration:
                self.stop_video()

    def start_video(self, first_frame):
        video_file = os.path.join(self.video_dir, f'video_{int(rospy.get_time())}.avi')
        height, width, _ = first_frame.shape
        self.video_writer = cv2.VideoWriter(video_file, cv2.VideoWriter_fourcc(*'MJPG'), self.frame_rate, (width, height))
        self.start_time = rospy.get_time()
        rospy.loginfo(f"Registrazione video iniziata: {video_file}")

    def stop_video(self):
        if self.video_writer:
            self.video_writer.release()
            rospy.loginfo("Registrazione video terminata.")
        self.video_writer = None
        self.recording = False

    def toggle_video_recording(self, msg):
        if msg.data:
            if not self.recording:
                self.recording = True
                rospy.loginfo("Inizio registrazione video.")
            else:
                rospy.loginfo("La registrazione è già avviata.")
        else:
            if self.recording:
                rospy.loginfo("La registrazione è in esecuzione, aspetta che si fermi.")
            else:
                rospy.loginfo("La registrazione è già ferma.")


if __name__ == '__main__':
    try:
        image_saver = DroneImageSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
