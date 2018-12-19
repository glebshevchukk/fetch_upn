import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import subprocess
import time

class RealCamera(object):

  def __init__(self,port):
    self.active_camera = None
    self.get_camera(port)
    self.current_img = None
    self.height = 100
    self.width = 100

  def get_camera(self,port):

    self.active_camera = cv2.VideoCapture(port)

    if not self.active_camera.isOpened():
      try:
        # Try to find and kill hanging cv2 process_ids.
        output = subprocess.check_output(['lsof -t /dev/video*'], shell=True)
        print('Found hanging cv2 process_ids: \n')
        print(output)
        print('Killing hanging processes...')
        for process_id in output.split('\n')[:-1]:
          subprocess.call(['kill %s' % process_id], shell=True)
        time.sleep(3)
        # Recapture webcam.
        self.active_camera = cv2.VideoCapture(port)
      except subprocess.CalledProcessError:
        raise ValueError(
              'Cannot connect to cameras. Try running: \n'
              'ls -ltrh /dev/video* \n '
              'to see which ports your webcams are connected to. Then hand those '
              'ports as a comma-separated list to --webcam_ports, e.g. '
              '--webcam_ports 0,1')

    # Verify camera is able to capture images.
    im = self.capture()
    assert im is not False
    self.current_img = im

  def capture(self):
    if not self.active_camera:
      print("No active camera set")
      return False

    data = self.active_camera.read()
    _, im = data

    im = np.array(im)
    self.current_img = im
    return im

  def view(self):
      fig, rgb_axis = plt.subplots()
      image_rows = self.height
      image_cols = self.width
      initial_image = np.zeros((image_rows, image_cols, 3))
      rgb_image = rgb_axis.imshow(initial_image, interpolation='nearest')


      def update_figure(frame_index):
        print("Updating figure")
        _ = frame_index
        while not self.current_img.any():
          image = self.current_img
      
        print("Got an image")
        image = image[..., [2, 1, 0]]
        rgb_image.set_array(image)
        return rgb_image
        
      # We must keep a reference to this animation in order for it to work.
      unused_animation = animation.FuncAnimation(
          fig, update_figure, interval=50, blit=True)
      mng = plt.get_current_fig_manager()
      mng.resize(*mng.window.maxsize())
      plt.show()

  def close(self):
    if self.active_camera:
      self.active_camera.release()


if __name__ == "__main__":
  camera = RealCamera(0)
  #for i in range(10):
  #  img = camera.capture()
  #print(img)
  camera.capture()
  camera.view()

  for i in range(1000):
    img = camera.capture()
    
