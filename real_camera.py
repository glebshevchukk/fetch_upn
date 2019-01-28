#Adapted from https://github.com/tensorflow/models/blob/master/research/tcn/dataset/webcam.py

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import multiprocessing
from multiprocessing import Process
import os
import subprocess
import sys
import time
import cv2
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import animation  # pylint: disable=g-import-not-at-top
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


class RealCamera(object):

  def __init__(self,port):

    self.get_camera(port)
    self.current_img = None
    self.height = 100
    self.width = 100

    # Create a queue per view for displaying and saving images.
    self.display_queues = [ImageQueue()]
    self.reconcile_queues = [ImageQueue()]

    # Create a queue for collecting all tuples of multi-view images to write to
    # disk.
    self.write_queue = ImageQueue()

    self.processes = []
    # Create a process to display collected images in real time.
    self.processes.append(Process(target=display_webcams, args=(self.display_queues,)))

    # Create a process to collect the latest simultaneous images from each view.
    self.processes.append(Process(
        target=reconcile, args=(self.reconcile_queues, self.write_queue,)))

    cameras=[self.active_camera]

    for (cam, dq, rq) in zip(cameras, self.display_queues, self.reconcile_queues):
      self.processes.append(Process(
          target=capture_webcam, args=(cam, dq, rq,)))
    print("Made all camera processes")

  def get_camera(self,port):

    camera = cv2.VideoCapture(port)

    if not camera.isOpened():
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
        camera = cv2.VideoCapture(port)
      except subprocess.CalledProcessError:
        raise ValueError(
              'Cannot connect to cameras.')

    # Verify camera is able to capture images.
    self.active_camera = camera
    #im = self.capture(-1)
    #assert im is not False

  def capture(self, timestep):

    if self.write_queue.empty():
      return None

    im = self.write_queue.get()[0]

    return np.array(im)


  def close(self):
    if self.active_camera:
      for q in self.display_queues + self.reconcile_queues:
        q.close()
      self.active_camera.release()
      sys.exit(0)


class ImageQueue(object):
  """An image queue holding each stream's most recent image.
  Basically implements a process-safe collections.deque(maxlen=1).
  """

  def __init__(self):
    self.lock = multiprocessing.Lock()
    self._queue = multiprocessing.Queue(maxsize=1)

  def append(self, data):
    with self.lock:
      if self._queue.full():
        # Pop the first element.
        _ = self._queue.get()
      self._queue.put(data)

  def get(self):
    with self.lock:
      return self._queue.get()

  def empty(self):
    return self._queue.empty()

  def close(self):
    return self._queue.close()


class WebcamViewer(object):
  """A class which displays a live stream from the webcams."""

  def __init__(self, display_queues):
    """Create a WebcamViewer instance."""
    self.height = 100
    self.width = 100
    self.queues = display_queues

  def _get_next_images(self):
    """Gets the next image to display."""
    # Wait for one image per view.
    not_found = True
    while not_found:
      if True in [q.empty() for q in self.queues]:
        # At least one image queue is empty; wait.
        continue
      else:
        # Retrieve the images.
        latest = [q.get() for q in self.queues]

        combined = np.concatenate(latest, axis=1)
      not_found = False
    return combined

  def run(self):
    """Displays the Kcam live stream in a window.
    This function blocks until the window is closed.
    """
    fig, rgb_axis = plt.subplots()

    print("Running webcam viewer")

    image_rows = self.height
    image_cols = self.width
    initial_image = np.zeros((image_rows, image_cols, 3))
    rgb_image = rgb_axis.imshow(initial_image, interpolation='nearest')

    def update_figure(frame_index):
      """Animation function for matplotlib FuncAnimation. Updates the image.
      Args:
        frame_index: The frame number.
      Returns:
        An iterable of matplotlib drawables to clear.
      """
      _ = frame_index
      images = self._get_next_images()
      rgb_image.set_array(images)
      return rgb_image,

    # We must keep a reference to this animation in order for it to work.
    unused_animation = animation.FuncAnimation(
        fig, update_figure, interval=50, blit=True)
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    plt.show()



#Independent method
def reconcile(queues, write_queue):
  """Gets a list of concurrent images from each view queue.
  This waits for latest images to be available in all view queues,
  then continuously:
  - Creates a list of current images for each view.
  - Writes the list to a queue of image lists to write to disk.
  Args:
    queues: A list of `ImageQueues`, holding the latest image from each webcam.
    write_queue: A multiprocessing.Queue holding lists of concurrent images.
  """
  # Loop forever.
  while True:
    # Wait till all queues have an image.
    if True in [q.empty() for q in queues]:
      continue
    else:
      # Retrieve all views' images.
      latest = [q.get() for q in queues]
      # Copy the list of all concurrent images to the write queue.
      write_queue.append(latest)

def get_image(camera):
  data = camera.read()
  _, im = data
  im = np.array(im)
  return im

#Independent process
def capture_webcam(camera, display_queue, reconcile_queue):
  """Captures images from simultaneous webcams, writes them to queues.
  Args:
    camera: A cv2.VideoCapture object representing an open webcam stream.
    display_queue: An ImageQueue.
    reconcile_queue: An ImageQueue.
  """
  while True:
    # Get images for all cameras.
    im = get_image(camera)

    im = preprocess_image(im)
    # Replace the current image in the display and reconcile queues.
    display_queue.append(im)
    reconcile_queue.append(im)


def display_webcams(display_queues):
  """Builds an WebcamViewer to animate incoming images, runs it."""
  viewer = WebcamViewer(display_queues)
  viewer.run()


def preprocess_image(np_image):

    np_image = np_image[...,[2,1,0]]
    image = Image.fromarray(np_image, 'RGB')

    #image = image.convert('L')
    #left, top, right, bottom
    image = image.crop((80, 0, 560, 480))
    image = image.resize((100, 100))

    return np.array(image)