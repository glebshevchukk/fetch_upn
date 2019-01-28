import tensorflow as tf
import glob
import os
import numpy as np
import pickle
from natsort import natsorted

from PIL import Image

def _int64_feature(value):
  return tf.train.Feature(int64_list=tf.train.Int64List(value=[value]))

def _float_feature(value):
  return tf.train.Feature(float_list=tf.train.FloatList(value=value))

def _bytes_feature(value):
  return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))

def preprocess_images(np_images,gif=False):
	np_images = np_images[0]
	images = []
	pil_images = []

	for np_image in np_images:

		np_image = np_image[...,[2,1,0]]
		image = Image.fromarray(np_image, 'RGB')

		#image = image.convert('L')
		#left, top, right, bottom
		image = image.crop((80, 0, 560, 480))
		image = image.resize((100, 100))

		images.append(np.array(image))

		pil_images.append(image)

	if gif:
                print(len(pil_images))
		pil_images[0].save('test_sequence.gif',
	               save_all=True,
	               append_images=pil_images[1:],
	               duration=1000,
	               loop=0)
	return np.array(images)


def convert_to(data_set, name):
  """Converts a dataset to tfrecords."""
  images = data_set['images']
  qts = data_set['qts']
  actions = data_set['actions']
  num_examples = data_set['images'].shape[0]

  T = images.shape[1]
  rows = images.shape[2]
  cols = images.shape[3]
  depth = images.shape[4]

  filename = os.path.join('/scr/kevin/unsupervised_upn/data/', name + '.tfrecords')
  print('Writing', filename)
  with tf.python_io.TFRecordWriter(filename) as writer:
	  for index in range(num_examples):
		  image_raw = images[index].tostring()
		  example = tf.train.Example(
		  features=tf.train.Features(
			  feature={
				  'T': _int64_feature(T),
				  'height': _int64_feature(rows),
				  'width': _int64_feature(cols),
				  'depth': _int64_feature(depth),
				  'actions': _float_feature(actions[index].flatten()),
				  'qts': _float_feature(qts[index].flatten()),
				  'images': _bytes_feature(image_raw)
			  }))
		  writer.write(example.SerializeToString())

def main():
	train_data_path = '/home/fetch/dev/upn/data/1k_reach_1/'
	imgs_paths = natsorted(glob.glob(train_data_path + 'images/*'))
	qts_paths = natsorted(glob.glob(train_data_path + 'qts/*'))
	acts_paths = natsorted(glob.glob(train_data_path + 'actions/*'))
	# imgs_paths = ['/scr/glebs/dev/upn/data/fetch/reach/1k_reach_0/images/images_541-542.pickle']
	# qts_paths = ['/scr/glebs/dev/upn/data/fetch/reach/1k_reach_0/qts/qts_541-542.pickle']
	# acts_paths = ['/scr/glebs/dev/upn/data/fetch/reach/1k_reach_0/actions/actions_541-542.pickle']
	all_imgs, all_qts, all_acts = [], [], []

	for i, img_path in enumerate(imgs_paths):
		print("Loaded %s" % img_path)
		gif = False
		if i is 0:
			gif=True
		all_imgs.append(preprocess_images(pickle.load(open(img_path, 'rb')),gif))
		all_acts.append(pickle.load(open(img_path.replace('img', 'action'), 'rb')))
		all_qts.append(pickle.load(open(img_path.replace('img', 'qt'), 'rb')))
	all_imgs = np.concatenate(all_imgs, axis=0)
	all_qts = np.concatenate(all_qts, axis=0)
	all_acts = np.concatenate(all_acts, axis=0)

	N, T, dX = all_qts.shape
	all_qts_norm = all_qts[:900].reshape(-1, dX)
	scale = np.diag(
		1.0 / np.maximum(np.std(all_qts_norm, axis=0), 1e-3))
	bias = - np.mean(
		all_qts_norm.dot(scale), axis=0)
	# Save the scale and bias.
	with open('/scr/glebs/dev/upn/data/fetch/reach/1k_reach_0/scale_and_bias_fetch_reach_random_1k_1.pkl', 'wb') as f:
		pickle.dump({'scale': scale, 'bias': bias}, f)
	all_qts = all_qts.reshape(-1, dX)
	all_qts = all_qts.dot(scale) + bias
	all_qts = all_qts.reshape(-1, T, dX)

	dataset_train = {'images': all_imgs[:900], 'qts': all_qts[:900], 'actions': all_acts[:900]}
	dataset_val = {'images': all_imgs[900:], 'qts': all_qts[900:], 'actions': all_acts[900:]}
	convert_to(dataset_train, '/scr/glebs/dev/upn/data/fetch/reach/1k_reach_1/tf/1k_0_train')
	convert_to(dataset_val, '/scr/glebs/dev/upn/data/fetch/reach/1k_reach_1/tf/1k_0_val')

if __name__ == "__main__":
	main()


