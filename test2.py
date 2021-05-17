import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from raspberryturk.core.data.dataset import Dataset
import tensorflow as tf
from random import randint
from notebooks.helpers import plot_confusion_matrix
from sklearn.metrics import confusion_matrix
tf.compat.v1.disable_eager_execution()
# because the version of tf for python3.8 causes the change of function
# add ".compat.v1" between tf and function to make sure it goes right
# you may like to delet it if you meet some mistake.


MAX_DISPLAY_STEP = 1000
LEARNING_RATE = 1e-4
TRAINING_ITERATIONS = 10000
DROPOUT = 0.5
START_BATCH_SIZE = 5
END_BATCH_SIZE = 100

d = Dataset.load_file('data/processed/dataset7.npz')

labels_count = d.y_train.shape[1]
image_size = 3600
image_width = 60
image_height = 60
labels = ['Knight', 'Bishop', 'Rook', 'Queen']

train_images = d.X_train
train_labels = d.y_train
validation_images = d.X_val
validation_labels = d.y_val

print(validation_labels)

piece_index = 0
img = train_images[piece_index].reshape(image_width, image_height)
plt.title(labels[train_labels[piece_index].argmax()])
plt.imshow(img, cmap=cm.gray)
plt.show()


def weight_variable(shape):
    initial = tf.compat.v1.truncated_normal(shape, stddev=0.1)
    return tf.compat.v1.Variable(initial)


def bias_variable(shape):
    initial = tf.compat.v1.constant(0.1, shape=shape)
    return tf.compat.v1.Variable(initial)


def conv2d(x0, W):
    return tf.compat.v1.nn.conv2d(x0, W, strides=[1, 1, 1, 1], padding='SAME')


def max_pool_2x2(x1):
    return tf.compat.v1.nn.max_pool(x1, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')


x = tf.compat.v1.placeholder('float', shape=[None, image_size])
y_ = tf.compat.v1.placeholder('float', shape=[None, labels_count])

W_conv1 = weight_variable([5, 5, 1, 32])
b_conv1 = bias_variable([32])

image = tf.compat.v1.reshape(x, [-1, image_width, image_height, 1])

h_conv1 = tf.compat.v1.nn.relu(conv2d(image, W_conv1) + b_conv1)
h_pool1 = max_pool_2x2(h_conv1)

W_conv2 = weight_variable([5, 5, 32, 64])
b_conv2 = bias_variable([64])

h_conv2 = tf.compat.v1.nn.relu(conv2d(h_pool1, W_conv2) + b_conv2)
h_pool2 = max_pool_2x2(h_conv2)

W_fc1 = weight_variable([15 * 15 * 64, 1024])
b_fc1 = bias_variable([1024])

h_pool2_flat = tf.compat.v1.reshape(h_pool2, [-1, 15 * 15 * 64])
h_fc1 = tf.compat.v1.nn.relu(tf.compat.v1.matmul(h_pool2_flat, W_fc1) + b_fc1)

keep_prob = tf.compat.v1.placeholder('float')
h_fc1_drop = tf.compat.v1.nn.dropout(h_fc1, keep_prob)

W_fc2 = weight_variable([1024, labels_count])
b_fc2 = bias_variable([labels_count])

y = tf.compat.v1.nn.softmax(tf.compat.v1.matmul(h_fc1_drop, W_fc2) + b_fc2)

cross_entropy = -tf.compat.v1.reduce_sum(y_ * tf.compat.v1.log(y))

train_step = tf.compat.v1.train.AdamOptimizer(LEARNING_RATE).minimize(cross_entropy)

correct_prediction = tf.compat.v1.equal(tf.argmax(y, 1), tf.compat.v1.argmax(y_, 1))
accuracy = tf.compat.v1.reduce_mean(tf.compat.v1.cast(correct_prediction, 'float'))

predict = tf.compat.v1.argmax(y, 1)
print(predict)

epochs_completed = 0
index_in_epoch = 0
num_examples = d.X_train.shape[0]


def next_batch(batch_size):
    global train_images
    global train_labels
    global index_in_epoch
    global epochs_completed

    start = index_in_epoch
    index_in_epoch += batch_size

    if index_in_epoch > num_examples:
        epochs_completed += 1
        perm = np.arange(num_examples)
        np.random.shuffle(perm)
        train_images = train_images[perm]
        train_labels = train_labels[perm]
        start = 0
        index_in_epoch = batch_size
        assert batch_size <= num_examples
    end = index_in_epoch
    return train_images[start:end], train_labels[start:end]


init = tf.compat.v1.global_variables_initializer()
sess = tf.compat.v1.InteractiveSession()

sess.run(init)
saver = tf.compat.v1.train.Saver()

train_accuracies = []
validation_accuracies = []
x_range = []

display_step = 10

previous_best = 0.0
for i in range(TRAINING_ITERATIONS):
    batch_size = int((END_BATCH_SIZE - START_BATCH_SIZE) * float(i) / TRAINING_ITERATIONS) + 5
    batch_xs, batch_ys = next_batch(batch_size)
    if i % display_step == 0 or (i + 1) == TRAINING_ITERATIONS:
        train_accuracy = accuracy.eval(feed_dict={x: batch_xs,
                                                  y_: batch_ys,
                                                  keep_prob: 1.0})
        val_len = 1000
        ind = randint(0, validation_images.shape[0] - val_len - 1)
        val_images_batch = validation_images[ind:ind + val_len]
        val_labels_batch = validation_labels[ind:ind + val_len]
        validation_accuracy = accuracy.eval(feed_dict={x: val_images_batch,
                                                       y_: val_labels_batch,
                                                       keep_prob: 1.0})
        print('training_accuracy / validation_accuracy => %.2f / %.2f for step %d' % (
            train_accuracy, validation_accuracy, i))
        train_accuracies.append(train_accuracy)
        validation_accuracies.append(validation_accuracy)
        x_range.append(i)
        if validation_accuracy > previous_best:
            previous_best = validation_accuracy
        if i % (display_step * 10) == 0 and i:
            display_step = min(MAX_DISPLAY_STEP, display_step * 10)

    sess.run(train_step, feed_dict={x: batch_xs, y_: batch_ys, keep_prob: DROPOUT})
print("Finished training, best batch validation model accuracy: %.5f" % (previous_best))
feed_deictt = {x: validation_images, keep_prob: 1.0}
pred = sess.run(predict, feed_dict={x: validation_images, keep_prob: 1.0})
full_validation_accuracy = accuracy.eval(feed_dict={x: validation_images,
                                                    y_: validation_labels,
                                                    keep_prob: 1.0})
print("Validation accuracy: %.5f" % full_validation_accuracy)
conf = confusion_matrix(validation_labels.argmax(axis=1), pred)

plot_confusion_matrix(conf, labels)
sess.close()
