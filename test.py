from raspberryturk.core.data.dataset import Dataset
import numpy as np
data = Dataset.load_file('data/processed/example_dataset.npz')
# data = np.load('data/processed/example_dataset.npz')

print(data.metadata)
np.set_printoptions(precision=4)
print(data.X_train.shape)
print(data.X_train)
print(data.y_train.shape)
print(data.y_train)
print(data.X_val.shape)
print(data.y_val.shape)
print(data.zca.shape)
