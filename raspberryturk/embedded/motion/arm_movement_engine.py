import numpy as np
# import os
import pickle
import logging
from raspberryturk import RaspberryTurkError, opt_path, cache_path
from sklearn.neighbors import KDTree


def _load_tree(logger):  # 寻找Tree的代码
    tree = None

    cached_tree_path = cache_path('arm_movement_engine.kdtree')
    try:
        with open(cached_tree_path, 'rb') as f:
            logger.info("Attemptimg to load kd-tree from cache...")
            tree = pickle.load(f)  # 在缓存文件当中找对应的KDTree
            logger.info("Successfully loaded kd-tree from cache.")
    except (OSError, IOError):
        logger.info("Failed to load kd-tree from {}".format(cached_tree_path))

    if tree is None:  # 如果没有找到tree
        try:
            pts_path = opt_path('arm_movement_engine_pts.npy')
            logger.info("Loading {}...".format(pts_path))
            with open(pts_path, 'rb') as f:
                pts = np.load(f)
                logger.info("Building kd-tree...")
                tree = KDTree(pts)
                logger.info("Done building kd-tree.")
                needs_to_write_cache = True
        except IOError:
            raise RaspberryTurkError("Arm movement engine can't find required file: {}".format(pts_path))
        else:
            try:
                with open(cached_tree_path, 'wb') as f:
                    logger.info("Writing kd-tree to cache...")
                    pickle.dump(tree, f)  # 保存内容
                    logger.info("Done writing kd-tree to cache.")
            except (OSError, IOError) as e:
                logger.warn("Failed to write kdtree to {}. Reason: {}.".format(cached_tree_path, e))

    return tree


class ArmMovementEngine(object):
    def __init__(self):
        logger = logging.getLogger(__name__)
        self._tree = _load_tree(logger)

    def convert_point(self, pt):
        pt = np.array(pt).reshape(-1, 2)  # 把数据转换成两列的形式
        index = self._tree.query(pt, return_distance=False).ravel()[0]
        # query 返回与传入point距离最近节点的数组i  ravel（）将数组将为一维数组， 取第0个值
        calibrated_offset = 105  # 应该是初始位姿
        return np.array([index / 1024 + calibrated_offset, index % 1024])
