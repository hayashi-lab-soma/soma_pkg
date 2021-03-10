import numpy as np

# Definitions
TREE_LOCATION_PATH = './soma_tools/data/TreeLocations_Mirais.txt'
TRAIN_DATASET = './soma_tools/data/train_dataset.txt'

if __name__ == '__main__':
    tree_locations = np.loadtxt(TREE_LOCATION_PATH, comments='#')
    num_trees = len(tree_locations)
    print("Number of trees:",num_trees)
