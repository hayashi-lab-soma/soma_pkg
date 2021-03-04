#!/usr/bin/env python
import numpy as np
import cv2
import networkx as nx
import matplotlib
import matplotlib.pyplot as plt

def main():
    print('python test via rosrun')
    print('OpenCV: {}'.format(cv2.__version__))


    G = nx.complete_graph(10)
    print('Gc:{},{}'.format(nx.number_of_nodes(G),nx.number_of_edges(G)))


    t = np.arange(0.0,2.0,0.01)
    s = 1 + np.sin(2*np.pi*t)

    fig,ax = plt.subplots()
    ax.plot(t, s)
    plt.show()

    return None

if __name__=='__main__':
    main()