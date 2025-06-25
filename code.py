import heapq
import matplotlib.pyplot as plt
import numpy as np


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])
