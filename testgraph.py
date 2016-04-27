from dijkstra import *


if __name__ == '__main__':
    g = Graph()
    g.addUniEdge('a', 'b', 5)
    g.addUniEdge('a', 'c', 3)
    g.addUniEdge('c', 'd', 2)
    g.addUniEdge('c', 'e', 10)
    g.addUniEdge('d', 'b', 1)
    g.addUniEdge('d', 'e', 3)
    g.addUniEdge('d', 'f', 5)
    g.addUniEdge('e', 'f', 1)
    g.addUniEdge('f', 'b', 7)
    g.dijkstra('a')
