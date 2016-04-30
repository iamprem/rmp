from minheap import minheap



class Vertex:
    """
    Vertex represents each node in the graph
    """
    def __init__(self, name):
        self.name = name
        self.adjEdges = set()
        self.prev = None            # Previous vertex on shortest path to source
        self.dist = float('Inf')    # Distance to source vertex
        self.heapIndex = None

        # Motion Planning DataStructure. Ignore these for general graph implementation.
        # self.bestCtrl = None     # Stores best control input that takes closer to q_rand
        # self.ctrlsPath = {}     # Stores control input as key and list of points from src to dest as values

    def getName(self):
        return self.name

    def getPrev(self):
        return self.prev

    def getDist(self):
        return self.dist

    def getHeapIndex(self):
        return self.heapIndex

    def setHeapIndex(self, heapIndex):
        self.heapIndex = heapIndex

    def setPrev(self, prev):
        self.prev = prev

    def setDist(self, dist):
        self.dist = dist

    def getAdjEdges(self):
        return self.adjEdges

    def addEdge(self, edge):
        self.adjEdges.add(edge)

    def removeEdge(self, edge):
        self.adjEdges.remove(edge)

    def __hash__(self):
        return hash(self.name)

    def __eq__(self, other):
        return self.name == other.name

    def __cmp__(self, other):
        return cmp(self.dist, other.dist)

    def __repr__(self):
        return repr(self)


class Edge:
    """
    Edge class represent a directional edge between a source and a destination Vertex
    """
    def __init__(self, src, dest, cost):
        self.scr = src
        self.dest = dest
        self.cost = cost

        # Motion Planning DataStructure. Ignore these for general graph implementation.
        self.ctrl = None    # Stores control from src that takes to dest which is closer to q_rand
        self.path = {}      # Stores control input as key and list of points from src to dest as values

    def getDest(self):
        return self.dest

    def getSrc(self):
        return self.scr

    def getCost(self):
        return self.cost

    def getSrcName(self):
        return self.scr.getName()

    def getDestName(self):
        return self.dest.getName()

    def setCost(self, cost):
        self.cost = cost
        return self

    def __repr__(self):
        return repr(self)



class Graph:
    """
    Defines the Graph Data Structure
    """
    def __init__(self):
        self.vertexMap = {}

    def addUniEdge(self, src, dest, cost, set_parent=False):
        u = self.getVertex(src, True)
        v = self.getVertex(dest, True)
        edge_uv = Edge(u, v, cost)
        u.addEdge(edge_uv)
        if set_parent:
            v.setPrev(u)
        return edge_uv

    def removeUniEdge(self, src, dest, remove_parent=True):
        u = self.getVertex(src)
        v = self.getVertex(dest)
        if remove_parent and v.getPrev() == u:
            v.setPrev(None)
        if u is not None and v is not None:
            for e in u.getAdjEdges():
                if e.getDest() == v:
                    u.removeEdge(e)
                    return True
        return False

    def addBiEdge(self, src, dest, cost):
        return self.addUniEdge(src, dest, cost), self.addUniEdge(dest, src, cost)

    def removeBiEdge(self, src, dest):
        return self.removeUniEdge(src, dest) and self.removeUniEdge(dest, src)

    def addVertex(self, u):
        return self.getVertex(u, True)

    def getVertex(self, src, create_new=False):
        if create_new and self.vertexMap.get(src) is None:
            self.vertexMap[src] = Vertex(src)
        return self.vertexMap.get(src)

    def dijkstra(self, srcName):

        src = self.getVertex(srcName)
        # Initialization
        for v in self.vertexMap.itervalues():
            v.setDist(float('Inf'))
            v.setPrev(None)

        src.setDist(0)
        Q = minheap(self.vertexMap.values())

        while len(Q.q) != 0:
            u = Q.extractMin()

            for e in u.getAdjEdges():
                alt = u.getDist() + e.getCost()
                v = e.getDest()
                if alt < v.getDist():
                    Q.heapDecreaseKey(v.getHeapIndex(), alt)
                    v.setPrev(u)
