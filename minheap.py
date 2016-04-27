class minheap:

    def __init__(self, x):
        self.q = list(x)
        self.buildMinHeap()

    def buildMinHeap(self):
        n = len(self.q)
        for pos in reversed(range(n/2 - 1)):
            self.minHeapify(pos)

    def minHeapify(self, pos):
        n = len(self.q)
        left = self.left(pos)
        right = self.righ(pos)
        smallest = None
        if left <= n - 1:
            if self.q[left].getDist() < self.q[pos].getDist():
                smallest = left
                self.q[left].setHeapIndex(pos)
                self.q[pos].setHeapIndex(left)
            else:
                self.q[left].setHeapIndex(left)
                self.q[pos].setHeapIndex(pos)
                smallest = pos
        else:
            self.q[pos].setHeapIndex(pos)
            smallest = pos

        if right <= n - 1:
            if self.q[right].getDist() < self.q[smallest].getDist():
                self.q[right].setHeapIndex(pos)
                self.q[pos].setHeapIndex(right)
                smallest = right
            else:
                self.q[right].setHeapIndex(right)
                self.q[pos].setHeapIndex(pos)
        if smallest != pos:
            self.q[pos], self.q[smallest] = self.q[smallest], self.q[pos]
            self.minHeapify(smallest)


    def extractMin(self):
        lastelt = self.q.pop()
        if self.q:
            returnitem = self.q[0]
            self.q[0] = lastelt
            self.minHeapify(0)
        else:
            returnitem = lastelt
        return returnitem

    def heapDecreaseKey(self, pos, key):
        if self.q[pos].getDist() < key:
            return
        self.q[pos].setDist(key)
        parent = self.parent(pos)
        while pos > 0 and self.q[parent].getDist() > self.q[pos].getDist():
            self.q[parent].setHeapIndex(pos)
            self.q[pos].setHeapIndex(parent)
            self.q[pos], self.q[parent] = self.q[parent], self.q[pos]
            pos = parent
            parent = self.parent(pos)

    def parent(self, pos):
        return (pos - 1) / 2

    def left(self, pos):
        return 2 * pos + 1

    def righ(self, pos):
        return 2 * pos + 2

