from operator import sub
from collections import deque
import pygame


class Visualizer:

    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    RED = (255, 0, 0)
    BLUE = (0, 0, 255)
    YELLOW = (255, 255, 0)
    PINK = (255, 192, 203)
    CYAN = (0, 255, 255)
    GREEN = (34, 139, 34)
    BROWN = (165, 42, 42)
    ENV_WIDTH = 800
    ENV_HEIGHT = 800
    RESOLUTION = ENV_WIDTH, ENV_HEIGHT

    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode(self.RESOLUTION)
        self.screen.fill((100, 186, 100))
        pygame.display.set_caption('Visualizer')
        pygame.display.update()

    def draw_square(self, point, size=10, color=GREEN):
        pygame.draw.rect(self.screen, color, (int(point[0]), int(point[1]), size, size))
        pygame.display.update()

    def define_obstacles(self):
        """
        Takes Mouse Clicks and Drags as input from user and returns its clockwise co-ordinates of the shape
        Note: If only two points are clicked, A rectangle will be returned
        :return:
        """
        OBSTACLE_DEFINED = False
        pygame.display.set_caption('Define Obstacles')
        pygame.display.update()
        obstacles = []
        poly_vertices = []
        while not OBSTACLE_DEFINED:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    OBSTACLE_DEFINED = True
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    start_pos = event.pos
                    print start_pos
                elif event.type == pygame.MOUSEBUTTONUP:
                    end_pos = event.pos
                    print end_pos
                    if start_pos == end_pos:
                        poly_vertices.append(end_pos)
                    else:
                        # If start and end position changed, Means mouse drag, So draw the rectangle
                        width, height = tuple(map(sub, end_pos, start_pos))
                        obstacles.append([start_pos,
                                          (start_pos[0], start_pos[1]+height),
                                          end_pos,
                                          (end_pos[0], end_pos[1]-height)])
                        pygame.draw.rect(self.screen, self.RED, start_pos + (width, height))
                        pygame.display.update()
                # On press ENTER draw the polygon from mouse clicked points
                elif event.type == pygame.KEYUP and event.key == pygame.K_RETURN:
                    print poly_vertices
                    if len(poly_vertices) < 2: continue
                    if len(poly_vertices) == 2:
                        start_pos, end_pos = poly_vertices[0], poly_vertices[1]
                        width, height = tuple(map(sub, end_pos, start_pos))
                        poly_vertices = [start_pos,
                                         (start_pos[0], start_pos[1]+height),
                                         end_pos,
                                         (end_pos[0], end_pos[1]-height)]
                    obstacles.append(poly_vertices)
                    pygame.draw.polygon(self.screen, self.RED, poly_vertices)
                    pygame.display.update()
                    poly_vertices = []
        return obstacles

    def plot_graph(self, tree, src):
        """
        Draw a Tree/Graph on pygame screen based on the graph/tree connectivity
        :param tree:
        :return:
        """
        node_list = deque([tree.getVertex(src)])
        while len(node_list):
            vtx = node_list.popleft()
            for edge in vtx.getAdjEdges():
                node_list.append(edge.getDest())
                self.draw_line(edge.getSrcName()[0:2], edge.getDestName()[0:2])
        print 'plot_graph:: Done!'

    def nh_plot_graph(self, tree, src):
        node_list = deque([tree.getVertex(src)])
        while len(node_list):
            vtx = node_list.popleft()
            for edge in vtx.getAdjEdges():
                node_list.append(edge.getDest())
                path = edge.path
                for i in range(len(path) - 1):
                    self.draw_line(path[i][0:2], path[i+1][0:2])
        print 'nh_plot_graph:: Done!'

    def draw_line(self, start_pos, end_pos, color=BLUE, line_width = 2):
        """
        Draws line between two co-ordinates (tuples). Tuples can also be in float.
        :param start_pos:
        :param end_pos:
        :param color:
        :return:
        """
        pygame.draw.line(self.screen, color, start_pos, end_pos, line_width)
        pygame.display.update()

    def draw_polygon(self, vertices, color=RED):
        """
        Draws polygon from the list of co-ordinates
        Order will be same as the order of vertices in the list
        :param vertices:
        :param color:
        :return:
        """
        pygame.draw.polygon(self.screen, color, vertices)
        pygame.display.update()

    def trace_path(self, goal):
        while goal.getPrev() is not None:
            prev = goal.getPrev()
            prev_point = prev.getName()[0:2]
            goal_point = goal.getName()[0:2]
            goal = prev
            self.draw_line(prev_point, goal_point, self.RED, 3)

    def nh_trace_path(self, goal):
        # Connect goal to nearest with straight line
        # prev = goal.getPrev()
        # prev_point = prev.getName()[0:2]
        # goal_point = goal.getName()[0:2]
        # self.draw_line(prev_point, goal_point, self.YELLOW, 3)
        # goal = prev
        while goal.getPrev() is not None:
            prev = goal.getPrev()
            for e in prev.getAdjEdges():
                if e.getDest() == goal:
                    path = e.path
                    for i in range(len(path) - 1):
                        self.draw_line(path[i][0:2], path[i+1][0:2], self.RED, 3)
            goal = prev

    def plot_points(self, points_list, color=YELLOW, line_width=1):
        for i in range(len(points_list) - 1):
            self.draw_line(points_list[i][0:2], points_list[i+1][0:2], color, line_width)
