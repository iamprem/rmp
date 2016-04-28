from operator import sub
import pygame


class Visualizer:

    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    RED = (255, 0, 0)
    BLUE = (0, 0, 255)
    ENV_WIDTH = 800
    ENV_HEIGHT = 800
    RESOLUTION = ENV_WIDTH, ENV_HEIGHT

    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode(self.RESOLUTION)
        self.screen.fill((100, 186, 100))
        pygame.display.set_caption('Visualizer')

    def define_obstacles(self):
        """
        Takes Mouse Clicks and Drags as input from user and returns its clockwise co-ordinates of the shape
        Note: If only two points are clicked, A rectangle will be returned
        :return:
        """
        OBSTACLE_DEFINED = False
        self.screen.fill((100, 186, 100))
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

    def plot_graph(self, tree):
        """
        Draw a Tree/Graph on pygame screen based on the graph/tree connectivity
        :param tree:
        :return:
        """
        for vtx in tree.vertexMap.values():
            for edge in vtx.getAdjEdges():
                self.draw_line(edge.getSrcName(), edge.getDestName())

    def draw_line(self, start_pos, end_pos, color=BLUE):
        """
        Draws line between two co-ordinates (tuples). Tuples can also be in float.
        :param start_pos:
        :param end_pos:
        :param color:
        :return:
        """
        pygame.draw.line(self.screen, color, start_pos, end_pos, 2)
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

