import pygame
from operator import sub
import sys
from pathplan import PathPlanner

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
ENV_WIDTH = 800
ENV_HEIGHT = 800
RESOLUTION = ENV_WIDTH, ENV_HEIGHT


pygame.init()
screen = pygame.display.set_mode(RESOLUTION)
screen.fill((100, 186, 100))
pygame.display.set_caption('First One')


pygame.display.update()
polygon_vertices = []
while True:
    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == pygame.MOUSEBUTTONDOWN:
            start_pos = event.pos
            print start_pos
        elif event.type == pygame.MOUSEBUTTONUP:
            end_pos = event.pos
            print end_pos
            if start_pos == end_pos:
                polygon_vertices.append(end_pos)
            else:
                width_height = tuple(map(sub, end_pos, start_pos))
                print width_height
                pygame.draw.rect(screen, RED, start_pos + width_height)
                pygame.display.update()

        elif event.type == pygame.KEYUP and event.key == pygame.K_RETURN:
            print event
            pygame.draw.polygon(screen, RED, polygon_vertices)
            pygame.display.update()
            polygon_vertices[:] = []
pygame.display.update()
