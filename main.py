from itertools import count
from tracemalloc import start
import pygame
import math
from queue import PriorityQueue



RED = (255, 0, 0)
GREEN = (0,255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)


WIDTH = 800

WINDOW = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Finding Algorithm Visualization")

class Node:

    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.width = width
        self.total_rows = total_rows

        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.neighbors = list()
        
    def get_pos(self):
        return self.row, self.col

    def is_closed(self):
        return self.color == RED

    def is_open(self):
        return self.color == GREEN
        
    def is_barrier(self):
        return self.color == BLACK

    def is_start(self):
        return self.color == ORANGE
    
    def is_end(self):
        return self.color == TURQUOISE
    
    def reset(self):
        self.color = WHITE

    def make_path(self):
        self.color = PURPLE

    def make_closed(self):
        self.color = RED

    def make_open(self):
        self.color = GREEN
        
    def make_barrier(self):
        self.color = BLACK

    def make_start(self):
        self.color = ORANGE
    
    def make_end(self):
        self.color = TURQUOISE

    def render_square(self, window):
        pygame.draw.rect(window, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid):
        self.neighbors = list()

        #  | 0 | 1 | 2 | 3 |
        # 0| o | # | o | o | 
        # 1| # | x | # | o |
        # 2| o |(#)| o | o |
        # 3| o | o | o | o |

        # verify if the row is not the last one (if there's any row bellow)
        # and verify if the node in the row bellow is a barrier
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): # DOWN
            self.neighbors.append(grid[self.row + 1][self.col])

        #  | 0 | 1 | 2 | 3 |
        # 0| o |(#)| o | o | 
        # 1| # | x | # | o |
        # 2| o | # | o | o |
        # 3| o | o | o | o |

        # verify if the row is not the first one (if there's any row above)
        # and verify if the node in the row above is a barrier
        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier(): # UP
            self.neighbors.append(grid[self.row - 1][self.col])

        #  | 0 | 1 | 2 | 3 |
        # 0| o | # | o | o | 
        # 1| # | x |(#)| o |
        # 2| o | # | o | o |
        # 3| o | o | o | o |

        # verify if the column is not the last one (if there's any row on the right side)
        # and verify if the node in the column on the right
        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): # RIGHT
            self.neighbors.append(grid[self.row][self.col + 1])


        #  | 0 | 1 | 2 | 3 |
        # 0| o | # | o | o | 
        # 1|(#)| x | # | o |
        # 2| o | # | o | o |
        # 3| o | o | o | o |

        # verify if the column is not the first one (if there's any row on the left side)
        # and verify if the node in the column on the left
        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): # LEFT
            self.neighbors.append(grid[self.row][self.col - 1])

    def __lt__(self, other):
        return False


def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)


def reconstruct_path(came_from, current_node, render_graphics):
    while current_node in came_from:
        current_node = came_from[current_node]
        current_node.make_path()
        render_graphics()




def algorithm(render_graphics, grid, starting_node, ending_node):
    count = 0
    open_set = PriorityQueue()
    # adding the starting node to the open set
    open_set.put((0, count, starting_node))
    came_from = {}
    # each node has a g score
    # current shortest distance to get from the starting node to this node
    g_score = {node: float("inf") for row in grid for node in row}
    g_score[starting_node] = 0
    # each node has a f score
    # mesure the distance from this node to the end
    f_score = {node: float("inf") for row in grid for node in row}  
    f_score[starting_node] = h(starting_node.get_pos(), ending_node.get_pos())
    # the open set doesn't allow us to check if somthing is inside or not
    # so we use open set hash, which makes it possible
    open_set_hash = {starting_node}


    while not open_set.empty():
        # quit the program during the algorithm
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        # .get() -> get the minimum element from the queue
        # (f_score, count, |current_node|
        #      0      1         (2)
        current_node = open_set.get()[2]
        open_set_hash.remove(current_node)

        # if the path is finished
        if current_node == ending_node:
            reconstruct_path(came_from, ending_node, render_graphics)
            ending_node.make_end()
            return True

        # loop through all the current node neighbors
        for neighbor in current_node.neighbors:
            # set the current node g score + 1 as reference
            temp_g_score = g_score[current_node] + 1
    
            if temp_g_score < g_score[neighbor]:
                # update
                came_from[neighbor] = current_node
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), ending_node.get_pos())
                # using open set hash is possible to check if some element is in the set
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()

        render_graphics()

        if current_node != starting_node:
            current_node.make_closed()

    return False


def make_grid(rows, width):
    # size of the node = screen width // number  of rows
    grid = list()
    node_size = width // rows

    for i in range(rows):
        grid.append([])
        for j in range(rows):
            node = Node(i, j, node_size, rows)
            grid[i].append(node)

    # grid = [
    #   [node, node, node],
    #   [node, node, node],
    #   [node, node, node]
    # ]
    return grid


def render_grid_lines(window, rows, width):
    node_size = width // rows
    for i in range(rows):
        pygame.draw.line(window, GREY, (0, i * node_size), (width, i * node_size))
        for j in range(rows):
            pygame.draw.line(window, GREY, (j * node_size, 0), (j * node_size, width))


def render_graphics(window, grid, rows, width):
    window.fill(WHITE)

    for row in grid:
        for node in row:
            node.render_square(window)
    
    render_grid_lines(window, rows, width)
    pygame.display.update()


def get_clicked_position(mouse_pos, rows, width):
    node_size = width // rows
    y, x = mouse_pos

    row = y // node_size
    col = x // node_size

    return row, col


def main(window, width):
    ROWS = 50
    grid = make_grid(ROWS, width)

    starting_node = None
    ending_node = None

    run = True

    while run:
        render_graphics(window, grid, ROWS, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            # left click
            if pygame.mouse.get_pressed()[0]:
                mouse_pos = pygame.mouse.get_pos()
                row, col = get_clicked_position(mouse_pos, ROWS, width)
                clicked_node = grid[row][col]
                print(f'left click at row: {row} col: {col}')

                if not starting_node and clicked_node != ending_node:
                    starting_node = clicked_node
                    starting_node.make_start()
                    print('start')

                elif not ending_node and clicked_node != starting_node:
                    ending_node = clicked_node
                    ending_node.make_end()
                    print('end')

                elif clicked_node != starting_node and clicked_node != ending_node:
                    clicked_node.make_barrier()
                    print('barrier')

            # right click
            elif pygame.mouse.get_pressed()[2]:
                mouse_pos = pygame.mouse.get_pos()
                row, col = get_clicked_position(mouse_pos, ROWS, width)
                clicked_node = grid[row][col]
                clicked_node.reset()

                if clicked_node == starting_node:
                    starting_node = None

                elif clicked_node == ending_node:
                    ending_node = None

            
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and not starting_node and ending_node:
                    for row in grid:
                        for node in row:
                            node.update_neighbors(grid)

                    algorithm(lambda: render_graphics(window, grid, ROWS, width), grid, starting_node, ending_node)

                if event.key == pygame.K_c:
                    starting_node = None
                    ending_node = None
                    grid = make_grid(ROWS, width)


main(WINDOW, WIDTH)




