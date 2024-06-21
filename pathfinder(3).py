import sys
import queue
import copy
import math

# pos: <x, y>
class MapPos:
    def __init__(self, x = 0, y = 0):
        self.x = x
        self.y = y

    def __add__(self, other):
        return MapPos(self.x + other.x, self.y + other.y)

    def __lt__(self, other):
        #if self.y < other.y:
        if self.x > other.x:
            return True
        else:
            return False

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return "[" + str(self.x) + ", " + str(self.y) + "]"

# block: <pos, cost> 
class Node:
    def __init__(self, pos: MapPos, cost):
        self.pos = pos
        self.cost = cost

    def __lt__(self, other):
        if self.cost < other.cost:
            return True
        elif self.cost == other.cost:
            if self.pos < other.pos:
                return True
            else:
                return False
        else:
            return False

    def __str__(self):
        return "<" + str(self.cost) + ", " + str(self.pos) + ">"


def step_cost(maze, cur, nxt):
    if maze[cur.x][cur.y] < maze[nxt.x][nxt.y]:
        return 1 + int(maze[nxt.x][nxt.y]) - int(maze[cur.x][cur.y])
    return 1

def heuristic_value(curr_node,dest, mode):
    if mode == 'manhattan':
        return (abs(curr_node.x-dest.x)+abs(curr_node.y-dest.y))
    if mode == 'euclidean':
        return math.sqrt(math.pow(abs(curr_node.x-dest.x),2)
                + math.pow(abs(curr_node.y-dest.y), 2))

    return (abs(curr_node.x-dest.x)+abs(curr_node.y-dest.y))

def out_print(maze):
    s = [' '.join(line) for line in maze]
    o = '\n'.join(s)
    print(o)

def dump_print(q):
    for e in q: print(e)
    print("--")

#BFS algo for the maze
def bfs(maze, dest: MapPos, start: MapPos, mode = 'bfs'):
    # to get neighbours of current node
    adj_cell_x = [1, 0, -1, 0]
    adj_cell_y = [0, 1, 0, -1]

    m, n = (len(maze), len(maze[0]))

    blk_visited = [[False for i in range(n)] for j in range(m)]
    blk_visited[start.x][start.y] = True

    q = []

    sol = Node(start, 0)
    q.append(sol)
    cost = 0

    parent_pos = [[MapPos() for j in range(n)] for i in range(m)]

    while len(q) > 0:
        if mode == 'ucs':
            q.sort() # sort by cost
            #dump_print(q)

        curr_block = q.pop(0)  # Dequeue the front cell
        curr_pos = curr_block.pos

        #if goal found than return cost
        if curr_pos.x == dest.x and curr_pos.y == dest.y:
            #print("Algorithm used = " + mode)
            # print path
            p = dest
            newmaze = copy.deepcopy(maze)
            while p != start:
                newmaze[p.x][p.y] = '*'
                p = parent_pos[p.x][p.y]
            newmaze[start.x][start.y] = '*'
            out_print(newmaze)
            return curr_block.cost

        x_pos = curr_pos.x
        y_pos = curr_pos.y

        for i in range(len(adj_cell_x)):
            
            x_pos = curr_pos.x + adj_cell_x[i]
            y_pos = curr_pos.y + adj_cell_y[i]
            post = MapPos(x_pos, y_pos)

            if x_pos < m and y_pos < n and x_pos >= 0 and y_pos >= 0:
                if maze[x_pos][y_pos] != 'X':
                    if not blk_visited[x_pos][y_pos]:
                        next_cell = Node(MapPos(x_pos, y_pos),
                                curr_block.cost 
                                + step_cost(maze, curr_pos, post))

                        blk_visited[x_pos][y_pos] = True

                        # record the path
                        parent_pos[x_pos][y_pos] = curr_pos

                        q.append(next_cell)

    return -1


def A_Star(maze, end, start, mode):
    # Create lists for open nodes and closed_list nodes
    open_list = queue.PriorityQueue()

    closed_list = [[False for i in range(len(maze[0]))] 
                        for j in range(len(maze))]
    closed_list[start.x][start.y] = True

    m, n = (len(maze), len(maze[0]))

    #neighbours direction
    adj_cell_x = [1, 0, -1, 0]
    adj_cell_y = [0, 1, 0, -1]

    # start node and goal node
    Start = Node(start, 0)
    goal = Node(end, 0)

    # Add the start node
    open_list.put((0, Start))

    parent_pos = [[MapPos() for j in range(n)] for i in range(m)]

    # Loop until the open list is empty
    while not open_list.empty():

        # Sort the open list to get the node with the lowest cost first
        # Get the node with the lowest cost
        current = open_list.get()    #getting least cost node
        curr_node = current[1]   #getting node in cuurent node
        curr_pos = curr_node.pos

        # Check if reached the goal
        if curr_pos.x == end.x and curr_pos.y == end.y:
            p = end
            newmaze = copy.deepcopy(maze)
            while p != start:
                newmaze[p.x][p.y] = '*'
                p = parent_pos[p.x][p.y]
            newmaze[start.x][start.y] = '*'
            out_print(newmaze)

            return curr_node.cost

        x_pos = curr_pos.x
        y_pos = curr_pos.y

        # Get neighbours
        for i in range(len(adj_cell_x)):

            x_pos = curr_pos.x + adj_cell_x[i]
            y_pos = curr_pos.y + adj_cell_y[i]
            post = MapPos(x_pos, y_pos)

            if x_pos < m and y_pos < n and x_pos >= 0 and y_pos >= 0:
                if maze[x_pos][y_pos] != 'X':
                    if not closed_list[x_pos][y_pos]:

                        neighbor = Node(MapPos(x_pos, y_pos), 
                                curr_node.cost 
                                + step_cost(maze, curr_pos, post))

                        #get heuristic value of neighbours
                        h = heuristic_value(neighbor.pos, end, mode) 

                        #getting f by f = h + g
                        f = h + neighbor.cost           

                        #adding neighbour to closed_list
                        closed_list[x_pos][y_pos] = True     

                        # record the path
                        parent_pos[x_pos][y_pos] = curr_pos

                        open_list.put((f, neighbor))

    return -1

def main():
    mapfile = sys.argv[1]
    algo = sys.argv[2]
    parm = ""
    if len(sys.argv) == 4: parm = sys.argv[3]

    mfile = open(mapfile, "r")
    line = mfile.readline()
    items = line.split()
    rows = int(items[0])
    cols = int(items[1])

    line = mfile.readline().rstrip("\n\r")
    items = line.split()
    x1 = int(items[0])-1
    y1 = int(items[1])-1

    line = mfile.readline().rstrip("\n\r")
    items = line.split()
    x2 = int(items[0])-1
    y2 = int(items[1])-1

    begpos = MapPos(x1, y1)
    endpos = MapPos(x2, y2)

    #print(begpos.x, begpos.y)
    #print(endpos.x, endpos.y)

    maze = []
    for i in range(rows):
        line = mfile.readline().rstrip("\n\r")
        items = line.split()
        maze.append(items)

    #print(maze)

    res = 0

    if algo == 'bfs':
        res = bfs(maze, endpos, begpos, 'bfs')
    elif algo == 'ucs':
        res = bfs(maze, endpos, begpos, 'ucs')
    elif algo == 'astar':
        res = A_Star(maze, endpos, begpos, parm)

    #print(res)
    if res == -1:
        print("null")

# main
if __name__ == '__main__':
    main()
