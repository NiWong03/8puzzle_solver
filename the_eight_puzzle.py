import heapq #needed for priority queue for uniform cost search
import time


# Creates each state of the puzzle as a node, includes position of the blank zero space,
# contains present state of the puzzle, cost to reach currNode node, and depth of node
# also includes reference to parent node if parent exists. 
class PuzzleNode:
    def __init__(self, state, operation= None, parent = None, cost = 0, depth = 0):
        self.state = state
        self.operation = operation
        self.parent = parent
        self.cost = cost
        self.depth = depth
        self.blank = self.findBlank()

    def __eq__(self,other):
        #state comparison between two nodes 
        return self.state == other.state
    
    def __lt__(self, other):
        #cost comparison for least cost between two nodes
        return self.cost < other.cost
    
    def __hash__(self):
        return hash(tuple(map(tuple, self.state)))  # allows node to be added to a set
    
    def findBlank(self):
        #loops through matrix and returns coords for blank
        for i in range(3):
            for j in range(3):
                if self.state[i][j] == 0:
                    return (i, j)

    def valid_operation(self):
        #to check if operation is possible in the bounds of the matrix 
        #if operation is valid, append to list of possible operations at given state
        operations = []
        row, col = self.blank
        if (row > 0):
            operations.append('up')
        if (row < 2):
            operations.append('down')
        if (col > 0):
            operations.append('left')
        if (col < 2):
            operations.append('right')
        return operations
    
    def newNode(self, operation):
        #generate new state for each operation
        row, col = self.blank
        node = [r.copy() for r in self.state]
        #depending on which operation was selected, move blank
        if (operation == 'up'):
            # decrementing because matrix is labeled top down
            # [ 1, 2, 3 ] row 0
            # [ 1, 2, 3 ] row 1
            # [ 1, 2, 3 ] row 2
            node[row][col], node[row - 1][col] = node[row - 1][col], node[row][col]
        if (operation == 'down'):
            node[row][col], node[row + 1][col] = node[row + 1][col], node[row][col]
        if (operation == 'left'):
            node[row][col], node[row][col - 1] = node[row ][col - 1], node[row][col]
        if (operation == 'right'):
            node[row][col], node[row][col + 1] = node[row ][col + 1], node[row][col]
        
        #increment cost and depth by one each, since we are one node deeper in search with each operation
        return PuzzleNode(node, parent=self, operation = operation, cost=self.cost + 1, depth=self.depth + 1)
    
    def childNode(self):
        successors = []
        #loop through all valid operations at given state, create new node for each operation
        #add new nodes to successors (children of currNode state)
        for operation in self.valid_operation():
            newNode = self.newNode(operation)
            successors.append(newNode)
        return successors


class UniformCostSearch:
    def __init__(self, initial_state):
        self.initial_state = initial_state
        self.goal_state = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]

    def solveUCS(self):
        initial_node = PuzzleNode(self.initial_state)
        frontier = [] #nodes to travel in search space
        explored = set() #set of nodes traveled
        maxHeap = len(frontier) #max size of queue
        heapq.heappush(frontier, (0, initial_node))  #push initial node
        startTime = time.process_time()
        
        while frontier:
            nodes_expanded = len(explored)      
            _, currNode = heapq.heappop(frontier)  # priority queue (_, cost is hardcoded to 0)
            if currNode.state == self.goal_state:#check if curr node is goal state
                endTime = time.process_time() 
                totalTime = endTime - startTime
                return self.solPath(currNode), nodes_expanded, maxHeap, totalTime # return goal state path and nodes expanded

            explored.add(currNode) # if not goal state, add node to explored set

            for successor in currNode.childNode():  # generate child node
                if successor not in explored:
                    heapq.heappush(frontier, (successor.cost, successor)) #add child to frontier
            maxHeap = (max(maxHeap, len(frontier)))
        endTime = time.process_time() 
        totalTime = endTime - startTime
        return None, nodes_expanded, maxHeap # return None and nodes expanded if no solution found

    def solPath(self, node):
        #returns path taken to reach goal node, from goal to start
        path = []
        
        while node:
            path.append(node.state)
            node = node.parent
        return path[::-1]  # return depths and path in reverse order from goal to start
    
class AStarTile:
    # g(n) = cost to get to a node (depth)
    #  h(n) = distance or # of misplaced tiles
    # f(n) = g(n) + f(n)

    def __init__(self, initial_state):
        self.initial_state = initial_state
        self.goal_state = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]

    def heuristic(self,state):
        # heuristic for mislplaced tiles, count will represent h(n)
        count = 0 # number of misplaced tiles
        for i in range(3):
            for j in range(3):
                if state[i][j] != self.goal_state[i][j] and state[i][j] != 0:
                    count += 1
        return count


    def solveMisplacedTile(self):
        initial_node = PuzzleNode(self.initial_state)
        frontier = [] #nodes to travel in search space
        explored = set() #set of nodes traveled
        maxHeap = len(frontier)
        heapq.heappush(frontier, (self.heuristic(initial_node.state), initial_node))    #push node with h(n) + g(n)
        startTime = time.process_time()

        while frontier:
            nodes_expanded = len(explored)
            total_cost, currNode = heapq.heappop(frontier) #priority queue, pops lowest cost node
            if currNode.state == self.goal_state:
                endTime = time.process_time()
                totalTime = endTime - startTime
                return self.solPath(currNode) , nodes_expanded , maxHeap , totalTime
            
            explored.add(tuple(map(tuple, currNode.state))) # if not goal state, add node to explored set
            for successor in currNode.childNode():
                state_tuple = tuple(map(tuple, successor.state))

                if state_tuple not in explored:
                    #calculate f(n) for each node
                    total_cost = successor.depth + self.heuristic(successor.state)
                    heapq.heappush(frontier, (total_cost, successor)) #add child to frontier
            maxHeap = (max(maxHeap, len(frontier)))
        endTime = time.process_time()
        totalTime = endTime - startTime
        return None, nodes_expanded, maxHeap, totalTime #return none if no solutions are found at bottom level
    
    def solPath(self, node):
        path = []
        while node:
            path.append(node.state)
            node = node.parent
        return path[::-1] #return path reversed from goal state to initial 
    
class AStarManhattan:
    # g(n) = cost to get to a node (depth)
    #  h(n) = manhattan distance
    # f(n) = g(n) + f(n)

    def __init__(self, initial_state):
        self.initial_state = initial_state
        self.goal_state = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]

    def heuristic2(self,state):
        # heuristic for manhattan distance, manhattanDist will represent h(n)
        manhattanDist = 0 # sum manhattan diststance
        for i in range(3):
            for j in range(3):
                stateTile = state[i][j]
                if stateTile != 0:
                    goalStateRow = (stateTile - 1) //3 # get row of tile in goal state
                    goalStateCol = (stateTile - 1) % 3 # get col of tile in goal state
                    manhattanDist += abs(i - goalStateRow) + abs(j - goalStateCol) # add to total manhattan distance
        return manhattanDist #sum of all tiles manhattan distance
    
    def solveManhattanDist(self):
        initial_node = PuzzleNode(self.initial_state)
        frontier = [] #nodes to travel in search space
        explored = set() #set of nodes traveled
        maxHeap = len(frontier)
        heapq.heappush(frontier, (self.heuristic2(initial_node.state), initial_node))    #push node with h(n) + g(n)
        startTime = time.process_time()

        while frontier:
            nodes_expanded = len(explored)
            total_cost, currNode = heapq.heappop(frontier) #priority queue, pops lowest cost node
            if currNode.state == self.goal_state:
                endTime = time.process_time()
                totalTime = endTime - startTime
                return self.solPath(currNode) , nodes_expanded, maxHeap, totalTime
            
            explored.add(tuple(map(tuple, currNode.state))) # if not goal state, add node to explored set
            for successor in currNode.childNode():
                state_tuple = tuple(map(tuple, successor.state))

                if state_tuple not in explored:
                    #calculate f(n) for each node
                    total_cost = successor.depth + self.heuristic2(successor.state)
                    heapq.heappush(frontier, (total_cost, successor)) #add child to frontier
            maxHeap = max(maxHeap, len(frontier))
        endTime = time.process_time()
        totalTime = endTime - startTime
        return None, nodes_expanded, maxHeap, totalTime
    
    def solPath(self, node):
        path = []
        while node:
            path.append(node.state)
            node = node.parent
        return path[::-1] #return path reversed from goal state to initial 
    

        

    # ------------------------------------------------main---------------------------------------------------------

if __name__ == "__main__":

    def puzzle_difficulty_mode():
        selected_difficulty = input( "You wish to use a default puzzle. Please enter a desired difficulty on a scale from 0 to 5." + '\n')
        if selected_difficulty == "0":
            print("Difficulty of 'Trivial' selected.")
            initial_state = [[1, 2, 3],[4, 5, 6],[0, 7, 8]]
            
        if (selected_difficulty == "1"):
            print("Difficulty of 'Very Easy' selected.")
            initial_state = [[1, 2, 3],[5, 0, 6],[4, 7, 8]]

        if (selected_difficulty == "2"):
            print("Difficulty of 'Easy' selected.")
            initial_state = [[1, 3, 6],[5, 0, 2],[4, 7, 8]]

        if (selected_difficulty == "3"):
            print("Difficulty of 'Doable' selected.")
            initial_state = [[1, 3, 6],[5, 0, 7],[4, 8, 2]]

        if (selected_difficulty == "4"):
            print("Difficulty of 'Oh Boy' selected.")
            initial_state = [[0, 7, 2],[4, 6, 1],[3, 5, 8]]

        if (selected_difficulty == "5"):
            print("Difficulty of 'Impossible' selected.")
            initial_state = [[7, 0, 2],[8, 5, 3],[6, 4, 1]]
        return initial_state

    userint = input("Welcome to my 8-Puzzle Solver. Type '1' to use a default puzzle, or '2' to create your own.")

    if (userint == "1"):
        initial_state = puzzle_difficulty_mode()


    if (userint == "2"):
        print("Enter your puzzle, using a zero to represent the blank. Please only enter valid 8-puzzles.")
        print("Enter the puzzle delimiting the numbers with a space. Type RETURN only when finished.")

        intp1 = input("Enter the first row:")
        intp2 = input("Enter the second row:")
        intp3 = input("Enter the third row:")
        # split each input into row of individual values and convert to integers
        row1 = [int(x) for x in intp1.split()]
        row2 = [int(x) for x in intp2.split()]
        row3 = [int(x) for x in intp3.split()]

        # put all inputs into one puzzle, print puzzle
        puzzle = [row1,row2,row3]
        for i in range(len(puzzle)):
            print(puzzle[i])

    if (userint == "2"):
        initial_state = [row1,row2,row3]

    userint = input("Enter Choice of Algorithm \n 1. Uniform Search Cost \n 2. A* Misplaced Tile \n 3. A* Manhattan Distance  \n ")
    if (userint == '1'):
        ucs = UniformCostSearch(initial_state)
        solPath, nodes_expanded, maxHeap , totalTime = ucs.solveUCS()
        if solPath:
            print("Solution Path:")
            for state in solPath:
                print(state, '\n')
            print("Nodes Expanded: ", nodes_expanded)
            print("Max size of queue: ", maxHeap)
            print("CPU Time: ", totalTime)
            
        else:
            print("No solution found.")
    if (userint == '2'):
        misplacedTile = AStarTile(initial_state)
        solPath, nodes_expanded, maxHeap, totalTime = misplacedTile.solveMisplacedTile()
        if solPath:
            print("Solution Path") 
            for state in solPath:
                node = PuzzleNode(state)
                g_n = node.depth  # g(n) is the depth
                h_n = misplacedTile.heuristic(state)  # Calculate h(n) using AStarTile's heuristic
                f_n = g_n + h_n 
                print(state, "f(n): " ,  f_n, '\n')
            print("Nodes Expanded: ", nodes_expanded)
            print("Max size of queue: ", maxHeap)
            print("CPU Time: ", totalTime)
        else:
            print("No solution found.")
    
    if (userint == '3'):
        manhattanDist = AStarManhattan(initial_state)
        solPath, nodes_expanded, maxHeap, totalTime = manhattanDist.solveManhattanDist()
        if solPath:
            print("Solution Path") 
            for state in solPath:
                node = PuzzleNode(state)
                g_n = node.depth  # g(n) is the depth
                h_n = manhattanDist.heuristic2(state)  # Calculate h(n) using AStarManhattan's heuristic
                f_n = g_n + h_n 
                print(state, "f(n): " ,  f_n, '\n')
            print("Nodes Expanded: ", nodes_expanded)
            print("Max size of queue: ", maxHeap)
            print("CPU Time: ", totalTime)
        else:
            print("No solution found.")
            