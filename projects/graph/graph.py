"""
Simple graph implementation
"""
from util import Stack, Queue  # These may come in handy


class Graph:

    """Represent a graph as a dictionary of vertices mapping labels to edges."""

    def __init__(self):
        self.vertices = {}

    def add_vertex(self, vertex_id):
        """
        Add a vertex to the graph.
        """
        self.vertices[vertex_id] = set()

    def add_edge(self, v1, v2):
        """
        Add a directed edge to the graph.
        """
        if v1 in self.vertices and v2 in self.vertices:
            self.vertices[v1].add(v2)
        else:
            raise IndexError("The vertex does not exist.")

    def get_neighbors(self, vertex_id):
        """
        Get all neighbors (edges) of a vertex.
        """
        return self.vertices[vertex_id]

    def bft(self, starting_vertex):
        """
        Print each vertex in breadth-first order
        beginning from starting_vertex.
        """
        # Create an empty queue
        bft_queue = Queue()
        # Create a set to store the visited nodes
        visited = set()
        # Init: enqueue the starting node
        bft_queue.enqueue(starting_vertex)
        # While the queue isn't empty
        while bft_queue.size() > 0:
            # Dequeue the first item
            v = bft_queue.dequeue()
            # Check if we have visited the node
            if v not in visited:
                # Mark as visited. (Add to the visited set)
                visited.add(v)
                print(v)
                # Add all neighbors to the queue.
                for next_vert in self.get_neighbors(v):
                    bft_queue.enqueue(next_vert)
        return visited

    def dft(self, starting_vertex):
        """
        Print each vertex in depth-first order
        beginning from starting_vertex.
        """
        # Create an empty stack
        dft_stack = Stack()
        # Create a set to store the visited nodes
        visited = set()
        # Init: Push the starting node
        dft_stack.push(starting_vertex)
        # While the stack isn't empty
        while dft_stack.size() > 0:
            # Pop the first item
            v = dft_stack.pop()
            # Check if we have visited the node
            if v not in visited:
                # Mark as visited. (Add to the visited set)
                visited.add(v)
                print(v)
                # Add all neighbors to the stack.
                for next_vert in self.get_neighbors(v):
                    dft_stack.push(next_vert)
        return visited

    # For recursive
    # Need to have a base case
    # Progress toward the base case
    # Call itself

    def dft_recursive(self, starting_vertex, visited=None):
        """
        Print each vertex in depth-first order
        beginning from starting_vertex.

        This should be done using recursion.
        """
        if visited == None:
            visited = set()
        # Check if we have been visited
        if starting_vertex not in visited:
            visited.add(starting_vertex)
            print(starting_vertex)
            # Base case: if no neighbors
            neighbors = self.get_neighbors(starting_vertex)
            # The base case for stopping the recursion is when there are no more unvisited nodes
            # if len(neighbors) == 0:
            #     return visited
            # If we do have neighbors, iterate over them and recurse for each one
            for neighbor in neighbors:
                self.dft_recursive(neighbor, visited)

    def bfs(self, starting_vertex, destination_vertex):
        """
        Return a list containing the shortest path from
        starting_vertex to destination_vertex in
        breath-first order.

        1. Find the target node.
        2. Keep track of how you got there.
        """

        # Create an empty queue and enqueue A PATH TO the starting vertex ID.
        q = Queue()
        # Create a Set to store visited vertices.
        visited = set()
        path = [starting_vertex]
        q.enqueue(path)
        # While the queue is not empty...
        while q.size() > 0:
            # Dequeue from the front of the line, this is our current path.
            current_path = q.dequeue()
        # current_node is the last thing in the path
            current_node = current_path[-1]
        # check if this is the target node
            if current_node == destination_vertex:
                # if so, return
                return current_path
        # Check if we've visited yet, if not:
            if current_node not in visited:
                # mark as visited
                visited.add(current_node)

                # get the current node's neighbors
                neighbors = self.get_neighbors(current_node)
                # iterate over the neighbors
                for neighbor in neighbors:
                    # add the neighbor to the path
                    neighbor_path = current_path.copy()
                    neighbor_path.append(neighbor)
                    # enqueue the neighbor's path
                    q.enqueue(neighbor_path)

    def dfs(self, starting_vertex, destination_vertex):
        """
        Return a list containing a path from
        starting_vertex to destination_vertex in
        depth-first order.
        """
        s = Stack()
        # Create a Set to store visited vertices.
        visited = set()
        path = [starting_vertex]
        s.push(path)
        # While the queue is not empty...
        while s.size() > 0:
            # Desueue from the front of the line, this is our current path.
            current_path = s.pop()
        # current_node is the last thing in the path
            current_node = current_path[-1]
        # check if this is the target node
            if current_node == destination_vertex:
                # if so, return
                return current_path
        # Check if we've visited yet, if not:
            if current_node not in visited:
                # mark as visited
                visited.add(current_node)

                # get the current node's neighbors
                neighbors = self.get_neighbors(current_node)
                # iterate over the neighbors
                for neighbor in neighbors:
                    # add the neighbor to the path
                    neighbor_path = current_path.copy()
                    neighbor_path.append(neighbor)
                    # enqueue the neighbor's path
                    s.push(neighbor_path)

    def dfs_recursive(self, starting_vertex, destination_vertex, visited=None, path=None):
        """
        Return a list containing a path from
        starting_vertex to destination_vertex in
        depth-first order.

        This should be done using recursion.
        """
        if visited == None:
            visited = set()

        if path == None:
            path = [starting_vertex]

        # Check if we have been visited
        if starting_vertex not in visited:
            visited.add(starting_vertex)
            print(starting_vertex)
            # The base case for stopping the recursion is when there are no more unvisited nodes
            neighbors = self.get_neighbors(starting_vertex)
            if starting_vertex == destination_vertex:
                return path
            # If we do have neighbors, iterate over them and recurse for each one
            for neighbor in neighbors:
                neighbor_path = path.copy()
                neighbor_path.append(neighbor)
                ret = self.dfs_recursive(
                    neighbor, destination_vertex, visited, neighbor_path)
                if ret:
                    return ret


if __name__ == '__main__':
    graph = Graph()  # Instantiate your graph
    # https://github.com/LambdaSchool/Graphs/blob/master/objectives/breadth-first-search/img/bfs-visit-order.png
    graph.add_vertex(1)
    graph.add_vertex(2)
    graph.add_vertex(3)
    graph.add_vertex(4)
    graph.add_vertex(5)
    graph.add_vertex(6)
    graph.add_vertex(7)
    graph.add_edge(5, 3)
    graph.add_edge(6, 3)
    graph.add_edge(7, 1)
    graph.add_edge(4, 7)
    graph.add_edge(1, 2)
    graph.add_edge(7, 6)
    graph.add_edge(2, 4)
    graph.add_edge(3, 5)
    graph.add_edge(2, 3)
    graph.add_edge(4, 6)

    '''
    Should print:
        {1: {2}, 2: {3, 4}, 3: {5}, 4: {6, 7}, 5: {3}, 6: {3}, 7: {1, 6}}
    '''
    print(graph.vertices)

    '''
    Valid BFT paths:
        1, 2, 3, 4, 5, 6, 7
        1, 2, 3, 4, 5, 7, 6
        1, 2, 3, 4, 6, 7, 5
        1, 2, 3, 4, 6, 5, 7
        1, 2, 3, 4, 7, 6, 5
        1, 2, 3, 4, 7, 5, 6
        1, 2, 4, 3, 5, 6, 7
        1, 2, 4, 3, 5, 7, 6
        1, 2, 4, 3, 6, 7, 5
        1, 2, 4, 3, 6, 5, 7
        1, 2, 4, 3, 7, 6, 5
        1, 2, 4, 3, 7, 5, 6
    '''
    graph.bft(1)

    '''
    Valid DFT paths:
        1, 2, 3, 5, 4, 6, 7
        1, 2, 3, 5, 4, 7, 6
        1, 2, 4, 7, 6, 3, 5
        1, 2, 4, 6, 3, 5, 7
    '''
    graph.dft(1)
    graph.dft_recursive(1)

    '''
    Valid BFS path:
        [1, 2, 4, 6]
    '''
    print(graph.bfs(1, 6))

    '''
    Valid DFS paths:
        [1, 2, 4, 6]
        [1, 2, 4, 7, 6]
    '''
    print(graph.dfs(1, 6))
    print(graph.dfs_recursive(1, 6))
