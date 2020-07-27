

def earliest_ancestor(ancestors, starting_node):
    parents = find_parents(ancestors, starting_node)
    if len(parents) == 0:
        return -1

    leafs = {}
    climb_tree(ancestors, starting_node, leafs)
    print(leafs)
    if len(leafs) == 1:
        return list(leafs.keys())[0]
    greatest_depth = leafs[list(leafs.keys())[0]]
    greatest_node = list(leafs.keys())[0]
    for parent in leafs:
        gen = leafs[parent]
        if gen > greatest_depth:
            greatest_depth = gen
            greatest_node = parent
    return greatest_node

# (parent, child)
# Idea: Write a helper function to find the parents for a given child.
# To finish: add a third variable, node_id. If we find two nodes with the same depth, we can compare the id's.


def find_parents(ancestors, child):
    parents = []
    for pair in ancestors:
        if pair[1] == child:
            parents.append(pair[0])
    return parents


def climb_tree(ancestors, node, leafs, depth=0):
    # Find the parents of the current node.
    parents = find_parents(ancestors, node)
    # Check to see if we have a leaf (current_node has no parents)
    if len(parents) == 0:
        leafs[node] = depth

    else:
        for parent in parents:
            climb_tree(ancestors, parent, leafs, depth+1)
