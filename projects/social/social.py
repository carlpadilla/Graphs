import random
import time

# this is bad! do not copy and paste


class Queue():
    def __init__(self):
        self.queue = []

    def enqueue(self, value):
        self.queue.append(value)

    def dequeue(self):
        if self.size() > 0:
            return self.queue.pop(0)
        else:
            return None

    def size(self):
        return len(self.queue)


class User:
    def __init__(self, name):
        self.name = name


class SocialGraph:
    def __init__(self):
        self.last_id = 0
        self.users = {}

        # this is your adjacency list representation of a graph
        self.friendships = {}

    def add_friendship(self, user_id, friend_id):
        """
        Creates a bi-directional friendship

        Therefore creates an undirected graph

        Makes TWO friendships
        """
        if user_id == friend_id:
            # print("WARNING: You cannot be friends with yourself")
            return False
        elif friend_id in self.friendships[user_id] or user_id in self.friendships[friend_id]:
            # print("WARNING: Friendship already exists")
            return False
        else:
            self.friendships[user_id].add(friend_id)
            self.friendships[friend_id].add(user_id)
            return True

    def add_user(self, name):
        """
        Create a new user with a sequential integer ID
        """
        self.last_id += 1  # automatically increment the ID to assign the new user
        self.users[self.last_id] = User(name)
        self.friendships[self.last_id] = set()

    def fisher_yates_shuffle(self, l):
        for i in range(0, len(l)):
            random_index = random.randint(i, len(l) - 1)
            l[random_index], l[i] = l[i], l[random_index]

    def populate_graph(self, num_users, avg_friendships):
        """
        Takes a number of users and an average number of friendships
        as arguments

        Creates that number of users and a randomly distributed friendships
        between those users.

        The number of users must be greater than the average number of friendships.
        """
        # Reset graph
        self.last_id = 0
        self.users = {}
        self.friendships = {}
        # !!!! IMPLEMENT ME

        # Add users
        for user in range(num_users):
            self.add_user(user)
            # starts at 1, up to and including num_users

        # * Hint 1: To create N random friendships,
        # you could create a list with all possible friendship combinations of user ids,

        friendship_combinations = []
        # O(n^2)
        for user in range(1, self.last_id + 1):
            for friend in range(user + 1, self.last_id + 1):
                friendship_combinations.append((user, friend))

        # [(1, 2), (1, 3), (2, 3)]
        # [(1, 1), (2, 1)]

        # shuffle the list
        self.fisher_yates_shuffle(friendship_combinations)

        # then grab the first N elements from the list.
        total_friendships = num_users * avg_friendships

        friends_to_make = friendship_combinations[:(total_friendships // 2)]

        # Create friendships
        for friendship in friends_to_make:
            self.add_friendship(friendship[0], friendship[1])

    # Hash the user name or something
    # if you are in the same bucket of the hash table, you are friends

    # another way of using randomness!

    def populate_graph_linear(self, num_users, avg_friendships):
        # Reset graph
        self.last_id = 0
        self.users = {}
        self.friendships = {}
        # !!!! IMPLEMENT ME

        # Add users
        for user in range(num_users):
            self.add_user(user)
            # starts at 1, up to and including num_users

        total_friendships = num_users * avg_friendships
        friendships_made = 0

    # until we've made the total friendships we want
        while friendships_made < total_friendships:
            # choose two user ids at random
            user = random.randint(1, self.last_id)
            friend = random.randint(1, self.last_id)
    # try to make them friends
            was_friendship_made = self.add_friendship(user, friend)
    # if that succeeds, increment a friendship counter
            if was_friendship_made:
                friendships_made += 1

    def get_all_social_paths(self, user_id):
        """
        Takes a user's user_id as an argument

        Returns a dictionary containing every user in that user's
        extended network with the shortest friendship path between them.

        The key is the friend's ID and the value is the path.

        Connected component: user's extended network

        BFT
        - breadth first, because "shortest friendship path"
        - traversal, for every user
        """
        q = Queue()
        visited = {}  # Note that this is a dictionary, not a set

        q.enqueue([user_id])

        while q.size() > 0:

            current_path = q.dequeue()
            current_node = current_path[-1]

            if current_node not in visited:
                visited[current_node] = current_path

                friends = self.friendships[current_node]

                for friend in friends:
                    friend_path = current_path.copy()
                    friend_path.append(friend)

                    q.enqueue(friend_path)

        return visited


if __name__ == '__main__':
    sg = SocialGraph()

    start_time = time.time()
    sg.populate_graph(1000, 50)
    end_time = time.time()

    print(end_time - start_time)

    start_time = time.time()
    sg.populate_graph_linear(1000, 50)
    end_time = time.time()
    print(end_time - start_time)
    connections = sg.get_all_social_paths(1)
    print(connections)

    # percentage of users in this user's extended social network
    # print("percentage of users in social network: ", len(connections) / 1000 * 100)

    # average degree of separation
    # aka, how many people do I need to introduce?
    # total = 0
    # for path in connections.values():
    #     total += len(path)

    # average = total / len(connections)

    # print("average degree of separation: ", average)
