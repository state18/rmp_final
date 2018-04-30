# This file houses my main contributions for the project.
# It contains RRT planning functions.

import math
import numpy as np
import random as rand
import klampt.model.collide as collide


# Given a "world" (robot, terrain, and obstacles), produce a path in the form of discretized configurations.
# TODO: Will possibly output robot movement instructions rather than just the path points.
def find_path(world, robot, q_init, q_goal, num_samples=300, max_iters=1):

    num_tries = 0
    path = []
    found_path = False
    while num_tries < max_iters and not found_path:
        rrt = gen_rrt(world, robot, q_init, num_samples)
        path = rrt.find_path(q_goal, world, robot)
        if path:
            found_path = True
        num_tries = num_tries + 1

    return path


def gen_rrt(world, robot, q_init, num_samples):
    # Used to check if a configuration results in collision in the world.
    coll_check = collide.WorldCollider(world)

    # First, build tree with initial configuration.
    rrt = RRT(q_init)

    # Now, for k iterations, try to expand the tree.
    for k in range(0, num_samples):
        q_rand = gen_rand_config(world)
        rrt.extend_tree(q_rand, world, robot, coll_check)

    return rrt


def gen_rand_config(world):
    # Generate a random configuration (x,y,t)
    # Note: This configuration does not need to be free of collision because it is
    # ultimately just used as a direction to extend the tree.
    # TODO: Get ranges from world info
    x_range = [-4, 4]
    y_range = [-4, 4]
    t_range = [0, 2*math.pi]
    # Sample from ranges
    x = rand.uniform(x_range[0], x_range[1])
    y = rand.uniform(y_range[0], y_range[1])
    t = rand.uniform(t_range[0], t_range[1])
    return [x, y, t]


class RRT:
    def __init__(self, q_init):
        # Since graph is growing, use adjacency list
        self.nodes = []
        self.max_x = -1
        self.max_y = -1
        self.min_x = 99999
        self.min_y = 99999
        self.adj_list = dict()
        self.num_nodes = 0
        self.add_node(q_init)

    def add_node(self, q_new, neighbors=None):
        if neighbors is None:
            neighbors = []

        self.num_nodes = self.num_nodes + 1
        node_new = Node(q_new[0], q_new[1], q_new[2], self.num_nodes)
        self.nodes.append(node_new)

        self.adj_list[node_new.id] = [node_new] + neighbors

        # For debugging: Log stats
        self.max_x = max(self.max_x, node_new.x)
        self.min_x = min(self.min_x, node_new.x)
        self.max_y = max(self.max_y, node_new.y)
        self.min_y = min(self.min_y, node_new.y)

        return node_new

    def get_nearest_neighbor(self, q):
        min_dist = 999999
        nearest_node_id = -1

        for node in self.nodes:
            # Using L1 distance on x and y for now
            dist = abs(node.x - q[0]) + abs(node.y - q[1])
            if dist < min_dist:
                min_dist = dist
                nearest_node_id = node.id
        return nearest_node_id

    def get_nearest_neighbors_by_angle(self, q, num_segs):
        nearest_neighbors = np.empty(num_segs, np.dtype(object))
        min_dists = np.ones(num_segs) * np.inf
        t_step = 2*math.pi / num_segs
        for node in self.nodes:
            dist = abs(node.x - q[0] + abs(node.y - q[1]))
            # Which segment does this node belong to?
            t = math.atan2(node.y - q[1], node.x - q[0])
            if t < 0:
                t = t + 2 * math.pi
            elif t >= 2 * math.pi:
                t = t - 2 * math.pi
            lower_bound = 0
            upper_bound = t_step
            for i in range(0, num_segs):
                if t >= lower_bound and t < upper_bound:
                    if dist < min_dists[i]:
                        min_dists[i] = dist
                        nearest_neighbors[i] = node
                    break
                else:
                    lower_bound = upper_bound
                    upper_bound = upper_bound + t_step

        # Sort nearest neighbors by distance.
        sort_indxs = np.argsort(min_dists)
        return nearest_neighbors[sort_indxs].tolist()






    def extend_tree(self, q, world, robot, coll_check, desired_mag=.3):
        node_near = self.adj_list[self.get_nearest_neighbor(q)][0]
        q_near = [node_near.x, node_near.y, node_near.t]
        # From q_near to q, advance incrementally a certain distance and check for collision
        # along the way.


        # First, rotate so the robot is facing towards q. Test for collision periodically.
        # Angular interpolation
        start_t = q_near[2]
        # The direction pointing from q_near to q
        goal_dir_t = math.atan2(q[1] - q_near[1], q[0] - q_near[0])
        if goal_dir_t < 0:
            goal_dir_t = goal_dir_t + 2*math.pi
        elif goal_dir_t >= 2*math.pi:
            goal_dir_t = goal_dir_t - 2*math.pi

        is_success = False
        t_step = 10
        # angle_diff = goal_dir_t - start_t
        # angle_diff = abs((angle_diff + 180) % 360) - 180
        angle_diff = math.atan2(math.sin(goal_dir_t - start_t), math.cos(goal_dir_t - start_t))

        curr_t = start_t
        rads_rotated = 0
        has_collided = False
        while rads_rotated < abs(angle_diff) and not has_collided:
            rads_rotated = rads_rotated + t_step
            curr_t = curr_t + np.sign(angle_diff) * t_step
            if curr_t < 0:
                curr_t = curr_t + 2*math.pi
            elif curr_t >= 2*math.pi:
                curr_t = curr_t - 2*math.pi

            # This is the last check we need, but don't overshoot it.
            if rads_rotated > abs(angle_diff):
                rads_rotated = abs(angle_diff)
                curr_t = goal_dir_t

            # Set current configuration in world.
            robot.setConfig([q_near[0], q_near[1], curr_t])
            # Check collision for current configuration.
            obj_colls = coll_check.robotObjectCollisions(world.robot(0))
            for i,j in obj_colls:
                # There is collision with an obstacle! We can't add this to the tree.
                # TODO: Possibly try the longer rotation if this one fails.
                has_collided = True
                return "TRAPPED"


        # Translate in a straight line incrementally towards x,y of q. Test for collision periodically.
        # 2D linear interpolation
        # Compute (x,y) after traveling dist towards q(x,y)

        x_diff = q[0] - q_near[0]
        y_diff = q[1] - q_near[1]
        mag = math.sqrt(x_diff ** 2 + y_diff ** 2)
        norm = [x_diff / mag, y_diff / mag]
        step_mag = .02
        curr_mag = step_mag
        curr_vec = [norm[0] * curr_mag, norm[1] * curr_mag]
        has_collided = False
        while curr_mag < desired_mag and not has_collided:
            # Compute new (x,y)
            curr_mag = curr_mag + step_mag
            if curr_mag > desired_mag:
                curr_mag = desired_mag
            curr_vec = [norm[0] * curr_mag, norm[1] * curr_mag]
            curr_x = curr_vec[0] + q_near[0]
            curr_y = curr_vec[1] + q_near[1]
            robot.setConfig([curr_x, curr_y, curr_t])
            obj_colls = coll_check.robotObjectCollisions(world.robot(0))
            for i, j in obj_colls:
                # There is collision with an obstacle! We can't add this to the tree.
                has_collided = True
                return "TRAPPED"

        #print(robot.getConfig())
        # If the rotation and translation steps were collision-free, add q_new to tree
        self.add_node([curr_x, curr_y, goal_dir_t], [node_near.id])


    def find_path(self, q_goal, world, robot):

        # Extract collision detector for attempts at linking goal to RRT.
        coll_check = collide.WorldCollider(world)

        # First, try to connect q_goal to a nearby configuration.
        # node_goal = self.adj_list[self.get_nearest_neighbor(q_goal)][0]

        # The idea is to look in 8 directions around the q_goal to find nearest neighbors. This is in case an abundance
        # of neighbors are not reachable from one side.
        num_segs = 8
        nearest_neighbors = self.get_nearest_neighbors_by_angle(q_goal, num_segs)
        chosen_neighbor = []

        # TODO: Try each of the neighbors, in order of closeness...
        for neighbor in nearest_neighbors:
            # First, rotate so the robot is facing towards q. Test for collision periodically.
            # Angular interpolation
            start_t = neighbor.t
            # The direction pointing from q_near to q
            goal_dir_t = math.atan2(q_goal[1] - neighbor.y, q_goal[0] - neighbor.x)
            if goal_dir_t < 0:
                goal_dir_t = goal_dir_t + 2 * math.pi
            elif goal_dir_t >= 2 * math.pi:
                goal_dir_t = goal_dir_t - 2 * math.pi

            is_success = False
            t_step = 10
            # angle_diff = goal_dir_t - start_t
            # angle_diff = abs((angle_diff + 180) % 360) - 180
            angle_diff = math.atan2(math.sin(goal_dir_t - start_t), math.cos(goal_dir_t - start_t))

            curr_t = start_t
            rads_rotated = 0
            has_collided = False
            while rads_rotated < abs(angle_diff) and not has_collided:
                rads_rotated = rads_rotated + t_step
                curr_t = curr_t + np.sign(angle_diff) * t_step
                if curr_t < 0:
                    curr_t = curr_t + 2 * math.pi
                elif curr_t >= 2 * math.pi:
                    curr_t = curr_t - 2 * math.pi

                # This is the last check we need, but don't overshoot it.
                if rads_rotated > abs(angle_diff):
                    rads_rotated = abs(angle_diff)
                    curr_t = goal_dir_t

                # Set current configuration in world.
                robot.setConfig([neighbor.x, neighbor.y, curr_t])
                # Check collision for current configuration.
                obj_colls = coll_check.robotObjectCollisions(world.robot(0))
                for i, j in obj_colls:
                    # There is collision with an obstacle! We can't add this to the tree.
                    # TODO: Possibly try the longer rotation if this one fails.
                    has_collided = True
                    return "TRAPPED"

            # Translate in a straight line incrementally towards x,y of q. Test for collision periodically.
            # 2D linear interpolation
            # Compute (x,y) after traveling dist towards q(x,y)

            x_diff = q_goal[0] - neighbor.x
            y_diff = q_goal[1] - neighbor.y
            mag = math.sqrt(x_diff ** 2 + y_diff ** 2)
            norm = [x_diff / mag, y_diff / mag]
            step_mag = .02
            curr_mag = step_mag
            curr_vec = [norm[0] * curr_mag, norm[1] * curr_mag]
            # This is key difference. The desired mag here is the entire distance from neighbor to q_goal.
            desired_mag = mag
            has_collided = False
            while curr_mag < desired_mag and not has_collided:
                # Compute new (x,y)
                curr_mag = curr_mag + step_mag
                if curr_mag > desired_mag:
                    curr_mag = desired_mag
                curr_vec = [norm[0] * curr_mag, norm[1] * curr_mag]
                curr_x = curr_vec[0] + neighbor.x
                curr_y = curr_vec[1] + neighbor.y
                robot.setConfig([curr_x, curr_y, curr_t])
                obj_colls = coll_check.robotObjectCollisions(world.robot(0))
                for i, j in obj_colls:
                    # There is collision with an obstacle! We can't add this to the tree.
                    has_collided = True
                    return "TRAPPED"

            chosen_neighbor = neighbor
            break

        # If we're here, the goal was successfully reached from a neighbor. Add goal to the tree.
        node_goal = self.add_node([q_goal[0], q_goal[1], goal_dir_t], [chosen_neighbor.id])

        # LEGACY
        #node_goal = self.adj_list[self.get_nearest_neighbor(q_goal)][0]

        prev_lookup = dict()
        queue = [node_goal.id]
        prev_lookup[node_goal.id] = -1
        visited = []

        while queue:
            parent_node_id = queue.pop(0)
            neighbor_ids = self.adj_list[parent_node_id][1:]

            for neighbor_id in neighbor_ids:
                if neighbor_id not in visited:
                    queue.append(neighbor_id)
                    node = self.adj_list[neighbor_id][0]
                    prev_lookup[node.id] = parent_node_id
                    visited.append(node.id)
                    if node is self.nodes[0]:
                        break

        path = []
        # Now, put together path.
        curr_node_id = prev_lookup[self.nodes[0].id]
        while curr_node_id is not -1:
            curr_node = self.adj_list[curr_node_id][0]
            path.append([curr_node.x, curr_node.y, curr_node.t])
            curr_node_id = prev_lookup[curr_node_id]

        #self.plot_path(path)
        return path

    def plot_path(self, path):

        import matplotlib as mpl
        mpl.use('Qt4Agg')
        import matplotlib.pyplot as plt


        # Iterate through each node and plot it and neighbors.
        for e in self.adj_list:
            node = self.adj_list[e][0]
            neighbors = self.adj_list[e][1:]
            for neighbor in neighbors:
                n_node = self.adj_list[neighbor][0]
                plt.plot([node.x, n_node.x], [node.y, n_node.y], '-bo')


        # Gather x's and y's
        x = []
        y = []
        for pair in path:
            x.append(pair[0])
            y.append(pair[1])


        # Plot them.
        plt.ion()
        plt.plot(x, y, '-ro')
        plt.ylim(-4, 4)
        plt.xlim(-4, 4)
        plt.axis('scaled')
        figure_manager = plt.get_current_fig_manager()
        figure_manager.window.showMaximized()
        plt.show()
        plt.pause(.001)

        pass


    def visualize_tree(self):
        raise NotImplementedError

class Node():
    def __init__(self, x, y, t, id_num):
        self.x = x
        self.y = y
        self.t = t
        self.id = id_num
