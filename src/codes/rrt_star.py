
"""
RRT_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import collections
import heapq
from scipy.misc import imread
import cv2
from matplotlib import cm

MAP_IMG = './maze.jpg'


class Utils:
    def __init__(self):
        self.env = Env()

        self.delta = 0.5
        self.img = self.env.img

    def is_intersect_circle(self, start, end):

        t = 0
        dist , theta = RrtStar.get_distance_and_angle(start, end)
        o = (start.x, start.y)
        d = (np.cos(theta), np.sin(theta))

        while True:
            shot = Node((o[0] + t * d[0], o[1] + t * d[1]))
            t = t + self.delta
            if self.get_dist(shot, end) <= self.delta:
                return False
            #print(int(shot.x), int(shot.y))
            #print(self.get_dist(shot, end))
            if self.is_inside_obs(shot):
                return True

        return False

    def is_collision(self, start, end):
        if self.is_inside_obs(start) or self.is_inside_obs(end):
            return True

        if self.is_intersect_circle(start, end):
            return True

        return False

    def is_inside_obs(self, node):
        delta = self.delta
        if(self.img[int(node.y)][int(node.x)] <= 200):
            return True

        return False

    @staticmethod
    def get_ray(start, end):
        orig = [start.x, start.y]
        direc = [end.x - start.x, end.y - start.y]
        return orig, direc

    @staticmethod
    def get_dist(start, end):
        return math.hypot(end.x - start.x, end.y - start.y)


class Plotting:
    def __init__(self, x_start, x_goal):
        self.xI, self.xG = x_start, x_goal
        self.env = Env()
        self.img = self.env.img


    def animation(self, nodelist, path, name, animation=False):
        self.plot_grid(name)
        self.plot_visited(nodelist, animation)
        self.plot_path(path)

    def animation_connect(self, V1, V2, path, name):
        self.plot_grid(name)
        self.plot_visited_connect(V1, V2)
        self.plot_path(path)

    def plot_grid(self, name):
        self.fig, ax = plt.subplots()

        plt.plot(self.xI[0], self.xI[1], "bs", linewidth=3)
        plt.plot(self.xG[0], self.xG[1], "gs", linewidth=3)

        plt.title(name)
        plt.axis("equal")


        plt.imshow(self.img, cmap=cm.Greys_r)
        #plt.pause(0.001)
        plt.draw()

    @staticmethod
    def plot_visited(nodelist, animation):


        if animation:
            count = 0
            for node in nodelist:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 100 == 0:
                        plt.pause(0.001)
        else:
            for node in nodelist:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    @staticmethod
    def plot_visited_connect(V1, V2):
        len1, len2 = len(V1), len(V2)

        for k in range(max(len1, len2)):
            if k < len1:
                if V1[k].parent:
                    plt.plot([V1[k].x, V1[k].parent.x], [V1[k].y, V1[k].parent.y], "-g")
            if k < len2:
                if V2[k].parent:
                    plt.plot([V2[k].x, V2[k].parent.x], [V2[k].y, V2[k].parent.y], "-g")

            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if k % 2 == 0:
                plt.pause(0.001)

        plt.pause(0.01)

    @staticmethod
    def plot_path(path):
        if len(path) != 0:
            plt.plot([x[0] for x in path], [x[1] for x in path], '-r', linewidth=2)
            plt.pause(0.01)
        plt.show()

class Env:
    def __init__(self):

        # img = imread(MAP_IMG, mode="L")
        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        # #img = cv2.dilate(img,kernel,iterations = 2)
        # img = cv2.erode(img,kernel,iterations = 2)
        self.img = []

        self.x_range = (0, 50)
        self.y_range = (0, 30)

        self.x_range = (0, 225)
        self.y_range = (0, 225)

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class RrtStar:
    def __init__(self, x_start, x_goal, step_len,
                 goal_sample_rate, search_radius, iter_max):

        self.s_start = Node(x_start)
        self.s_goal = Node(x_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max
        self.vertex = [self.s_start]
        self.path = []

        self.env = Env()
        self.plotting = Plotting(x_start, x_goal)
        self.utils = Utils()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range


    def selectStartGoalPoints(self, start, goal):

        if self.utils.is_inside_obs(start) :
            print "start fail "
            print(start.x,start.y)
            return False
        else:
            self.s_start = start
            self.vertex = [self.s_start]
            self.plotting.xI = (self.s_start.x, self.s_start.y)

        if self.utils.is_inside_obs(goal) :
            print "goal fail"
            print(goal.x, goal.y)
            return False
        else:
            self.s_goal = goal
            self.plotting.xG =  (self.s_goal.x, self.s_goal.y)

        return True


    def planning(self):
        #plt.imshow(self.env.img, cmap=cm.Greys_r)
        if self.selectStartGoalPoints(self.s_start, self.s_goal):
            self.plotting.img = self.env.img
            for k in range(self.iter_max):
                node_rand = self.generate_random_node(self.goal_sample_rate)
                node_near = self.nearest_neighbor(self.vertex, node_rand)
                node_new = self.new_state(node_near, node_rand)

                if k % 500 == 0:
                    print(k)

                if node_new and not self.utils.is_collision(node_near, node_new):
                    neighbor_index = self.find_near_neighbor(node_new)
                    self.vertex.append(node_new)

                    if neighbor_index:
                        self.choose_parent(node_new, neighbor_index)
                        self.rewire(node_new, neighbor_index)

            index = self.search_goal_parent()

            if index != []:
                self.path = self.extract_path(self.vertex[index])
                self.plotting.animation(self.vertex, self.path, "rrt*, N = " + str(self.iter_max))
                return self.path
            else:
                #self.plotting.animation(self.vertex, self.path, "rrt*, N = " + str(self.iter_max))

                print("fail")



    def new_state(self, node_start, node_goal):
        dist, theta = self.get_distance_and_angle(node_start, node_goal)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))

        node_new.parent = node_start

        return node_new

    def choose_parent(self, node_new, neighbor_index):
        cost = [self.get_new_cost(self.vertex[i], node_new) for i in neighbor_index]

        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_new.parent = self.vertex[cost_min_index]

    def rewire(self, node_new, neighbor_index):
        for i in neighbor_index:
            node_neighbor = self.vertex[i]

            if self.cost(node_neighbor) > self.get_new_cost(node_new, node_neighbor):
                node_neighbor.parent = node_new

    def search_goal_parent(self):
        dist_list = [math.hypot(n.x - self.s_goal.x, n.y - self.s_goal.y) for n in self.vertex]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.cost(self.vertex[i]) for i in node_index
                         if not self.utils.is_collision(self.vertex[i], self.s_goal)]

            if cost_list != []:
                return node_index[int(np.argmin(cost_list))]
            else:
                return []

        return []

    def get_new_cost(self, node_start, node_end):
        dist, _ = self.get_distance_and_angle(node_start, node_end)

        return self.cost(node_start) + dist

    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.randint(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.randint(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    def find_near_neighbor(self, node_new):
        n = len(self.vertex) + 1
        r = min(self.search_radius * math.sqrt((math.log(n) / n)), self.step_len)

        dist_table = [math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.vertex]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
                            not self.utils.is_collision(node_new, self.vertex[ind])]

        return dist_table_index

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    @staticmethod
    def cost(node_p):
        node = node_p
        cost = 0.0

        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost

    def update_cost(self, parent_node):
        OPEN = queue.QueueFIFO()
        OPEN.put(parent_node)

        while not OPEN.empty():
            node = OPEN.get()

            if len(node.child) == 0:
                continue

            for node_c in node.child:
                node_c.Cost = self.get_new_cost(node, node_c)
                OPEN.put(node_c)

    def extract_path(self, node_end):
        path = [[self.s_goal.x, self.s_goal.y]]
        node = node_end

        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


def main():
    x_start = (18, 8)  # Starting node
    x_goal = (146, 31)  # Goal node


    rrt_star = RrtStar(x_start, x_goal, 10, 0.2, 200, 7000)
    rrt_star.planning()


if __name__ == '__main__':
    main()
