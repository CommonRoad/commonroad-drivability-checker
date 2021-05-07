import networkx as nx
import numpy as np
from collections import defaultdict


class RoutePlanner:
    def __init__(self, lanelet_network):
        self.lanelet_network = lanelet_network
        self.create_graph_from_lanelet_network()

    def create_graph_from_lanelet_network(self):
        """ Build a graph from the lanelet network. The length of a lanelet is assigned as weight to
            its outgoing edges as in Bender P., Ziegler J., Stiller C., "Lanelets: Efficient Map
            Representation for Autonomous Driving",  IEEE Intelligent Vehicles Symposium, 2014.
            The edge weight between adjacent lanelets is set to zero.
        """
        self.graph = nx.DiGraph()
        nodes = list()
        edges = list()
        for lanelet in self.lanelet_network.lanelets:
            nodes.append(lanelet.lanelet_id)
            if lanelet.successor:
                for successor in lanelet.successor:
                    l = self.lanelet_network.find_lanelet_by_id(successor)
                    edges.append((lanelet.lanelet_id, l.lanelet_id, {'weight': lanelet.distance[-1]}))
            if lanelet.adj_left:
                l = self.lanelet_network.find_lanelet_by_id(lanelet.adj_left)
                edges.append((lanelet.lanelet_id, l.lanelet_id,
                              {'weight': 0, 'same_dir': lanelet.adj_left_same_direction}))
            if lanelet.adj_right:
                l = self.lanelet_network.find_lanelet_by_id(lanelet.adj_right)
                edges.append((lanelet.lanelet_id, l.lanelet_id,
                              {'weight': 0, 'same_dir': lanelet.adj_right_same_direction}))
        self.graph.add_nodes_from(nodes)
        self.graph.add_edges_from(edges)

    def find_all_shortest_paths(self, source_lanelet_id, target_lanelet_id):
        return list(nx.all_shortest_paths(self.graph,
                                          source=source_lanelet_id,
                                          target=target_lanelet_id))

    def find_all_simple_paths(self, source_lanelet_id, target_lanelet_id):
        return list(nx.all_simple_paths(self.graph,
                                        source=source_lanelet_id,
                                        target=target_lanelet_id))

    def find_all_lanelets_leading_to_goal(self, source_lanelet_id, target_lanelet_id, allow_overtaking=True):
        lanelet_ids_leading_to_goal = set()
        if source_lanelet_id == target_lanelet_id:
            cur_lanelet = self.lanelet_network.find_lanelet_by_id(source_lanelet_id)
            lanelet_ids_leading_to_goal.add(source_lanelet_id)
            if cur_lanelet.adj_left:
                if (cur_lanelet.adj_left_same_direction or
                        (not cur_lanelet.adj_left_same_direction and allow_overtaking)):
                    lanelet_ids_leading_to_goal.add(cur_lanelet.adj_left)
            if cur_lanelet.adj_right and cur_lanelet.adj_right_same_direction:
                lanelet_ids_leading_to_goal.add(cur_lanelet.adj_right)
        else:
            simple_paths = self.find_all_simple_paths(source_lanelet_id, target_lanelet_id)
            for p in simple_paths:
                flag = True
                pre_lanelet = self.lanelet_network.find_lanelet_by_id(p[0])
                for i, l_id in enumerate(p[1:-1]):
                    cur_lanelet = self.lanelet_network.find_lanelet_by_id(l_id)
                    if ((l_id == pre_lanelet.adj_left and
                         not pre_lanelet.adj_left_same_direction) or
                            (l_id == pre_lanelet.adj_right and
                             not pre_lanelet.adj_right_same_direction)):
                        if p[i + 2] in cur_lanelet.successor:
                            flag = False
                            break
                    pre_lanelet = cur_lanelet
                if flag:
                    lanelet_ids_leading_to_goal = lanelet_ids_leading_to_goal.union(set(p))
            if allow_overtaking:
                overtaking_lanelets = set()
                for lanelet_id in lanelet_ids_leading_to_goal:
                    lanelet = self.lanelet_network.find_lanelet_by_id(lanelet_id)
                    if lanelet.adj_left and not pre_lanelet.adj_left_same_direction:
                        overtaking_lanelets.add(lanelet.adj_left)
                lanelet_ids_leading_to_goal = lanelet_ids_leading_to_goal.union(overtaking_lanelets)
        return lanelet_ids_leading_to_goal

    def _lanelets_leading_to_goal_for_path(self, path, allow_overtaking):
        lanelet_ids_leading_to_goal = set()
        flag = True
        pre_lanelet = self.lanelet_network.find_lanelet_by_id(path[0])
        for i, l_id in enumerate(path[1:-1]):
            cur_lanelet = self.lanelet_network.find_lanelet_by_id(l_id)
            if ((l_id == pre_lanelet.adj_left and
                 not pre_lanelet.adj_left_same_direction) or
                    (l_id == pre_lanelet.adj_right and
                     not pre_lanelet.adj_right_same_direction)):
                if path[i + 2] in cur_lanelet.successor:
                    flag = False
                    break
            pre_lanelet = cur_lanelet
        if flag:
            lanelet_ids_leading_to_goal = lanelet_ids_leading_to_goal.union(set(path))
        if allow_overtaking:
            overtaking_lanelets = set()
            for lanelet_id in lanelet_ids_leading_to_goal:
                lanelet = self.lanelet_network.find_lanelet_by_id(lanelet_id)
                overtaking_lanelets = overtaking_lanelets.union(self.find_all_adjacent_lanelets(lanelet))
                if lanelet.adj_left and not pre_lanelet.adj_left_same_direction:
                    overtaking_lanelets.add(lanelet.adj_left)
            lanelet_ids_leading_to_goal = lanelet_ids_leading_to_goal.union(overtaking_lanelets)
        return lanelet_ids_leading_to_goal

    def find_all_adjacent_lanelets(self, lanelet):
        right = left = lanelet
        adjacent_lanelet_ids = set()
        while (left.adj_left != None and left.adj_left_same_direction):
            adjacent_lanelet_ids.add(left.adj_left)
            left = self.lanelet_network.find_lanelet_by_id(left.adj_left)

        while (right.adj_right != None and right.adj_right_same_direction):
            adjacent_lanelet_ids.add(right.adj_right)
            right = self.lanelet_network.find_lanelet_by_id(right.adj_right)
        return adjacent_lanelet_ids

    def find_reference_path_to_goal(self, source_lanelet_id, target_lanelet_id, allow_overtaking=True):
        """ Not working in many situations."""
        shortest_paths = self.find_all_shortest_paths(source_lanelet_id,
                                                      target_lanelet_id)
        reference_paths = defaultdict()
        # find the shortest path with least lane changes
        for path_number, shortest_path in enumerate(shortest_paths):
            reference_lanelets = [self.lanelet_network.find_lanelet_by_id(shortest_path[0])]
            initial_lanelet_id = shortest_path[0]
            lane_changes_cnt = 0
            for i, id in enumerate(shortest_path[1:]):
                lanelet = self.lanelet_network.find_lanelet_by_id(id)
                preceding_lanelet = self.lanelet_network.find_lanelet_by_id(shortest_path[i])

                adjacent_lanelets = set()
                if preceding_lanelet.adj_left:
                    adjacent_lanelets.add(preceding_lanelet.adj_left)
                if preceding_lanelet.adj_right:
                    adjacent_lanelets.add(preceding_lanelet.adj_right)

                if id in adjacent_lanelets:
                    del reference_lanelets[-1]
                    if not preceding_lanelet.lanelet_id == initial_lanelet_id:
                        lane_changes_cnt += 1
                    reference_lanelets.append(lanelet)
                else:
                    reference_lanelets.append(lanelet)
            reference_paths[(lane_changes_cnt, path_number)] = reference_lanelets

        key, reference_lanelets = min(reference_paths.items(), key=lambda x: x[0][0])
        lanelets_leading_to_goal = self._lanelets_leading_to_goal_for_path(shortest_paths[key[1]], allow_overtaking)
        center_vertices = reference_lanelets[0].center_vertices
        for i in range(1, len(reference_lanelets)):
            if np.isclose(center_vertices[-1],
                          reference_lanelets[i].center_vertices[0]).all():
                idx = 1
            else:
                idx = 0
            center_vertices = np.concatenate((center_vertices,
                                              reference_lanelets[i].center_vertices[idx:]))
        return center_vertices, lanelets_leading_to_goal
