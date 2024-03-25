import numpy as np
from RRTTree import RRTTree
import time
import math

class RRTInspectionPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob, coverage):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env, task="ip")

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob
        self.coverage = coverage
        
        #our
        self.all_inspection_points = self.planning_env.inspection_points
        self.all_seen_inspection_points = None
        self.coverage_of_all_seen = 0.0
        self.edges_start_end = {}
        self.k = 5
        self.removed_edges = {} #dict[edge] = bool #dict[edge] = bool, true if it was already removed from the tree

    def plan(self):
        '''
        Compute and return the plan. 
        The function should return a numpy array containing the states in the configuration space.
        '''
        start_time = time.time()

        # initialize an empty plan.
        plan = []

        # TODO: Task 2.4
        conf_start_state = self.planning_env.start
        inspected_points = self.planning_env.get_inspected_points(conf_start_state)
        #We do not use here self.add_vertex on purpose
        start_id = self.tree.add_vertex(conf_start_state, inspected_points)
        self.all_seen_inspection_points = inspected_points
        self.coverage_of_all_seen = self.planning_env.compute_coverage(self.all_seen_inspection_points)

        #for debugging
        i = 0

        # your stopping condition should look like this: 
        while self.tree.max_coverage < self.coverage:
            conf_rand_state = self.sample_random()
            #check if rand config is valid 
            if not self.planning_env.config_validity_checker(conf_rand_state):
                continue
            #check if the rand config adds new inspection points that were not seen
            if not self.check_if_new_ipoints(conf_rand_state):
                continue

            conf_near_id, conf_near_state = self.tree.get_nearest_config(conf_rand_state)
            conf_new_state = self.extend(conf_near_state, conf_rand_state)
            if not self.planning_env.config_validity_checker(conf_new_state):
                continue

            if self.planning_env.edge_validity_checker(conf_near_state, conf_new_state):
                #I removed the part that checks if conf_new_state equals conf_goal_state - is it ok?
                conf_new_id = self.add_vertex(conf_near_state, conf_new_state)
                dist = self.planning_env.robot.compute_distance(conf_near_state, conf_new_state)
                self.add_edge(conf_near_id, conf_new_id, dist)

                #Adding rewire
                k_nearest_ids, _ = self.tree.get_k_nearest_neighbors(conf_new_state, self.k)
                    
                #conf_start_state could not be child of anyone
                if not np.array_equal(conf_new_state, conf_start_state):
                    for potential_father_id in k_nearest_ids:
                        self.rewire(potential_father_id, conf_new_id)
                
                for potential_child_id in k_nearest_ids:
                    if(potential_child_id != start_id):
                        self.rewire(conf_new_id, potential_child_id)
            
            i += 1
            print(i)
            if i % 100 == 0:
                print("Max coverage: ", self.tree.max_coverage)
                print("All points seen sor far: ", self.coverage_of_all_seen)
                print("Num of vertices: ", len(self.tree.vertices))

        print("End: Max coverage: ", self.tree.max_coverage)
        print("End: All points seen sor far: ", self.coverage_of_all_seen)
        print("End: Num of vertices: ", len(self.tree.vertices))

        plan = self.construct_path(self.tree.max_coverage_id)
                    
        # print total path cost and time
        print('Total cost of path: {:.2f}'.format(self.compute_cost(plan)))
        print('Total time: {:.2f}'.format(time.time()-start_time))
        print('Plan: ', plan)

        return np.array(plan)

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances 
        between steps in the configuration space.
        @param plan A given plan for the robot.
        '''
        # TODO: Task 2.4
        cost = 0.0
        if len(plan) == 0:
            cost = math.inf
        for i in range(len(plan) - 1):
            config_1 = self.tree.get_vertex_for_config(plan[i]).config
            config_2 = self.tree.get_vertex_for_config(plan[i + 1]).config
            cost += self.planning_env.robot.compute_distance(config_1, config_2)
        
        return cost

    def extend(self, near_config, rand_config):
        '''
        Compute and return a new configuration for the sampled one.
        @param near_config The nearest configuration to the sampled configuration.
        @param rand_config The sampled configuration.
        '''
        # TODO: Task 2.4
        if self.ext_mode == "E2":
            step_size = 10
            dist = np.linalg.norm(rand_config - near_config)
            if step_size >= dist:
                return rand_config            
            rand_config = near_config + (((rand_config - near_config)/dist) * step_size)

        return rand_config

    def check_if_new_ipoints(self, conf):
        ipoints_rand_config = self.planning_env.get_inspected_points(conf)
        if len(ipoints_rand_config) == 0:
            return False
        union_ipoints = self.planning_env.compute_union_of_points(self.all_seen_inspection_points, 
                                                                  ipoints_rand_config)
        coverage = self.planning_env.compute_coverage(union_ipoints)
        if coverage > self.tree.max_coverage:
            return True
        return False
    
    #our function
    def sample_random(self) -> np.array:
        if np.random.uniform(0, 1) < self.goal_prob:
            return self.tree.vertices[self.tree.max_coverage_id].config
        
        # With probability 1 - p_bias, sample randomly within link limits
        lower_limit, upper_limit = -math.pi, math.pi
        conf = []
        for i in range(self.planning_env.robot.dim):           
            conf.append(np.random.uniform(lower_limit, upper_limit))
        return np.array(conf)
    
        """# Sample random point within a circle centered at the node position with radius=max_coverage_radius
        if np.random.uniform(0, 1) < self.goal_prob:
            #sample around the config with the max coverage
            max_coverage_config = self.tree.max_coverage_config
            radius = np.random.uniform(0, 2*np.pi)
            conf = []
            for i in self.dim:           
                x = node_position[0] * np.cos(radius)
                y = node_position[1] * np.sin(radius)
                conf.append((x, y))
            return conf
        
        lower_limit, upper_limit = -math.pi, math.pi
        conf = []
        for i in self.dim:           
            conf.append(np.random.uniform(lower_limit, upper_limit))
        return np.array(conf)"""
    
    #our
    def add_edge(self, sid, eid, edge_cost):
        self.add_edge_start_end(sid, eid)
        self.tree.add_edge(sid, eid, edge_cost)

    #our
    def add_edge_start_end(self, sid, eid):
        if sid not in self.edges_start_end:
            self.edges_start_end[sid] = []
        self.edges_start_end[sid].append(eid)

    #our
    def add_vertex(self, config_father, config_new_child):
        ipoints_conf_father = self.tree.get_vertex_for_config(config_father).inspected_points
        ipoints_conf_new_child = self.planning_env.get_inspected_points(config_new_child)
        ipoints_union = self.planning_env.compute_union_of_points(ipoints_conf_father, ipoints_conf_new_child)
        
        self.all_seen_inspection_points = self.planning_env.compute_union_of_points(
                    self.all_seen_inspection_points,
                    ipoints_conf_new_child)
        self.coverage_of_all_seen = self.planning_env.compute_coverage(
                    self.all_seen_inspection_points)

        return self.tree.add_vertex(config_new_child, ipoints_union)

    #our
    def construct_path(self, dest_id):
        path_goal_to_start = []
    
        curr_id = dest_id
        start_id = 0 
        if curr_id != None:
            while (curr_id != start_id):
                path_goal_to_start.append(self.tree.vertices[curr_id].config)
                curr_id = self.tree.edges[curr_id]
            path_goal_to_start.append(self.planning_env.start)
        
        path_start_to_goal = np.flipud(path_goal_to_start)
         #should be a shape of (N, 4)
        path = np.stack(path_start_to_goal)
        return path
    
    #our
    def rewire(self, x_potential_parent_id, x_child_id) -> None:
        '''
        Implement the rewire method
        @param x_potential_parent_id - candidte to become a parent
        @param x_child_id - the id of the child vertex
        return None
        '''
        # TODO
        #do rewire only if edge is new
        if self.is_edge_new(x_child_id, x_potential_parent_id):
            config_potential_parent = self.tree.vertices[x_potential_parent_id].config
            config_child = self.tree.vertices[x_child_id].config
            if self.planning_env.edge_validity_checker(config_potential_parent, config_child):
                curr_ipoints_conf_child = self.tree.get_vertex_for_config(config_child).inspected_points

                ipoints_conf_potential_parent = self.tree.get_vertex_for_config(config_potential_parent).inspected_points
                ipoints_conf_child = self.planning_env.get_inspected_points(config_child)
                ipoints_union = self.planning_env.compute_union_of_points(ipoints_conf_potential_parent, 
                                                                          ipoints_conf_child)            

                if len(ipoints_union) == len(curr_ipoints_conf_child):
                    cost_new_edge = self.planning_env.robot.compute_distance(config_child, config_potential_parent)
                    cost_parent = self.tree.vertices[x_potential_parent_id].cost
                    cost_child = self.tree.vertices[x_child_id].cost
                    if cost_new_edge + cost_parent < cost_child:
                        self.do_rewire(x_child_id, config_child, x_potential_parent_id, config_potential_parent, ipoints_union)

                if len(ipoints_union) > len(curr_ipoints_conf_child):
                    cost_new_edge = self.planning_env.robot.compute_distance(config_child, config_potential_parent)
                    cost_parent = self.tree.vertices[x_potential_parent_id].cost
                    cost_child = self.tree.vertices[x_child_id].cost
                    if cost_new_edge + cost_parent < cost_child:
                        self.do_rewire(x_child_id, config_child, x_potential_parent_id, config_potential_parent, ipoints_union)

    #our
    def do_rewire(self, x_child_id, config_child, x_potential_parent_id, config_potential_parent, ipoints_union):
        prev_father = self.tree.edges[x_child_id]
    
        #removing the old edge from edges_start_end
        if prev_father in self.edges_start_end:
            if x_child_id in self.edges_start_end[prev_father]:
                self.edges_start_end[prev_father].remove(x_child_id)
                if len(self.edges_start_end[prev_father]) == 0:
                    del self.edges_start_end[prev_father]
            #else:
                #setting cost of prev_father to zero and removing the ipoints of prev_father from his children
                # self.tree.vertices[prev_father].cost = 0.0
                # conf_father_of_father = self.tree.vertices[self.tree.edges[prev_father]].config
                # ipoitns_to_remove = self.planning_env.get_inspected_points(conf_father_of_father)
                # ipoitns_of_father = self.tree.vertices[prev_father].inspected_points
                # # Remove elements of array2 from array1
                # ipoints_reduced = np.setdiff1d(ipoitns_of_father, ipoitns_to_remove)
                # self.tree.vertices[prev_father].set_inspected_points(ipoints_reduced)
                # self.propagate_to_children(prev_father)

        #updating the inspected_points of child
        self.tree.vertices[x_child_id].set_inspected_points(ipoints_union)

        #updating max coverage
        self.tree.set_max_coverage(x_child_id, config_child, ipoints_union)

        #there is no need to update self.all_seen_inspection_points and self.coverage_of_all_seen

        #adding the new edge
        dist = self.planning_env.robot.compute_distance(config_child, config_potential_parent)
        self.add_edge(x_potential_parent_id, x_child_id, dist)
        
        self.propagate_to_children(x_child_id)

    #our
    def is_edge_new(self, id_child, id_father):
        if (id_child == id_father):
            return False
        
        if (id_child in self.tree.edges):
            if self.tree.edges[id_child] == id_father:
                return False
        if (id_child in self.edges_start_end):
            if id_father in self.edges_start_end[id_child]:
                return False
        if (id_father in self.tree.edges):
            if self.tree.edges[id_father] == id_child:
                return False
        if (id_father in self.edges_start_end):
            if id_child in self.edges_start_end[id_father]:
                return False
        
        return True
    
    #our function
    def propagate_to_children(self, id_father_origin):
        if id_father_origin in self.edges_start_end:#it means it has children
            ids_father = []
            ids_father.append(id_father_origin)

            for id_father in ids_father:
                config_father = self.tree.vertices[id_father].config
                ipoints_conf_father = self.tree.get_vertex_for_config(config_father).inspected_points
                vertices_to_change = self.edges_start_end[id_father]
                for v_id in vertices_to_change:
                    #updating ipoints
                    config_child = self.tree.vertices[v_id].config
                    ipoints_conf_child = self.planning_env.get_inspected_points(config_child)
                    ipoints_union = self.planning_env.compute_union_of_points(ipoints_conf_father, 
                                                                          ipoints_conf_child)
                    self.tree.vertices[v_id].set_inspected_points(ipoints_union)

                    #updating cost
                    cost_edge = self.planning_env.robot.compute_distance(config_father, config_child)
                    cost_father = self.tree.vertices[id_father].cost
                    self.tree.vertices[v_id].set_cost(cost_edge + cost_father)

                    #updating max coverage
                    self.tree.set_max_coverage(v_id, config_child, ipoints_union)
                    
                    if v_id in self.edges_start_end and len(self.edges_start_end[v_id]) != 0:
                        ids_father.append(v_id)

                    
    