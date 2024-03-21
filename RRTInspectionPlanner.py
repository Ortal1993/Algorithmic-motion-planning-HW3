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
        self.edges_start_end = {}

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

        #for debugging
        i = 0

        # your stopping condition should look like this: 
        while self.tree.max_coverage < self.coverage:
            conf_rand_state = self.sample_random()
            #TODO - check if config_validity_checker is neccesary
            if not self.planning_env.config_validity_checker(conf_rand_state):
                continue
            conf_near_id, conf_near_state = self.tree.get_nearest_config(conf_rand_state)
            conf_new_state = self.extend(conf_near_state, conf_rand_state)
            if not np.array_equal(conf_near_state, conf_new_state):            
                if self.planning_env.edge_validity_checker(conf_near_state, conf_new_state):
                    #I removed the part that checks if conf_new_state equals conf_goal_state - is it ok?
                    conf_new_id = self.add_vertex(conf_near_state, conf_new_state)
                    dist = self.planning_env.robot.compute_distance(conf_near_state, conf_new_state)
                    self.add_edge(conf_near_id, conf_new_id, dist)  

            i += 1
            print(i)
            print("tree.max_coverage: ", self.tree.max_coverage)              

        plan = self.construct_path(self.tree.max_coverage_id)
                    
        # print total path cost and time
        print('Total cost of path: {:.2f}'.format(self.compute_cost(plan)))
        print('Total time: {:.2f}'.format(time.time()-start_time))

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

    #our function
    def sample_random(self) -> np.array:
        if np.random.uniform(0, 1) < self.goal_prob:
            return self.tree.vertices[self.tree.max_coverage_id].config #TODO - not sure about it!!
        # With probability 1 - p_bias, sample randomly within link limits
        lower_limit, upper_limit = -math.pi, math.pi
        conf = []
        for i in range(self.planning_env.robot.dim):           
            conf.append(np.random.uniform(lower_limit, upper_limit))
        return np.array(conf)
    
        """# Sample random point within a circle centered at the node position with radius=max_coverage_radius
        if np.random.uniform(0, 1) < self.goal_prob:
            #sample arount the config with the max coverage
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
    
    def add_edge(self, sid, eid, edge_cost):
        self.add_edge_start_end(sid, eid)
        self.tree.add_edge(sid, eid, edge_cost)

    #our
    def add_edge_start_end(self, sid, eid):
        if sid not in self.edges_start_end:
            self.edges_start_end[sid] = []
        self.edges_start_end[sid].append(eid)

    def add_vertex(self, config_father, config_new_child):
        ipoints_conf_father = self.planning_env.get_inspected_points(config_father)
        ipoints_conf_new_child = self.planning_env.get_inspected_points(config_new_child)
        ipoints_union = self.planning_env.compute_union_of_points(ipoints_conf_father, 
                                                                ipoints_conf_new_child)

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

    