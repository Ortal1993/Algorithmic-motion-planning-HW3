import numpy as np
from RRTTree import RRTTree
import time
import math

class RRTMotionPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob):
        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob

        #our
        self.edges_start_end = {}
        self.goal_id = -1

    def plan(self):
        '''
        Compute and return the plan. 
        The function should return a numpy array containing the states in the configuration space.
        '''
        start_time = time.time()

        # initialize an empty plan.
        start_id = self.tree.add_vertex(self.planning_env.start)
        
        #for debugging
        i = 0

        #######
        while not self.tree.is_goal_exists(self.planning_env.goal):
            conf_rand_state = self.sample_random()
            #TODO - check if config_validity_checker is neccesary
            if (not self.planning_env.config_validity_checker(conf_rand_state)):
                continue
            conf_near_id, conf_near_state = self.tree.get_nearest_config(conf_rand_state)
            if conf_near_id == self.goal_id:
                continue
            conf_new_state = self.extend(conf_near_state, conf_rand_state)
            if not np.array_equal(conf_near_state, conf_new_state):            
                if self.planning_env.edge_validity_checker(conf_near_state, conf_new_state):                
                    conf_new_id = -1
                    if np.array_equal(conf_new_state, self.planning_env.goal):
                        if (self.goal_id == -1):                
                            conf_new_id = self.tree.add_vertex(conf_new_state)
                            dist = self.planning_env.robot.compute_distance(conf_near_state, conf_new_state)
                            self.add_edge(conf_near_id, conf_new_id, dist)
                            self.goal_id = conf_new_id
                        else:
                            conf_new_id = self.goal_id
                    else:
                        conf_new_id = self.tree.add_vertex(conf_new_state)
                        dist = self.planning_env.robot.compute_distance(conf_near_state, conf_new_state)
                        self.add_edge(conf_near_id, conf_new_id, dist)
            
            i += 1
            print(i)

        plan = self.construct_path(self.goal_id)
                    
        # print total path cost and time
        print('Total cost of path: {:.2f}'.format(self.compute_cost(plan)))
        print('Total time: {:.2f}'.format(time.time()-start_time))

        return np.array(plan)

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps in the configuration space.
        @param plan A given plan for the robot.
        '''
        # TODO: Task 2.3
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
        # TODO: Task 2.3
        #TODO - check if we need to convert each config to euclidean coords and than compute?
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
            return self.planning_env.goal
        # With probability 1 - p_bias, sample randomly within link limits
        lower_limit, upper_limit = -math.pi, math.pi
        conf = []
        for i in range(self.planning_env.robot.dim):           
            conf.append(np.random.uniform(lower_limit, upper_limit))
        return np.array(conf)

    def add_edge(self, sid, eid, edge_cost):
        self.add_edge_start_end(sid, eid)
        self.tree.add_edge(sid, eid, edge_cost)

    #our
    def add_edge_start_end(self, sid, eid):
        if sid not in self.edges_start_end:
            self.edges_start_end[sid] = []
        self.edges_start_end[sid].append(eid)

    #our
    def construct_path(self, dest_id):
        '''
        calculate path in the configuration space.
        should be a shape of (N, 4) for N configurations, including start and goal configurations.
        '''
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
    
    