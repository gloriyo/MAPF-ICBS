import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location
from multi_agent_planner import ma_star,get_sum_of_cost
import math
import math
import copy

import numpy

'''
# Developer's cNOTE regarding Python's mutable default arguments:
#       The responsibiliy of preserving mutable values of passed arguments and 
#       preventing retention of local mutable defaults by assigning immuatable default values (i.e. param=None) in parameters
#       is the responsiblity of the function being called upon
#       PEP 505 - None-aware operators: https://www.python.org/dev/peps/pep-0505/#syntax-and-semantics
'''

def detect_collision(path1, path2, pos=None):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    assert pos is None
    if pos is None:
        pos = []
    t_range = max(len(path1),len(path2))
    for t in range(t_range):
        loc_c1 =get_location(path1,t)
        loc_c2 = get_location(path2,t)
        loc1 = get_location(path1,t+1)
        loc2 = get_location(path2,t+1)
        if loc1 == loc2:
            pos.append(loc1)
            return pos,t
        if[loc_c1,loc1] ==[loc2,loc_c2]:
            pos.append(loc2)
            pos.append(loc_c2)
            return pos,t
        
       
    return None


def detect_collisions(paths, ma_list, collisions=None):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    if collisions is None:
        collisions = []
    for ai in range(len(paths)-1):
        for aj in range(ai+1,len(paths)):
            if detect_collision(paths[ai],paths[aj]) !=None:
                position,t = detect_collision(paths[ai],paths[aj])

                # find meta-agent group for each agent
                ma_i = {ai}
                ma_j = {aj}
                # check if agents in collision are already in meta-agents 
                for ma in ma_list:
                    if ai in ma:
                        ma_i = ma
                    elif aj in ma:
                        ma_j = ma

                collisions.append({'a1':ai, 'ma1':ma_i,
                                'a2':aj, 'ma2':ma_j,
                                'loc':position,
                                'timestep':t+1})
    return collisions

def count_all_collisions_pair(path1, path2):
    collisions = 0
    t_range = max(len(path1),len(path2))
    for t in range(t_range):
        loc_c1 =get_location(path1,t)
        loc_c2 = get_location(path2,t)
        loc1 = get_location(path1,t+1)
        loc2 = get_location(path2,t+1)
        if loc1 == loc2 or [loc_c1,loc1] ==[loc2,loc_c2]:
            collisions += 1
    return collisions

def count_all_collisions(paths):
    collisions = 0
    for i in range(len(paths)-1):
        for j in range(i+1,len(paths)):
            ij_collisions = count_all_collisions_pair(paths[i],paths[j])
            collisions += ij_collisions

    # print("number of collisions: ", collisions)
    return collisions    
  
def standard_splitting(collision, constraints=None):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    if constraints is None:
        constraints = []


    if len(collision['loc'])==1:
        constraints.append({'agent':collision['a1'],
                            'meta_agent': collision['ma1'],
                            'loc':collision['loc'],
                            'timestep':collision['timestep'],
                            'positive':False
                            })
        constraints.append({'agent':collision['a2'],
                            'meta_agent': collision['ma2'],
                            'loc':collision['loc'],
                            'timestep':collision['timestep'],
                            'positive':False
                            })
    else:
        constraints.append({'agent':collision['a1'],
                            'meta_agent': collision['ma1'],
                            'loc':[collision['loc'][0],collision['loc'][1]],
                            'timestep':collision['timestep'],
                            'positive':False
                            })
        constraints.append({'agent':collision['a2'],
                            'meta_agent': collision['ma'],
                            'loc':[collision['loc'][1],collision['loc'][0]],
                            'timestep':collision['timestep'],
                            'positive':False
                            })
    return constraints

    # pass


def disjoint_splitting(collision, constraints=None):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    if constraints is None:
        constraints = []

    a = random.choice([('a1','ma1'), ('a2','ma2')]) # chosen agent
    agent = a[0]
    meta_agent = a[1]

    if len(collision['loc'])==1:
        constraints.append({'agent':collision[agent],
                            'meta_agent': collision[meta_agent],
                            'loc':collision['loc'],
                            'timestep':collision['timestep'],
                            'positive':True
                            })
        constraints.append({'agent':collision[agent],
                            'meta_agent': collision[meta_agent],
                            'loc':collision['loc'],
                            'timestep':collision['timestep'],
                            'positive':False
                            })
    else:
        if a[0] == collision['a1']:
            constraints.append({'agent':collision[agent],
                                'loc':[collision['loc'][0],collision['loc'][1]],
                                'timestep':collision['timestep'],
                                'positive':True
                                })
            constraints.append({'agent':collision[agent],
                                'loc':[collision['loc'][0],collision['loc'][1]],
                                'timestep':collision['timestep'],
                                'positive':False
                                })
        else:
            constraints.append({'agent':collision[agent],
                                'loc':[collision['loc'][1],collision['loc'][0]],
                                'timestep':collision['timestep'],
                                'positive':True
                                })
            constraints.append({'agent':collision[agent],
                                'loc':[collision['loc'][1],collision['loc'][0]],
                                'timestep':collision['timestep'],
                                'positive':False
                                })
    return constraints


def paths_violate_constraint(constraint, paths, rst=None):
    assert constraint['positive'] is True
    if rst is None:
        rst = []

    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst

def combined_constraints(constraints, new_constraints, updated_constraints=None):
    assert updated_constraints is None

    if isinstance(new_constraints, list):
        updated_constraints = copy.deepcopy(new_constraints)
    else:
        updated_constraints = [new_constraints]

    print('combining constraints:')
    print('const1: ', constraints)
    print('const2: ', updated_constraints)


    for c in constraints:
        if c not in updated_constraints:
            updated_constraints.append(c)
    print(new_constraints)

    assert len(updated_constraints) <= len(constraints) + len(new_constraints)
    return updated_constraints


def bypass_found(curr_cost, new_cost, curr_collisions_num, new_collisions_num):
    # return False
    if curr_cost == new_cost \
        and (new_collisions_num < curr_collisions_num):
        return True
    return False


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        # self.discarded_agents = []
        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['ma_collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1
        

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node


    def find_solution(self, disjoint):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()
        
        if disjoint:
            splitter = disjoint_splitting
        else:
            splitter = standard_splitting
        print("USING: ", splitter)

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {
            'cost':0,
            'constraints': [],
            'paths': [],
            'ma_collisions': [],
            'agent_collisions': None, # matrix of collisions in history between pairs of (meta-)agents
            'ma_list': [] # [{a1,a2}, ... ]
        }       
        
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = ma_star(self.my_map, self.starts, self.goals, self.heuristics,
                          [i], root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            
            root['paths'].append(path)

        print (root ['paths'])


        root['cost'] = get_sum_of_cost(root['paths'])
        root['ma_collisions'] = detect_collisions(root['paths'], root['ma_list'])
        root['agent_collisions'] = numpy.zeros((self.num_of_agents, self.num_of_agents))
        self.push_node(root)


        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to ap['ma_collisions'].index(chosen_collision)list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit


        # algorithm for detecting cardinality
        # as 'non-cardinal' or 'semi-cardinal' or 'cardinal'
        # using standard splitting
        def detect_cardinal_conflict(self, p, collision):
            cardinality = 'non-cardinal'
            new_constraints = standard_splitting(collision)

            print('new constraints:')
            for nc in new_constraints:
                print(nc)

            for c in p['constraints']:
                if c not in new_constraints:
                    new_constraints.append(c)
                        
            ma1 = collision['ma1'] #agent a1
            # to-do: replace with coupled a* for ma-cbs
            alt_paths1 = ma_star(self.my_map,self.starts, self.goals,self.heuristics,list(ma1),new_constraints)
            print(alt_paths1)
            # bigger = False
            # for i in range(len(ma1)): 
            #     if len(alt_path1[i])> len(p['paths'][ma1[i]]):
            #         bigger =True

            curr_paths = []
            for a1 in ma1:
                curr_paths.append(p['paths'][a1])
                
            # get costs for the meta agent
            curr_cost = get_sum_of_cost(curr_paths)
            
            alt_cost = 0 # write inline if later
            if alt_paths1:
                alt_cost = get_sum_of_cost(alt_paths1)

            if not alt_paths1 or alt_cost > curr_cost:
                cardinality = 'semi-cardinal'
                
                print('alt_path1 takes longer or is empty. at least semi-cardinal.')
                
            ma2 = collision['ma2'] #agent a2

            alt_paths2 = ma_star(self.my_map,self.starts, self.goals,self.heuristics,list(ma2),new_constraints)
            print(alt_paths2)

            # for i in range(len(ma2)):
            #     if len(alt_path1[i])> len(p['paths'][ma2[i]]):
            #         bigger =True
            # if not alt_path2 or bigger:
            curr_paths = []
            for a2 in ma2:
                curr_paths.append(p['paths'][a2])
                
            # get costs for the meta agent
            curr_cost = get_sum_of_cost(curr_paths)
            
            alt_cost = 0 # write inline if later
            if alt_paths2:
                alt_cost = get_sum_of_cost(alt_paths2)

            if not alt_paths2 or alt_cost > curr_cost:
                cardinality = 'semi-cardinal'
                if cardinality == 'semi-cardinal':
                    cardinality = 'cardinal'
                    
                    print('identified cardinal conflict')

                else:
                    cardinality = 'semi-cardinal'
                    
                    print('alt_path2 takes longer or is empty. semi-cardinal.')   
                
            return cardinality        


        def should_merge(collision, p, N=0):
            # aSH
            CM = 0
            ma1 = collision['ma1']
            ma2 = collision['ma2']
            
            for ai in ma1:
                for aj in ma2:
                    if ai > aj:
                        ai, aj = aj, ai
                    CM += p['agent_collisions'][ai][aj]
            if CM > N:
                return True
            return False

            
        def generate_child(constraints, paths, group_collisions, ma_list):
            collisions = detect_collisions(paths, ma_list)
            cost = get_sum_of_cost(paths)
            child_node = {
                'cost':cost,
                'constraints': copy.deepcopy(['constraints']),
                'paths': copy.deepcopy(paths), # {0: {'path':[..path...]}, ... , n: {'path':[..path...]} # not sure if other keys are needed
                'ma_collisions': collisions,
                'agent_collisions':copy.deepcopy(p['agent_collisions']), # matrix of collisions in history between pairs of simple agents
                'ma_list': copy.deepcopy(p['ma_list']) # [{a1,a2}, ... ]
            }
            return child_node

        # returns new merged agents (the meta-agent), and updated list of ma_list
        def merge_agents(self, collision, p):

            # constraints = standard_splitting(collision)
            
            # collision simple agents and their meta-agent group
            a1 = collision['a1']
            a2 = collision['a2']
            ma1 = collision['ma1']
            ma2 = collision['ma2']


            # group1 = {a1}
            # group2 = {a2}

            # new_constraints = copy.deepcopy(p['constraints'])


            ###### REPLACE ###########
            # # check if agents in collision are already part of a meta-agent solution found
            # for ma in p['ma_list']:
            #     if a1 in ma:
            #         group1 = ma
            #         # we will merge two groups so we do not want to detect this old meta-agent in the future again, same as group2
            #         p['ma_list'].remove(ma)
            #     elif a2 in ma:
            #         group2 = ma
            #         p['ma_list'].remove(ma)

            meta_agent = set.union(ma1, ma2)

            # # remove existing internal constraints
            # for i, constraint in enumerate(new_constraints):
            #     if ma1 == constraint['meta_agent'] and ma2 == constraint['meta_agent']:
            #         new_constraints.remove 

            # replace old meta-agents with merged meta-agent
            for constraint in new_constraints:
                if ma1 == constraint['meta_agent'] or ma2 == constraint['meta_agent']:
                    constraint['meta_agent'] = meta_agent



            ma_list = copy.deepcopy(['ma_list'])
            ma_list.append(meta_agent)

            return meta_agent, ma_list

        # ATTENTION: THE CBS LOOOOOOOOOOOOP ============@#￥#%#@￥@#%##@￥======  STARTS ---#￥%------   HERE  ---- @
        # normal CBS with disjoint and standard splitting
        while len(self.open_list) > 0:
            if self.num_of_generated > 50000:
                print('reached maximum number of nodes. Returning...')
                return None #what are u guys doing here  Admiring the 50000 nodes?
            print('\n')  
            p = self.pop_node()
            if p['ma_collisions'] == []:
                self.print_results(p)
                for pa in p['paths']:
                    print(pa)
                return p['paths'], self.num_of_generated, self.num_of_expanded # number of nodes generated/expanded for comparing implementations


            print('\nNode expanded. Collisions: ', p['ma_collisions'])
            print('Trying to find cardinal conflict.')

            # USING STANDARD SPLITTING
            # select a cardinal conflict;
            # if none, select a semi-cardinal conflict
            # if none, select a random conflict
            collision_cardinalities = None
            collision_cardinalities = []
            chosen_collision = None
            new_constraints = None
            collision_type = None
            for collision in p['ma_collisions']:
                collision_type = detect_cardinal_conflict(self, p, collision)
                if collision_type == 'cardinal' and new_constraints is None:    
                    print('Detected cardinal collision. Chose it.')
                    print(collision)

                    chosen_collision = collision
                    # collision_type = 'cardinal'
                    break

            if not new_constraints:
                assert new_constraints is None
                
                for collision in p['ma_collisions']:
                    collision_type = detect_cardinal_conflict(self, p, collision)
                    if collision_type == 'semi-cardinal':    
                        
                        print('Detected semi-cardinal collision. Chose it.')
                        print(collision)
                        
                        chosen_collision = collision
                        # collision_type = 'semi-cardinal'
                        break
                if not new_constraints:
                    assert new_constraints is None    
                    chosen_collision = p['ma_collisions'][0] 
                    assert chosen_collision is not None
                    collision_type = 'non-cardinal'
                    print('No cardinal or semi-cardinal conflict. Randomly choosing...')


            # keep track of collisions in history (aSh)
            chosen_a1 = chosen_collision['a1']
            chosen_a2 = chosen_collision['a2']
            if chosen_a1 > chosen_a2:
                # swap to only fill half of the matrix
                chosen_a1, chosen_a2 = chosen_a2, chosen_a1
            p['agent_collisions'][chosen_a1][chosen_a2] += 1


            # implementing bypassing conflicts
            # if collision_type != 'cardinal' and find_bypass(self,p, chosen_collision, collision_type):
            #         continue

            print('NEW CONSTS:')
            new_constraints = splitter(chosen_collision)

            # child_nodes = None
            child_nodes = []
            assert child_nodes == []
            bypass_successful = False
            for constraint in new_constraints:
                # q = {'cost':0,
                #     'constraints': [constraint],
                #     'paths':[],
                #     'ma_collisions':[],
                #     'discarded_agents': []
                # }
                # for c in p['constraints']:
                #     if c not in q['constraints']:
                #         q['constraints'].append(c)
                # for pa in p['paths']:
                #     q['paths'].append(pa)
                
                # for da in p['discarded_agents']:
                #     q['discarded_agents'].append(da)
                
                updated_constraints = combined_constraints(p['constraints'], constraint)
                q = generate_child(updated_constraints, p['paths'], p['agent_collisions'], p['ma_list'])

                ai = constraint['meta_agent']

                # to-do: replace with coupled a* for ma-cbs
                path = ma_star(self.my_map,self.starts, self.goals,self.heuristics,list(ai),q['constraints']) 
                
                if path is not None:
                    q['paths'][ai]= path
                    # task 4
                    continue_flag = False
                    if constraint['positive']:
                        vol = paths_violate_constraint(constraint,q['paths'])
                        for v in vol:
                            path_v = ma_star(self.my_map,self.starts, self.goals,self.heuristics,list(v),q['constraints'])
                            if path_v  is None:
                                continue_flag = True
                            else:
                                q['paths'][v] = path_v
                        if continue_flag:
                            continue
                    
                    q['ma_collisions'] = detect_collisions(q['paths'])
                    q['cost'] = get_sum_of_cost(q['paths'])
                    # CHECK BYPASS HERE.......
                    #     if q['cost'] == p['cost'] \
                    #         and (len(q['ma_collisions']) < len(p['ma_collisions'])):

                    # if bypass is found, push only the current child and exit loop


                    # assert that bypass is not possible if cardinal
                    if collision_type == 'cardinal':
                        assert bypass_found(p['cost'], q['cost'], len(p['ma_collisions']), len(q['ma_collisions'])) == False

                    # conflict should be resolved due to new constraints; compare costs and total number of collisions
                    if collision_type != 'cardinal' \
                            and bypass_found(p['cost'], q['cost'], len(p['ma_collisions']), len(q['ma_collisions'])):
                        print('bypass found')
                        self.push_node(q)
                        
                        bypass_successful = True
                        break # break out of constraint loop
                    child_nodes.append(copy.deepcopy(q))
            else: # bypass found
                continue # start of while loop

            assert not bypass_successful

            # MA-CBS
            if should_merge(collision,p):
                # returns meta_agent, ma_list
                meta_agent, updated_ma_list = merge_agents(self, collision, p)

                # Update paths
                meta_agent_paths = ma_star(self.my_map,self.starts, self.goals,self.heuristics,list(meta_agent),p['constraints'])

                # if can be 
                if meta_agent_paths:
                    assert meta_agent not in p['ma_list']

                    # Update collisions, cost
                    updated_node = generate_child(p['constraints'],  meta_agent_paths, p['agent_collisions'], updated_ma_list) 


                    # print('agents {}, {} merged into agent {}'.format(collision['a1'], a2, meta_agent))

                    # Merge & restart
                    # if merge_restart():
                        # restart_search()
                    self.push_node(updated_node)    

                    continue # start of while loop

                
            assert len(child_nodes) <= 2
            print('bypass not found')
            for n in child_nodes:
                self.push_node(n)     
                    
        return None


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        
        # file = "nodes-generated.csv"
        # result_file = open(file, "a", buffering=1)
        # result_file.write("{}\n".format(self.num_of_generated))

        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
