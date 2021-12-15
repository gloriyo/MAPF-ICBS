import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
import math
import copy

'''
# Developer's cNOTE regarding Python's mutable default arguments:
#       The responsibiliy of preserving mutable values of passed arguments and 
#       preventing retention of local mutable defaults by assigning immuatable default values (i.e. param=None) in parameters
#       is the responsiblity of the function being called upon
#       PEP 505 - None-aware operators: https://www.python.org/dev/peps/pep-0505/#syntax-and-semantics
'''

def detect_collision(path1, path2, pos=None):
    '''Return the first collision that occurs between two robot paths

    Parameters
    ----------
    path1 : list
        A list of tuples representing locations in the first agent's path
    path2 : list
        A list of tuples representing locations in the second agent's path

    Returns
    -------
    list
        a list of location(s) for the first vertex or edge collision
    '''
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


def detect_collisions(paths, collisions=None):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    if collisions is None:
        collisions = []
    for i in range(len(paths)-1):
        for j in range(i+1,len(paths)):
            if detect_collision(paths[i],paths[j]) !=None:
                position,t = detect_collision(paths[i],paths[j])
                collisions.append({'a1':i,
                                'a2':j,
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
                            'loc':collision['loc'],
                            'timestep':collision['timestep'],
                            'positive':False
                            })
        constraints.append({'agent':collision['a2'],
                            'loc':collision['loc'],
                            'timestep':collision['timestep'],
                            'positive':False
                            })
    else:
        constraints.append({'agent':collision['a1'],
                            'loc':[collision['loc'][0],collision['loc'][1]],
                            'timestep':collision['timestep'],
                            'positive':False
                            })
        constraints.append({'agent':collision['a2'],
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

    agent = random.randint(0,1)
    a = 'a'+str(agent +1)
    if len(collision['loc'])==1:
        constraints.append({'agent':collision[a],
                            'loc':collision['loc'],
                            'timestep':collision['timestep'],
                            'positive':True
                            })
        constraints.append({'agent':collision[a],
                            'loc':collision['loc'],
                            'timestep':collision['timestep'],
                            'positive':False
                            })
    else:
        if agent == 0:
            constraints.append({'agent':collision[a],
                                'loc':[collision['loc'][0],collision['loc'][1]],
                                'timestep':collision['timestep'],
                                'positive':True
                                })
            constraints.append({'agent':collision[a],
                                'loc':[collision['loc'][0],collision['loc'][1]],
                                'timestep':collision['timestep'],
                                'positive':False
                                })
        else:
            constraints.append({'agent':collision[a],
                                'loc':[collision['loc'][1],collision['loc'][0]],
                                'timestep':collision['timestep'],
                                'positive':True
                                })
            constraints.append({'agent':collision[a],
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
    updated_constraints = copy.deepcopy(new_constraints)

    for c in constraints:
        if c not in updated_constraints:
            updated_constraints.append(c)
    print(new_constraints)
    assert len(new_constraints) <= 2
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
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
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
        root = {'cost': 0,
                'constraints': [],
                # 'paths': [],
                'agents': {},
                'collisions': [],
                'discarded_agents': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            # root['paths'].append(path)

            # thank you Gloria!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            
            root['agents'][i] = {
                'path': path[i],
                'coupled_agents': [i]
            }

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # # Task 3.1: Testing
        # print(root['collisions'])

        # # Task 3.2: Testing
        # print('Root Collisions:')
        # for collision in root['collisions']:
        #     # print(standard_splitting(collision))
        #     print(disjoint_splitting(collision))


        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to ap['collisions'].index(chosen_collision)list of constraints (using your
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
                        
            a1 = collision['a1'] #agent a1
            alt_path1 = a_star(self.my_map,self.starts[a1], self.goals[a1],self.heuristics[a1],a1,new_constraints)
            print(alt_path1)
            if not alt_path1 or len(alt_path1) > len(p['paths'][a1]):
                cardinality = 'semi-cardinal'
                
                print('alt_path1 takes longer or is empty. at least semi-cardinal.')
                
            a2 = collision['a2'] #agent a2
            alt_path2 = a_star(self.my_map,self.starts[a2], self.goals[a2],self.heuristics[a2],a2,new_constraints)
            print(alt_path2)
            if not alt_path2 or len(alt_path2) > len(p['paths'][a2]):
                if cardinality == 'semi-cardinal':
                    cardinality = 'cardinal'
                    
                    print('identified cardinal conflict')

                else:
                    cardinality = 'semi-cardinal'
                    
                    print('alt_path2 takes longer or is empty. semi-cardinal.')   
                
            return cardinality
 
        def should_merge(collision, p, N=0):
           
            a1 = collision['a1']
            a2 = collision['a2']
    
            if count_all_collisions_pair(p['paths'][a1], p['paths'][a2]) >= N:
                return True
                
            return False

        # returns a new node with merged agents, based on collision at hand
        def merge_agents(self, collision, p):

            # constraints = standard_splitting(collision)
            
            a1 = collision['a1']
            a2 = collision['a2']

            group1 = {a1}
            group2 = {a2}

            # child node with new meta agent # to-do move node init after 
            q = {
                'cost':0,
                'constraints': copy.deepcopy(p['constraints']),
                # 'paths':[],
                'agents': copy.deepcopy(p['agents']), # {0: {'path':[..path...]}, ... , n: {'path':[..path...]} # not sure if other keys are needed
                'collisions': None,
                'group_collisions':copy.deepcopy(p['group_collisions']), # list of sets of numbers of collisions between (meta-)agents
                'meta_agents': copy.deepcopy(p['meta_agents']) # [{a1,a2}, ... ]
            }

            # check if agents in collision are already part of a meta-agent solution found
            for ma in q['meta_agents']:
                if a1 in ma:
                    group1 = ma
                elif a2 in ma:
                    group2 = ma
                
            new_constraints = None # to-do

            # # solve new group
            # paths for agents in meta-agent
            temp_agents = copy.deepcopy(p['agents'])
            meta_agent = None
            # agent 1 then 2
            # group 1
            for a in group1:
                path = a_star(self.my_map,self.starts[a1], self.goals[a1], self.heuristics[a1], \
                            a1,new_constraints)
                if path is None:
                    print('solution doesn\'t exist')
                    break # return None
                temp_agents[a] = { 
                    'path': path
                }
            else: # continue
                # group_2
                for a in group2:
                    path = a_star(self.my_map,self.starts[a2], self.goals[a2], self.heuristics[a2], \
                                a2,new_constraints)
                    if path is None:
                        print('solution doesn\'t exist')
                        break # return None
                    temp_agents[a] = { 
                        'path': path
                    }
                else: # solution found
                    meta_agent = set.union(group1, group2)         

            
            if meta_agent is None:
                temp_agents = copy.deepcopy(p['agents'])
                # repeat above in reverse order # agent 2 then 1
                pass


            if meta_agent:
                assert meta_agent not in q['meta_agents']

                ## to-do constraints meta-agent constraints: group1 + group 2

                q['agents'] = temp_agents
                q['collisions'] = detect_collisions(q['agents'], q['meta_agents'])
                q['cost'] = get_sum_of_cost(q['paths'])
                q['meta_agents'].append(meta_agent)
 

            print('agents {}, {} merged into agent {}'.format(a1, a2, meta_agent))


            assert q
            return q       


            
        # normal CBS with disjoint and standard splitting
        while len(self.open_list) > 0:
            if self.num_of_generated > 50000:
                print('reached maximum number of nodes. Returning...')
                return None
            print('\n')
            p = self.pop_node()
            if p['collisions'] == []:
                self.print_results(p)
                for pa in p['paths']:
                    print(pa)
                return p['paths'], self.num_of_generated, self.num_of_expanded # number of nodes generated/expanded for comparing implementations


            print('\nNode expanded. Collisions: ', p['collisions'])
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
            for collision in p['collisions']:
                consts = None
                consts = splitter(collision)

                print(consts)
                assert len(consts) == 2

                cardinality = detect_cardinal_conflict(self, p, collision)

                # cardinality = detect_cardinality(self, disjoint, p, collision, consts) # method b: costs
                collision_cardinalities.append([cardinality, consts]) # change to dictionary...
                if cardinality == 'cardinal' and new_constraints is None:    
                    print('Detected cardinal collision. Chose it.')
                    print(collision)
                    assert collision

                    chosen_collision = collision
                    collision_type = 'cardinal'
                    new_constraints = consts
                    break

            if not new_constraints:
                assert new_constraints is None
                
                for collision in p['collisions']:
                    # consts = splitter(collision) ## NO....

                    assert len(consts) == 2
                    cardinality = collision_cardinalities[p['collisions'].index(collision)][0]
                    consts = collision_cardinalities[p['collisions'].index(collision)][1]
                    assert len(consts) == 2
                    if cardinality == 'semi-cardinal':    
                        
                        print('Detected semi-cardinal collision. Chose it.')
                        print(collision)
                        
                        chosen_collision = collision
                        collision_type = 'semi-cardinal'
                        new_constraints = consts
                        break
                if not new_constraints:
                    assert new_constraints is None    
                    chosen_collision = p['collisions'][0] 
                    assert chosen_collision is not None
                    collision_type = 'non-cardinal'
                    cardinality = 'non-cardinal'


                    # new_constraints = splitter(chosen_collision)
                    new_constraints = collision_cardinalities[0][1]
                    print('No cardinal or semi-cardinal conflict. Randomly choosing...')

            # implementing bypassing conflicts
            # if collision_type != 'cardinal' and find_bypass(self,p, chosen_collision, collision_type):
            #         continue
            assert new_constraints is not None
            print('NEW CONSTS:')
            print(new_constraints)

            # child_nodes = None
            child_nodes = []
            assert child_nodes == []
            bypass_successful = False
            for constraint in new_constraints:
                q = {'cost':0,
                    'constraints': [constraint],
                    'paths':[],
                    'collisions':[],
                    'discarded_agents': []
                }
                for c in p['constraints']:
                    if c not in q['constraints']:
                        q['constraints'].append(c)
                for pa in p['paths']:
                    q['paths'].append(pa)
                
                for da in p['discarded_agents']:
                    q['discarded_agents'].append(da)
                
                ai = constraint['agent']
                path = a_star(self.my_map,self.starts[ai], self.goals[ai],self.heuristics[ai],ai,q['constraints'])
                
                if path is not None:
                    q['paths'][ai]= path
                    # task 4
                    continue_flag = False
                    if constraint['positive']:
                        vol = paths_violate_constraint(constraint,q['paths'])
                        for v in vol:
                            path_v = a_star(self.my_map,self.starts[v], self.goals[v],self.heuristics[v],v,q['constraints'])
                            if path_v  is None:
                                continue_flag = True
                            else:
                                q['paths'][v] = path_v
                        if continue_flag:
                            continue
                    
                    q['collisions'] = detect_collisions(q['paths'])
                    q['cost'] = get_sum_of_cost(q['paths'])
                    # CHECK BYPASS HERE.......
                    #     if q['cost'] == p['cost'] \
                    #         and (len(q['collisions']) < len(p['collisions'])):

                    # if bypass is found, push only the current child and exit loop
                    cardinalities_index = p['collisions'].index(chosen_collision)
                    current_cardinality = collision_cardinalities[cardinalities_index][0]

                    # assert that bypass is not possible if cardinal
                    if current_cardinality == 'cardinal':
                        assert bypass_found(p['cost'], q['cost'], len(p['collisions']), len(q['collisions'])) == False

                    if current_cardinality != 'cardinal' \
                            and bypass_found(p['cost'], q['cost'], len(p['collisions']), len(q['collisions'])):
                        print('bypass found')
                        self.push_node(q)
                        
                        bypass_successful = True
                        break # break out of constraint loop
                    child_nodes.append(copy.deepcopy(q))

            # if bypass has not been found
            if not bypass_successful:

                # MA-CBS
                print('asdfasdfasdfasdf                              ',should_merge(collision,p))
                if should_merge(collision,p):
                    q_with_new_agent = merge_agents(self, collision, p)
                    
                    # Merge & restart
                    # if merge_restart():
                        # restart_search()
                    self.push_node(q_with_new_agent)    
                    continue

                
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
