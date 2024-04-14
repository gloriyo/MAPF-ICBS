import time as timer
import heapq
import random
# from single_agent_planner import compute_heuristics, a_star, get_location
# from multi_agent_planner import ll_solver, get_sum_of_cost, compute_heuristics, get_location

from a_star_class import A_Star, get_sum_of_cost, compute_heuristics, get_location

import copy

import numpy

'''
   ## Reference to class
'''
######
'''
# Developer's cNOTE regarding Python's mutable default arguments:
#       The responsibiliy of preserving mutable values of passed arguments and 
#       preventing retention of local mutable defaults by assigning immuatable default values (i.e. param=None) in parameters
#       is the responsiblity of the function being called upon
#       PEP 505 - None-aware operators: https://www.python.org/dev/peps/pep-0505/#syntax-and-semantics
'''

def generate_child(constraints, paths, agent_collisions, ma_list):

    assert isinstance(ma_list , list)

    collisions = detect_collisions(paths, ma_list)
    cost = get_sum_of_cost(paths)
    child_node = {
        'cost':cost,
        'constraints': copy.deepcopy(constraints),
        'paths': copy.deepcopy(paths), # {0: {'path':[..path...]}, ... , n: {'path':[..path...]} # not sure if other keys are needed
        'ma_collisions': collisions,
        'agent_collisions':copy.deepcopy(agent_collisions), # matrix of collisions in history between pairs of simple agents
        'ma_list': copy.deepcopy(ma_list) # [{a1,a2}, ... ]
    }
    return child_node

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
        loc_c1 = get_location(path1,t)
        loc_c2 = get_location(path2,t)
        loc1 = get_location(path1,t+1)
        loc2 = get_location(path2,t+1)
        # vertex collision
        if loc1 == loc2:
            pos.append(loc1)
            return pos,t
        # edge collision
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

                # find meta-agents of agents in collision 
                assert isinstance(ma_list , list)
                ma_i = get_ma_of_agent(ai, ma_list)
                assert isinstance(ma_list , list)
                ma_j = get_ma_of_agent(aj, ma_list)

                # check if internal collision in the same meta-agent
                if ma_i != ma_j:
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
                            'meta_agent': collision['ma2'],
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

    print(agent, collision)

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
        if agent == 'a1':
            constraints.append({'agent':collision[agent],
                                'meta_agent': collision[meta_agent],
                                'loc':[collision['loc'][0],collision['loc'][1]],
                                'timestep':collision['timestep'],
                                'positive':True
                                })
            constraints.append({'agent':collision[agent],
                                'meta_agent': collision[meta_agent],
                                'loc':[collision['loc'][0],collision['loc'][1]],
                                'timestep':collision['timestep'],
                                'positive':False
                                })
        else:
            constraints.append({'agent':collision[agent],
                                'meta_agent': collision[meta_agent],
                                'loc':[collision['loc'][1],collision['loc'][0]],
                                'timestep':collision['timestep'],
                                'positive':True
                                })
            constraints.append({'agent':collision[agent],
                                'meta_agent': collision[meta_agent],
                                'loc':[collision['loc'][1],collision['loc'][0]],
                                'timestep':collision['timestep'],
                                'positive':False
                                })
    return constraints

# get the meta-agent an agent is a part of
# do NOT use for constraints, use key 'meta-agent' in constraint
def get_ma_of_agent(agent, ma_list):

    assert isinstance(ma_list , list)
    for ma in ma_list:
        # print(ma, ma_list)
        if agent in ma:
            # print(agent, ma)
            return ma
    raise BaseException('No meta-agent found for agent')
                        

# find meta-agents of the agents that violates constraint
def meta_agents_violate_constraint(constraint, paths, ma_list, violating_ma=None):
    assert constraint['positive'] is True
    if violating_ma is None:
        violating_ma = []

    for i in range(len(paths)):
        ma_i = get_ma_of_agent(i, ma_list)

        if ma_i == constraint['meta_agent'] or ma_i in violating_ma:
            continue


        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                # if ma_i not in violating_ma:
                    violating_ma.append(ma_i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                # if ma_i not in violating_ma:
                violating_ma.append(ma_i)

    return violating_ma


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

    # print('combining constraints:')
    # print('const1: ', constraints)
    # print('const2: ', updated_constraints)

    for c in constraints:
        if c not in updated_constraints:
            updated_constraints.append(c)

    assert len(updated_constraints) <= len(constraints) + len(new_constraints)
    return updated_constraints


def bypass_found(curr_cost, new_cost, curr_collisions_num, new_collisions_num):
    if curr_cost == new_cost \
        and (new_collisions_num < curr_collisions_num):
        return True
    return False

def should_merge(collision, p, N=0):
    a1 = collision['a1']
    a2 = collision['a2']

    if a1 > a2:
        a1, a2 = a2, a1
    assert a1 < a2
    p['agent_collisions'][a1][a2] += 1

    if p['agent_collisions'][a1][a2] > N:
        return True

    ma1 = collision['ma1']
    ma2 = collision['ma2']
    
    # check it is same meta-agent
    assert ma1 != ma2
    assert not (a2 in ma1 or a1 in ma2)

    return False


class ICBS_Solver(object):
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
        print("> Generate node {} with cost {}".format(self.num_of_generated, node['cost']))
        self.num_of_generated += 1
        

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("> Expand node {} with cost {}".format(id, node['cost']))
        self.num_of_expanded += 1
        return node

    def empty_tree(self):
        self.open_list.clear()

    # algorithm for detecting cardinality
    # as 'non-cardinal' or 'semi-cardinal' or 'cardinal'
    # using standard splitting
    def detect_cardinal_conflict(self, AStar, p, collision):
        cardinality = 'non-cardinal'

        # temporary constraints (standard splitting) for detecting cardinal collision purposes
        temp_constraints = standard_splitting(collision)


        ma1 = collision['ma1'] #agent a1

        # print('Sending ma1 in collision {} to A* '.format(ma1))

        assert temp_constraints[0]['meta_agent'] == ma1
        path1_constraints = combined_constraints(p['constraints'], temp_constraints[0])
        astar_ma1 = AStar(self.my_map,self.starts, self.goals,self.heuristics,list(ma1),path1_constraints)
        alt_paths1 = astar_ma1.find_paths()

        # get current paths of meta-agent
        curr_paths = []
        for a1 in ma1:

            not_nested_list = p['paths'][a1]
            assert any(isinstance(i, list) for i in not_nested_list) == False


            curr_paths.append(p['paths'][a1])

        # print(curr_paths)
        # print(alt_paths1)

        # get costs for the meta agent
        curr_cost = get_sum_of_cost(curr_paths)
        
        alt_cost = 0 # write inline if later
        if alt_paths1:
            alt_cost = get_sum_of_cost(alt_paths1)

        # print('\t oldcost:{} newcost:{}'.format(curr_cost, alt_cost))

        if not alt_paths1 or alt_cost > curr_cost:
            cardinality = 'semi-cardinal'
            
            print('alt_path1 takes longer or is empty. at least semi-cardinal.')
            
            
        ma2 = collision['ma2'] #agent a2

        # print('Sending ma2 in collision {} to A* '.format(ma2))

        assert temp_constraints[1]['meta_agent'] == ma2
        path2_constraints = combined_constraints(p['constraints'], temp_constraints[1])
        astar_ma2 = AStar(self.my_map,self.starts, self.goals,self.heuristics,list(ma2),path2_constraints)
        alt_paths2 = astar_ma2.find_paths()

        # if not alt_path2 or bigger:
        curr_paths = []
        for a2 in ma2:    
            not_nested_list = p['paths'][a2]
            assert any(isinstance(i, list) for i in not_nested_list) == False

            curr_paths.append(p['paths'][a2])
            
        # print(curr_paths)
        # print(alt_paths2)

        # get costs for the meta agent
        curr_cost = get_sum_of_cost(curr_paths)
        
        alt_cost = 0 # write inline if later
        if alt_paths2:
            alt_cost = get_sum_of_cost(alt_paths2)

        # print('\t oldcost:{} newcost:{}'.format(curr_cost, alt_cost))

        if not alt_paths2 or alt_cost > curr_cost:
            # cardinality = 'semi-cardinal'
            if cardinality == 'semi-cardinal':
                cardinality = 'cardinal'
                
                # print('identified cardinal conflict')

            else:
                cardinality = 'semi-cardinal'
                
                # print('alt_path2 takes longer or is empty. semi-cardinal.')   
        # print('cardinality: ', cardinality)
            
        return cardinality        

    # returns new merged agents (the meta-agent), and updated list of ma_list
    def merge_agents(self, collision, ma_list):

        # constraints = standard_splitting(collision)
        
        # collision simple agents and their meta-agent group
        a1 = collision['a1']
        a2 = collision['a2']
        ma1 = collision['ma1']
        ma2 = collision['ma2']

        meta_agent = set.union(ma1, ma2)

        print('new merged meta_agent ', meta_agent)

        assert meta_agent not in ma_list

        ma_list.remove(ma1)
        ma_list.remove(ma2)
        ma_list.append(meta_agent)

        return meta_agent, ma_list


    def find_solution(self, disjoint):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint         - use disjoint splitting or not
        """

        self.start_time = timer.time()
        
        if disjoint:
            splitter = disjoint_splitting
        else:
            splitter = standard_splitting

        AStar = A_Star

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
            astar = AStar(self.my_map, self.starts, self.goals, self.heuristics, [i], root['constraints'])
            path = astar.find_paths()


            if path is None:
                raise BaseException('No solutions')
            root['ma_list'].append({i})
            root['paths'].extend(path)



        root['cost'] = get_sum_of_cost(root['paths'])
        root['ma_collisions'] = detect_collisions(root['paths'], root['ma_list'])
        root['agent_collisions'] = numpy.zeros((self.num_of_agents, self.num_of_agents))
        self.push_node(root)



        # ATTENTION: THE CBS LOOOOOOOOOOOOP ============@#￥#%#@￥@#%##@￥======  STARTS ---#￥%------   HERE  ---- @
        # normal CBS with disjoint and standard splitting
        while len(self.open_list) > 0:
            if self.num_of_generated > 50000:
                print('reached maximum number of nodes. Returning...')
                return None 
            print('\n')  
            p = self.pop_node()
            if p['ma_collisions'] == []:
                self.print_results(p)
                # for pa in p['paths']:
                #     # print('asfasdfasdf       ',pa)
                return p['paths'], self.num_of_generated, self.num_of_expanded # number of nodes generated/expanded for comparing implementations


            print('Node expanded. Collisions: ', p['ma_collisions'])
            for pa in p['paths']:
                print(pa)

            print('\n> Find Collision Type')

            # USING STANDARD SPLITTING
            # select a cardinal conflict;
            # if none, select a semi-cardinal conflict
            # if none, select a random conflict
            chosen_collision = None
            new_constraints = None
            collision_type = None
            for collision in p['ma_collisions']:

                print(collision)

                collision_type = self.detect_cardinal_conflict(AStar, p, collision)
                if collision_type == 'cardinal' and new_constraints is None:    
                    print('Detected cardinal collision. Chose it.')
                    print(collision)

                    chosen_collision = collision
                    # collision_type = 'cardinal'
                    break

            else: # no cardinal collisions found
                for collision in p['ma_collisions']:
                    collision_type = self.detect_cardinal_conflict(AStar, p, collision)
                    if collision_type == 'semi-cardinal':    
                        
                        print('Detected semi-cardinal collision. Chose it.')
                        print(collision)
                        chosen_collision = collision
                        # collision_type = 'semi-cardinal'
                        break

                else: # no semi-cardinal collision found
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


            new_constraints = splitter(chosen_collision)

            print('OLD CONSTS:')
            print(p['constraints'])      

            print('NEW CONSTS:')
            print(new_constraints)
            print('\n')
            # child_nodes = None
            child_nodes = []
            assert child_nodes == []
            bypass_successful = False
            for constraint in new_constraints:
                print(constraint)
                
                updated_constraints = combined_constraints(p['constraints'], constraint)
                q = generate_child(updated_constraints, p['paths'], p['agent_collisions'], p['ma_list'])


                assert isinstance(p['ma_list'] , list)
                assert isinstance(q['ma_list'] , list)

                ma = constraint['meta_agent']

                print('\nSending meta_agent {} of constrained agent {} to A* '.format(ma, constraint['agent']))
                print('\twith constraints ', q['constraints'])

                for a in ma:
                    print (q['paths'][a])

                astar = AStar(self.my_map,self.starts, self.goals,self.heuristics,list(ma),q['constraints'])
                paths = astar.find_paths()

                if paths is not None:
                    
                    for i in range(len(paths)):
                                print (paths[i])
                    for i, agent in enumerate(ma):

                        

                        not_nested_list = paths[i]
                        assert any(isinstance(j, list) for j in not_nested_list) == False


                        q['paths'][agent] = paths[i]

                    if constraint['positive']:
                        # vol = paths_violate_constraint(constraint,q['paths'])
                        violating_ma_list = meta_agents_violate_constraint(constraint, q['paths'], q['ma_list'])
                        no_solution = False
                        for v_ma in violating_ma_list:
                            
                            print('\nSending meta-agent violating constraint {} to A* '.format(v_ma))
                            print('\twith constraints ', q['constraints'])

                            for a in v_ma:
                                print (q['paths'][a])


                            v_ma_list = list(v_ma) # should use same list for all uses
                            astar_v_ma = AStar(self.my_map,self.starts,self.goals,self.heuristics,v_ma_list,q['constraints'])
                            paths_v_ma = astar_v_ma.find_paths()



                            # replace paths of meta-agent with new paths found
                            if paths_v_ma is not None:

                                for i in range(len(v_ma_list)):
                                    print (paths_v_ma[i])

                                for i, agent in enumerate(v_ma_list):

                                    assert paths_v_ma[i] is not None
                                    print(paths_v_ma[i])

                                    not_nested_list = paths_v_ma[i]
                                    assert any(isinstance(j, list) for j in not_nested_list) == False


                                    q['paths'][agent] = paths_v_ma[i]
                            else:
                                print("no solution, moving on to next constraint")   
                                no_solution = True
                                break # move on the next constraint
                                
                        if no_solution:
                            continue # move on to the next constraint

                    q['ma_collisions'] = detect_collisions(q['paths'],q['ma_list'])

                    if chosen_collision in q['ma_collisions']:
                        print(q['paths'])
                        print('\nOH NO!!!!! chosen_collision is still in child :\'(')
                        print(chosen_collision)

                    assert chosen_collision not in q['ma_collisions']

                    q['cost'] = get_sum_of_cost(q['paths'])


                    # assert that bypass is not possible if cardinal
                    if collision_type == 'cardinal':
                        assert bypass_found(p['cost'], q['cost'], len(p['ma_collisions']), len(q['ma_collisions'])) == False

                    # conflict should be resolved due to new constraints; compare costs and total number of collisions
                    if collision_type != 'cardinal' \
                            and bypass_found(p['cost'], q['cost'], len(p['ma_collisions']), len(q['ma_collisions'])):
                        print('> Take Bypass')
                        self.push_node(q)
                        
                        bypass_successful = True
                        break # break out of constraint loop
                    assert not bypass_successful
                    child_nodes.append(copy.deepcopy(q))

            if bypass_successful:
                continue # start of while loop

            assert not bypass_successful

            # MA-CBS
            if should_merge(collision, p, 7):
                print('> Merge meta-agents into a new')
                # returns meta_agent, ma_list
                meta_agent, updated_ma_list = self.merge_agents(collision, p['ma_list'])


                # updated constraints
                updated_constraints = copy.deepcopy(p['constraints'])
                for c in updated_constraints:
                    if c['meta_agent'].issubset(meta_agent):
                        c['meta_agent'] = meta_agent

                print('Sending newly merged meta_agent {} to A* '.format(meta_agent))
                print('\twith constraints ', updated_constraints)

                for a in meta_agent:
                    print (p['paths'][a])


                # Update paths
                ma_astar = AStar(self.my_map,self.starts, self.goals,self.heuristics,list(meta_agent), updated_constraints)
                ma_paths = ma_astar.find_paths()


                # if can be 
                if ma_paths:

                    for i in range(len(meta_agent)):
                        print (ma_paths[i])
                                        
                    updated_paths = copy.deepcopy(p['paths'])

                    for i, agent in enumerate(meta_agent):
                        
                        assert isinstance(i, int)

                        not_nested_list = ma_paths[i]
                        assert any(isinstance(j, list) for j in not_nested_list) == False



                        updated_paths[agent] = ma_paths[i]


                    # for a in meta_agent:
                    #     print (updated_paths[a])

                    # Update collisions, cost
                    updated_node = generate_child(updated_constraints, updated_paths, p['agent_collisions'], updated_ma_list) 


                    # print('agents {}, {} merged into agent {}'.format(collision['a1'], a2, meta_agent))

                    # Merge & restart
                    # restart with only updated node with merged agents
                    self.empty_tree()

                    assert self.open_list == []

                    self.push_node(updated_node)    

                    continue # start of while loop
            else:
                print("do not merge")
                
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


        print("Solution:")
        for i in range(len(node['paths'])):
            print("agent", i, ": ", node['paths'][i])
