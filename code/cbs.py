import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
import copy

def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    t_range = max(len(path1),len(path2))
    for t in range(t_range):
        loc_c1 =get_location(path1,t)
        loc_c2 = get_location(path2,t)
        loc1 = get_location(path1,t+1)
        loc2 = get_location(path2,t+1)
        if loc1 == loc2:
            return [loc1],t
        if[loc_c1,loc1] ==[loc2,loc_c2]:
            return [loc2,loc_c2],t
        
       
    return None
    # pass


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions =[]
    for i in range(len(paths)-1):
        for j in range(i+1,len(paths)):
            if detect_collision(paths[i],paths[j]) !=None:
                position,t = detect_collision(paths[i],paths[j])
                collisions.append({'a1':i,
                                'a2':j,
                                'loc':position,
                                'timestep':t+1})
    return collisions

    # pass


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
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


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
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
        if agent ==0:
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
    # pass



def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
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

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            # print(standard_splitting(collision))
            print(disjoint_splitting(collision),'\n\n')


        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        
        def detect_cardinal(self, collision, p):
            cardinality = 'non-cardinal'

            q = copy.deepcopy(p)
            # new_constraints = standard_splitting(collision)

            new_constraints = disjoint_splitting(collision)

            # for c in p['constraints']:
            #     if c not in new_constraints:
            #         new_constraints.append(c)

            alt_constraint1 = [new_constraints[0]]
            alt_constraint2 = [new_constraints[1]]

            for c in p['constraints']:
                if c not in alt_constraint1:
                    alt_constraint1.append(c)
                if c not in alt_constraint2:
                    alt_constraint2.append(c)   

            a1 = collision['a1'] #agent a1
         
            alt_path1 = a_star(self.my_map,self.starts[a1], self.goals[a1],self.heuristics[a1],a1,alt_constraint1)
            q['paths'][a1] = alt_path1
            
            if new_constraints[0]['positive']:
                vol = paths_violate_constraint(new_constraints[0],q['paths'])
                for v in vol:
                    path_v = a_star(self.my_map,self.starts[v], self.goals[v],self.heuristics[v],v,alt_constraint1)
                    if path_v  is None:
                        alt_path1 = None
                        break
                    else:
                        q['paths'][v] = path_v
            q['cost'] = get_sum_of_cost(q['paths'])


            if not alt_path1 or q['cost'] > p['cost']:
                cardinality = 'semi-cardinal'
            # if not alt_path1 or len(alt_path1)>len(p['paths'][a1]):
            #     cardinality = 'semi-cardinal'    
                print('alt_path1 takes longer or is empty. at least semi-cardinal.')
                
            a2 = collision['a2'] #agent a2
            alt_path2 = a_star(self.my_map,self.starts[a2], self.goals[a2],self.heuristics[a2],a2,alt_constraint2)
            if not alt_path2 or len(alt_path2) > len(p['paths'][a2]):
                if cardinality == 'semi-cardinal':
                    cardinality = 'cardinal'
                    
                    print('identified cardinal conflict')
  
                else:
                    cardinality = 'semi-cardinal'
                    
                    print('alt_path2 takes longer or is empty. semi-cardinal.')   
                
            return cardinality   



        while len(self.open_list) > 0:
            p = self.pop_node()
            if p['collisions'] == []:
                # self.print_results(p)
                return p['paths']

            chosen_collision = None
            
            # select a cardinal conflict;
            # if none, select a semi-cardinal conflict
            # if none, select a random conflict
            for collision in p['collisions']:
                if detect_cardinal(self, collision, p) == 'cardinal':
                    chosen_collision = collision
                    break
            if not chosen_collision:
                for collision in p['collisions']:
                    if detect_cardinal(self, collision, p) == 'semi-cardinal':
                        chosen_collision = collision
                        break
                if not chosen_collision:
                    chosen_collision = p['collisions'].pop(0)
            
            # chosen_collision = p['collisions'].pop(0)

            # constraints = standard_splitting(chosen_collision)
            constraints = disjoint_splitting(chosen_collision)

            for constraint in constraints:
                q = {'cost':0,
                    'constraints': [constraint],
                    'paths':[],
                    'collisions':[]
                }
                for c in p['constraints']:
                    if c not in q['constraints']:
                        q['constraints'].append(c)
                for pa in p['paths']:
                    q['paths'].append(pa)
                
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
                    self.push_node(q)     
        return None


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))