## Multi-Agent Path Finding - Improved Conflict Based Search ##

Improved CBS (ICBS) is an extension of the Conflict Based Search algorithm developed for the purpose of finding optimal paths for multiple agents on any given map without collisions. The algorithm has many practical applications, especially in the field of autonomous robotics. 

### Background - CBS ###
Originally developed by a dedicated team of MAPF researchers at the AAAI Association, Conflict Based Search (CBS) is a two-level algorithm that guarentees optimal paths for each agent in a map internally represented as a graph. At the higher-level, CBS generates a constraint tree (CT) and conducts a seach on the tree based on conflicts between agents. At the lower-level, a search is conducted to find an optimal path for an agent with a set of given constraints.

CBS resolves conflicts by generating constraints for the agents involved in the collision. For any specific collision, constraints may be generated for the paths of agents involved, to prevent the collision from occuring. Each node on the CT maintains a set of constraints from any previous collisions found by its predecessors and a set of paths consistent with the current set of constraints. Given the paths, a new conflict can be chosen to be resolved with additional constraints. Therefore, each node generated in the CT aims to resolve a specific conflict found in its current set of paths until an optimal set of collision-free paths is found, concluding our search. 
