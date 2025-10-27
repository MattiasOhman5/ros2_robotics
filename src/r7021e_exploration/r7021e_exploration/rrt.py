#!/home/ros2_ws/.venv/bin/python3
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import random
import math
from scipy.spatial import cKDTree
from scipy.ndimage import distance_transform_edt
from skimage.draw import line

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y

        self.parent = None
        self.children = []
        self.cost = 0.0
        self.path_length = 0.0
        self.obstacle_cost = 0.0

    def add_child(self, x, y):
        child_node = Node(x, y)
        child_node.parent = self
        self.children.append(child_node)
        return child_node
    
    def __repr__(self):
        parent_str = f"({self.parent.x:.2f}, {self.parent.y:.2f})" if self.parent else "None"
        return f"Node(x={self.x:.2f}, y={self.y:.2f}, parent={parent_str}, children={len(self.children)})"
        

class RRT():
    def __init__(self, node, occupancy_grid, num_iterations, goal):
        self.root_node = node
        self.nodes = [self.root_node]
        self.og = occupancy_grid
        self.obstacle_weight = 1000.0

        self.num_iterations = num_iterations
        self.tree_size = 1
        self.step_size = 0.1
        self.sample_goal_bias = 0.05
        self.goal = goal
        self.r_goal = 0.35
        self.best_cost = float('inf')
        self.best_node = None
        self.best_path_length = float('inf')

        self.d_safe = 0.1
        self.sigma = 0.03
        
        grid = self.convert2grid(self.og)
        obstacle_mask = (grid != 100).astype(np.uint8)
        dist_cells = distance_transform_edt(obstacle_mask)
        self.distance_map = dist_cells * self.og.info.resolution

        self.node_coords = [(node.x, node.y)]
        self.kd_tree = cKDTree(self.node_coords)

        self.gamma_rrt = 3.0
        self.d = 2

        self.sols_found = 0

    def add_node(self, new_node):
        self.nodes.append(new_node)
        self.node_coords.append((new_node.x, new_node.y))
        self.tree_size += 1

        if self.tree_size % 50 == 0:
            self.kd_tree = cKDTree(self.node_coords)

    # primitive procedures from https://arxiv.org/pdf/1105.1186

    def sample_free(self):
        #if random.random() < self.sample_goal_bias:
        #    return self.goal

        x = self.og.info.origin.position.x + random.uniform(0, self.og.info.width * self.og.info.resolution)
        y = self.og.info.origin.position.y + random.uniform(0, self.og.info.height * self.og.info.resolution)
        x_rand = (x, y)
        return x_rand
    
    def informed_sample(self, x_start, x_goal, c_best):
        # informed RRT* från den här: https://arxiv.org/pdf/1404.2334
        x0, y0 = x_start
        x1, y1 = x_goal

        # den teoretiskt lägsta kostnaden man kan få vore att gå fågelvägen mellan start och mål
        c_min = math.hypot(x1 - x0, y1 - y0)

        # sampla från en ellips som täcker start och mål efter att man hittat en lösning
        # ... best_cost är infinite när man startar koden
        if not math.isfinite(self.best_path_length) or c_best <= c_min:
            return self.sample_free()
        
        # ellipsen har sin mittpunkt exakt mellan starten och målet
        ellipse_center_x = 0.5 * (x0 + x1)
        ellipse_center_y = 0.5 * (y0 + y1)

        # behöver theta för att kunna rotera ellipsen
        theta = math.atan2(y1 - y0, x1 - x0)

        # r1 är det långt strecket i ellipsen, som går i en oroterad ellips går längs x-axeln
        r1 = self.best_path_length / 2.0 # formeln för denna är skriven i artikeln, Algorithm 2: rad 5
        # r2 är det kort strecket i ellipsen (längs y-axeln)
        r2 = math.sqrt(self.best_path_length**2 - c_min**2) / 2.0 # formeln för denna är skriven i artikeln, Algorithm 2: rad 6

        # Algorithm 2: rad 8, SampleUnitBall
        u = random.random()
        v = random.random()
        r = math.sqrt(u) # Man tar roten ur här för uniform distribution, annars klumpar sig punkterna runt origo
        angle = 2.0 * math.pi * v
        x_ball = r * math.cos(angle)
        y_ball = r * math.sin(angle)

        # Algorithm 2: rad 9
        x_ell = r1 * x_ball # man sträcker ut bollen i x-led
        y_ell = r2 * y_ball # man sträcker ut bollen i y-led

        # man applicerar rotation, hade kunnat använda rotationsmatris
        x_rot = x_ell * math.cos(theta) - y_ell * math.sin(theta)
        y_rot = x_ell * math.sin(theta) + y_ell * math.cos(theta)

        # bollen vi samplade var runt origo, man behöver translatera den baserat på mitten av ellipsen
        x_rand = x_rot + ellipse_center_x
        y_rand = y_rot + ellipse_center_y

        return (x_rand, y_rand)
    
    def nearest(self, x_rand):
        # att använda kd-tree ska ha bättre tidskomplexitet för både near och nearest än att loop igenom samtliga noder
        distance, idx = self.kd_tree.query(x_rand)
        return self.nodes[idx]
    
    def near(self, x_new, radius):
        indices = self.kd_tree.query_ball_point(x_new, radius)
        nodes = []
        for i in indices:
            nodes.append(self.nodes[i])
        return nodes
    
    def steer(self, x_nearest, x_rand):
        # kollar vilken riktning vi samplat åt och väljer en punkt, step_size långt ifrån ursprungspunkten
        theta = math.atan2(x_rand[1] - x_nearest.y, x_rand[0] - x_nearest.x)
        x_new = x_nearest.x + self.step_size * math.cos(theta)
        y_new = x_nearest.y + self.step_size * math.sin(theta)
        return (x_new, y_new)
    
    def collision_free(self, x_nearest, x_new):
        x0, y0 = self.world_to_grid(x_nearest)
        x1, y1 = self.world_to_grid(x_new)

        # line-funktionen från skimage ritar en linje mellan 2 celler och returnerar alla celler den passerar igenom
        # finns också nånting som heter Bresenham's algoritm men den var gode komplicerad
        row, col = line(y0, x0, y1, x1)

        # fick index out of bound vi något tillfälle, borde inte ha fått det dock
        for gy, gx in zip(row, col):
            if gx < 0 or gy < 0 or gx >= self.og.info.width or gy >= self.og.info.height:
                return False

            index = gy * self.og.info.width + gx
            value = self.og.data[index]

            # en cell är occupied om det har värdet 100
            if value == 100 or value == -1:
                return False

        return True
    
    # fågelvägen mellan 2 punkter
    def line_cost(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])
    
    # använd en målzon för annars måste man sampla exakta målkoordinaterna för att hitta lösning
    def in_goal_region(self, x):
        return math.hypot(x[0] - self.goal[0], x[1] - self.goal[1]) < self.r_goal # den behövde sättas större in inflate för annars kan man inte nå många frontiers
            
    # helper

    def world_to_grid(self, point):
        x, y = point
        gx = int((x - self.og.info.origin.position.x) / self.og.info.resolution)
        gy = int((y - self.og.info.origin.position.y) / self.og.info.resolution)
        return gx, gy
    
    # man beräknar radien dynamiskt enligt https://arxiv.org/pdf/1105.1186
    def get_radius(self):
        d = self.d
        gamma_rrt = self.gamma_rrt
        n = max(self.tree_size, 2)

        #radius = min(gamma_rrt * (math.log(n) / n) ** (1 / d), self.step_size)
        radius = gamma_rrt * (math.log(n) / n) ** (1 / d)
        return radius
    
    def plot_tree(self):
        xs = []
        ys = []
        gx = self.goal[0]
        gy = self.goal[1]

        for node in self.nodes:
            xs.append(node.x)
            ys.append(node.y)
            
            for child in node.children:
                plt.plot([node.x, child.x], [node.y, child.y], 'b-', linewidth=0.8)


        plt.scatter(xs, ys, s=10, c='red')
        plt.scatter(gx, gy, s=20, c='green', marker='X')


        if self.best_node:
             path, _ = self.extract_best_path()
             px, py = zip(*path)
             plt.plot(px, py, 'y-', linewidth=1.5, label="best path")

    # vi gör run_RRT för att bygga upp trädet men behöver en funktion för att plocka ut den bästa vägen
    def extract_best_path(self):
        if self.best_node is None:
            return [], self.best_cost
        
        path = []
        current = self.best_node
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent

        path.reverse() # flippar ordningen så att listan går från start -> mål istället
        return path, self.best_cost
    
    def get_obstacle_dist(self, x, y):
        gx, gy = self.world_to_grid((x, y))
        return self.distance_map[gy, gx]
    
    def convert2grid(self, occupancy_grid_msg):
        width = occupancy_grid_msg.info.width
        height = occupancy_grid_msg.info.height

        map_grid = np.array(occupancy_grid_msg.data).reshape((height, width))

        return map_grid
    
    # the main algorithm
    def get_clearance_score(self, x, y):
        """Returns higher values for open areas, lower for constricted regions"""
        gx, gy = self.world_to_grid((x, y))
        
        # Sample distances in 8 directions around the point
        angles = [0, 45, 90, 135, 180, 225, 270, 315]
        search_radius = int(0.5 / self.og.info.resolution)  # search 0.5m in each direction
        
        clearances = []
        for angle_deg in angles:
            angle_rad = math.radians(angle_deg)
            for r in range(1, search_radius):
                test_gx = gx + int(r * math.cos(angle_rad))
                test_gy = gy + int(r * math.sin(angle_rad))
                
                if (test_gx < 0 or test_gy < 0 or 
                    test_gx >= self.og.info.width or test_gy >= self.og.info.height):
                    clearances.append(r * self.og.info.resolution)
                    break
                    
                index = test_gy * self.og.info.width + test_gx
                if self.og.data[index] == 100 or self.og.data[index] == -1:
                    clearances.append(r * self.og.info.resolution)
                    break
            else:
                clearances.append(search_radius * self.og.info.resolution)
        
        # Use minimum clearance (narrowest direction) as constriction measure
        min_clearance = min(clearances)
        avg_clearance = sum(clearances) / len(clearances)
        
        # Combine both: narrow passages have low min AND low avg
        return min_clearance * 0.7 + avg_clearance * 0.3


    def run_RRT(self):
        # vi kör en bestämd mängd iterationer och sparar den bästa lösningen
        i = 0
        while i < self.num_iterations:
            i += 1
            # man samplar över hela kartan tills man hittar första lösningen och därefter samplar i en ellips (informed RRT*)
            x_rand = self.informed_sample((self.root_node.x, self.root_node.y), (self.goal[0], self.goal[1]), self.best_path_length)
            x_nearest = self.nearest(x_rand)
            x_new = self.steer(x_nearest, x_rand)

            # -- detta block handlar i stort sett bara om att koppla den nya noden till den parent som ger lägst cost (bäst väg)
            if self.collision_free((x_nearest.x, x_nearest.y), x_new):
                x_near = self.near(x_new, self.get_radius())
                x_new_node = Node(x_new[0], x_new[1])
                
                segment_length = self.line_cost(x_new, (x_nearest.x, x_nearest.y))
                x_new_node.path_length = x_nearest.path_length + segment_length

                obstacle_dist = self.get_obstacle_dist(x_new[0], x_new[1])
                clearance = self.get_clearance_score(x_new[0], x_new[1])
                
                obstacle_cost = self.obstacle_weight * math.exp(-clearance / 0.3)

                #obstacle_cost = self.obstacle_weight / (obstacle_dist + 1e-6)
                x_new_node.obstacle_cost = obstacle_cost
                
                x_new_node.parent = x_nearest
                x_new_node.cost = x_nearest.cost + segment_length + obstacle_cost
                
                best_parent = x_nearest
                best_cost = x_new_node.cost
                best_length = x_new_node.path_length

                for node in x_near:
                    if self.collision_free((node.x, node.y), x_new):
                        segment_length = self.line_cost((node.x, node.y), x_new)
                        path_length = node.path_length + segment_length
                        cost = node.cost + segment_length + obstacle_cost
                        if cost < best_cost:
                            best_parent = node
                            best_cost = cost
                            best_length = path_length

                x_new_node.parent = best_parent
                x_new_node.cost = best_cost
                x_new_node.path_length = best_length
                best_parent.children.append(x_new_node)
                self.add_node(x_new_node)
            # ----------------------------------------------------------------------------------------------------------------- 

            # det dom kallar rewiring. Man kollar om ifall det går att göra andra noder i närheten billigare genom att sätta den nya noden
            # som dess parent
                for node in x_near:
                    if node == best_parent:
                        continue

                    if self.collision_free((x_new_node.x, x_new_node.y), (node.x, node.y)):
                        segment_length = self.line_cost((x_new_node.x, x_new_node.y), (node.x, node.y))
                        new_cost = x_new_node.cost + segment_length
                        new_length = x_new_node.path_length + segment_length
                        if new_cost < node.cost:
                            if node.parent:
                                node.parent.children.remove(node)
                            node.parent = x_new_node 
                            node.cost = new_cost
                            node.path_length = new_length
                            x_new_node.children.append(node)
            
            # -----------------------------------------------------------------------------------------------------------------

            # vi sparar den bäst väg vi hittar eftersom vi kör den en bestämd mängd iterationer
                if self.in_goal_region(x_new):
                    #self.sols_found += 1
                    if x_new_node.cost < self.best_cost:
                        self.best_cost = x_new_node.cost
                        self.best_node = x_new_node
                        print(f"Found new best node, cost = {self.best_cost:.3f}")

                    if x_new_node.path_length < self.best_path_length:
                        self.best_path_length = x_new_node.path_length
                        print(f"Found new shortest geometric path = {self.best_path_length:.3f}")

                    #if self.sols_found == 5:
                    #    return

        print("finished RRT")          
        return
    