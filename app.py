from flask import Flask, render_template, request, jsonify
import numpy as np
import time
import os

app = Flask(__name__)

class RRTPlanner:
    def __init__(self, start, goal, obstacles, step_size=0.05, max_iterations=5000, goal_sample_rate=0.2):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = obstacles
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.goal_sample_rate = goal_sample_rate
        self.vertices = [self.start]
        self.edges = []
        self.parents = {tuple(self.start): None}
        
    def check_collision(self, point1, point2):
        # Check if the line segment between point1 and point2 intersects with any obstacle
        for obstacle in self.obstacles:
            x1, y1, width, height = obstacle
            # Convert obstacle coordinates to rectangle corners
            rect_left, rect_right = x1, x1 + width
            rect_top, rect_bottom = y1, y1 + height
            
            # Line segment parameters
            x3, y3 = point1
            x4, y4 = point2
            
            # Check if either endpoint is inside the rectangle
            if (rect_left <= x3 <= rect_right and rect_top <= y3 <= rect_bottom) or \
               (rect_left <= x4 <= rect_right and rect_top <= y4 <= rect_bottom):
                return True
                
            # Check line segment intersection with rectangle edges
            def ccw(A, B, C):
                return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
            
            def intersect(A, B, C, D):
                return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
            
            # Check intersection with rectangle edges
            rect_corners = [
                (rect_left, rect_top), (rect_right, rect_top),
                (rect_right, rect_bottom), (rect_left, rect_bottom)
            ]
            
            for i in range(4):
                if intersect(
                    point1, point2,
                    rect_corners[i], rect_corners[(i+1)%4]
                ):
                    return True
        
        return False
        
    def steer(self, from_point, to_point):
        direction = to_point - from_point
        distance = np.linalg.norm(direction)
        if distance < self.step_size:
            return to_point
        else:
            normalized_direction = direction / distance
            return from_point + normalized_direction * self.step_size
            
    def nearest_vertex(self, point):
        distances = [np.linalg.norm(point - vertex) for vertex in self.vertices]
        return self.vertices[np.argmin(distances)]
        
    def plan(self):
        start_time = time.time()
        iterations = 0
        path_found = False
        
        for i in range(self.max_iterations):
            iterations = i + 1
            # Random sampling with increased goal bias
            if np.random.random() < self.goal_sample_rate:
                random_point = self.goal
            else:
                random_point = np.array([np.random.uniform(0, 1), np.random.uniform(0, 1)])
            
            # Find nearest vertex
            nearest = self.nearest_vertex(random_point)
            
            # Steer towards random point
            new_vertex = self.steer(nearest, random_point)
            
            # Check if new vertex causes collision
            if not self.check_collision(nearest, new_vertex):
                # Add vertex and edge
                self.vertices.append(new_vertex)
                self.edges.append((tuple(nearest), tuple(new_vertex)))
                self.parents[tuple(new_vertex)] = tuple(nearest)
                
                # Try to connect to goal if close enough
                distance_to_goal = np.linalg.norm(new_vertex - self.goal)
                if distance_to_goal < self.step_size * 2:  # Increased connection radius
                    if not self.check_collision(new_vertex, self.goal):
                        self.vertices.append(self.goal)
                        self.edges.append((tuple(new_vertex), tuple(self.goal)))
                        self.parents[tuple(self.goal)] = tuple(new_vertex)
                        path = self._extract_path()
                        if path:  # Only return if a valid path is found
                            path_found = True
                            smoothed_path = self._smooth_path(path)
                            planning_time = time.time() - start_time
                            return {
                                'path': smoothed_path,
                                'stats': {
                                    'planning_time': planning_time,
                                    'iterations': iterations,
                                    'path_length': self._compute_path_length(smoothed_path),
                                    'vertices_explored': len(self.vertices),
                                    'path_nodes': len(smoothed_path),
                                    'success': True
                                }
                            }
                            
            # Early termination if taking too long
            if i > 0 and i % 1000 == 0:
                path = self._try_connect_to_goal()
                if path:
                    path_found = True
                    smoothed_path = self._smooth_path(path)
                    planning_time = time.time() - start_time
                    return {
                        'path': smoothed_path,
                        'stats': {
                            'planning_time': planning_time,
                            'iterations': iterations,
                            'path_length': self._compute_path_length(smoothed_path),
                            'vertices_explored': len(self.vertices),
                            'path_nodes': len(smoothed_path),
                            'success': True
                        }
                    }
        
        # Final attempt to connect to goal
        path = self._try_connect_to_goal()
        planning_time = time.time() - start_time
        if path:
            path_found = True
            smoothed_path = self._smooth_path(path)
            return {
                'path': smoothed_path,
                'stats': {
                    'planning_time': planning_time,
                    'iterations': iterations,
                    'path_length': self._compute_path_length(smoothed_path),
                    'vertices_explored': len(self.vertices),
                    'path_nodes': len(smoothed_path),
                    'success': True
                }
            }
        
        return {
            'path': None,
            'stats': {
                'planning_time': planning_time,
                'iterations': iterations,
                'path_length': 0,
                'vertices_explored': len(self.vertices),
                'path_nodes': 0,
                'success': False
            }
        }

    def _compute_path_length(self, path):
        """Compute the total length of the path"""
        if not path or len(path) < 2:
            return 0
        
        length = 0
        for i in range(len(path) - 1):
            length += np.linalg.norm(np.array(path[i]) - np.array(path[i + 1]))
        return length

    def _try_connect_to_goal(self):
        """Try to connect the closest vertex to the goal"""
        distances_to_goal = [np.linalg.norm(self.goal - vertex) for vertex in self.vertices]
        closest_idx = np.argmin(distances_to_goal)
        closest_vertex = self.vertices[closest_idx]
        
        if not self.check_collision(closest_vertex, self.goal):
            self.vertices.append(self.goal)
            self.edges.append((tuple(closest_vertex), tuple(self.goal)))
            self.parents[tuple(self.goal)] = tuple(closest_vertex)
            return self._extract_path()
        return None

    def _extract_path(self):
        if tuple(self.goal) not in self.parents:
            return None
            
        path = []
        current = tuple(self.goal)
        while current is not None:
            path.append(current)
            current = self.parents[current]
        return list(reversed(path))
        
    def _smooth_path(self, path):
        """Simple path smoothing by removing redundant waypoints"""
        if len(path) <= 2:
            return path
            
        smoothed = [path[0]]
        current_idx = 0
        
        while current_idx < len(path) - 1:
            # Try to connect to furthest possible point
            for j in range(len(path) - 1, current_idx, -1):
                if not self.check_collision(path[current_idx], path[j]):
                    smoothed.append(path[j])
                    current_idx = j
                    break
            else:
                current_idx += 1
                if current_idx < len(path):
                    smoothed.append(path[current_idx])
                    
        return smoothed

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/plan', methods=['POST'])
def plan_path():
    data = request.get_json()
    start = data['start']
    goal = data['goal']
    obstacles = data['obstacles']
    
    planner = RRTPlanner(start, goal, obstacles)
    result = planner.plan()
    
    return jsonify({
        'path': result['path'] if result['path'] else [],
        'vertices': [list(v) for v in planner.vertices],
        'edges': [[list(e[0]), list(e[1])] for e in planner.edges],
        'stats': result['stats']
    })

if __name__ == '__main__':
    port = int(os.environ.get('PORT', 5001))
    app.run(host='0.0.0.0', port=port, debug=False)
