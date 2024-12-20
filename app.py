from flask import Flask, render_template, request, jsonify
import numpy as np

app = Flask(__name__)

class RRTPlanner:
    def __init__(self, start, goal, obstacles, step_size=0.05, max_iterations=2000):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = obstacles
        self.step_size = step_size
        self.max_iterations = max_iterations
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
        
    def plan(self):
        for _ in range(self.max_iterations):
            # Random sampling
            if np.random.random() < 0.1:  # 10% chance to sample goal
                random_point = self.goal
            else:
                random_point = np.array([np.random.uniform(0, 1), np.random.uniform(0, 1)])
            
            # Find nearest vertex
            distances = [np.linalg.norm(random_point - vertex) for vertex in self.vertices]
            nearest_idx = np.argmin(distances)
            nearest_vertex = np.array(self.vertices[nearest_idx])
            
            # Extend towards random point
            direction = random_point - nearest_vertex
            norm = np.linalg.norm(direction)
            if norm > 0:
                direction = direction / norm
                new_vertex = nearest_vertex + direction * self.step_size
                
                # Check if new vertex causes collision
                if not self.check_collision(nearest_vertex, new_vertex):
                    # Add vertex and edge
                    self.vertices.append(new_vertex)
                    self.edges.append((tuple(nearest_vertex), tuple(new_vertex)))
                    self.parents[tuple(new_vertex)] = tuple(nearest_vertex)
                    
                    # Check if we can connect to goal
                    if np.linalg.norm(new_vertex - self.goal) < self.step_size:
                        if not self.check_collision(new_vertex, self.goal):
                            self.vertices.append(self.goal)
                            self.edges.append((tuple(new_vertex), tuple(self.goal)))
                            self.parents[tuple(self.goal)] = tuple(new_vertex)
                            return self._extract_path()
        
        return None

    def _extract_path(self):
        path = []
        current = tuple(self.goal)
        while current is not None:
            path.append(current)
            current = self.parents[current]
        return list(reversed(path))

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/plan', methods=['POST'])
def plan_path():
    data = request.get_json()
    start = data['start']
    goal = data['goal']
    obstacles = data['obstacles']  # List of [x, y, width, height]
    
    planner = RRTPlanner(start, goal, obstacles)
    path = planner.plan()
    
    return jsonify({
        'path': path if path else [],
        'vertices': [list(v) for v in planner.vertices],
        'edges': [[list(e[0]), list(e[1])] for e in planner.edges]
    })

if __name__ == '__main__':
    app.run(debug=True, port=5001)
