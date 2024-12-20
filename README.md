# RRT (Rapidly-exploring Random Tree) Path Planning Visualizer

An interactive web application for visualizing the RRT path planning algorithm with obstacle avoidance.

## Features
- Interactive canvas for setting start and goal positions
- Draw rectangular obstacles using right-click and drag
- Visualize the RRT exploration tree
- See the final path from start to goal
- Real-time path planning with obstacle avoidance

## Installation

1. Clone the repository:
```bash
git clone https://github.com/mihir-prakash/Robotic-Algorithm-Visualizer.git
cd Robotic-Algorithm-Visualizer
```

2. Install the required packages:
```bash
pip install -r requirements.txt
```

## Usage

1. Run the Flask application:
```bash
python app.py
```

2. Open your web browser and go to `http://localhost:5001`

3. Using the interface:
   - Right-click and drag to draw rectangular obstacles (gray)
   - Left-click to set the start point (green)
   - Left-click again to set the goal point (red)
   - Click "Plan Path" to run the RRT algorithm
   - Click "Clear" to reset the canvas

## How It Works

The application implements the RRT (Rapidly-exploring Random Tree) algorithm for path planning:
1. Randomly samples points in the space
2. Grows a tree from the start position
3. Checks for collisions with obstacles
4. Connects to the goal when possible
5. Returns the path from start to goal

## Technologies Used
- Backend: Python, Flask
- Frontend: HTML5 Canvas, JavaScript
- Algorithm: RRT with collision detection
