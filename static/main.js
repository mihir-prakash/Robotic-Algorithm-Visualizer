const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const planButton = document.getElementById('planButton');
const clearButton = document.getElementById('clearButton');
const statusText = document.getElementById('status');

let start = null;
let goal = null;
let obstacles = [];
let isSettingStart = true;
let isDrawingObstacle = false;
let currentObstacle = null;

// Constants for visualization
const POINT_RADIUS = 5;
const START_COLOR = '#4CAF50';
const GOAL_COLOR = '#f44336';
const TREE_COLOR = '#2196F3';
const PATH_COLOR = '#FFC107';
const OBSTACLE_COLOR = '#424242';
const PATH_WIDTH = 3;
const TREE_WIDTH = 1;

function drawPoint(point, color) {
    ctx.beginPath();
    ctx.arc(point[0] * canvas.width, point[1] * canvas.height, POINT_RADIUS, 0, 2 * Math.PI);
    ctx.fillStyle = color;
    ctx.fill();
}

function drawLine(from, to, color, width) {
    ctx.beginPath();
    ctx.moveTo(from[0] * canvas.width, from[1] * canvas.height);
    ctx.lineTo(to[0] * canvas.width, to[1] * canvas.height);
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.stroke();
}

function drawObstacle(obstacle) {
    const [x, y, width, height] = obstacle;
    ctx.fillStyle = OBSTACLE_COLOR;
    ctx.fillRect(
        x * canvas.width,
        y * canvas.height,
        width * canvas.width,
        height * canvas.height
    );
}

function clearCanvas() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    // Redraw all obstacles
    obstacles.forEach(drawObstacle);
    if (start) drawPoint(start, START_COLOR);
    if (goal) drawPoint(goal, GOAL_COLOR);
}

function resetPlanner() {
    start = null;
    goal = null;
    obstacles = [];
    isSettingStart = true;
    isDrawingObstacle = false;
    currentObstacle = null;
    planButton.disabled = true;
    clearCanvas();
    statusText.textContent = 'Right click and drag to draw obstacles. Left click to set start point (green), then goal point (red)';
    updateStats(null);
}

async function planPath() {
    try {
        statusText.textContent = 'Planning path...';
        planButton.disabled = true;
        updateStats(null);  // Reset stats while planning

        const response = await fetch('/plan', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ start, goal, obstacles })
        });

        const data = await response.json();
        
        clearCanvas();  // Clear and redraw obstacles
        
        // Draw the tree
        data.edges.forEach(edge => {
            drawLine(edge[0], edge[1], TREE_COLOR, TREE_WIDTH);
        });

        // Draw the path
        if (data.path.length > 1) {
            for (let i = 0; i < data.path.length - 1; i++) {
                drawLine(data.path[i], data.path[i + 1], PATH_COLOR, PATH_WIDTH);
            }
            statusText.textContent = 'Path found!';
        } else {
            statusText.textContent = 'No path found.';
        }

        // Redraw start and goal points on top
        drawPoint(start, START_COLOR);
        drawPoint(goal, GOAL_COLOR);
        
        updateStats(data.stats);
        planButton.disabled = false;
    } catch (error) {
        console.error('Error:', error);
        statusText.textContent = 'Error occurred while planning path.';
        updateStats(null);
        planButton.disabled = false;
    }
}

function updateStats(stats) {
    if (!stats) {
        document.getElementById('planStatus').textContent = 'Waiting to plan...';
        document.getElementById('planTime').textContent = '-';
        document.getElementById('iterations').textContent = '-';
        document.getElementById('pathLength').textContent = '-';
        document.getElementById('verticesExplored').textContent = '-';
        document.getElementById('pathNodes').textContent = '-';
        return;
    }
    
    // Update status
    const statusElem = document.getElementById('planStatus');
    if (stats.success) {
        statusElem.textContent = 'Path Found!';
        statusElem.className = 'stat-value success';
    } else {
        statusElem.textContent = 'No Path Found';
        statusElem.className = 'stat-value failure';
    }
    
    // Update other statistics
    document.getElementById('planTime').textContent = `${stats.planning_time.toFixed(3)} seconds`;
    document.getElementById('iterations').textContent = stats.iterations.toLocaleString();
    document.getElementById('pathLength').textContent = stats.path_length.toFixed(3);
    document.getElementById('verticesExplored').textContent = stats.vertices_explored.toLocaleString();
    document.getElementById('pathNodes').textContent = stats.path_nodes.toLocaleString();
}

canvas.addEventListener('click', (event) => {
    if (event.button === 0) {  // Left click
        const rect = canvas.getBoundingClientRect();
        const x = (event.clientX - rect.left) / canvas.width;
        const y = (event.clientY - rect.top) / canvas.height;
        
        if (isSettingStart) {
            start = [x, y];
            drawPoint(start, START_COLOR);
            isSettingStart = false;
            statusText.textContent = 'Click to set goal point (red)';
        } else if (!goal) {
            goal = [x, y];
            drawPoint(goal, GOAL_COLOR);
            planButton.disabled = false;
            statusText.textContent = 'Click "Plan Path" to start planning';
        }
    }
});

canvas.addEventListener('mousedown', (event) => {
    if (event.button === 2) {  // Right click
        event.preventDefault();
        const rect = canvas.getBoundingClientRect();
        const x = (event.clientX - rect.left) / canvas.width;
        const y = (event.clientY - rect.top) / canvas.height;
        
        isDrawingObstacle = true;
        currentObstacle = [x, y, 0, 0];
    }
});

canvas.addEventListener('mousemove', (event) => {
    if (isDrawingObstacle && currentObstacle) {
        const rect = canvas.getBoundingClientRect();
        const x = (event.clientX - rect.left) / canvas.width;
        const y = (event.clientY - rect.top) / canvas.height;
        
        currentObstacle[2] = x - currentObstacle[0];  // width
        currentObstacle[3] = y - currentObstacle[1];  // height
        
        clearCanvas();
        // Draw the current obstacle
        drawObstacle(currentObstacle);
    }
});

canvas.addEventListener('mouseup', (event) => {
    if (event.button === 2 && isDrawingObstacle && currentObstacle) {  // Right click release
        isDrawingObstacle = false;
        // Normalize negative dimensions
        if (currentObstacle[2] < 0) {
            currentObstacle[0] += currentObstacle[2];
            currentObstacle[2] = Math.abs(currentObstacle[2]);
        }
        if (currentObstacle[3] < 0) {
            currentObstacle[1] += currentObstacle[3];
            currentObstacle[3] = Math.abs(currentObstacle[3]);
        }
        obstacles.push([...currentObstacle]);
        currentObstacle = null;
        clearCanvas();
    }
});

// Prevent context menu on right click
canvas.addEventListener('contextmenu', (event) => {
    event.preventDefault();
});

clearButton.addEventListener('click', resetPlanner);
planButton.addEventListener('click', planPath);

// Initial reset
resetPlanner();
