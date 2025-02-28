# ROS Web Dashboard Documentation

A comprehensive guide for building web-based dashboards for ROS robots using roslibjs and ROSBridge.

## Table of Contents
- [Overview](#overview)
- [Setup and Installation](#setup-and-installation)
- [Core Functionalities](#core-functionalities)
  - [Connecting to ROSBridge](#connecting-to-rosbridge)
  - [Topic Subscription](#topic-subscription)
  - [Topic Publishing](#topic-publishing)
- [UI Components](#ui-components)
  - [Joystick Control](#joystick-control)
  - [Camera Feed](#camera-feed)
  - [2D Map Integration](#2d-map-integration)
  - [3D Map Integration](#3d-map-integration)
  - [Data Display](#data-display)
- [Complete Implementation](#complete-implementation)
- [Troubleshooting](#troubleshooting)
- [Additional Resources](#additional-resources)

## Overview

This documentation provides reusable code templates for creating a web-based dashboard for ROS-powered robots. Using roslibjs and ROSBridge, you can build interactive interfaces that enable:

- Real-time robot control
- Sensor data visualization
- Camera feed display
- 2D and 3D map rendering
- Robot state monitoring

## Setup and Installation

### Prerequisites
- A running ROS environment
- ROSBridge server installed and configured
- Basic understanding of HTML, CSS, and JavaScript

### Basic HTML Structure

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ROS Web Dashboard</title>
    <!-- Required libraries -->
    <script src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.9.0/nipplejs.min.js"></script>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f0f0f0;
        }
        .dashboard {
            display: flex;
            flex-wrap: wrap;
            gap: 20px;
        }
        #joystick {
            width: 200px;
            height: 200px;
            background-color: #ddd;
            border-radius: 50%;
        }
        #map-container {
            width: 600px;
            height: 400px;
            background-color: #ccc;
        }
        #camera-feed {
            width: 640px;
            height: 480px;
            background-color: #000;
        }
    </style>
</head>
<body>
    <div class="dashboard">
        <!-- Dashboard components will be added here -->
    </div>
    <script>
        // JavaScript code will be added here
    </script>
</body>
</html>
```

## Core Functionalities

### Connecting to ROSBridge

The first step is establishing a connection to the ROSBridge server, which acts as a bridge between your web application and the ROS ecosystem.

```javascript
let ros;

function connectToROS(url = 'ws://localhost:9090') {
    ros = new ROSLIB.Ros({
        url: url
    });

    ros.on('connection', () => {
        console.log('Connected to ROSBridge server.');
        // Initialize components after successful connection
        initializeDashboard();
    });

    ros.on('error', (error) => {
        console.error('Error connecting to ROSBridge server:', error);
    });

    ros.on('close', () => {
        console.log('Connection to ROSBridge server closed.');
    });
}

function initializeDashboard() {
    // Initialize all dashboard components
    setupJoystick();
    setupCameraFeed();
    setup2DMap();
    setup3DMap();
    setupDataDisplay();
}

// Connect when page loads
window.onload = () => {
    connectToROS();
};
```

### Topic Subscription

Subscribe to ROS topics to receive real-time data from your robot.

```javascript
function subscribeToTopic(topicName, messageType, callback) {
    const topic = new ROSLIB.Topic({
        ros: ros,
        name: topicName,
        messageType: messageType
    });

    topic.subscribe(callback);
    console.log(`Subscribed to topic: ${topicName}`);
    
    return topic; // Return for future unsubscribe if needed
}

// Example: Subscribe to odometry data
function setupOdometryListener() {
    subscribeToTopic('/odom', 'nav_msgs/Odometry', (message) => {
        // Update position display
        document.getElementById('position-x').textContent = message.pose.pose.position.x.toFixed(2);
        document.getElementById('position-y').textContent = message.pose.pose.position.y.toFixed(2);
        document.getElementById('position-z').textContent = message.pose.pose.position.z.toFixed(2);
        
        // Update orientation display (convert quaternion to degrees if needed)
        // Additional processing as required
    });
}
```

### Topic Publishing

Send commands to your robot by publishing messages to ROS topics.

```javascript
function publishToTopic(topicName, messageType, messageData) {
    const topic = new ROSLIB.Topic({
        ros: ros,
        name: topicName,
        messageType: messageType
    });

    const message = new ROSLIB.Message(messageData);
    topic.publish(message);
    console.log(`Published to ${topicName}:`, messageData);
}

// Example: Send movement command
function sendMoveCommand(linearX, angularZ) {
    const twistMessage = {
        linear: { x: linearX, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angularZ }
    };
    
    publishToTopic('/cmd_vel', 'geometry_msgs/Twist', twistMessage);
}
```

## UI Components

### Joystick Control

Implement a virtual joystick for intuitive robot control.

```html
<div id="joystick"></div>
```

```javascript
function setupJoystick() {
    const joystick = nipplejs.create({
        zone: document.getElementById('joystick'),
        mode: 'static',
        position: { left: '50%', top: '50%' },
        color: 'blue',
        size: 150
    });

    // Maximum speeds
    const maxLinearSpeed = 1.0; // m/s
    const maxAngularSpeed = 1.0; // rad/s

    joystick.on('move', (evt, data) => {
        // Calculate linear and angular velocities based on joystick position
        const distance = Math.min(data.distance, 75); // Cap at 75px
        const angle = data.angle.radian;
        
        const linearX = Math.sin(angle) * maxLinearSpeed * (distance / 75);
        const angularZ = -Math.cos(angle) * maxAngularSpeed * (distance / 75);

        sendMoveCommand(linearX, angularZ);
    });

    joystick.on('end', () => {
        // Stop the robot when joystick is released
        sendMoveCommand(0, 0);
    });
    
    console.log('Joystick control initialized');
}
```

### Camera Feed

Display real-time camera feed from your robot.

```html
<div class="component-container">
    <h3>Camera Feed</h3>
    <img id="camera-feed" alt="Camera Feed">
</div>
```

```javascript
function setupCameraFeed(topicName = '/camera/image_raw/compressed') {
    const imageTopic = new ROSLIB.Topic({
        ros: ros,
        name: topicName,
        messageType: 'sensor_msgs/CompressedImage'
    });

    imageTopic.subscribe((message) => {
        const imageElement = document.getElementById('camera-feed');
        imageElement.src = "data:image/jpeg;base64," + message.data;
    });
    
    console.log('Camera feed initialized');
}
```

### 2D Map Integration

Display a 2D map of the environment with robot position.

```html
<div class="component-container">
    <h3>2D Map</h3>
    <div id="map-2d"></div>
</div>
```

```javascript
function setup2DMap() {
    // For a complete 2D map implementation, consider using libraries like:
    // - ROS2D (https://github.com/RobotWebTools/ros2djs)
    // - Leaflet with custom ROS integration
    
    const mapTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/map',
        messageType: 'nav_msgs/OccupancyGrid'
    });

    mapTopic.subscribe((message) => {
        // Process and render map data
        // This is a simplified example - actual implementation will depend on your visualization library
        console.log('Map data received:', message);
        
        // Example using ROS2D (if included in your project)
        // const viewer = new ROS2D.Viewer({
        //     divID: 'map-2d',
        //     width: 600,
        //     height: 400
        // });
        // const map = new ROS2D.OccupancyGridClient({
        //     ros: ros,
        //     rootObject: viewer.scene,
        //     continuous: true
        // });
    });
    
    console.log('2D map initialized');
}
```

### 3D Map Integration

Visualize 3D sensor data from your robot.

```html
<div class="component-container">
    <h3>3D Map</h3>
    <div id="map-3d"></div>
</div>
```

```javascript
function setup3DMap() {
    // For a complete 3D map implementation, consider using libraries like:
    // - Three.js for 3D rendering
    // - ros3djs (https://github.com/RobotWebTools/ros3djs)
    
    const pointCloudTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/point_cloud',
        messageType: 'sensor_msgs/PointCloud2'
    });

    pointCloudTopic.subscribe((message) => {
        // Process and render 3D point cloud data
        // This is a simplified example - actual implementation will depend on your visualization library
        console.log('Point cloud data received');
        
        // Example using ros3djs (if included in your project)
        // const viewer = new ROS3D.Viewer({
        //     divID: 'map-3d',
        //     width: 600,
        //     height: 400,
        //     antialias: true
        // });
        // const pointCloud = new ROS3D.PointCloud2({
        //     ros: ros,
        //     topic: '/point_cloud',
        //     material: { size: 0.01, color: 0x00ff00 }
        // });
        // viewer.addObject(pointCloud);
    });
    
    console.log('3D map initialized');
}
```

### Data Display

Create UI elements to display robot sensor data and state information.

```html
<div class="component-container">
    <h3>Robot Data</h3>
    <div id="data-display">
        <div class="data-group">
            <h4>Position</h4>
            <p>X: <span id="position-x">0.00</span> m</p>
            <p>Y: <span id="position-y">0.00</span> m</p>
            <p>Z: <span id="position-z">0.00</span> m</p>
        </div>
        <div class="data-group">
            <h4>Battery</h4>
            <p>Level: <span id="battery-level">0</span>%</p>
            <p>Status: <span id="battery-status">Unknown</span></p>
        </div>
        <div class="data-group">
            <h4>Speed</h4>
            <p>Linear: <span id="linear-speed">0.00</span> m/s</p>
            <p>Angular: <span id="angular-speed">0.00</span> rad/s</p>
        </div>
    </div>
</div>
```

```javascript
function setupDataDisplay() {
    // Subscribe to battery state
    subscribeToTopic('/battery_state', 'sensor_msgs/BatteryState', (message) => {
        document.getElementById('battery-level').textContent = (message.percentage * 100).toFixed(0);
        document.getElementById('battery-status').textContent = determineBatteryStatus(message);
    });
    
    // Subscribe to velocity
    subscribeToTopic('/cmd_vel', 'geometry_msgs/Twist', (message) => {
        document.getElementById('linear-speed').textContent = message.linear.x.toFixed(2);
        document.getElementById('angular-speed').textContent = message.angular.z.toFixed(2);
    });
    
    console.log('Data displays initialized');
}

function determineBatteryStatus(batteryMessage) {
    // Logic to determine battery status based on voltage, current, etc.
    if (batteryMessage.percentage < 0.2) return 'Low';
    if (batteryMessage.power_supply_status === 1) return 'Charging';
    return 'Normal';
}
```

## Complete Implementation

Here's a complete implementation combining all the components:

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ROS Web Dashboard</title>
    <script src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.9.0/nipplejs.min.js"></script>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f5f5f5;
            color: #333;
        }
        .dashboard {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            max-width: 1200px;
            margin: 0 auto;
        }
        .component-container {
            background-color: white;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
            padding: 15px;
        }
        h3 {
            margin-top: 0;
            border-bottom: 1px solid #eee;
            padding-bottom: 10px;
        }
        #joystick {
            width: 200px;
            height: 200px;
            background-color: #eee;
            border-radius: 50%;
            margin: 0 auto;
        }
        #camera-feed {
            width: 100%;
            max-width: 640px;
            height: auto;
            background-color: #000;
            border-radius: 4px;
            display: block;
            margin: 0 auto;
        }
        #map-2d, #map-3d {
            width: 100%;
            height: 300px;
            background-color: #eee;
            border-radius: 4px;
        }
        .data-group {
            margin-bottom: 15px;
        }
        .data-group h4 {
            margin: 0 0 5px 0;
            color: #555;
        }
        .data-group p {
            margin: 5px 0;
        }
        #connection-status {
            position: fixed;
            top: 10px;
            right: 10px;
            padding: 5px 10px;
            border-radius: 4px;
            font-weight: bold;
        }
        .connected {
            background-color: #d4edda;
            color: #155724;
        }
        .disconnected {
            background-color: #f8d7da;
            color: #721c24;
        }
        #connection-form {
            margin-bottom: 20px;
            text-align: center;
        }
        button {
            background-color: #007bff;
            color: white;
            border: none;
            padding: 8px 15px;
            border-radius: 4px;
            cursor: pointer;
        }
        button:hover {
            background-color: #0069d9;
        }
        input[type="text"] {
            padding: 8px;
            width: 250px;
            border: 1px solid #ccc;
            border-radius: 4px;
        }
    </style>
</head>
<body>
    <header>
        <h1>ROS Web Dashboard</h1>
        <div id="connection-form">
            <input type="text" id="rosbridge-address" value="ws://localhost:9090" placeholder="ROSBridge WebSocket URL">
            <button onclick="connectToROSBridge()">Connect</button>
        </div>
        <div id="connection-status" class="disconnected">Disconnected</div>
    </header>
    
    <div class="dashboard">
        <div class="component-container">
            <h3>Robot Control</h3>
            <div id="joystick"></div>
        </div>
        
        <div class="component-container">
            <h3>Camera Feed</h3>
            <img id="camera-feed" alt="Camera Feed">
        </div>
        
        <div class="component-container">
            <h3>2D Map</h3>
            <div id="map-2d"></div>
        </div>
        
        <div class="component-container">
            <h3>3D Visualization</h3>
            <div id="map-3d"></div>
        </div>
        
        <div class="component-container">
            <h3>Robot Data</h3>
            <div id="data-display">
                <div class="data-group">
                    <h4>Position</h4>
                    <p>X: <span id="position-x">0.00</span> m</p>
                    <p>Y: <span id="position-y">0.00</span> m</p>
                    <p>Z: <span id="position-z">0.00</span> m</p>
                </div>
                <div class="data-group">
                    <h4>Battery</h4>
                    <p>Level: <span id="battery-level">0</span>%</p>
                    <p>Status: <span id="battery-status">Unknown</span></p>
                </div>
                <div class="data-group">
                    <h4>Speed</h4>
                    <p>Linear: <span id="linear-speed">0.00</span> m/s</p>
                    <p>Angular: <span id="angular-speed">0.00</span> rad/s</p>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        let ros;
        
        function connectToROSBridge() {
            const rosbridgeAddress = document.getElementById('rosbridge-address').value;
            const statusElement = document.getElementById('connection-status');
            
            // Disconnect if already connected
            if (ros) {
                ros.close();
            }
            
            // Update status
            statusElement.className = 'disconnected';
            statusElement.textContent = 'Connecting...';
            
            // Connect to ROSBridge
            ros = new ROSLIB.Ros({
                url: rosbridgeAddress
            });
            
            ros.on('connection', () => {
                console.log('Connected to ROSBridge.');
                statusElement.className = 'connected';
                statusElement.textContent = 'Connected';
                initializeDashboard();
            });
            
            ros.on('error', (error) => {
                console.error('Error connecting to ROSBridge:', error);
                statusElement.className = 'disconnected';
                statusElement.textContent = 'Connection Error';
            });
            
            ros.on('close', () => {
                console.log('Connection to ROSBridge closed.');
                statusElement.className = 'disconnected';
                statusElement.textContent = 'Disconnected';
            });
        }
        
        function initializeDashboard() {
            setupJoystick();
            setupCameraFeed();
            setup2DMap();
            setup3DMap();
            setupDataDisplay();
        }
        
        function subscribeToTopic(topicName, messageType, callback) {
            const topic = new ROSLIB.Topic({
                ros: ros,
                name: topicName,
                messageType: messageType
            });
            
            topic.subscribe(callback);
            console.log(`Subscribed to topic: ${topicName}`);
            return topic;
        }
        
        function publishToTopic(topicName, messageType, messageData) {
            const topic = new ROSLIB.Topic({
                ros: ros,
                name: topicName,
                messageType: messageType
            });
            
            const message = new ROSLIB.Message(messageData);
            topic.publish(message);
        }
        
        function setupJoystick() {
            const joystickElement = document.getElementById('joystick');
            
            // Clear any existing joystick
            joystickElement.innerHTML = '';
            
            const joystick = nipplejs.create({
                zone: joystickElement,
                mode: 'static',
                position: { left: '50%', top: '50%' },
                color: '#007bff',
                size: 150
            });
            
            const maxLinearSpeed = 1.0; // m/s
            const maxAngularSpeed = 1.0; // rad/s
            
            joystick.on('move', (evt, data) => {
                const distance = Math.min(data.distance, 75);
                const angle = data.angle.radian;
                
                const linearX = Math.sin(angle) * maxLinearSpeed * (distance / 75);
                const angularZ = -Math.cos(angle) * maxAngularSpeed * (distance / 75);
                
                // Update speed display
                document.getElementById('linear-speed').textContent = linearX.toFixed(2);
                document.getElementById('angular-speed').textContent = angularZ.toFixed(2);
                
                // Send command to robot
                publishToTopic('/cmd_vel', 'geometry_msgs/Twist', {
                    linear: { x: linearX, y: 0, z: 0 },
                    angular: { x: 0, y: 0, z: angularZ }
                });
            });
            
            joystick.on('end', () => {
                // Stop the robot when joystick is released
                publishToTopic('/cmd_vel', 'geometry_msgs/Twist', {
                    linear: { x: 0, y: 0, z: 0 },
                    angular: { x: 0, y: 0, z: 0 }
                });
                
                // Update speed display
                document.getElementById('linear-speed').textContent = '0.00';
                document.getElementById('angular-speed').textContent = '0.00';
            });
            
            console.log('Joystick initialized');
        }
        
        function setupCameraFeed(topicName = '/camera/image_raw/compressed') {
            const imageTopic = new ROSLIB.Topic({
                ros: ros,
                name: topicName,
                messageType: 'sensor_msgs/CompressedImage'
            });
            
            imageTopic.subscribe((message) => {
                const imageElement = document.getElementById('camera-feed');
                imageElement.src = "data:image/jpeg;base64," + message.data;
            });
            
            console.log('Camera feed initialized');
        }
        
        function setup2DMap() {
            // Simplified implementation - in a real application, you would use
            // a visualization library appropriate for your needs
            console.log('2D map initialized');
            
            subscribeToTopic('/map', 'nav_msgs/OccupancyGrid', (message) => {
                console.log('Map data received');
                // Process and display map data
            });
        }
        
        function setup3DMap() {
            // Simplified implementation - in a real application, you would use
            // a 3D visualization library like Three.js or ros3djs
            console.log('3D map initialized');
            
            subscribeToTopic('/point_cloud', 'sensor_msgs/PointCloud2', (message) => {
                console.log('Point cloud data received');
                // Process and display point cloud data
            });
        }
        
        function setupDataDisplay() {
            // Subscribe to odometry for position
            subscribeToTopic('/odom', 'nav_msgs/Odometry', (message) => {
                document.getElementById('position-x').textContent = message.pose.pose.position.x.toFixed(2);
                document.getElementById('position-y').textContent = message.pose.pose.position.y.toFixed(2);
                document.getElementById('position-z').textContent = message.pose.pose.position.z.toFixed(2);
            });
            
            // Subscribe to battery state
            subscribeToTopic('/battery_state', 'sensor_msgs/BatteryState', (message) => {
                document.getElementById('battery-level').textContent = (message.percentage * 100).toFixed(0);
                document.getElementById('battery-status').textContent = determineBatteryStatus(message);
            });
            
            console.log('Data displays initialized');
        }
        
        function determineBatteryStatus(batteryMessage) {
            if (batteryMessage.percentage < 0.2) return 'Low';
            if (batteryMessage.power_supply_status === 1) return 'Charging';
            return 'Normal';
        }
        
        // Automatically try to connect when the page loads
        window.onload = () => {
            // Add a slight delay to ensure all elements are loaded
            setTimeout(connectToROSBridge, 500);
        };
    </script>
</body>
</html>
```

## Troubleshooting

Common issues and their solutions:

### Connection Problems

**Problem**: Cannot connect to ROSBridge server.

**Solutions**:
- Ensure ROSBridge is running: `roslaunch rosbridge_server rosbridge_websocket.launch`
- Check if the WebSocket URL is correct (including protocol, IP, and port)
- Verify there are no firewall issues blocking WebSocket connections
- Try using localhost (`ws://localhost:9090`) when running on the same machine

### Topic Subscription Issues

**Problem**: Not receiving data from subscribed topics.

**Solutions**:
- Verify the topic exists: `rostopic list`
- Check topic message type: `rostopic info /your_topic`
- Ensure the topic is active: `rostopic echo /your_topic`
- Confirm the message structure matches what your code expects

### Camera Feed Not Displaying

**Problem**: Camera feed image is not showing up.

**Solutions**:
- Ensure the camera topic is publishing: `rostopic hz /camera/image_raw/compressed`
- Verify the message type (CompressedImage vs. Image)
- Check browser console for JavaScript errors
- Try using a direct image topic if compression is causing issues

## Additional Resources

- [roslibjs Documentation](https://github.com/RobotWebTools/roslibjs)
- [ROSBridge Wiki](http://wiki.ros.org/rosbridge_suite)
- [ROS 2D Visualization](https://github.com/RobotWebTools/ros2djs)
- [ROS 3D Visualization](https://github.com/RobotWebTools/ros3djs)
- [nipplejs Documentation](https://github.com/yoannmoinet/nipplejs)