// depreciated. real front end in diarc/gui

var ws;
function setConnected(connected) {
    $("#connect").prop("disabled", connected);
    $("#disconnect").prop("disabled", !connected);
}

function connect() {
    ws = new WebSocket('ws://localhost:8080/user');
    ws.onmessage = function(data) {
        helloWorld(data.data);
    };
    ws.onopen = function() {
        console.log("Connected to user WebSocket");
        setConnected(true);
    };
    ws.onclose = function() {
        console.log("User WebSocket is in disconnected state");
        setConnected(false);
    };
    ws.onerror = function(error) {
        console.log("WebSocket error: " + error);
    };
}

function disconnect() {
    if (ws) {
        ws.close();
        ws = null;
    }
}

function sendData() {
    if (ws && ws.readyState === WebSocket.OPEN) {
        var data = JSON.stringify({ 'user': $("#user").val() });
        ws.send(data);
    }
}

function helloWorld(message) {
    $("#helloworldmessage").append(" " + message + "");
}

var wsMapGui;
var isConnectingMapGui = false; // Flag to track if a connection attempt is underway

function connectMapGui() {
    if (!wsMapGui || wsMapGui.readyState === WebSocket.CLOSED) {
        wsMapGui = new WebSocket('ws://localhost:8080/map');
        wsMapGui.onmessage = function(event) {
            var data = JSON.parse(event.data); // Parse the incoming message data
            if (data.mapImageUrl) {
                showMapData(event.data); // Handle map data
            } else if (data.position && data.orientation) {
                showRobotPose(event.data); // Handle robot pose data
            } else if (data.keyLocations) { // Check if key locations data is present
                showKeyLocations(event.data); // Handle key locations data
            } else if (data.success !== undefined) { // Check if success message data is present
                updateGoToLocationMessage(data); // Handle goToLocation response
            } else {
                console.log("Received unhandled data type:", data);
            }
        };
        wsMapGui.onopen = function() {
            console.log("Connected to MapGui WebSocket");
        };
        wsMapGui.onerror = function(error) {
            console.log("WebSocket error: ", error);
        };
        wsMapGui.onclose = function(event) {
            console.log("WebSocket connection closed with code:", event.code, "and reason:", event.reason);
            wsMapGui = null; // Reset the WebSocket reference
            // Implement a backoff reconnection strategy here if needed
        };
    } else {
        console.log("WebSocket is already open.");
    }
}


function fetchMapData() {
    if (wsMapGui && wsMapGui.readyState === WebSocket.OPEN) {
        wsMapGui.send(JSON.stringify({ action: 'fetchMapData' }));
    } else {
        console.log("WebSocket is not open. Queuing request.");
        connectMapGui(); // Try reconnecting if not connected
        setTimeout(fetchMapData, 500); // Retry after 500ms
    }
}
function showMapData(message) {
    var data = JSON.parse(message);
    if (data.mapImageUrl) {
        $("#mapImage").attr("src", data.mapImageUrl); // Set the new map image

        // Optional: Clear existing markers when loading new map data
        $(".location-marker, #robotMarker").remove(); // Remove old markers to avoid duplicates

        console.log("Map image updated.");
    } else {
        console.log("Error updating map image:", data.error);
    }
}



function navigateToPoint(x, y, quatX, quatY, quatZ, quatW) {
    if (wsMapGui && wsMapGui.readyState === WebSocket.OPEN) {
        console.log("Sending navigation point to WebSocket.");
        wsMapGui.send(JSON.stringify({
            action: 'navigateToPoint',
            x: parseFloat(x),
            y: parseFloat(y),
            quatX: parseFloat(quatX),
            quatY: parseFloat(quatY),
            quatZ: parseFloat(quatZ),
            quatW: parseFloat(quatW)
        }));
    } else {
        console.log("WebSocket is not open. Attempting to connect...");
        connectMapGui();
        setTimeout(() => navigateToPoint(x, y, quatX, quatY, quatZ, quatW), 100); // Retry after 100 ms
    }
}
function updateGoToLocationMessage(data) {
    var msgElement = $('#goToLocationMsg');
    msgElement.text(data.message); // Update the message text
    if (data.success) {
        msgElement.removeClass('alert-danger').addClass('alert-success');
    } else {
        msgElement.removeClass('alert-success').addClass('alert-danger');
    }
    msgElement.show(); // Make sure the message is visible
}

function updateRobotLocation() {
    if (wsMapGui && wsMapGui.readyState === WebSocket.OPEN) {
        wsMapGui.send(JSON.stringify({ action: 'updateRobotLocation' }));
    }
}

function fetchRobotPose() {
    if (wsMapGui && wsMapGui.readyState === WebSocket.OPEN) {
        wsMapGui.send(JSON.stringify({ action: 'fetchRobotPose' }));
    }
}
function showRobotPose(message) {
    var data = JSON.parse(message);
    if (data.position && data.orientation) {
        console.log("Received Robot Pose: Position - " + JSON.stringify(data.position) +
                    ", Orientation - " + JSON.stringify(data.orientation));
        $("#posePosition").text("Position - X: " + data.position.x + ", Y: " + data.position.y + ", Z: " + data.position.z);
        $("#poseOrientation").text("Orientation - X: " + data.orientation.x + ", Y: " + data.orientation.y +
                                  ", Z: " + data.orientation.z + ", W: " + data.orientation.w);

        // Check if robot marker exists, if not create it
        var robotMarker = $("#robotMarker");
        if (!robotMarker.length) {
            $('#mapData').append('<div id="robotMarker" style="position: absolute;"></div>');
            robotMarker = $("#robotMarker"); // Re-select after appending
        }

        // Update robot marker position on the map
        robotMarker.css({
            left: data.robotPixelPosition.x + 'px',
            top: data.robotPixelPosition.y + 'px',
            display: 'block'
        });
    } else {
        console.log("Error fetching robot pose:", data.error);
    }
}


function fetchKeyLocations() {
    if (wsMapGui && wsMapGui.readyState === WebSocket.OPEN) {
        wsMapGui.send(JSON.stringify({ action: 'fetchKeyLocations' }));
    }
}
function showKeyLocations(message) {
    var data = JSON.parse(message);
    if (data && data.keyLocations) {
        console.log("Received Key Locations: ", data.keyLocations);
        var keyLocations = data.keyLocations;

        // Iterate through each key location and manage the marker
        Object.keys(keyLocations).forEach(function(key) {
            var location = keyLocations[key];
            var x = location.x; // Assuming these are pixel coordinates
            var y = location.y;

            var markerId = 'marker-' + key.replace(/[^a-zA-Z0-9]/g, ''); // Normalize key for HTML ID usage
            var existingMarker = $('#' + markerId);
            if (existingMarker.length) {
                // Update position of existing marker
                existingMarker.css({left: x + 'px', top: y + 'px'});
            } else {
                // Create new marker and append to mapData
                $('#mapData').append('<div id="' + markerId + '" class="location-marker" style="left: ' + x + 'px; top: ' + y + 'px;"></div>');
            }
        });

        $("#keyLocations").text(JSON.stringify(data.keyLocations));
    } else {
        console.log("Error fetching key locations:", data.error);
    }
}


$(function() {
    $("#fetchMapData").click(function() {
        connectMapGui();
        setTimeout(fetchMapData, 100); // Delay the fetch to give connection time to open
    });
    $("#navigateToPoint").click(function() {
        console.log("Navigate to Point button clicked.");
        var x = prompt("Enter X coordinate:");
        var y = prompt("Enter Y coordinate:");
        console.log("Coordinates provided by user: X =", x, ", Y =", y);

        // Default quaternion values for no rotation
        var quatX = 0.0;
        var quatY = 0.0;
        var quatZ = 0.0;
        var quatW = 1.0;

        if (x !== null && y !== null) { // Only proceed if x and y are provided
            console.log("Connecting to Map GUI WebSocket.");
            connectMapGui();
            setTimeout(function() { navigateToPoint(x, y, quatX, quatY, quatZ, quatW); }, 100); // Delay to ensure connection is open
        } else {
            console.log("Navigation cancelled or incomplete coordinates provided.");
        }
    });
    $("#updateRobotLocation").click(function() {
        connectMapGui();
        setTimeout(updateRobotLocation, 100); // Delay to ensure connection is open
    });
    $("#fetchRobotPose").click(function() {
        connectMapGui();
        setTimeout(fetchRobotPose, 100); // Delay to ensure connection is open
    });
    $("#fetchKeyLocations").click(function() {
        connectMapGui();
        setTimeout(fetchKeyLocations, 100); // Delay to ensure connection is open
    });
    $("form").on('submit', function(e) {
        e.preventDefault();
    });
    $("#connect").click(function() {
        connect();
    });
    $("#disconnect").click(function() {
        disconnect();
    });
    $("#send").click(function() {
        sendData();
    });
});


