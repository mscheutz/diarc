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
        wsMapGui.onmessage = function(data) {
            showMapData(data.data);
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
//function showMapData(message) {
//    $("#mapData").append("<p>" + message + "</p>");
//}
function showMapData(message) {
    var data = JSON.parse(message);
    if (data.mapImageUrl) {
        $("#mapData").html("<img src='" + data.mapImageUrl + "' />");
    } else {
        $("#mapData").text("Error: " + data.error);
    }
}


function navigateToPoint(x, y) {
    if (wsMapGui && wsMapGui.readyState === WebSocket.OPEN) {
        wsMapGui.send(JSON.stringify({ action: 'navigateToPoint', x: x, y: y }));
    }
}

function updateRobotLocation() {
    if (wsMapGui && wsMapGui.readyState === WebSocket.OPEN) {
        wsMapGui.send(JSON.stringify({ action: 'updateRobotLocation' }));
    }
}






$(function() {
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
    $("#fetchMapData").click(function() {
        connectMapGui();
        setTimeout(fetchMapData, 100); // Delay the fetch to give connection time to open
    });
    $("#navigateToPoint").click(function() {
        var x = prompt("Enter X coordinate:");
        var y = prompt("Enter Y coordinate:");
        connectMapGui();
        setTimeout(function() { navigateToPoint(x, y); }, 100); // Delay to ensure connection is open
    });
    $("#updateRobotLocation").click(function() {
        connectMapGui();
        setTimeout(updateRobotLocation, 100); // Delay to ensure connection is open
    });
});
