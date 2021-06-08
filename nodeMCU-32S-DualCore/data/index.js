var gateway = `ws://${window.location.hostname}/ws`;
var websocket;

// Initialize Elements
window.addEventListener('load', onLoad);
function onLoad(event) {
    initWebSocket();
    initButtonButtons();
}

// WebSocket handling
function initWebSocket() {
    websocket = new WebSocket(gateway);
    websocket.onopen    = onOpen;
    websocket.onclose   = onClose;
    websocket.onmessage = onMessage;
}

function onOpen(event) {}

function onClose(event) { setTimeout(initWebSocket, 2000);}

// Sync Message Handling
function onMessage(event) {
    let data = JSON.parse(event.data);
    
    // Update Button
    if(data.Button=="red_Button"){
        document.getElementById('red_Button').className = data.status;
    } 
    else if(data.Button=="blue_Button"){
        document.getElementById('blue_Button').className = data.status;
    } else if (data.Button=="None") {
        document.getElementById('status').value = data;
    }
}

// Button Handling
function initButtonButtons() {
    document.getElementById('red_Button').addEventListener('click', toggle);
    document.getElementById('blue_Button').addEventListener('click', toggle);
}

// Send Button Commands
function toggle(event) {
    websocket.send(JSON.stringify({'Button':this.id}));
}