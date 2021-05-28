var gateway = `ws://${window.location.hostname}/ws`;
var websocket;

// Initialize Elements
window.addEventListener('load', onLoad);
function onLoad(event) {
    initWebSocket();
    initLEDButtons();
}

// WebSocket handling
function initWebSocket() {
    console.log('Trying to open a WebSocket connection...');
    websocket = new WebSocket(gateway);
    websocket.onopen    = onOpen;
    websocket.onclose   = onClose;
    websocket.onmessage = onMessage;
}

function onOpen(event) {
    console.log('Connection opened');
}

function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
}

// Sync Message Handling
function onMessage(event) {
    let data = JSON.parse(event.data);
    console.log(data);
    // Update LED Button
    if(data.led=="red_LED"){
        document.getElementById('red_LED').className = data.status;
    } 
    else if(data.led=="blue_LED"){
        document.getElementById('blue_LED').className = data.status;
    }    
    else {console.log("Unkown Sync Message");}
    
}

// LED Button Handling
function initLEDButtons() {
    document.getElementById('red_LED').addEventListener('click', toggle);
    document.getElementById('blue_LED').addEventListener('click', toggle);
}

// Send LED Button Commands
function toggle(event) {
    websocket.send(JSON.stringify({'led':this.id}));
}