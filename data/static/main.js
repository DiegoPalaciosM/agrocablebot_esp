var topnavH = document.getElementById("topnav").offsetHeight + "px";

const sections = document.querySelectorAll("section");

sections.forEach(section => {
    section.style.paddingTop = topnavH;
    section.style.minHeight = `calc(100vh - ${topnavH})`;
})

const cards = document.querySelectorAll(".card");

cards.forEach(center => {
    center.style.transform = `translateY(calc((50% - ${topnavH})/2))`;
})

$.get("mqttHost", (data, status) => {
    document.getElementById("host").outerHTML = `Host MQTT actual: ${data}`;
});

var gateway = `ws://${window.location.host}/info`;
var websocket;
function initWebSocket() {
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
}
function onOpen(event) {
    console.log('Connection opened');
}

function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
}
function onMessage(event) {
    var json = JSON.parse(event.data)
    document.getElementById("curCA").outerHTML = json["curCA"];
    document.getElementById("curCB").outerHTML = json["curCB"];
    document.getElementById("curCC").outerHTML = json["curCC"];
    document.getElementById("curCD").outerHTML = json["curCD"];
}
window.addEventListener('load', onLoad);
function onLoad(event) {
    initWebSocket();
}