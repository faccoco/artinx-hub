var cvs, ctx;
let img, currentRes;
let allPoints = [];
const ImageUrl = "http://127.0.0.1:5630/img/RadarCenter";
const AllPosition = ["标志点1", "标志点2", "...3", "4", "5", "6"];
const TotalPointCounts = AllPosition.length;

$(document).ready(() => {
    initCanvas();
    initImg();
    initButtons();
    initTable();
    initMouse();
    setInterval(updateLog(), 100);
});

function initCanvas() {
    cvs = document.getElementById('radarCenter');
    ctx = cvs.getContext('2d');
    ctx.alpha = false;
    ctx.willReadFrequently = true;
    ctx.imageSmoothingEnabled = false;
    currentRes = document.getElementById('results');
}

function initImg() {
    img = new Image();
    img.onload = function () {
        ctx.clearRect(0, 0, cvs.width, cvs.height);
        cvs.width = img.width;
        cvs.height = img.height;
        ctx.drawImage(img, 0, 0);
    };
    img.src = ImageUrl;
}

function initMouse() {
    cvs.addEventListener('mousemove', getPos, true);
    cvs.addEventListener('click', writePoint, true);
    cvs.onauxclick = popPoint;
}

function getPos(evt) {
    let rect = cvs.getBoundingClientRect();
    let x = parseInt(evt.clientX - rect.left);
    let y = parseInt(evt.clientY - rect.top);
    let porperty = ctx.getImageData(x, y, 1, 1).data;
    results.innerHTML = '<table style="width:100%;table-layout:fixed"><td>X: '
        + x + '</td><td>Y: ' + y + '</td><td>R: '
        + porperty[0] + '</td><td>G: ' + porperty[1] + '</td><td>B: '
        + porperty[2] + '</td></table>';
    return {x, y};
}

function popPoint() {
    if (allPoints.length > 0) {
        allPoints.pop();
        updateTable();
    }
};

function writePoint(evt) {
    if (allPoints.length < TotalPointCounts) {
        allPoints.push(getPos(evt));
        updateTable();
    }
}

const AllButtons = [["ReloadPage", function () {
    window.location.reload();
}], ["ResetAll", function () {
    allPoints = [];
    updateTable();
}], ["Send", function () {
    if (allPoints.length == TotalPointCounts)
        fetch("/radar_points", {
            method: "POST",
            body: JSON.stringify(allPoints)
        });
}], ["CloseTab", function () {
    window.close();
}]];

function initButtons() {
    AllButtons.forEach(function (element) {
        let button = $("<label class=\"mdui-list-item mdui-ripple\">" +
            "<div class=\"mdui-list-item-content mdui-text-truncate\">" + element[0] + "</div > " +
            "<div class=\"mdui-fab-mini\">" +
            "<input type=\"button\" id=\"filter_" + element[0] + "\"/>" +
            "<i class=\"mdui-btn-icon\"></i>" +
            "</div>" +
            "</label>");
        $("#filter").append(button);
        $("#filter_" + element[0]).click(element[1]);
    });
}

function initTable() {
    AllPosition.forEach(function (element) {
        let line = $("<tr><td>" + element + "</td></tr>");
        let val = $("<td>not selected</td>");
        watches[element] = val;
        line.append(val);
        $("#watches").append(line);
    });
}

function updateTable() {
    for (let i = 0; i < allPoints.length; ++i)
        watches[AllPosition[i]].html("X: " + allPoints[i].x + " Y: " + allPoints[i].y);
    for (let i = allPoints.length; i < TotalPointCounts; ++i)
        watches[AllPosition[i]].html("not selected");
}

function updateLog() {
    let logDiv = $("#logs");
    let keepDown = false;
    if (logDiv[0].scrollTop + logDiv[0].clientHeight >= logDiv[0].scrollHeight - 200.0) {
        keepDown = true;
    }
    fetch("/log").then(res => {
        if (!res.ok) {
            throw new Error(res.status + "");
        }
        return res.text();
    }).then(data => {
        if (data === "") return;
        if (this.prev) {
            data = this.prev + data;
        }
        let logs = data.split('\n');
        for (let log of logs.slice(0, -1)) {
            $("#logs").append("<p>" + log + "</p>");
        }
        this.prev = logs[logs.length - 1];
    });
    if (keepDown) {
        logDiv[0].scrollTop = logDiv[0].scrollHeight - logDiv[0].clientHeight;
    }
}
