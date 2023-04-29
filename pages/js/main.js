import * as echarts from "./echarts/echarts.esm.js"

let filters = {};
let images = {};
let $watchVals = {};
let checkRadar = false;
let locateTab;

let watchCharts = {};
let watchChartOptions = {};
let startTimePoint = new Date() - 0;

let updateInterval = setInterval(() => { }, 100000);
window.settings = new Proxy(
    {
        updateFreq,
        logLength,
    },
    {
        get: (target, p, receiver) => $('#' + p).val(),
        set: function (target, p, value, receiver) {
            $('#' + p).val(value);
            $('#' + p).parents('.mdui-row').children('.settingsDisplay').text(value);
            switch (p) {
                case 'updateFreq':
                    clearInterval(updateInterval);
                    updateInterval = value > 0 ?
                        setInterval(updateAll, 1000. / value) :
                        setInterval(() => { }, 100000);
                    break;
            }
            return true;
        }
    }
);
settings.logLength = 100;
$(document).ready(function () {
    settings.updateFreq = 20.;   //start updating
});

function updateAll() {
    updateFilter();
    updateWatches();
    updateRadar();
}

function updateWatches() {
    fetch("/watch").then(res => res.json()).then(data => {
        for (let k in data) {
            if (!$watchVals.hasOwnProperty(k)) {
                newTableRow(k);
                $('#tr-' + k + ' input:checkbox').attr('checked', false);
                if (!isNaN(+data[k])) {
                    newChart(k);
                    $('#chart-' + k).hide();
                } else {
                    $('#tr-' + k + ' .mdui-checkbox').hide();
                }
            }
            $watchVals[k].html(data[k]);
            if (watchChartOptions.hasOwnProperty(k) && !isNaN(+data[k])) {
                let chartData = watchChartOptions[k].series[0].data;
                if (chartData.length >= settings.logLength) {
                    chartData.splice(0, chartData.length - settings.logLength + 1);
                }
                chartData.push(
                    [(new Date() - startTimePoint) / 1000., +data[k]]
                );
                watchCharts[k].setOption(watchChartOptions[k]);
            }
        }
    })
}

function newTableRow(k) {
    let $tr = $($('#trTemp')[0].content.firstElementChild)
        .clone(true)
        .attr('id', 'tr-' + k);
    $tr.children('td').eq(1).text(k);
    let $val = $tr.children('td').eq(2);
    $watchVals[k] = $val;
    $("#watchesTable").append($tr);
}

function newChart(key) {
    $('#watches').append(
        $($('#chartTemp')[0].content.firstElementChild)
            .clone(true)
            .attr('id', 'chart-' + key)
    );
    let chart = echarts.init($('#chart-' + key)[0]);
    let chartOption = {
        title: {
            text: key
        },
        tooltip: {},
        xAxis: {
            type: 'value',
            min: 'dataMin',
            max: 'dataMax',
        },
        yAxis: {
            type: 'value'
        },
        series: [
            {
                name: key,
                type: 'line',
                //large: true,
                symbol: 'none',
                data: []
            }
        ],
        animation: false    //如果为真，坐标网格会显示混乱（实质上是这些线移动过渡动画过慢）
    };
    chart.setOption(chartOption);
    watchCharts[key] = chart;
    watchChartOptions[key] = chartOption;
}

function updateStatus() {
    fetch("/status").then(res => {
        if (!res.ok) {
            throw new Error(res.status + "");
        }
        return res.text();
    }).then(data => {
        $("#current_status").html(data);
    }).catch(err => {
        $("#current_status").html(`<p>${err}<p>`);
    });
}

function updateFilter() {
    fetch("/filter", {
        method: "POST"
    }).then(res => res.json()).then(data => {
        //data: [[name|uint64_t, false], [name|uint64_t, true], ...]
        if (!(data && Object.keys(data).length === 0 && Object.getPrototypeOf(data) === Object.prototype))
            for (let v of data) {
                if (filters[v[0]] === undefined) {
                    let checkbox = $("<label class=\"mdui-list-item mdui-ripple\">" +
                        "<div class=\"mdui-list-item-content mdui-text-truncate\">" + v[0].split(/([-])/)[0] + "</div > " +
                        "<div class=\"mdui-checkbox\">" +
                        "<input type=\"checkbox\" id=\"filter_" + v[0] + "\"/>" +
                        "<i class=\"mdui-checkbox-icon\"></i>" +
                        "</div>" +
                        "</label>");
                    $("#filter").append(checkbox);
                    let inner = $("#filter_" + v[0]);
                    inner.change(function () {
                        filters[v[0]] = this.checked;
                        if (this.checked)
                            $(images[v[0]]).show();
                        else
                            $(images[v[0]]).hide();
                        fetch("/filter", {
                            method: "POST",
                            body: JSON.stringify(filters)
                        })
                    });
                }
                $("#filter_" + v[0])[0].checked = v[1];
                if (!images.hasOwnProperty(v[0])) {
                    let img = $("<img class=\"mdui-img-fluid image\" id=\"current_image\" src=\"\" alt=\"\"/>")
                    img[0].src = "/img/" + v[0];
                    img.on("click", function (e) {
                        let x = e.pageX - this.offsetLeft;
                        let y = e.pageY - this.offsetTop;
                        fetch("/radar", {
                            method: "POST",
                            body: JSON.stringify([x, y])
                        })
                    });
                    images[v[0]] = img[0];
                    $("#images").append(img);
                }
                filters[v[0]] = v[1];
            }
    });
}

function updateRadar() {
    if (!checkRadar) {
        checkRadar = true;
        setTimeout(() => {
            fetch("/radar", { method: "GET" }).then(res => res.json()).then(data => {
                if (data) {
                    let checkbox = $("<label class=\"mdui-list-item mdui-ripple\">" +
                        "<div class=\"mdui-list-item-content mdui-text-truncate\">Radar Loc-Cal</div > " +
                        "<div class=\"mdui-checkbox\">" +
                        "<input type=\"checkbox\" id=\"filter_radar_cal\"/>" +
                        "<i class=\"mdui-checkbox-icon\"></i>" +
                        "</div>" +
                        "</label>");
                    $("#filter").prepend(checkbox);
                    let inner = $("#filter_radar_cal");
                    inner.change(function () {
                        if (this.checked)
                            locateTab = window.open("radar_locate.html", "Radar Locate");
                        else
                            locateTab.close();
                    });
                }
            })
        }, 300);
    }
}

function exitServer() {
    fetch("/exit");
    try {
        locateTab.close();
    } catch (e) {
    }
    setTimeout(function () {
        window.close();
    }, 514);
}
