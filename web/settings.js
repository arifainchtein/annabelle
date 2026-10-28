// Annabelle settings: the Daffodil page's System Info (set time, GPS) and WiFi Config panels,
// without the sensor and config-switch parts. Talks to /AnnabelleServlet (AnnabelleWifiManager):
// GET formName=GetWebData for the current values, POST for the forms. Clock and GPS changes are
// queued and applied by Annabelle's loop(), which reports back in GetWebData.settingsResult.
var webData = null;

function $(id) { return document.getElementById(id); }
function show(id, on) { $(id).classList.toggle("d-none", !on); }

function loadWebData() {
    return fetch("/AnnabelleServlet?formName=GetWebData", { cache: "no-store" })
        .then(function (r) {
            // 503 = Annabelle already has 2 replies going out (AnnabelleWifiManager gate) - ask again shortly
            if (r.status === 503) return new Promise(function (ok) { setTimeout(ok, 1500); }).then(loadWebData).then(function () { return null; });
            return r.json();
        })
        .then(function (j) { if (!j) return webData; webData = j; refreshHeader(); return j; });
}

function post(params) {
    var body = new URLSearchParams(params);
    return fetch("/AnnabelleServlet", { method: "POST", body: body })
        .then(function (r) { return r.text().then(function (t) { if (!r.ok) throw new Error(t); return t; }); });
}

function refreshHeader() {
    if (!webData) return;
    $("host-name").textContent = webData.hostname || "Annabelle";
    $("chip-lora").style.color = webData.lora ? "#0d6efd" : "#c62828";
    $("chip-net").style.color = webData.internetAvailable ? "#2e7d32" : "#c62828";
    $("chip-ssid").textContent = webData.stationmode ? (webData.ssid || "") : "Access point " + (webData.soft_ap_ssid || "");
}

// ---------------------------------------------------------------- System Info / Set Time / GPS

function localInputValue(d) {
    function p(n) { return (n < 10 ? "0" : "") + n; }
    return d.getFullYear() + "-" + p(d.getMonth() + 1) + "-" + p(d.getDate()) + "T" + p(d.getHours()) + ":" + p(d.getMinutes()) + ":" + p(d.getSeconds());
}

function fillSettings() {
    var d = webData;
    $("serial-number").textContent = d.serialnumber || "-";
    $("firmware").textContent = d.firmware || "-";
    $("board-temp").textContent = (d.boardTemperature === null || d.boardTemperature === undefined) ? "-" : Number(d.boardTemperature).toFixed(1) + " °C";
    $("ip-address").textContent = d.ipAddress || "-";
    $("ping-time").textContent = d.internetAvailable ? d.internetPingTime + " ms" : "no internet";
    $("ntp-status").textContent = d.ntpSynced ? "synced" : "not synced";
    $("uptime").textContent = fmtDuration(d.uptimeSec || 0);
    show("internet-time-button", !!d.ntpSynced);
    $("latitude").value = d.latitude === undefined ? "" : d.latitude;
    $("longitude").value = d.longitude === undefined ? "" : d.longitude;
    $("settings-result").textContent = d.settingsResult || "";
}

$("settings-btn").addEventListener("click", function () {
    var open = $("settings-panel").classList.contains("d-none");
    show("wifi-panel", false);
    show("raw-panel", false);
    show("settings-panel", open);
    if (open) {
        $("date-input").value = localInputValue(new Date());
        loadWebData().then(fillSettings);
    }
});
$("settings-close").addEventListener("click", function () { show("settings-panel", false); });

// Sends a queued form, then reads the outcome Annabelle's loop() wrote a second or two later.
function settingsForm(params) {
    $("settings-result").textContent = "Sending...";
    post(params)
        .then(function (reply) {
            $("settings-result").textContent = reply;
            setTimeout(function () { loadWebData().then(fillSettings).then(poll); }, 2000);
        })
        .catch(function (e) { $("settings-result").textContent = "Failed: " + e.message; });
}

$("manual-time-submit").addEventListener("click", function () {
    var v = $("date-input").value;
    if (!v) { alert("Please input a valid time"); return; }
    if (v.length === 16) v += ":00";  // some browsers leave the seconds off
    settingsForm({ formName: "ManualSetTime", time: v });
});

$("internet-time-button").addEventListener("click", function () {
    settingsForm({ formName: "SetTimeViaInternet" });
});

$("save-gps-button").addEventListener("click", function () {
    var lat = $("latitude").value.trim(), lon = $("longitude").value.trim();
    if (lat === "" || isNaN(Number(lat))) { alert("Please input the latitude"); return; }
    if (lon === "" || isNaN(Number(lon))) { alert("Please input the longitude"); return; }
    settingsForm({ formName: "SetGPS", lat: lat, long: lon });
});

// ---------------------------------------------------------------- WiFi

function setWifiMode(station) {
    $("wifi-switch").checked = station;
    $("wifi-switch-label").textContent = station ? "Join a WiFi network" : "Be an access point";
    document.querySelectorAll(".station-mode").forEach(function (el) { el.classList.toggle("d-none", !station); });
    document.querySelectorAll(".ap-mode").forEach(function (el) { el.classList.toggle("d-none", station); });
}

function fillWifi() {
    var d = webData;
    var ssids = [];
    try { ssids = JSON.parse(d.ssids || "[]"); } catch (e) { }
    var sel = $("ssid");
    sel.innerHTML = "";
    var seen = {};
    ssids.sort(function (a, b) { return b.rssi - a.rssi; }).forEach(function (s) {
        if (!s.ssid || seen[s.ssid]) return;
        seen[s.ssid] = true;
        var o = document.createElement("option");
        o.value = s.ssid;
        o.textContent = s.ssid + "   " + s.rssi + " dBm" + (s.secure ? "  (password)" : "");
        sel.appendChild(o);
    });
    if (d.ssid && !seen[d.ssid]) {
        var o = document.createElement("option");
        o.value = o.textContent = d.ssid;
        sel.appendChild(o);
    }
    if (d.ssid) sel.value = d.ssid;
    $("wifi-password").value = "";
    $("hostname").value = d.hostname || "";
    $("ap-address").value = d.soft_ap_ssid || "";
    $("ap-password").value = "";
    $("ap-hostname").value = d.hostname || "";
    $("wifi-result").textContent = "";
    setWifiMode(!!d.stationmode);
}

$("wifi-btn").addEventListener("click", function () {
    var open = $("wifi-panel").classList.contains("d-none");
    show("settings-panel", false);
    show("raw-panel", false);
    show("wifi-panel", open);
    if (open) loadWebData().then(fillWifi);
});
$("wifi-switch").addEventListener("change", function () { setWifiMode($("wifi-switch").checked); });
$("wifi-cancel").addEventListener("click", function () { show("wifi-panel", false); });

$("wifi-submit").addEventListener("click", function () {
    var station = $("wifi-switch").checked;
    var params, host;
    if (station) {
        host = $("hostname").value.trim();
        params = { formName: "ConfigSTA", ssid: $("ssid").value, pass: $("wifi-password").value, host: host };
        if (!params.ssid) { alert("Please choose an SSID"); return; }
    } else {
        host = $("ap-hostname").value.trim();
        params = { formName: "ConfigAP", apaddress: $("ap-address").value.trim(), pass: $("ap-password").value, host: host };
        if (!params.apaddress) { alert("Please input the access point name"); return; }
    }
    if (!host) { alert("Please input the host name"); return; }
    if (!confirm("Save and restart Annabelle?")) return;
    $("wifi-result").textContent = "Saving...";
    post(params)
        .then(function (reply) {
            $("wifi-result").textContent = reply + (station
                ? ". Reconnect your phone to " + params.ssid + " and open http://" + host + ".local in about 30 s."
                : ". Join the " + params.apaddress + " network and open http://192.168.4.1 in about 30 s.");
        })
        .catch(function (e) { $("wifi-result").textContent = "Failed: " + e.message; });
});

// ---------------------------------------------------------------- Raw data

$("host-name").addEventListener("click", function () {
    show("settings-panel", false);
    show("wifi-panel", false);
    show("raw-panel", true);
    loadWebData().then(function (d) {
        var html = '<table class="table table-sm table-striped">';
        Object.keys(d).forEach(function (k) {
            var v = d[k];
            if (k === "ssids") {
                try { v = JSON.parse(v).map(function (s) { return esc(s.ssid) + " " + s.rssi + " dBm ch " + s.channel; }).join("<br>"); } catch (e) { v = esc(v); }
            } else {
                v = esc(v);
            }
            html += "<tr><td>" + esc(k) + "</td><td>" + v + "</td></tr>";
        });
        $("raw-table").innerHTML = html + "</table>";
    });
});
$("raw-close").addEventListener("click", function () { show("raw-panel", false); });

loadWebData();
