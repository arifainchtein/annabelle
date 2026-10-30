// Annabelle telepathons page: polls /Telepathons (Annabelle.ino fillTelepathonsJson) every 30 s.
// Each device is a stack of cards, one per packet: the newest is in front and the arrows (or a
// swipe) go back through the ones Annabelle kept (/TelepathonHistory). Devices whose packets check
// out are listed first, by name; sources that never sent a packet that checked out (another
// device's struct of the same size, bit errors) are piled up underneath as junk.
var OP_NAMES = { 0: "Unknown", 1: "Pulse Sleep", 2: "No LED", 3: "Full", 4: "Cloudy", 5: "Comma" };

// Card types - must match HeardCardType in HeardEntry.h, values in the order noteXxxPacket() sends them.
// f: [label, unit, decimals] or a function(value) returning the text.
// tabs: which of the card's tabs each value goes on, one letter per f entry - s Sensors, p Power, r Radio.
var CARD_TYPES = {
    1: { name: "Data", checksum: true, f: [["Battery", "V", 2], ["Battery current", "mA", 1], ["Panel", "V", 2], ["Panel current", "mA", 1],
        ["V50_I", "V", 2], ["Mode", opName], ["Sleep", "s", 0], ["Forecast", forecastText], ["Clock", clockText]], tabs: "pppppssrs" },
    2: { name: "Data", checksum: false, f: [["Fish tank", "cm", 1], ["Sump trough", "cm", 1], ["RTC battery", "V", 2], ["Micro temp", "°C", 0],
        ["Light", "lux", 0], ["Alert code", "", 0], ["Clock", clockText]], tabs: "sspssss" },
    3: { name: "Data", checksum: true, f: [["Battery", "V", 2], ["Battery current", "mA", 1], ["Solar", "V", 2], ["Solar current", "mA", 1],
        ["Fence avg", "kV", 2], ["Energizer battery", "V", 2], ["Temperature", "°C", 1], ["Clock", clockText], ["Sleep", "s", 0]], tabs: "ppppsssss" },
    4: { name: "Data", checksum: false, f: [["Solar", "V", 2], ["Capacitor", "V", 2], ["Flow 1 total", "ml", 0], ["Flow 2 total", "ml", 0],
        ["Temperature", "°C", 0], ["Clock", clockText]], tabs: "ppssss" },
    5: { name: "Data", checksum: false, f: [["Temperature", "°C", 1], ["Clock", clockText]], tabs: "ss" },
    6: { name: "Vital signs", checksum: true, f: [["Resets", "", 0], ["Sequence", "", 0], ["Wake voltage", "V", 3], ["Min voltage", "V", 3],
        ["TX duration", "ms", 0], ["TX failures", "", 0], ["Wake drift", "s", 0], ["Last awake", "s", 0], ["Wakes", "", 0]], tabs: "prpprrppp" },
    7: { name: "Identity", checksum: true, f: [] },
    8: { name: "Comma record", checksum: true, f: [["Voltage", "V", 2], ["Index", "", 0], ["Total", "", 0]], tabs: "ppp" },
    9: { name: "Power profile", checksum: true, f: [["Capacity", "mAh", 0]], tabs: "p" }
};

// Daffodil functions (config switch, DigitalStablesData.h FUN_*/DAFFODIL_*): the name and the labels of
// the readings Annabelle.ino daffodilSensors() sends for it in "sens", after the board temperature and the
// outdoor temperature/humidity (OUTDOOR, shown for any function that has the sensor).
var OUTDOOR = [["Outdoor temperature", "°C", 1], ["Outdoor humidity", "%", 0]];
var FLOW = ["Flow 1", "L/min", 1], TOTAL = ["Flow 1 total", mlText], TANK1 = ["Tank 1 pressure", "psi", 2], TROUGH = ["Trough level", "cm", 0];
var FUNCTIONS = {
    1: { name: "One flow meter", f: [FLOW, TOTAL] },
    2: { name: "Two flow meters", f: [FLOW, TOTAL, ["Flow 2", "L/min", 1], ["Flow 2 total", mlText]] },
    3: { name: "Flow meter and tank", f: [FLOW, TOTAL, TANK1] },
    4: { name: "One tank", f: [TANK1] },
    5: { name: "Two tanks", f: [TANK1, ["Tank 2 pressure", "psi", 2]] },
    6: { name: "Septic tank", f: [["Level", "cm", 0], ["Maximum", "cm", 0]] },
    7: { name: "Water trough", f: [TROUGH, ["Trough minimum", "cm", 0], ["Trough maximum", "cm", 0]] },
    8: { name: "Temperature and soil moisture", f: [] },  // its readings are OUTDOOR
    9: { name: "Light detector", f: [] },
    10: { name: "Voltage monitor", f: [["Sensor 1", "V", 2]] },
    11: { name: "Water trough and tank", f: [TROUGH, TANK1] },
    12: { name: "Two water troughs", f: [TROUGH, ["Trough 2 level", "cm", 0]] },
    13: { name: "Water trough and water temperature", f: [TROUGH, ["Water temperature", "°C", 1]] }
};
// Flow totals as the Daffodil's own page shows them: ml up to 1 L, then litres.
function mlText(v) {
    if (v > 10000) return (v / 1000).toFixed(0) + " L";
    if (v > 1000) return (v / 1000).toFixed(2) + " L";
    return Math.round(v) + " ml";
}

// The Sensors tab's function block for a Daffodil (front of the stack only - older cards don't keep it).
function functionRows(t) {
    var fn = FUNCTIONS[t.fn] || { name: "Function " + t.fn, f: [] }, v = t.sens || [];
    var html = row("Function", esc(fn.name));
    var temp = valueText(["Board temperature", "°C", 0], v[0]);
    if (temp !== null) html += row("Board temperature", temp);
    OUTDOOR.forEach(function (field, i) {
        var text = valueText(field, v[i + 1]);
        if (text !== null) html += row(field[0], text);
    });
    fn.f.forEach(function (field, i) {
        var text = valueText(field, v[i + 1 + OUTDOOR.length]);
        if (text !== null) html += row(field[0], text);
    });
    return html;
}

var last = null;        // last /Telepathons JSON
var lastFetchMs = 0;    // browser time it arrived, to keep the clocks ticking between polls
var nav = {};           // per stack (row:gen): { idx, cards } - idx 0 is the live front card
var junkOpen = false;

function pad(n) { return (n < 10 ? "0" : "") + n; }
function esc(s) {
    return String(s).replace(/[&<>"']/g, function (c) { return { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]; });
}

// Annabelle's RTC "epoch" is local wall-clock time counted as if it were UTC, so it is read back
// with the UTC getters.
function fmtTime(epoch) {
    if (!epoch) return "-";
    var d = new Date(epoch * 1000);
    return pad(d.getUTCHours()) + ":" + pad(d.getUTCMinutes()) + ":" + pad(d.getUTCSeconds());
}
function fmtDateTime(epoch) {
    var d = new Date(epoch * 1000);
    return pad(d.getUTCDate()) + "/" + pad(d.getUTCMonth() + 1) + " " + fmtTime(epoch);
}
function fmtAgo(sec) {
    if (sec < 0) sec = 0;
    if (sec < 60) return sec + " s ago";
    if (sec < 3600) return Math.floor(sec / 60) + " min " + (sec % 60) + " s ago";
    if (sec < 86400) return Math.floor(sec / 3600) + " h " + Math.floor((sec % 3600) / 60) + " min ago";
    return Math.floor(sec / 86400) + " d " + Math.floor((sec % 86400) / 3600) + " h ago";
}
function fmtDuration(sec) { return fmtAgo(sec).replace(" ago", ""); }

function opName(v) { return OP_NAMES[v] || ("Mode " + v); }
// Daffodil opMode bit 1: the device holds a forecast from Annabelle less than 31 min old.
function forecastText(v) { return (v & 2) ? "fresh" : '<span class="bad">none in the last 31 min</span>'; }
function clockPlain(v) {
    var s = Math.round(v);
    return s === 0 ? "on time" : Math.abs(s) + " s " + (s > 0 ? "fast" : "slow");
}

// Did Annabelle's WeatherForecastUpdate reach this Daffodil, and did its clock get corrected?
// From fillTelepathonsJson's "sync" (Annabelle.ino noteSync): the device only reports whether its
// forecast is fresh, so these are the times Annabelle saw that, not exact reception times.
function syncLine(s) {
    var w = s.fresh
        ? '<span style="color:#2e7d32">&#10003; Weather received</span> - fresh since ' + fmtTime(s.freshSince)
        : '<span class="bad">&#10007; Weather not received</span>' + (s.lastFresh ? " - last fresh " + fmtDateTime(s.lastFresh) : " since Annabelle started");
    var t = s.correctedAt
        ? '<span style="color:#2e7d32">&#10003; Clock corrected</span> ' + fmtDateTime(s.correctedAt) + " (was " + clockPlain(s.correctedBy) + ")"
        : '<span class="bad">&#10007; No clock correction seen</span>' + (s.offset !== undefined ? " - " + clockPlain(s.offset) : "");
    return '<div style="font-size:13px;margin:2px 0 4px 0">' + w + "<br>" + t + "</div>";
}

// WiFi of a Daffodil (fillTelepathonsJson "wifi"): what its latest data packet reported, and the
// on/off button. The request is held by Annabelle and resent until the device's data shows the
// change (DeviceCommandQueue.h) - an awake device answers in seconds, a sleeping one at its next wake.
var WIFI_STATES = { 1: "own hotspot", 2: "router, no internet", 3: "router + internet" };
var CMD_TIMEOUT_MIN = 15;
function wifiLine(t) {
    var w = t.wifi, c = w.cmd, on = w.status > 0;
    var html = on ? '<span style="color:#2e7d32">&#10003; WiFi on</span> - ' + esc(WIFI_STATES[w.status] || "status " + w.status)
        : '<span class="bad">&#10007; WiFi off</span>';
    var pending = c && c.state === 1;
    if (pending) {
        html += '<br><span style="color:#ef6c00">Turning ' + (c.on ? "on" : "off") + "... sent " + c.sends + (c.sends === 1 ? " time, " : " times, ")
            + fmtAgo(c.ageSec) + " - a sleeping device answers when it next wakes</span>";
    } else if (c && c.state === 2) {
        html += '<br><span style="color:#555">Turned ' + (c.on ? "on" : "off") + " from this page " + fmtAgo(c.ageSec) + "</span>";
    } else if (c && c.state === 3) {
        html += '<br><span class="bad">Turning ' + (c.on ? "on" : "off") + " gave up after " + CMD_TIMEOUT_MIN + " min - device not heard</span>";
    }
    var btn = pending ? '<button class="btn btn-outline-secondary btn-sm" disabled>Waiting...</button>'
        : '<button class="btn ' + (on ? "btn-outline-secondary" : "btn-primary") + ' btn-sm" data-wifi="' + (on ? "off" : "on") + '" data-serial="' + esc(t.serial) + '">'
          + (on ? "Turn WiFi off" : "Turn WiFi on") + "</button>";
    return '<div class="d-flex justify-content-between align-items-center" style="font-size:13px;margin:2px 0 4px 0;gap:8px"><div>'
        + html + "</div>" + btn + "</div>";
}

function sendWifi(btn) {
    btn.disabled = true;
    fetch("/AnnabelleServlet", { method: "POST", body: new URLSearchParams({
        formName: "DeviceWifi", serial: btn.getAttribute("data-serial"), state: btn.getAttribute("data-wifi") }) })
        .then(function (r) { return r.text(); })
        .then(function (t) {
            if (t !== "Queued") alert(t);
            setTimeout(poll, 1500);
        })
        .catch(function () { btn.disabled = false; alert("Cannot reach Annabelle"); });
}

// Device clock minus real time - TopTank ran ~2 min slow for days (2026-10-11).
function clockText(v) {
    var s = Math.round(v);
    var text = s === 0 ? "on time" : (Math.abs(s) + " s " + (s > 0 ? "fast" : "slow"));
    return Math.abs(s) > 30 ? '<span class="bad">' + text + "</span>" : text;
}

function valueText(field, v) {
    if (v === null || v === undefined) return null;
    if (typeof field[1] === "function") return field[1](v);
    // Garbage floats from misread structs: show them as they are, but readable.
    var text = (Math.abs(v) >= 1e6 || (v !== 0 && Math.abs(v) < 1e-3)) ? Number(v).toExponential(2) : Number(v).toFixed(field[2]);
    return esc(text) + (field[1] ? " " + field[1] : "");
}

// Green while a device is reporting normally, amber when it has missed a while, red when silent.
// Sleeping Daffodils and Langleys report every sleep-time seconds, so they get more slack.
var SLEEP_VALUE = { 1: 6, 3: 8 };  // card type -> index of its sleep time
function ageColor(sec, card) {
    var i = card ? SLEEP_VALUE[card.ty] : undefined;
    var sleep = (i !== undefined && card.v && card.v[i] > 0) ? card.v[i] : 0;
    var expected = sleep > 0 ? sleep + 60 : 300;
    if (sec <= expected * 1.5) return "#2e7d32";
    if (sec <= expected * 3) return "#ef6c00";
    return "#c62828";
}

function row(label, value) {
    return '<div class="hc-row"><span>' + label + "</span><span>" + value + "</span></div>";
}

function navKey(t) { return t.row + ":" + t.gen; }

// One card's values for one tab, under a small heading saying which packet they came from.
function tabSection(card, tab, nowSec) {
    var type = CARD_TYPES[card.ty] || { name: "Type " + card.ty, f: [], tabs: "" };
    var v = card.v || [], html = "";
    type.f.forEach(function (field, i) {
        if ((type.tabs || "").charAt(i) !== tab) return;
        var text = valueText(field, v[i]);
        if (text !== null) html += row(field[0], text);
    });
    if (!html) return "";
    return '<div class="hc-sub">' + esc(type.name) + " · " + fmtAgo(nowSec - card.t)
        + (card.ok ? "" : ' · <span class="bad">' + (type.checksum ? "checksum failed" : "name or clock not believable") + "</span>") + "</div>" + html;
}

// The card's tabs (2026-10-23). Front of the stack: the newest data packet, the newest vital signs
// and whatever else came last (fillTelepathonsJson "front", "data", "vital"). Further back: that one
// packet, split the same way.
var TAB_NAMES = { s: "Sensors", p: "Power", r: "Radio" };
var cardTab = {};   // device name -> s/p/r, remembered on this phone
try { cardTab = JSON.parse(localStorage.getItem("annabelleCardTabs")) || {}; } catch (e) { }

function isData(c) { return c && c.ty >= 1 && c.ty <= 5; }

function cardBody(t, card, nowSec, isFront) {
    var cards = [card];
    if (isFront) {
        var data = isData(card) ? card : t.data, vital = card.ty === 6 ? card : t.vital;
        cards = [data, vital];
        if (card !== data && card !== vital) cards.push(card);  // comma record, power profile, identity
    }
    var dataCard = cards[0] && isData(cards[0]) ? cards[0] : null;
    var tab = cardTab[t.name] || "s";
    var age = nowSec - card.t;
    var html = '<div class="hc-age" style="color:' + (isFront ? ageColor(age, dataCard) : "#555") + '">'
        + (isFront ? "Last heard " : esc((CARD_TYPES[card.ty] || { name: "Type " + card.ty }).name) + " · ")
        + fmtAgo(age) + " (" + fmtDateTime(card.t) + ")</div>";
    if (dataCard && dataCard.ty === 1 && dataCard.v && (dataCard.v[7] & 1)) html += '<div class="tp-warn">USB jumper fitted</div>';  // opMode bit 0
    html += '<div class="hc-tabs">' + Object.keys(TAB_NAMES).map(function (k) {
        return '<button type="button" class="' + (k === tab ? "active" : "") + '" data-tab="' + k + '" data-name="' + esc(t.name) + '">' + TAB_NAMES[k] + "</button>";
    }).join("") + "</div>";
    var body = "";
    if (tab === "r") {
        if (t.sync && isFront) body += syncLine(t.sync);
        if (t.wifi && isFront) body += wifiLine(t);
        body += row("Signal", card.rssi + " dBm, SNR " + Number(card.snr).toFixed(1));
    }
    if (tab === "s" && t.fn && isFront) body += functionRows(t);
    cards.forEach(function (c) { if (c) body += tabSection(c, tab, nowSec); });
    if (!body) body = '<div class="hc-empty">' + (isFront ? "Nothing on " + TAB_NAMES[tab] + " heard yet" : "No " + TAB_NAMES[tab] + " values in this packet") + "</div>";
    return html + '<div class="hc-body">' + body + "</div>";
}

// Product logo for the kinds that have one (web/logos, put into the page by build_web.py as
// KIND_LOGO_DATA); the grey badge otherwise, and always on junk - a junk "Daffodil" is only a
// packet of that size.
var KIND_LOGO_DATA = window.KIND_LOGO_DATA || {};
function kindHtml(kind, junk) {
    if (!junk && KIND_LOGO_DATA[kind]) return '<img class="hc-logo" src="' + KIND_LOGO_DATA[kind] + '" alt="' + esc(kind) + '" title="' + esc(kind) + '">';
    return '<span class="badge bg-secondary">' + esc(kind || "?") + "</span>";
}

function stackHtml(t, nowSec, junk) {
    var key = navKey(t);
    var st = nav[key] || { idx: 0 };
    var total = t.cards;
    var card = st.idx > 0 && st.cards ? st.cards[st.idx] : t.front;
    if (!card) return "";
    var depth = Math.min(3, total - 1 - st.idx);
    var html = '<div class="col-12 col-md-6 col-lg-4' + (junk ? " junk" : "") + '"><div class="stack d' + Math.max(0, depth) + '">';
    html += '<div class="hc' + (st.idx > 0 ? " older" : "") + '" data-key="' + key + '">';
    html += '<div class="hc-top"><span class="hc-name">' + esc(t.name) + "</span>" + kindHtml(t.kind, junk) + "</div>";
    html += cardBody(t, card, nowSec, st.idx === 0);
    html += '<div class="hc-nav">'
        + '<button class="btn btn-outline-secondary btn-sm" data-nav="older" data-key="' + key + '"' + (st.idx >= total - 1 ? " disabled" : "") + ">&#9664;</button>"
        + '<span style="font-size:13px">' + (st.idx + 1) + " / " + total + (st.idx > 0 ? ' · <a href="#" data-nav="latest" data-key="' + key + '">latest</a>' : "") + "</span>"
        + '<button class="btn btn-outline-secondary btn-sm" data-nav="newer" data-key="' + key + '"' + (st.idx === 0 ? " disabled" : "") + ">&#9654;</button>"
        + "</div>";
    html += '<div class="hc-foot">' + t.okCount + " good · " + (t.badCount ? '<span class="bad">' + t.badCount + " rejected</span>" : "0 rejected")
        + ' · <span style="font-family:monospace">' + esc(t.serial) + "</span></div>";
    html += "</div></div></div>";
    return html;
}

function render() {
    if (!last) return;
    var nowSec = last.now + Math.floor((Date.now() - lastFetchMs) / 1000);
    renderClock(nowSec);
    renderWeather(last.weather);
    var byName = function (a, b) { return a.name.localeCompare(b.name, undefined, { sensitivity: "base" }); };
    var real = last.telepathons.filter(function (t) { return t.real && t.cards; }).sort(byName);
    var junk = last.telepathons.filter(function (t) { return !t.real && t.cards; }).sort(byName);
    var junkPackets = junk.reduce(function (s, t) { return s + t.badCount; }, 0);

    document.getElementById("summary").textContent = real.length + " devices, " + junk.length + " junk sources · Annabelle up "
        + fmtDuration(Math.floor(last.uptimeSec + (Date.now() - lastFetchMs) / 1000));
    if (diagOpen) renderDiagnostics();
    document.getElementById("telepathons").innerHTML = real.map(function (t) { return stackHtml(t, nowSec, false); }).join("")
        || '<div class="col-12 text-muted">No device has sent a packet that checked out since Annabelle started.</div>';

    var html = "";
    if (junk.length) {
        html += '<div class="panel pile-head" id="pile-head">'
            + '<div class="d-flex justify-content-between"><strong>Junk</strong><span>' + (junkOpen ? "hide &#9650;" : "show &#9660;") + "</span></div>"
            + '<div style="font-size:13px;color:#6c757d">' + junk.length + " sources, " + junkPackets + " packets that did not check out: "
            + esc(junkNames(junk)) + "</div>";
        if (!junkOpen) {
            html += '<div class="pile-deck">';
            for (var i = 0; i < Math.min(4, junk.length); i++) html += '<div style="top:' + (i * 3) + "px;left:" + (i * 6) + 'px"></div>';
            html += "</div>";
        }
        html += "</div>";
        // Junk rows come without a card since 2026-10-16 (keeps /Telepathons small) - a plain list.
        if (junkOpen) html += '<div class="panel" style="overflow-x:auto"><table class="diag-table" style="white-space:nowrap"><tr><th>Name</th><th>Serial</th><th class="num">Rejected</th>'
            + '<th class="num">Signal</th><th class="num">Last heard</th></tr>' + junk.map(function (t) {
                return "<tr><td>" + esc(t.name) + '</td><td style="font-family:monospace;font-size:12px">' + esc(t.serial) + '</td><td class="num">'
                    + t.badCount + '</td><td class="num">' + t.rssi + ' dBm</td><td class="num">' + fmtAgo(Math.max(0, nowSec - t.lastEpoch)) + "</td></tr>";
            }).join("") + "</table></div>";
    }
    document.getElementById("junk").innerHTML = html;
}

// Diagnostics tab (2026-10-16): the radio statistics Annabelle.ino keeps since boot (RadioStats),
// the last corrupted packets, and each device's packet counts - all from the /Telepathons poll.
var diagOpen = false;

function showTab(diag) {
    diagOpen = diag;
    document.getElementById("tab-devices").classList.toggle("d-none", diag);
    document.getElementById("tab-diag").classList.toggle("d-none", !diag);
    document.getElementById("tab-btn-devices").classList.toggle("active", !diag);
    document.getElementById("tab-btn-diag").classList.toggle("active", diag);
    try { localStorage.setItem("annabelleTab", diag ? "diag" : "devices"); } catch (e) { }
    render();
}

// "12 (4.1/h)" - a count since boot and its hourly rate
function perHour(n, upSec) {
    if (!upSec) return String(n);
    var rate = n * 3600 / upSec;
    return n + ' <span style="color:#6c757d">(' + (rate < 10 ? rate.toFixed(1) : Math.round(rate)) + "/h)</span>";
}
function flag(text, isBad) { return isBad ? '<span class="bad">' + text + "</span>" : text; }
function drow(label, value, help) {
    return '<div class="kv"><span' + (help ? ' title="' + esc(help) + '"' : "") + ">" + label + "</span><span>" + value + "</span></div>";
}
function avgMs(total, n) { return n ? Math.round(total / n) : 0; }

// The device a packet probably came from: the real device whose usual signal is closest (within 8 dB).
function likelySource(rssi) {
    var best = null, bestDiff = 9;
    last.telepathons.forEach(function (t) {
        if (!t.real) return;
        var d = Math.abs(t.rssi - rssi);
        if (d < bestDiff) { bestDiff = d; best = t.name; }
    });
    return best;
}

function renderDiagnostics() {
    var r = last.radio, up = last.uptimeSec;
    var el = document.getElementById("diag-radio");
    if (!r) { el.innerHTML = "<h5>Radio</h5>This Annabelle firmware has no radio statistics."; return; }
    var reads = r.rxPackets + r.noPacket;
    el.innerHTML = "<h5>Radio <small style=\"font-size:13px;color:#6c757d\">since Annabelle started, " + fmtDuration(up) + " ago</small></h5>"
        + drow("Not listening", flag(r.deafPct + "%", r.deafPct >= 10), "Share of the time the radio was not receiving: handling a packet, or sending")
        + drow("Packets received", perHour(r.rxPackets, up))
        + drow("Corrupted packets", perHour(r.noPacket, up) + (reads ? " - " + Math.round(100 * r.noPacket / reads) + "% of all" : ""),
            "Arrived damaged (CRC error) - see Corrupted packets for whose")
        + (r.rxDropped !== undefined ? drow("Dropped, queue full", flag(perHour(r.rxDropped, up), r.rxDropped > 0), "Read from the radio but lost because the main loop was 4 packets behind") : "")
        + (r.dio0Stuck !== undefined ? drow("Radio unstuck", perHour(r.dio0Stuck, up), "The radio's interrupt line was stuck high - cleared by the safety net") : "")
        + "<hr style=\"margin:8px 0\">"
        + drow("Read after", avgMs(r.latencyMsTotal, reads) + " ms avg, max " + r.latencyMsMax + " ms", "Interrupt -> packet read. A second packet in that time replaces the first")
        + drow("Read more than 1 s late", flag(perHour(r.latencyOver1s, up), r.latencyOver1s > 0))
        + drow("Busy per packet", avgMs(r.busyMsTotal, r.rxPackets) + " ms avg, max " + r.busyMsMax + " ms", "Radio not listening while a packet is taken out of it")
        + drow("Busy more than 1 s", flag(perHour(r.busyOver1s, up), r.busyOver1s > 0), "A Daffodil sends its vital signs 1 s after its data - a busy spell over 1 s loses them")
        + "<hr style=\"margin:8px 0\">"
        + drow("Sends", perHour(r.txCount, up) + " - " + (r.txMsTotal / 1000).toFixed(1) + " s, longest " + r.txMsMax + " ms", "Weather forecasts and device commands")
        + drow("Overwritten by a send", flag(perHour(r.txClobbered, up), r.txClobbered > 0), "A received packet not yet read when a send started")
        + (r.lockWaitMsMax !== undefined ? drow("Waited for a send", "max " + r.lockWaitMsMax + " ms" + (r.lockWaitOver1s ? ", " + flag(r.lockWaitOver1s + " over 1 s", true) : ""),
            "A packet arrived while Annabelle was sending - the reader waits until the send is over") : "")
        + (r.loopMsMax !== undefined ? drow("Longest main loop", r.loopMsMax + " ms" + (r.loopOver1s ? ", " + r.loopOver1s + " over 1 s" : ""),
            "Pi serial commands, weather download, web requests - packets are read separately, but the queue fills if this is long") : "")
        + (r.resetReason !== undefined ? "<hr style=\"margin:8px 0\">"
            + drow("Last restart", esc(r.resetReason) + (r.resetStage ? " - during " + esc(r.resetStage) : ""),
                "Why Annabelle last restarted, and what its main loop was doing at that moment")
            + drow("WiFi", (r.wifiReconnects ? flag(r.wifiReconnects + " reconnects", true) : "no drops")
                + (r.wifiDownSecMax ? ", longest gap " + fmtDuration(r.wifiDownSecMax) : ""),
                "Annabelle retries the house WiFi every minute while it is gone, and restarts after 15 minutes without it")
            + (r.lowMemRestarts !== undefined ? drow("Low-memory restarts", r.lowMemRestarts ? flag(r.lowMemRestarts + " since power-on", true) : "none",
                "Annabelle restarts when the largest free block stays under 6 KB for 10 minutes, rather than wait to crash") : "") : "")
        + (r.heapFree !== undefined ? "<hr style=\"margin:8px 0\">"
            + drow("Free memory", Math.round(r.heapFree / 1024) + " KB (lowest " + Math.round(r.heapMin / 1024) + " KB)")
            + drow("Largest free block", flag(Math.round(r.heapBlock / 1024) + " KB", r.heapBlock < 12288),
                "The biggest single free piece of memory - when it gets small, things start to fail") : "")
        + (r.replyPeak !== undefined ? drow("Web replies", "largest " + (r.replyPeak / 1024).toFixed(1) + " of 8 KB, building used "
            + (r.arenaPeak / 1024).toFixed(1) + " of 16 KB" + (r.replyBusy ? ", " + r.replyBusy + " busy" : "")
            + (r.replyTooBig ? ", " + flag(r.replyTooBig + " too big", true) : ""),
            "Page replies are built in fixed memory set aside at start, one at a time - 'busy' means a request came while another was going out (the page just asks again)") : "")
        + (r.webBusy !== undefined ? drow("Web requests", (r.webBusy + r.webLowMemory) ? flag(r.webBusy + " busy, "
            + r.webLowMemory + " low memory", r.webLowMemory > 0) : "none turned away",
            "At most 2 replies go out at once, and none while memory is low, so the page can't run Annabelle out of memory - 'busy' requests are simply asked again by the page") : "");

    // Which step shrinks the largest free block (Annabelle.ino fragBegin/fragEnd, 2026-10-17)
    if (r.frag && r.frag.length) {
        var fr = r.frag.slice().sort(function (a, b) { return a[1] - b[1]; });
        var accounted = fr.reduce(function (s, f) { return s + f[1]; }, 0);
        var ft = '<hr style="margin:8px 0"><div style="font-size:13px;color:#6c757d;margin-bottom:2px">Largest free block: '
            + Math.round(r.fragBootBlock / 1024) + " KB at boot, " + Math.round(r.heapBlock / 1024) + " KB now. Change by step:</div>"
            + '<table class="diag-table"><tr><th>Step</th><th class="num">Change</th><th class="num">Runs</th></tr>';
        fr.forEach(function (f) {
            ft += "<tr><td>" + esc(f[0]) + '</td><td class="num">' + flag((f[1] / 1024).toFixed(1) + " KB", f[1] < -4096) + '</td><td class="num">' + f[2] + "</td></tr>";
        });
        ft += '<tr><td style="color:#6c757d">elsewhere (web, WiFi, LoRa task)</td><td class="num">'
            + ((r.heapBlock - r.fragBootBlock - accounted) / 1024).toFixed(1) + ' KB</td><td></td></tr></table>';
        el.innerHTML += ft;
    }

    var c = document.getElementById("diag-corrupt");
    var list = r.corrupt || [];
    var html = "<h5>Corrupted packets <small style=\"font-size:13px;color:#6c757d\">last " + list.length + "</small></h5>";
    if (!list.length) {
        html += '<div class="ok">None since Annabelle started</div>';
    } else {
        html += '<table class="diag-table"><tr><th>Time</th><th class="num">Signal</th><th class="num">SNR</th><th>Probably from</th></tr>';
        list.forEach(function (p) {
            // The far devices all arrive at about -110 dBm - too close together to tell apart.
            var near = p[1] > -90;
            var src = near ? likelySource(p[1]) : null;
            html += "<tr><td>" + fmtTime(p[0]) + '</td><td class="num">' + p[1] + ' dBm</td><td class="num">' + p[2] + "</td><td>"
                + (near ? flag(src ? esc(src) : "a nearby device", true) : '<span style="color:#6c757d">a far device</span>') + "</td></tr>";
        });
        html += "</table>";
    }
    html += '<div class="diag-note">Signal says whose a packet was: a nearby device arrives strong (about -70 dBm), far ones weak (about -110). '
        + "A strong packet with a low SNR (under ~5) was damaged on the way - overlapping with another transmission, or distorted by the sender.</div>";
    c.innerHTML = html;

    var real = last.telepathons.filter(function (t) { return t.real; })
        .sort(function (a, b) { return a.name.localeCompare(b.name, undefined, { sensitivity: "base" }); });
    var nowSec = last.now + Math.floor((Date.now() - lastFetchMs) / 1000);
    var d = '<h5>Devices</h5><div style="overflow-x:auto"><table class="diag-table" style="white-space:nowrap"><tr><th>Device</th><th class="num">Good</th><th class="num">Failed checksum</th>'
        + '<th class="num">Per hour</th><th class="num">Signal</th><th class="num">SNR</th><th class="num">Last heard</th></tr>';
    real.forEach(function (t) {
        var badPct = t.okCount + t.badCount ? Math.round(100 * t.badCount / (t.okCount + t.badCount)) : 0;
        d += "<tr><td>" + esc(t.name) + '</td><td class="num">' + t.okCount + '</td><td class="num">'
            + flag(t.badCount + (t.badCount ? " (" + badPct + "%)" : ""), badPct >= 10) + '</td><td class="num">'
            + (up ? Math.round(t.okCount * 3600 / up) : "-") + '</td><td class="num">' + t.rssi + ' dBm</td><td class="num">' + t.snr
            + '</td><td class="num">' + fmtAgo(Math.max(0, nowSec - t.lastEpoch)) + "</td></tr>";
    });
    d += '</table></div><div class="diag-note">Counts since Annabelle started. A packet the radio could not decode at all is not '
        + "counted against any device - it shows under Corrupted packets.</div>";
    document.getElementById("diag-devices").innerHTML = d;
    renderRadioHistory();
}

// /RadioHistory: one row per 10 minutes since Annabelle started (up to 12 h). Fetched at most every
// 2 minutes, only while the Diagnostics tab is open.
var radioHistory = null, radioHistoryFetchMs = 0;
function fetchRadioHistory() {
    if (Date.now() - radioHistoryFetchMs < 120000) return;
    radioHistoryFetchMs = Date.now();
    fetch("/RadioHistory", { cache: "no-store" }).then(function (r) {
            if (r.status === 503) { radioHistoryFetchMs = 0; return null; }  // busy - try again next refresh
            return r.json();
        })
        .then(function (h) { if (h) { radioHistory = h; renderRadioHistory(); } })
        .catch(function () { });
}
function renderRadioHistory() {
    var el = document.getElementById("diag-history");
    if (!el) return;
    fetchRadioHistory();
    var h = radioHistory;
    if (!h || !h.rows) { el.innerHTML = "<h5>History</h5>Loading... (not available on older firmware)"; return; }
    var html = '<h5>History <small style="font-size:13px;color:#6c757d">every ' + Math.round(h.periodSec / 60) + " min, newest first</small></h5>";
    if (!h.rows.length) html += "The first row appears 10 minutes after Annabelle starts.";
    else {
        // wider than a phone - scrolls sideways inside the panel
        html += '<div style="overflow-x:auto"><table class="diag-table" style="white-space:nowrap"><tr><th>Time</th><th class="num">Free KB</th><th class="num">Block KB</th><th class="num">Packets</th>'
            + '<th class="num">Corrupted</th><th class="num">Late</th><th class="num">Avg delay</th><th class="num">Loop max</th><th class="num">Send wait</th></tr>';
        h.rows.slice().reverse().forEach(function (x) {
            html += "<tr><td>" + fmtTime(x[0]) + '</td><td class="num">' + Math.round(x[1] / 1024) + '</td><td class="num">'
                + flag(Math.round(x[2] / 1024), x[2] < 12288) + '</td><td class="num">' + x[3] + '</td><td class="num">' + x[4]
                + '</td><td class="num">' + flag(x[5], x[5] > 0) + '</td><td class="num">' + x[6] + ' ms</td><td class="num">'
                + x[7] + ' ms</td><td class="num">' + x[8] + " ms</td></tr>";
        });
        html += "</table></div>";
    }
    el.innerHTML = html;
}

// "FISH ×3, SUMP" - the same garbled name can come with several different serials.
function junkNames(junk) {
    var counts = {}, order = [];
    junk.forEach(function (t) {
        if (!counts[t.name]) { counts[t.name] = 0; order.push(t.name); }
        counts[t.name]++;
    });
    return order.map(function (n) { return counts[n] > 1 ? n + " \u00d7" + counts[n] : n; }).join(", ");
}

// ---------------------------------------------------------------- Weather forecast broadcast

var weatherOpen = false;
var LORA_ERRORS = { 1: "LoRa init failed", 2: "CAD timeout", 3: "channel busy", 4: "TX failed" };
var DOWNLOAD_ERRORS = { 2: "download failed", 3: "no internet" };

// The line under the header: when the last WeatherForecastUpdate went out (Annabelle.ino
// sendWeatherForecast), or in red why it didn't.
function renderWeather(w) {
    var line = document.getElementById("weather-line");
    document.getElementById("weather-send").disabled = !!(w && w.pending);
    if (!w) { line.textContent = ""; return; }
    if (w.pending) {
        line.innerHTML = '<span style="color:#555">Sending weather forecast...</span>';
    } else if (!w.t) {
        line.innerHTML = '<span style="color:#555">No weather forecast sent since Annabelle started</span>';
    } else if (w.lora === 0) {
        line.innerHTML = '<span id="weather-link" title="Show the forecast that went out">Weather forecast sent ' + fmtTime(w.t) + "</span>"
            + (w.retries ? " (" + w.retries + (w.retries === 1 ? " retry)" : " retries)") : "")
            + (DOWNLOAD_ERRORS[w.download] ? ' <span style="color:#ef6c00">- previous forecast, ' + DOWNLOAD_ERRORS[w.download] + "</span>" : "");
    } else {
        var why = w.lora === -1 ? "no forecast data" + (DOWNLOAD_ERRORS[w.download] ? " (" + DOWNLOAD_ERRORS[w.download] + ")" : "")
            : w.lora === 5 ? "channel busy after " + w.retries + " retries"
            : (LORA_ERRORS[w.lora] || "LoRa error " + w.lora);
        line.innerHTML = '<span class="bad">Weather forecast NOT sent ' + fmtTime(w.t) + ": " + esc(why) + "</span>";
    }
    renderWeatherPanel(w);
}

function renderWeatherPanel(w) {
    var panel = document.getElementById("weather-panel");
    var show = weatherOpen && w && w.t && w.lora === 0 && w.forecasts;
    panel.classList.toggle("d-none", !show);
    if (!show) return;
    var days = ["Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"];
    var html = '<h5>Forecast sent ' + fmtTime(w.t) + '</h5><table class="table table-sm" style="font-size:13px;margin:0">'
        + "<tr><th>Time</th><th>Temp</th><th>Cloud</th><th>Humidity</th><th>Rain</th><th>Pressure</th></tr>";
    w.forecasts.forEach(function (f) {
        var d = new Date(f.t * 1000);  // forecast times are real Unix time
        html += "<tr><td>" + (f.t ? days[d.getDay()] + " " + pad(d.getHours()) + ":" + pad(d.getMinutes()) : "-") + "</td>"
            + "<td>" + Number(f.temp).toFixed(1) + " °C</td><td>" + f.cloud + " %</td><td>" + Math.round(f.humidity) + " %</td>"
            + "<td>" + Number(f.rain).toFixed(1) + " mm</td><td>" + Math.round(f.pressure) + " hPa</td></tr>";
    });
    panel.innerHTML = html + "</table>";
}

function sendWeatherNow() {
    var btn = document.getElementById("weather-send");
    btn.disabled = true;
    fetch("/AnnabelleServlet", { method: "POST", body: new URLSearchParams({ formName: "SendWeatherNow" }) })
        .then(function (r) { return r.text(); })
        .then(function (t) {
            if (t !== "Queued") alert(t);
            setTimeout(poll, 1500);  // shows "Sending...", then the outcome a few seconds later
        })
        .catch(function () { btn.disabled = false; alert("Cannot reach Annabelle"); });
}

function renderClock(nowSec) {
    // Phone local time, counted as UTC the same way as Annabelle's.
    var phoneLocal = Math.floor(Date.now() / 1000) - new Date().getTimezoneOffset() * 60;
    var drift = nowSec - phoneLocal;
    document.getElementById("clock").textContent = "Annabelle " + fmtDateTime(nowSec);
    var el = document.getElementById("drift");
    if (Math.abs(drift) > 30) {
        el.textContent = "Annabelle's clock is " + fmtDuration(Math.abs(drift)) + (drift > 0 ? " ahead of" : " behind")
            + " this phone (" + fmtTime(phoneLocal) + ") - set it under Settings.";
        el.classList.remove("d-none");
    } else {
        el.classList.add("d-none");
    }
}

function findStack(key) {
    if (!last) return null;
    for (var i = 0; i < last.telepathons.length; i++) if (navKey(last.telepathons[i]) === key) return last.telepathons[i];
    return null;
}

// Older cards come from /TelepathonHistory, fetched when the user first steps back and kept until
// they return to the front.
function step(key, dir) {
    var t = findStack(key);
    if (!t) return;
    var st = nav[key] || (nav[key] = { idx: 0 });
    if (dir === "latest") { st.idx = 0; st.cards = null; render(); return; }
    var go = function () {
        var max = (st.cards ? st.cards.length : t.cards) - 1;
        st.idx = Math.max(0, Math.min(max, st.idx + (dir === "older" ? 1 : -1)));
        if (st.idx === 0) st.cards = null;
        render();
    };
    if (st.cards || dir !== "older") { go(); return; }
    fetch("/TelepathonHistory?row=" + t.row + "&gen=" + t.gen, { cache: "no-store" })
        .then(function (r) { return r.json(); })
        .then(function (j) { if (j.cards && j.cards.length) { st.cards = j.cards; go(); } })
        .catch(function () { document.getElementById("status").textContent = "Cannot reach Annabelle"; });
}

document.addEventListener("click", function (e) {
    var b = e.target.closest("[data-nav]");
    if (b) { e.preventDefault(); step(b.getAttribute("data-key"), b.getAttribute("data-nav")); return; }
    if (e.target.closest("#pile-head")) { junkOpen = !junkOpen; render(); }
    if (e.target.closest("#weather-link")) { weatherOpen = !weatherOpen; render(); }
    if (e.target.closest("#weather-send")) sendWeatherNow();
    var tb = e.target.closest("[data-tab]");
    if (tb) {
        cardTab[tb.getAttribute("data-name")] = tb.getAttribute("data-tab");
        try { localStorage.setItem("annabelleCardTabs", JSON.stringify(cardTab)); } catch (err) { }
        render();
        return;
    }
    var wb = e.target.closest("[data-wifi]");
    if (wb) sendWifi(wb);
});

// Swipe left for older, right for newer.
var touchX = null, touchKey = null;
document.addEventListener("touchstart", function (e) {
    var c = e.target.closest(".hc");
    touchKey = c ? c.getAttribute("data-key") : null;
    touchX = e.touches[0].clientX;
}, { passive: true });
document.addEventListener("touchend", function (e) {
    if (!touchKey || touchX === null) return;
    var dx = e.changedTouches[0].clientX - touchX;
    if (Math.abs(dx) > 50) step(touchKey, dx < 0 ? "older" : "newer");
    touchKey = touchX = null;
}, { passive: true });

function poll() {
    fetch("/Telepathons", { cache: "no-store" })
        .then(function (r) {
            // 503 = Annabelle is still sending another reply (one at a time, fixed memory) - ask again shortly
            if (r.status === 503) { setTimeout(poll, 1500); return null; }
            return r.json();
        })
        .then(function (j) {
            if (!j) return;
            last = j;
            lastFetchMs = Date.now();
            // Forget browsing state for rows that were reused for another device.
            var live = {};
            j.telepathons.forEach(function (t) { live[navKey(t)] = true; });
            Object.keys(nav).forEach(function (k) { if (!live[k]) delete nav[k]; });
            document.getElementById("status").textContent = "";
            render();
        })
        .catch(function () {
            document.getElementById("status").textContent = "Cannot reach Annabelle - retrying...";
        });
}

poll();
document.getElementById("tab-btn-devices").addEventListener("click", function () { showTab(false); });
document.getElementById("tab-btn-diag").addEventListener("click", function () { showTab(true); });
try { diagOpen = localStorage.getItem("annabelleTab") === "diag"; } catch (e) { }
if (diagOpen) showTab(true);
// Every poll makes Annabelle build an ~8 KB reply: every 30 s, skipped while the page is in the
// background, and once straight away when it comes back. Reload the page for a fresh one sooner.
setInterval(function () { if (!document.hidden) poll(); }, 30000);
document.addEventListener("visibilitychange", function () { if (!document.hidden) poll(); });
// Only the clock ticks between polls - redrawing the cards every second would swallow taps on their buttons.
setInterval(function () {
    if (last) renderClock(last.now + Math.floor((Date.now() - lastFetchMs) / 1000));
}, 1000);
