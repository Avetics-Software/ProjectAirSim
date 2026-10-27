// The "Drone mission" tab of the map editor (actor_editor.html): plan a mission on the
// map the way QGC's plan view does. A mission is a list of steps per drone; the map
// shows them as numbered pins, the server (gcs/mission_plan.py) checks them as the
// ground station will upload them and returns the route to draw.
// Uses the page's globals: map, $, api, r6, colour, setStatus.

const Mission = (() => {
  const ACTIONS = [["", "(no movement: only camera, hold ...)"], ["takeoff", "Take off"],
                   ["goto", "Waypoint"], ["land", "Land"], ["rtl", "Return home"]];
  const ADD = [["goto", "Waypoint"], ["takeoff", "Takeoff"], ["land", "Land"], ["rtl", "Return home"],
               ["gimbal", "Camera angle"], ["hold", "Hold"], ["look_at", "Look at"], ["wait", "Wait"],
               ["photo", "Photo"], ["heading", "Heading"], ["speed", "Speed"], ["track", "Track"]];
  const TOOLS = [["goto", "Waypoint"], ["look_at", "Look-at point"], ["land", "Landing"]];

  let M = null;            // state.mission
  let drone = null;        // selected drone name
  let sel = -1;            // selected step
  let tool = "goto";
  let results = {};        // drone -> preview result
  let timer = null;
  let dirty = false;
  const layer = L.layerGroup();

  const steps = () => (M.data.drones[drone] ||= { steps: [] }).steps;
  const hovers = () => (M.known_models[M.models[drone]] || { hovers: true }).hovers;
  const num = (v) => (v === "" || v === null || v === undefined || isNaN(+v) ? undefined : +v);
  const esc = (t) => String(t).replace(/[&<>]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[c]));

  function action(s) { return ["takeoff", "goto", "land", "rtl"].find((k) => s[k]) || ""; }
  function place(s) {
    if (s.goto) return s.goto;
    if (s.land && typeof s.land === "object") return s.land;
    return null;
  }
  function lastAlt(upto) {
    for (let k = upto - 1; k >= 0; k--) {
      const s = steps()[k];
      if (s.goto && s.goto.alt !== undefined) return s.goto.alt;
      if (s.takeoff) return s.takeoff;
    }
    return 30;
  }

  function summary(s) {
    const parts = [];
    if (s.takeoff) parts.push(`takeoff ${s.takeoff} m`);
    if (s.goto) parts.push(`waypoint${s.goto.alt !== undefined ? ` ${s.goto.alt} m` : ""}`);
    if (s.land) parts.push(typeof s.land === "object" ? `land, heading ${s.land.heading}°` : "land");
    if (s.rtl) parts.push("return home");
    if (s.heading !== undefined) parts.push(`heading ${s.heading}`);
    if (s.gimbal) parts.push("camera " + Object.entries(s.gimbal).map(([k, v]) => `${k} ${v}`).join(", "));
    if (s.look_at !== undefined) parts.push(s.look_at && s.look_at !== "none" ? "look at" : "look-at off");
    if (s.hold) parts.push(`hold ${s.hold} s`);
    if (s.wait) parts.push(`wait ${s.wait} s`);
    if (s.photo) parts.push("photo");
    if (s.speed) parts.push(`speed ${s.speed}`);
    if (s.track !== undefined) parts.push(`track ${typeof s.track === "object" ? (s.track.target || "on") : s.track}`);
    return parts.join(" · ") || "(empty step)";
  }

  // --- Edits ----------------------------------------------------------------------
  function touch() {
    dirty = true;
    render();
    draw();
    schedule();
  }

  function addStep(kind, at) {
    const list = steps();
    const s = {};
    const alt = lastAlt(list.length);
    if (kind === "goto") s.goto = { lat: 0, lon: 0 };
    else if (kind === "takeoff") s.takeoff = alt;
    else if (kind === "land") s.land = true;
    else if (kind === "rtl") s.rtl = true;
    else if (kind === "gimbal") s.gimbal = { pitch: -45, yaw: 0 };
    else if (kind === "hold") s.hold = 5;
    else if (kind === "look_at") s.look_at = "none";
    else if (kind === "wait") s.wait = 5;
    else if (kind === "photo") s.photo = true;
    else if (kind === "heading") s.heading = 0;
    else if (kind === "speed") s.speed = 6;
    else if (kind === "track") s.track = "on";
    // A takeoff leads the list; a goto without a place yet takes the map centre
    if (kind === "goto") { const c = map.getCenter(); Object.assign(s.goto, { lat: r6(c.lat), lon: r6(c.lng), alt }); }
    const index = kind === "takeoff" ? 0 : (at ?? list.length);
    list.splice(index, 0, s);
    sel = index;
    touch();
  }

  function mapClick(e) {
    if (!M) return;
    const lat = r6(e.latlng.lat), lon = r6(e.latlng.lng);
    const list = steps();
    const alt = lastAlt(list.length);
    const ended = list.some((s) => s.land || s.rtl);
    if (ended) return setStatus("The mission ends at its land or return step; remove that first, or select a step and drag its pin.", "err");
    if (tool === "goto") {
      list.push({ goto: { lat, lon, alt } });
    } else if (tool === "look_at") {
      list.push({ look_at: { lat, lon } });
    } else if (hovers()) {
      list.push({ goto: { lat, lon, alt } }, { land: true });
    } else {
      // A fixed-wing lands on a runway: heading along the last leg if there is one
      const prev = [...list].reverse().find((s) => s.goto);
      const heading = prev ? Math.round(bearing(prev.goto, { lat, lon })) : 0;
      list.push({ land: { lat, lon, heading, approach_m: 400, approach_alt: 30 } });
    }
    sel = list.length - (tool === "land" && hovers() ? 2 : 1);
    touch();
  }

  function bearing(a, b) {
    const dy = b.lat - a.lat, dx = (b.lon - a.lon) * Math.cos(a.lat * Math.PI / 180);
    return (Math.atan2(dx, dy) * 180 / Math.PI + 360) % 360;
  }

  // --- Panel -----------------------------------------------------------------------
  function field(label, id, value, attrs = "") {
    return `<div><label>${label}</label><input id="${id}" value="${value ?? ""}" ${attrs}></div>`;
  }

  function render() {
    $("m-drone").innerHTML = M.drones.map((n) => `<option ${n === drone ? "selected" : ""}>${n}</option>`).join("");
    $("m-model").innerHTML = Object.entries(M.known_models)
      .map(([k, v]) => `<option value="${k}" ${k === M.models[drone] ? "selected" : ""}>${esc(v.label)}</option>`).join("");
    const r = results[drone];
    const ul = $("m-steps");
    ul.innerHTML = "";
    steps().forEach((s, k) => {
      const li = document.createElement("li");
      li.className = (k === sel ? "sel " : "") + (r && r.error_step === k + 1 ? "bad" : "");
      li.innerHTML = `<span class="num">${k + 1}</span><span>${esc(summary(s))}</span>
        <span class="mv"><button data-a="up" title="Move up">↑</button><button data-a="down" title="Move down">↓</button>
        <button data-a="del" class="danger" title="Remove">✕</button></span>`;
      li.onclick = (e) => {
        const a = e.target.dataset && e.target.dataset.a;
        if (a === "del") { steps().splice(k, 1); sel = Math.min(sel, steps().length - 1); return touch(); }
        if (a === "up" && k > 0) { const l = steps(); [l[k - 1], l[k]] = [l[k], l[k - 1]]; sel = k - 1; return touch(); }
        if (a === "down" && k < steps().length - 1) { const l = steps(); [l[k + 1], l[k]] = [l[k], l[k + 1]]; sel = k + 1; return touch(); }
        sel = k; render(); draw();
      };
      ul.appendChild(li);
    });
    if (!steps().length) ul.innerHTML = `<li class="muted" style="cursor:default">No steps yet. Click the map to add waypoints, or add a takeoff.</li>`;
    renderStep();
    renderResult();
    $("m-tools").innerHTML = TOOLS.map(([v, t]) =>
      `<label class="tool"><input type="radio" name="m-tool" value="${v}" ${v === tool ? "checked" : ""}> ${t}</label>`).join("");
    document.querySelectorAll("input[name=m-tool]").forEach((el) => { el.onchange = () => { tool = el.value; }; });
  }

  function renderStep() {
    const box = $("m-step");
    const s = steps()[sel];
    box.hidden = !s;
    if (!s) return;
    const act = action(s);
    const p = place(s) || {};
    const fw = !hovers();
    const g = s.gimbal || {};
    const gimbalYawKey = g.heading !== undefined ? "heading" : "yaw";
    const lookAt = s.look_at && s.look_at !== "none" ? s.look_at : null;
    const runway = s.land && typeof s.land === "object";
    box.innerHTML = `
      <div class="grid">
        <div style="grid-column:1/3"><label>Step ${sel + 1}: movement</label>
          <select id="s-action">${ACTIONS.map(([v, t]) => `<option value="${v}" ${v === act ? "selected" : ""}>${t}</option>`).join("")}</select></div>
        ${act === "takeoff" ? field("Height (m)", "s-takeoff", s.takeoff, 'type="number" min="1"') : ""}
        ${act === "goto" ? field("Latitude", "s-lat", p.lat, 'type="number" step="0.000001"') + field("Longitude", "s-lon", p.lon, 'type="number" step="0.000001"')
          + field("Height (m)", "s-alt", p.alt, 'type="number" min="0" placeholder="keep the last"') : ""}
        ${act === "land" ? `<div style="grid-column:1/3"><label class="chk"><input type="checkbox" id="s-runway" ${runway ? "checked" : ""}>
            ${fw ? "Runway landing (a fixed-wing needs one)" : "Land at a chosen spot (otherwise where it is)"}</label></div>` : ""}
        ${runway ? field("Latitude", "s-lat", p.lat, 'type="number" step="0.000001"') + field("Longitude", "s-lon", p.lon, 'type="number" step="0.000001"')
          + field("Heading (deg)", "s-lheading", p.heading, 'type="number"') + field("Approach (m)", "s-approach", p.approach_m, 'type="number" min="1" placeholder="default"')
          + field("Approach height (m)", "s-approach-alt", p.approach_alt, 'type="number" min="1" placeholder="default"') : ""}
      </div>
      <hr>
      <div class="grid">
        ${fw ? "" : field("Heading (deg)", "s-heading", s.heading ?? "", 'placeholder="keep"')}
        ${field("Hold (s)", "s-hold", s.hold ?? "", 'type="number" min="0"' + (fw ? ' placeholder="circles"' : ""))}
        ${field("Wait (s)", "s-wait", s.wait ?? "", 'type="number" min="0"')}
        ${field("Speed (m/s)", "s-speed", s.speed ?? "", 'type="number" min="0.1" placeholder="mission"')}
        ${field("Camera pitch", "s-gpitch", g.pitch ?? "", 'type="number" min="-90" max="90" placeholder="keep"')}
        <div><label>Camera yaw</label>
          <div style="display:flex;gap:4px"><select id="s-gkind" style="width:auto"><option value="yaw" ${gimbalYawKey === "yaw" ? "selected" : ""}>from nose</option>
          <option value="heading" ${gimbalYawKey === "heading" ? "selected" : ""}>compass</option></select>
          <input id="s-gyaw" type="number" value="${g[gimbalYawKey] ?? ""}" placeholder="keep"></div></div>
        <div style="grid-column:1/3"><label class="chk"><input type="checkbox" id="s-photo" ${s.photo ? "checked" : ""}> Take a photo here</label></div>
        <div style="grid-column:1/3"><label>Track</label><input id="s-track" value="${s.track === undefined ? "" : (typeof s.track === "object" ? esc(s.track.target || "on") : esc(s.track))}"
          placeholder="blank: none · on · off · a target name"></div>
        <div style="grid-column:1/3"><label>Look at</label>
          <div style="display:flex;gap:4px;align-items:center">
            <span class="muted" style="flex:1">${lookAt ? `${lookAt.lat}, ${lookAt.lon}` : (s.look_at === "none" ? "off (stops looking at a point)" : "unchanged")}</span>
            <button id="s-look-pick">Pick on map</button><button id="s-look-off">Off</button><button id="s-look-clear">Unset</button></div></div>
      </div>`;
    wireStep(s);
  }

  function wireStep(s) {
    const on = (id, fn) => { const el = $(id); if (el) el.addEventListener("change", () => { fn(el); touch(); }); };
    $("s-action").addEventListener("change", () => {
      const v = $("s-action").value, here = place(s) || map.getCenter();
      const at = { lat: r6(here.lat), lon: r6(here.lng ?? here.lon) };
      for (const k of ["takeoff", "goto", "land", "rtl"]) delete s[k];
      if (v === "takeoff") s.takeoff = 30;
      else if (v === "goto") s.goto = { ...at, alt: lastAlt(sel) };
      else if (v === "land") s.land = hovers() ? true : { ...at, heading: 0 };
      else if (v === "rtl") s.rtl = true;
      touch();
    });
    on("s-takeoff", (el) => { s.takeoff = num(el.value) ?? 30; });
    const setPlace = (key, val) => { const t = s.goto || s.land; if (val === undefined) delete t[key]; else t[key] = val; };
    on("s-lat", (el) => setPlace("lat", num(el.value)));
    on("s-lon", (el) => setPlace("lon", num(el.value)));
    on("s-alt", (el) => setPlace("alt", num(el.value)));
    on("s-runway", (el) => {
      if (el.checked) { const here = map.getCenter(); s.land = { lat: r6(here.lat), lon: r6(here.lng), heading: 0 }; } else s.land = true;
    });
    on("s-lheading", (el) => setPlace("heading", num(el.value) ?? 0));
    on("s-approach", (el) => setPlace("approach_m", num(el.value)));
    on("s-approach-alt", (el) => setPlace("approach_alt", num(el.value)));
    const opt = (key) => (el) => { const v = num(el.value); if (v === undefined) delete s[key]; else s[key] = v; };
    on("s-heading", opt("heading")); on("s-hold", opt("hold")); on("s-wait", opt("wait")); on("s-speed", opt("speed"));
    const gimbal = () => {
      const pitch = num($("s-gpitch").value), yaw = num($("s-gyaw").value), kind = $("s-gkind").value;
      const g = {};
      if (pitch !== undefined) g.pitch = pitch;
      if (yaw !== undefined) g[kind] = yaw;
      if (Object.keys(g).length) s.gimbal = g; else delete s.gimbal;
    };
    on("s-gpitch", gimbal); on("s-gyaw", gimbal); on("s-gkind", gimbal);
    on("s-photo", (el) => { if (el.checked) s.photo = true; else delete s.photo; });
    on("s-track", (el) => {
      const v = el.value.trim();
      if (!v) delete s.track; else if (v === "on") s.track = "on"; else if (v === "off") s.track = "off"; else s.track = v;
    });
    $("s-look-pick").onclick = () => { pick = (ll) => { s.look_at = { lat: r6(ll.lat), lon: r6(ll.lng) }; touch(); }; setStatus("Click the map for the look-at point."); };
    $("s-look-off").onclick = () => { s.look_at = "none"; touch(); };
    $("s-look-clear").onclick = () => { delete s.look_at; touch(); };
  }
  let pick = null;

  function renderResult() {
    const r = results[drone];
    const box = $("m-result");
    if (!r) { box.textContent = ""; return; }
    if (r.error) { box.className = "err"; box.textContent = r.error; return; }
    box.className = "ok";
    const t = r.time_s ? `, about ${Math.floor(r.time_s / 60)}:${String(r.time_s % 60).padStart(2, "0")}` : "";
    box.textContent = r.steps ? `${r.steps} steps, ${r.items} mission items, route ${r.length_m} m${t}, up to ${Math.round(r.max_alt)} m` : "";
  }

  // --- Map ---------------------------------------------------------------------------
  function pinFor(latlng, label, col, onMove, extra = "") {
    const icon = L.divIcon({ className: "", iconSize: [26, 26], iconAnchor: [13, 13],
      html: `<div class="pin ${extra}" style="background:${col};width:26px;height:26px">${label}</div>` });
    const m = L.marker(latlng, { icon, draggable: true }).addTo(layer);
    m.on("dragend", () => { const ll = m.getLatLng(); onMove(r6(ll.lat), r6(ll.lng)); touch(); });
    return m;
  }

  function draw() {
    layer.clearLayers();
    M.drones.forEach((name, di) => {
      const r = results[name];
      const chosen = name === drone, col = colour(di + 3);
      if (r && r.route && r.route.length) {
        L.polyline([[M.origin[0], M.origin[1]], ...r.route], { color: col, weight: chosen ? 4 : 2, opacity: chosen ? 0.95 : 0.45,
          dashArray: chosen ? null : "4 6", interactive: false }).addTo(layer);
      }
      if (!chosen) return;
      const list = (M.data.drones[name] || {}).steps || [];
      list.forEach((s, k) => {
        const p = place(s), isSel = k === sel;
        if (p && p.lat !== undefined) {
          const mk = pinFor([p.lat, p.lon], k + 1, col, (lat, lon) => { p.lat = lat; p.lon = lon; sel = k; },
                            (isSel ? "selpin " : "") + (s.land ? "landpin" : ""));
          mk.on("click", () => { sel = k; render(); draw(); });
        }
        if (s.look_at && s.look_at !== "none") {
          const mk = L.marker([s.look_at.lat, s.look_at.lon], { draggable: true, icon: L.divIcon({ className: "", iconSize: [22, 22], iconAnchor: [11, 11],
            html: `<div class="pin eye" title="Look at (step ${k + 1})">◎${k + 1}</div>` }) }).addTo(layer);
          mk.on("dragend", () => { const ll = mk.getLatLng(); s.look_at = { lat: r6(ll.lat), lon: r6(ll.lng) }; touch(); });
        }
      });
    });
  }

  // --- Preview and save --------------------------------------------------------------
  function schedule(delay = 300) {
    clearTimeout(timer);
    timer = setTimeout(check, delay);
  }

  async function check() {
    const res = await api("/api/mission/preview", { data: M.data, models: M.models });
    if (res.error) { setStatus(res.error, "err"); return; }
    results = res.drones;
    render();
    draw();
  }

  async function save(finish) {
    const res = await api("/api/mission/save", { data: M.data, models: M.models, finish });
    if (res.error) { setStatus(`Not saved: ${res.error}`, "err"); return false; }
    dirty = false;
    const msg = `Saved ${res.saved}\nIn the sim's console: mission load ${M.data.mission}`;
    if (res.problems.length) setStatus(`${msg}\nThe ground station will refuse:\n${res.problems.join("\n")}`, "err");
    else setStatus(msg, "ok");
    return true;
  }

  function init(mission) {
    M = mission;
    drone = M.drones[0];
    layer.addTo(map);
    $("m-file").textContent = M.path + (M.exists ? "" : "  (new: created when you save)");
    $("m-add").innerHTML = ADD.map(([v, t]) => `<option value="${v}">${t}</option>`).join("");
    $("m-add-btn").onclick = () => addStep($("m-add").value, sel >= 0 ? sel + 1 : undefined);
    $("m-drone").onchange = () => { drone = $("m-drone").value; sel = -1; render(); draw(); };
    $("m-model").onchange = () => { M.models[drone] = $("m-model").value; touch(); };
    $("m-name").value = M.data.mission || "";
    $("m-name").onchange = () => { M.data.mission = $("m-name").value.trim(); dirty = true; schedule(); };
    $("m-origin").value = M.data.origin || "start";
    $("m-origin").onchange = () => { M.data.origin = $("m-origin").value; dirty = true; };
    $("m-speed").value = M.data.speed ?? "";
    $("m-speed").onchange = () => { const v = num($("m-speed").value); if (v === undefined) delete M.data.speed; else M.data.speed = v; touch(); };
    $("m-record").checked = !!M.data.record;
    $("m-record").onchange = () => { if ($("m-record").checked) M.data.record = true; else delete M.data.record; dirty = true; };
    $("m-actors").value = M.data.actors || "";
    $("m-actors").onchange = () => { const v = $("m-actors").value.trim(); if (v) M.data.actors = v; else delete M.data.actors; dirty = true; };
    render();
    draw();
    schedule(0);
    const first = steps().find((s) => place(s) && place(s).lat !== undefined);
    return first ? [place(first).lat, place(first).lon] : null;
  }

  return {
    init, save, draw, mapClick: (e) => { if (pick) { const f = pick; pick = null; f(e.latlng); } else mapClick(e); },
    show(on) { if (!M) return; if (on) { layer.addTo(map); render(); draw(); } else map.removeLayer(layer); },
    get dirty() { return dirty; },
    get active() { return !!M; },
  };
})();
