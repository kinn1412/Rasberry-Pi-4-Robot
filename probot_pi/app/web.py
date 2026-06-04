"""Flask dashboard: live telemetry + control + tuning + demo scenarios.

Transport is Server-Sent Events (telemetry stream) + REST POST (commands/tuning)
— no socket.io client lib, so it works on an isolated robot WiFi with no internet
and nothing to vendor. The front end (below) is plain JS with canvas strip charts.

The 100 Hz supervisor loop runs in its own thread and writes snapshots into the
shared ControlState; this server only reads them. Commands/tuning flow the other
way through ControlState, which the loop reads each tick.
"""
import json
import time

from flask import Flask, Response, jsonify, request

from probot_pi.bsp import params as P
from probot_pi.app import scenarios


def make_app(control):
    app = Flask(__name__)

    @app.route("/")
    def index():
        return INDEX_HTML

    @app.route("/api/stream")
    def stream():
        def gen():
            while True:
                payload = json.dumps(control.get_snapshot())
                yield f"data: {payload}\n\n"
                time.sleep(1.0 / 15.0)
        return Response(gen(), mimetype="text/event-stream")

    @app.route("/api/scenarios")
    def list_scenarios():
        return jsonify(scenarios.names())

    @app.route("/api/command", methods=["POST"])
    def set_command():
        d = request.get_json(force=True)
        control.set_command(v=d.get("v"), w=d.get("w"))
        return jsonify(ok=True)

    @app.route("/api/mode", methods=["POST"])
    def set_mode():
        m = request.get_json(force=True).get("mode")
        control.set_mode({"idle": P.MODE_IDLE, "run": P.MODE_RUN, "estop": P.MODE_ESTOP}[m])
        return jsonify(ok=True)

    @app.route("/api/estop", methods=["POST"])
    def estop():
        control.estop()
        return jsonify(ok=True)

    @app.route("/api/fuzzy", methods=["POST"])
    def set_fuzzy():
        control.set_fuzzy(bool(request.get_json(force=True).get("enabled")))
        return jsonify(ok=True)

    @app.route("/api/tuning", methods=["POST"])
    def set_tuning():
        control.set_tuning(**request.get_json(force=True))
        return jsonify(ok=True)

    @app.route("/api/scenario", methods=["POST"])
    def run_scenario():
        name = request.get_json(force=True).get("name")
        try:
            control.start_scenario(scenarios.make(name))
        except KeyError:
            return jsonify(ok=False, error="unknown scenario"), 400
        return jsonify(ok=True)

    @app.route("/api/stop", methods=["POST"])
    def stop():
        control.stop()
        return jsonify(ok=True)

    return app


INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>probot dashboard</title>
<style>
  :root{--bg:#0e1116;--panel:#171b22;--mut:#8b949e;--fg:#e6edf3;--acc:#2f81f7;--ok:#3fb950;--warn:#d29922;--err:#f85149}
  *{box-sizing:border-box} body{margin:0;background:var(--bg);color:var(--fg);font:13px/1.4 ui-monospace,Menlo,Consolas,monospace}
  h1{font-size:15px;margin:0} h2{font-size:12px;color:var(--mut);margin:0 0 8px;text-transform:uppercase;letter-spacing:.06em}
  .wrap{max-width:1100px;margin:0 auto;padding:14px}
  .bar{display:flex;flex-wrap:wrap;gap:10px;align-items:center;justify-content:space-between;margin-bottom:12px}
  .stats{display:flex;flex-wrap:wrap;gap:6px}
  .chip{background:var(--panel);border:1px solid #232a33;border-radius:6px;padding:4px 8px}
  .chip b{color:var(--mut);font-weight:400}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
  .panel{background:var(--panel);border:1px solid #232a33;border-radius:8px;padding:12px}
  button{background:#21262d;color:var(--fg);border:1px solid #30363d;border-radius:6px;padding:7px 12px;cursor:pointer;font:inherit}
  button:hover{border-color:var(--acc)}
  button.on{background:var(--ok);color:#04130a;border-color:var(--ok)}
  button.go{background:var(--acc);color:#021024;border-color:var(--acc)}
  button.stop{background:#3d2222;border-color:#5a2a2a}
  button.estop{background:var(--err);color:#1a0303;border-color:var(--err);font-weight:700}
  .row{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin-bottom:8px}
  .slider{display:flex;align-items:center;gap:8px;margin:6px 0}
  .slider label{width:120px;color:var(--mut)} .slider input{flex:1} .slider .v{width:62px;text-align:right}
  canvas{width:100%;height:130px;display:block;background:#0b0e13;border-radius:6px}
  .leg{display:flex;gap:12px;flex-wrap:wrap;color:var(--mut);margin:6px 0 2px;font-size:11px}
  .leg i{display:inline-block;width:10px;height:3px;vertical-align:middle;margin-right:4px}
  .badge{padding:2px 8px;border-radius:6px;font-weight:700}
  .scn{color:var(--mut)} .full{grid-column:1/3}
</style>
</head>
<body>
<div class="wrap">
  <div class="bar">
    <h1>probot · fuzzy supervisor</h1>
    <div class="stats">
      <span class="chip">mode <b id="mode" class="badge">—</b></span>
      <span class="chip"><b>link</b> <span id="link">—</span></span>
      <span class="chip"><b>rate</b> <span id="rate">—</span>Hz</span>
      <span class="chip"><b>over</b> <span id="over">—</span></span>
      <span class="chip"><b>fault</b> <span id="fault">—</span></span>
      <span class="chip"><b>vbat</b> <span id="vbat">—</span>V</span>
      <span class="chip"><b>rx/bad/drop</b> <span id="rxs">—</span></span>
    </div>
  </div>

  <div class="grid">
    <div class="panel">
      <h2>Control</h2>
      <div class="row">
        <button class="go" onclick="post('/api/mode',{mode:'run'})">START</button>
        <button class="stop" onclick="post('/api/stop',{})">STOP</button>
        <button class="estop" onclick="post('/api/estop',{})">E-STOP</button>
        <button id="fz" onclick="toggleFuzzy()">FUZZY —</button>
      </div>
      <div class="slider"><label>v (m/s)</label><input id="vS" type="range" min="-0.5" max="0.5" step="0.01" value="0" oninput="cmd()"><span class="v" id="vV">0.00</span></div>
      <div class="slider"><label>w (rad/s)</label><input id="wS" type="range" min="-3" max="3" step="0.05" value="0" oninput="cmd()"><span class="v" id="wV">0.00</span></div>
      <div class="row"><button onclick="zero()">center v,w → 0</button></div>
    </div>

    <div class="panel">
      <h2>Demo scenarios</h2>
      <div class="row" id="scnBtns"></div>
      <div class="scn">running: <b id="scnLabel">—</b> <span id="scnPct"></span></div>
      <div style="height:8px;background:#0b0e13;border-radius:5px;margin-top:8px;overflow:hidden"><div id="scnBar" style="height:100%;width:0;background:var(--acc)"></div></div>
    </div>

    <div class="panel full">
      <h2>Live tuning (no LUT rebuild)</h2>
      <div class="slider"><label>k_yaw</label><input id="t_k_yaw" type="range" min="0" max="2" step="0.05" value="1" oninput="tune()"><span class="v" id="t_k_yaw_v">1.00</span></div>
      <div class="slider"><label>k_trac</label><input id="t_k_trac" type="range" min="0" max="2" step="0.05" value="1" oninput="tune()"><span class="v" id="t_k_trac_v">1.00</span></div>
      <div class="slider"><label>slip_scale (°/s)</label><input id="t_slip_scale_dps" type="range" min="10" max="120" step="1" value="60" oninput="tune()"><span class="v" id="t_slip_scale_dps_v">60</span></div>
      <div class="slider"><label>slip_expect</label><input id="t_slip_expect_gain" type="range" min="0" max="1" step="0.01" value="0" oninput="tune()"><span class="v" id="t_slip_expect_gain_v">0.00</span></div>
    </div>

    <div class="panel full">
      <h2>Wheel speed (rad/s)</h2>
      <div class="leg"><span><i style="background:#3fb950"></i>meas L</span><span><i style="background:#58a6ff"></i>meas R</span><span><i style="background:#9e6a03"></i>ref L</span><span><i style="background:#bc8cff"></i>ref R</span></div>
      <canvas id="cWheel"></canvas>
    </div>
    <div class="panel">
      <h2>Heading</h2>
      <div class="leg"><span><i style="background:#f85149"></i>e_psi (°)</span><span><i style="background:#d29922"></i>r_err (°/s)</span></div>
      <canvas id="cHead"></canvas>
    </div>
    <div class="panel">
      <h2>Fuzzy outputs</h2>
      <div class="leg"><span><i style="background:#3fb950"></i>λ</span><span><i style="background:#f85149"></i>Δω_yaw</span><span><i style="background:#d29922"></i>σ_err</span></div>
      <canvas id="cFuzz"></canvas>
    </div>
  </div>
</div>

<script>
const N=300;
function $(id){return document.getElementById(id);}
function fmt(x,d){return (x===undefined||x===null)?'—':Number(x).toFixed(d);}
function post(url,obj){return fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(obj)});}

class Chart{
  constructor(cv,series){this.cv=cv;this.ctx=cv.getContext('2d');this.series=series;this.data=series.map(()=>[]);this.fix();
    addEventListener('resize',()=>this.fix());}
  fix(){const r=this.cv.getBoundingClientRect();this.cv.width=r.width*devicePixelRatio;this.cv.height=r.height*devicePixelRatio;
    this.ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);this.W=r.width;this.H=r.height;}
  push(vals){for(let i=0;i<this.series.length;i++){const a=this.data[i];a.push(vals[i]);if(a.length>N)a.shift();}this.draw();}
  draw(){const c=this.ctx,W=this.W,H=this.H;c.clearRect(0,0,W,H);
    let mn=Infinity,mx=-Infinity;for(const a of this.data)for(const v of a){if(v==null||isNaN(v))continue;if(v<mn)mn=v;if(v>mx)mx=v;}
    if(mn===Infinity){mn=-1;mx=1;} if(mx-mn<1e-6){mn-=1;mx+=1;} const pad=(mx-mn)*0.1;mn-=pad;mx+=pad;
    c.strokeStyle='#222a33';c.lineWidth=1;c.beginPath();
    if(mn<0&&mx>0){const y=H-(0-mn)/(mx-mn)*H;c.moveTo(0,y);c.lineTo(W,y);} c.stroke();
    c.fillStyle='#6b7280';c.font='10px monospace';c.fillText(mx.toFixed(2),3,11);c.fillText(mn.toFixed(2),3,H-3);
    for(let i=0;i<this.series.length;i++){const a=this.data[i];c.strokeStyle=this.series[i];c.lineWidth=1.5;c.beginPath();
      for(let j=0;j<a.length;j++){const v=a[j];if(v==null||isNaN(v))continue;const x=a.length<2?0:j/(N-1)*W;const y=H-(v-mn)/(mx-mn)*H;
        j===0?c.moveTo(x,y):c.lineTo(x,y);} c.stroke();}}
}
const wheel=new Chart($('cWheel'),['#3fb950','#58a6ff','#9e6a03','#bc8cff']);
const head =new Chart($('cHead'),['#f85149','#d29922']);
const fuzz =new Chart($('cFuzz'),['#3fb950','#f85149','#d29922']);

// build scenario buttons
fetch('/api/scenarios').then(r=>r.json()).then(list=>{const c=$('scnBtns');
  list.forEach(n=>{const b=document.createElement('button');b.textContent=n;b.onclick=()=>post('/api/scenario',{name:n});c.appendChild(b);});});

let cmdT=0;
function cmd(){const v=parseFloat($('vS').value),w=parseFloat($('wS').value);$('vV').textContent=v.toFixed(2);$('wV').textContent=w.toFixed(2);
  const now=Date.now();if(now-cmdT<40)return;cmdT=now;post('/api/command',{v:v,w:w});}
function zero(){$('vS').value=0;$('wS').value=0;cmd();}
let fuzzyOn=true;
function toggleFuzzy(){fuzzyOn=!fuzzyOn;post('/api/fuzzy',{enabled:fuzzyOn});}
let tuneT=0;
function tune(){const ids=['k_yaw','k_trac','slip_scale_dps','slip_expect_gain'];const o={};
  ids.forEach(k=>{const el=$('t_'+k);o[k]=parseFloat(el.value);const lbl=$('t_'+k+'_v');lbl.textContent=(k==='slip_scale_dps')?el.value:parseFloat(el.value).toFixed(2);});
  const now=Date.now();if(now-tuneT<60)return;tuneT=now;post('/api/tuning',o);}

let userTuning=false;
document.querySelectorAll('input[type=range]').forEach(el=>{el.addEventListener('pointerdown',()=>userTuning=true);el.addEventListener('pointerup',()=>setTimeout(()=>userTuning=false,400));});

const es=new EventSource('/api/stream');
es.onmessage=ev=>{const s=JSON.parse(ev.data);
  const mb=$('mode');mb.textContent=s.mode||'—';mb.style.background=s.mode==='RUN'?'#3fb950':(s.mode==='ESTOP'?'#f85149':'#30363d');mb.style.color=s.mode==='IDLE'?'#e6edf3':'#04130a';
  const lk=$('link');lk.textContent=s.link_ok?'OK':'STALE';lk.style.color=s.link_ok?'#3fb950':'#f85149';
  $('rate').textContent=fmt(s.rate,0);$('over').textContent=s.overruns??'—';
  const f=(s.fault_names&&s.fault_names.length)?s.fault_names.join(','):'none';const fe=$('fault');fe.textContent=f;fe.style.color=(f==='none')?'#8b949e':'#f85149';
  $('vbat').textContent=fmt(s.vbat,1);$('rxs').textContent=`${s.rx_frames??0}/${s.rx_bad??0}/${s.drops??0}`;
  $('fz').textContent='FUZZY '+(s.fuzzy_enabled?'ON':'OFF');$('fz').className=s.fuzzy_enabled?'on':'';fuzzyOn=s.fuzzy_enabled;
  $('scnLabel').textContent=s.scenario||'—';
  const pct=s.scenario_progress==null?0:Math.round(s.scenario_progress*100);$('scnPct').textContent=s.scenario?`(${pct}%)`:'';$('scnBar').style.width=pct+'%';
  if(!userTuning&&s.tuning){for(const k in s.tuning){const el=$('t_'+k);if(el){el.value=s.tuning[k];const lbl=$('t_'+k+'_v');if(lbl)lbl.textContent=(k==='slip_scale_dps')?Math.round(s.tuning[k]):Number(s.tuning[k]).toFixed(2);}}}
  wheel.push([s.omega_meas_l,s.omega_meas_r,s.omega_ref_l,s.omega_ref_r]);
  head.push([s.e_psi_deg,s.r_err_dps]);
  fuzz.push([s.lam,s.dw_yaw,s.sigma_err]);
};
es.onerror=()=>{$('link').textContent='no server';};
</script>
</body>
</html>
"""
