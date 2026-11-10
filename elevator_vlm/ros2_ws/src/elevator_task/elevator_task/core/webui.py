"""Web UI. 標準ライブラリの HTTP サーバだけで動く.

  GET  /                 画面
  GET  /api/state?since= 状態と記録（JSON）
  GET  /stream.mjpg      カメラ映像（指差した点と判定の結果を重ねる）
  GET  /snapshot.jpg     カメラ画像1枚
  GET  /extra/<名前>.jpg 追加のカメラ（外から見る固定カメラなど）の画像1枚
  POST /api/plan         {"instructions": "1行に1指示"}   指示文から計画を作る
  POST /api/set_plan     {"plan": {...}}                  計画の JSON を直接読み込む
  POST /api/execute      表示中の計画を承認して実行する
  POST /api/stop         停止
  POST /api/reset        停止を解除
  POST /api/trigger      {"name": "..."}                  設定で追加したボタン
"""
from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional, Tuple
from urllib.parse import parse_qs, urlparse

from .runtime import TaskRuntime

PAGE = r"""<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>画像と言語の指示によるタスク実行</title>
<style>
:root{--paper:#F2F4F6;--sheet:#fff;--ink:#18222D;--steel:#56626E;--line:#C9D1D9;--panel:#1B232C;--lamp:#F2A900;--vlm:#3949AB;--ok:#1B7F4B;--ng:#B3261E}
*{box-sizing:border-box}
body{margin:0;background:var(--paper);color:var(--ink);font-family:"BIZ UDPGothic","Hiragino Sans","Yu Gothic UI","Meiryo","Noto Sans JP",sans-serif;font-size:15px;line-height:1.6}
header{background:var(--panel);color:#fff;padding:10px 16px;display:flex;flex-wrap:wrap;gap:6px 16px;align-items:baseline}
header h1{font-size:1.05rem;margin:0}
header span{font-size:.82rem;color:#C9D1D9}
main{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1fr);gap:14px;padding:14px;max-width:1280px;margin:0 auto}
@media(max-width:900px){main{grid-template-columns:minmax(0,1fr)}}
section{background:var(--sheet);border:1px solid var(--line);border-radius:6px;padding:12px 14px;margin:0 0 14px}
h2{font-size:.95rem;margin:0 0 8px;color:var(--vlm)}
img#cam{width:100%;height:auto;background:#000;border-radius:4px;display:block;aspect-ratio:4/3}
.extras{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}
.extras figure{margin:0}.extras img{width:100%;height:auto;background:#000;border-radius:4px;display:block;aspect-ratio:4/3}
.extras figcaption{font-size:.8rem;color:var(--steel)}
textarea{width:100%;min-height:5.4em;font:inherit;padding:8px;border:1px solid var(--line);border-radius:4px;resize:vertical}
button{font:inherit;font-weight:700;padding:8px 14px;border-radius:4px;border:1px solid var(--line);background:#fff;color:var(--ink);cursor:pointer}
button.primary{background:var(--vlm);border-color:var(--vlm);color:#fff}
button.stop{background:var(--ng);border-color:var(--ng);color:#fff}
button:disabled{opacity:.45;cursor:not-allowed}
.row{display:flex;flex-wrap:wrap;gap:8px;margin-top:8px}
.badge{display:inline-block;padding:.05em .6em;border-radius:4px;font-size:.8rem;font-weight:700;background:var(--panel);color:var(--lamp)}
.badge.ok{background:var(--ok);color:#fff}.badge.ng{background:var(--ng);color:#fff}
table{border-collapse:collapse;width:100%;font-size:.86rem}
th,td{border-bottom:1px solid var(--line);padding:6px 6px;text-align:left;vertical-align:top}
th{color:var(--steel);font-weight:700;white-space:nowrap}
tr.cur td{background:#FFF7DD}
code,pre{font-family:ui-monospace,Consolas,"Courier New",monospace;font-size:.8rem}
#log{height:260px;overflow:auto;background:var(--panel);color:#E6EDF3;padding:8px;border-radius:4px;white-space:pre-wrap;word-break:break-all;margin:0}
#msg{min-height:1.6em;font-size:.86rem;color:var(--steel);margin-top:6px}
details{margin-top:8px}summary{cursor:pointer;color:var(--steel);font-size:.86rem}
dl{display:grid;grid-template-columns:auto 1fr;gap:2px 12px;margin:0;font-size:.86rem}
dt{color:var(--steel)}dd{margin:0;word-break:break-all}
</style>
</head>
<body>
<header><h1>画像と言語の指示によるタスク実行</h1><span id="hd">接続中…</span></header>
<main>
<div>
<section><h2>カメラ映像</h2><img id="cam" src="/stream.mjpg" alt="ロボットのカメラ映像"></section>
<section id="extrasec" style="display:none"><h2>外からの映像</h2><div id="extras" class="extras"></div></section>
<section><h2>状態</h2>
<dl>
<dt>実行</dt><dd><span id="task" class="badge">-</span> <span id="taskmsg"></span></dd>
<dt>VLM</dt><dd><span id="vlm" class="badge">-</span> <span id="vlminfo"></span></dd>
<dt>直近の判定</dt><dd id="chk">-</dd>
<dt>オドメトリ</dt><dd id="pose">-</dd>
<dt>入力</dt><dd id="rstat">-</dd>
<dt>記録の保存先</dt><dd id="rundir">-</dd>
</dl>
<div class="row"><button class="stop" id="bstop">停止</button><button id="breset">停止を解除</button><span id="extra" class="row" style="margin:0"></span></div>
</section>
</div>
<div>
<section><h2>1. 指示文（1行に1つ）</h2>
<textarea id="inst"></textarea>
<div class="row"><button class="primary" id="bplan">計画を作る</button></div>
<details><summary>計画の JSON を直接編集する／読み込む</summary>
<textarea id="pjson" placeholder='{"steps": [...]}'></textarea>
<div class="row"><button id="bload">この JSON を計画にする</button></div></details>
<div id="msg"></div>
</section>
<section><h2>2. 計画の確認と承認</h2>
<div id="planstate"></div>
<table id="plan"><thead><tr><th>#</th><th>指示</th><th>いつ</th><th>何を</th><th>時間切れ</th><th>状態</th></tr></thead><tbody></tbody></table>
<div class="row"><button class="primary" id="bexec" disabled>承認して実行</button></div>
</section>
<section><h2>記録</h2><pre id="log"></pre></section>
</div>
</main>
<script>
const $=id=>document.getElementById(id);
let since=0, filled=false, buttonsMade=false, planShown="", extraNames=[], extraTick=0;
const PH={pending:"未着手",waiting:"条件待ち",acting:"動作中",done:"完了",failed:"中止"};
const TS={idle:"待機",running:"実行中",done:"完了",failed:"失敗",stopped:"停止"};
async function post(path,body){
  try{
    const r=await fetch(path,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body||{})});
    const j=await r.json(); $("msg").textContent=j.message||""; return j;
  }catch(e){ $("msg").textContent="送信に失敗しました: "+e; return {ok:false}; }
}
function esc(s){return String(s).replace(/[&<>"]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));}
function whenText(w){ if(w.type==="now") return "前の手順が終わったらすぐ";
  if(w.type==="when_true") return "条件が成り立ったら<br><code>"+esc(w.condition)+"</code>";
  return "一度成り立たなくなり、再び成り立ったら<br><code>"+esc(w.condition)+"</code>"; }
function doText(d){ if(d.skill==="rotate") return "旋回 "+d.angle_deg+"°";
  if(d.skill==="move_straight") return "直進 "+d.distance_m+" m";
  if(d.skill==="wait") return "待機 "+d.seconds+" 秒";
  return "点へ移動<br><code>"+esc(d.target)+"</code>"; }
function fmtEvent(e){ const o=Object.assign({},e); delete o.id; delete o.wall; delete o.kind;
  const t=new Date(e.wall*1000).toLocaleTimeString("ja-JP",{hour12:false});
  return t+" "+e.kind+" "+JSON.stringify(o); }
async function tick(){
  let s; try{ s=await (await fetch("/api/state?since="+since)).json(); }catch(e){ $("hd").textContent="サーバに接続できません"; return; }
  $("hd").textContent="時刻 "+s.time.toFixed(1)+" s";
  if(!filled){ $("inst").value=s.instructions.join("\n"); filled=true; }
  if(!buttonsMade){ extraNames=s.extra_cameras.map(c=>c.name);
    if(extraNames.length){ $("extrasec").style.display=""; for(const c of s.extra_cameras){ const f=document.createElement("figure");
      f.innerHTML='<img id="ex_'+esc(c.name)+'" alt="'+esc(c.label)+'"><figcaption>'+esc(c.label)+'</figcaption>'; $("extras").appendChild(f);} } }
  if(extraNames.length && (extraTick++%2===0)){ for(const n of extraNames){ $("ex_"+n).src="/extra/"+n+".jpg?t="+Date.now(); } }
  if(!buttonsMade){ buttonsMade=true; for(const b of s.buttons){ const el=document.createElement("button"); el.textContent=b.label; el.onclick=()=>post("/api/trigger",{name:b.name}); $("extra").appendChild(el);} }
  const t=$("task"); t.textContent=TS[s.task_state]||s.task_state; t.className="badge"+(s.task_state==="done"?" ok":(s.task_state==="failed"||s.task_state==="stopped"?" ng":""));
  $("taskmsg").textContent=s.task_message+(s.stopped?"（停止中）":"");
  const v=$("vlm"); v.textContent=s.vlm.ok?"接続":"未接続"; v.className="badge "+(s.vlm.ok?"ok":"ng");
  $("vlminfo").textContent=s.vlm.base_url+" / "+s.vlm.model+(s.vlm.ok?"":"（"+s.vlm.info+"）");
  $("chk").textContent=s.last_check?((s.last_check.answer||"判定できず")+"（"+s.last_check.latency_s+" 秒）"+(s.last_check.error?" "+s.last_check.error:"")):"-";
  $("pose").textContent=s.pose?("x="+s.pose[0]+" y="+s.pose[1]+" 向き="+(s.pose[2]*180/Math.PI).toFixed(1)+"°"):"未受信";
  $("rstat").textContent=s.robot_status.length?s.robot_status.join("、"):"すべて受信中";
  $("rundir").textContent=s.run_dir||"-";
  const ps={none:"計画はまだありません。",planning:"VLM が計画を作成中です…",ready:"計画を確認し、問題がなければ承認してください。"+(s.plan_latency_s?"（生成 "+s.plan_latency_s+" 秒）":""),error:"計画を作れませんでした: "+s.plan_error};
  $("planstate").textContent=ps[s.plan_state]||"";
  // 作った計画は編集欄にも写す（条件文や目標の言い回しを変えて試せるように）
  if(s.plan){ const pj=JSON.stringify(s.plan,null,2); if(pj!==planShown){ planShown=pj; $("pjson").value=pj; } }
  const tb=$("plan").tBodies[0]; tb.innerHTML="";
  if(s.plan){ s.plan.steps.forEach((st,i)=>{ const ex=s.executor.steps[i]; const tr=document.createElement("tr");
    if(s.task_state==="running"&&s.executor.current===i) tr.className="cur";
    let stat=ex?(PH[ex.phase]||ex.phase):"-";
    if(ex&&ex.phase==="waiting") stat+="<br>判定 "+ex.checks+" 回"+(ex.streak_answer?"（"+ex.streak_answer+"×"+ex.streak+"）":"")+(st.when.type==="after_false_then_true"?(ex.seen_false?"<br>不成立を確認済み":"<br>不成立を未確認"):"");
    if(ex&&ex.message) stat+="<br>"+esc(ex.message);
    tr.innerHTML="<td>"+(i+1)+"</td><td>"+esc(st.instruction)+"</td><td>"+whenText(st.when)+"</td><td>"+doText(st.do)+"</td><td>"+st.timeout_s+" 秒</td><td>"+stat+"</td>";
    tb.appendChild(tr); }); }
  $("bexec").disabled=!(s.plan_state==="ready"&&s.task_state!=="running"&&!s.stopped);
  $("bplan").disabled=(s.plan_state==="planning"||s.task_state==="running");
  if(s.events.length){ const log=$("log"); const stick=log.scrollTop+log.clientHeight>=log.scrollHeight-8;
    for(const e of s.events){ since=Math.max(since,e.id); log.textContent+=fmtEvent(e)+"\n"; }
    if(log.textContent.length>200000) log.textContent=log.textContent.slice(-150000);
    if(stick) log.scrollTop=log.scrollHeight; }
}
$("bplan").onclick=()=>post("/api/plan",{instructions:$("inst").value});
$("bload").onclick=()=>post("/api/set_plan",{plan:$("pjson").value});
$("bexec").onclick=()=>post("/api/execute");
$("bstop").onclick=()=>post("/api/stop");
$("breset").onclick=()=>post("/api/reset");
setInterval(tick,500); tick();
</script>
</body>
</html>
"""


def _make_handler(runtime: TaskRuntime, stream_hz: float):
    period = 1.0 / max(stream_hz, 0.5)

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt: str, *args: Any) -> None:  # アクセスログは出さない
            return

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj: Any, code: int = 200) -> None:
            self._send(code, json.dumps(obj, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")

        def _result(self, result: Tuple[bool, str]) -> None:
            self._json({"ok": bool(result[0]), "message": result[1]})

        def do_GET(self) -> None:  # noqa: N802
            url = urlparse(self.path)
            if url.path in ("/", "/index.html"):
                self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
            elif url.path == "/api/state":
                try:
                    since = int(parse_qs(url.query).get("since", ["0"])[0])
                except ValueError:
                    since = 0
                self._json(runtime.state(since))
            elif url.path == "/snapshot.jpg":
                jpeg = runtime.overlay_jpeg()
                if jpeg is None:
                    self._send(503, b"no image yet", "text/plain")
                else:
                    self._send(200, jpeg, "image/jpeg")
            elif url.path.startswith("/extra/") and url.path.endswith(".jpg"):
                jpeg = runtime.extra_jpeg(url.path[len("/extra/"):-len(".jpg")])
                if jpeg is None:
                    self._send(503, b"no image yet", "text/plain")
                else:
                    self._send(200, jpeg, "image/jpeg")
            elif url.path == "/stream.mjpg":
                self._stream()
            else:
                self._send(404, b"not found", "text/plain")

        def _stream(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.close_connection = True
            try:
                while True:
                    jpeg = runtime.overlay_jpeg()
                    if jpeg is not None:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpeg)).encode("ascii") + b"\r\n\r\n" + jpeg + b"\r\n")
                        self.wfile.flush()
                    time.sleep(period)
            except (BrokenPipeError, ConnectionError, OSError):
                return

        def do_POST(self) -> None:  # noqa: N802
            url = urlparse(self.path)
            try:
                length = int(self.headers.get("Content-Length") or 0)
                raw = self.rfile.read(length) if length > 0 else b""
                body: Dict[str, Any] = json.loads(raw.decode("utf-8")) if raw.strip() else {}
                if not isinstance(body, dict):
                    raise ValueError("JSON のオブジェクトを送ってください")
            except (ValueError, UnicodeDecodeError) as e:
                self._json({"ok": False, "message": f"リクエストを読めません: {e}"}, 400)
                return
            if url.path == "/api/plan":
                self._result(runtime.request_plan(str(body.get("instructions", ""))))
            elif url.path == "/api/set_plan":
                self._result(runtime.set_plan(body.get("plan")))
            elif url.path == "/api/execute":
                self._result(runtime.execute())
            elif url.path == "/api/stop":
                self._result(runtime.stop())
            elif url.path == "/api/reset":
                self._result(runtime.reset())
            elif url.path == "/api/trigger":
                self._result(runtime.trigger(str(body.get("name", ""))))
            else:
                self._json({"ok": False, "message": "not found"}, 404)

    return Handler


class WebUI:
    def __init__(self, runtime: TaskRuntime, host: str = "0.0.0.0", port: int = 8080, stream_hz: float = 5.0):
        self.server = ThreadingHTTPServer((host, port), _make_handler(runtime, stream_hz))
        self.server.daemon_threads = True
        self.port = self.server.server_address[1]
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self.server.serve_forever, name="webui", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.server.shutdown()
        self.server.server_close()
