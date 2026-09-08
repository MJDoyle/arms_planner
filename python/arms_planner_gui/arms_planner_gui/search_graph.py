"""Live view of the assembly search.

`arms-plan --trace-dfs` writes one JSON object per line, prefixed "[dfs] ", into
the same stdout stream the viewer already reads while planning.  This module
collects those events into a graph and serves it on its own port, so the search
can be watched and picked over in a browser tab while it is still running.

The graph alone shows where the search went; the per-part rejection tally beside
it shows why it did not go anywhere else, which is usually the question being
asked when a plan fails.
"""

from __future__ import annotations

import json
import threading
from collections import Counter
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# A long search can produce tens of thousands of states.  Past a few thousand the
# picture stops being readable anyway, so keep the newest and say so.
_MAX_NODES = 4000


class SearchGraph:
    """Thread-safe store of the search as it unfolds."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.reset()

    def reset(self) -> None:
        with self._lock:
            self.parts: dict[str, dict] = {}
            self.nodes: dict[int, dict] = {}
            self.edges: list[dict] = []
            self.rejects: dict[int, list[dict]] = {}
            self.current: int | None = None
            self.running = False
            self.truncated = False
            self._order: list[int] = []

    # ---- ingest ----

    def feed(self, line: str) -> bool:
        """Consume one planner output line.  Returns True if it was an event."""
        if not line.startswith("[dfs] "):
            return False
        try:
            ev = json.loads(line[6:])
        except json.JSONDecodeError:
            return False

        kind = ev.get("ev")
        with self._lock:
            if kind == "parts":
                self.parts = {str(k): v for k, v in ev.get("map", {}).items()}
                self.running = True

            elif kind == "visit":
                nid = int(ev["id"])
                node = self._node(nid)
                node["parts"] = ev.get("parts", [])
                node["visits"] = node.get("visits", 0) + 1
                if node["state"] == "frontier":
                    node["state"] = "visited"
                self.current = nid

            elif kind == "edge":
                src, dst = int(ev["from"]), int(ev["to"])
                self._node(src)
                tgt = self._node(dst)
                if not tgt.get("parts"):
                    tgt["parts"] = ev.get("parts", [])
                self.edges.append({"from": src, "to": dst, "part": int(ev["part"])})

            elif kind == "reject":
                src = int(ev["from"])
                self.rejects.setdefault(src, []).append({
                    "part": int(ev["part"]),
                    "why": ev.get("why", ""),
                    "by": ev.get("by", []),
                })

            elif kind in ("dead", "goal"):
                self._node(int(ev["id"]))["state"] = kind

        return True

    def _node(self, nid: int) -> dict:
        n = self.nodes.get(nid)
        if n is None:
            n = {"id": nid, "parts": [], "state": "frontier", "visits": 0}
            self.nodes[nid] = n
            self._order.append(nid)
            if len(self._order) > _MAX_NODES:
                drop = self._order.pop(0)
                self.nodes.pop(drop, None)
                self.rejects.pop(drop, None)
                self.truncated = True
        return n

    def finish(self) -> None:
        with self._lock:
            self.running = False
            self.current = None

    # ---- serialise ----

    def snapshot(self) -> dict:
        with self._lock:
            total = len(self.parts) or max(
                (len(n["parts"]) for n in self.nodes.values()), default=0)

            nodes = []
            for n in self.nodes.values():
                nodes.append({**n, "depth": total - len(n["parts"])})

            # Why the search kept turning back.  A part that is rejected at almost
            # every state it appears in is the one holding the plan up, and the
            # blockers tell you what to look at.
            tally: dict[str, dict] = {}
            for lst in self.rejects.values():
                for r in lst:
                    e = tally.setdefault(str(r["part"]), {"count": 0, "why": Counter(),
                                                          "by": Counter()})
                    e["count"] += 1
                    e["why"][r["why"]] += 1
                    for b in r["by"]:
                        e["by"][str(b)] += 1

            blocked = [
                {"part": pid,
                 "count": e["count"],
                 "why": e["why"].most_common(1)[0][0],
                 "by": [b for b, _ in e["by"].most_common(3)]}
                for pid, e in tally.items()
            ]
            blocked.sort(key=lambda d: -d["count"])

            ids = set(self.nodes)
            return {
                "running": self.running,
                "truncated": self.truncated,
                "parts": self.parts,
                "nodes": nodes,
                "edges": [e for e in self.edges if e["from"] in ids and e["to"] in ids],
                "current": self.current,
                "blocked": blocked,
                "counts": {
                    "nodes": len(self.nodes),
                    "dead": sum(1 for n in self.nodes.values() if n["state"] == "dead"),
                },
            }


def build_figure(snap: dict):
    """The search as a Plotly figure, for embedding in the viewer's own panel.

    Same layout as the standalone page — one column per part removed — but drawn
    with Plotly so it can live inside a viser tab.  Returns None if Plotly is not
    installed, so the viewer can fall back to offering the browser page.
    """
    try:
        import plotly.graph_objects as go
    except ImportError:
        return None

    parts = snap.get("parts", {})
    pname = lambda i: (parts.get(str(i)) or {}).get("name", f"part {i}")

    # Column per depth, rows in the order states first appeared.
    pos, rows = {}, {}
    for n in sorted(snap["nodes"], key=lambda d: d["id"]):
        d = n["depth"]
        r = rows.get(d, 0)
        rows[d] = r + 1
        pos[n["id"]] = (d, r)

    ex, ey = [], []
    for e in snap["edges"]:
        a, b = pos.get(e["from"]), pos.get(e["to"])
        if a and b:
            ex += [a[0], b[0], None]
            ey += [a[1], b[1], None]

    colour = {"dead": "#e2564a", "goal": "#3fbf7f"}
    nx, ny, nc, nt = [], [], [], []
    for n in snap["nodes"]:
        p = pos.get(n["id"])
        if not p:
            continue
        nx.append(p[0])
        ny.append(p[1])
        nc.append("#4da3ff" if n["id"] == snap.get("current")
                  else colour.get(n["state"], "#39485c"))
        nt.append(f"state {n['id']}<br>{len(n['parts'])} parts in place<br>"
                  + "<br>".join(pname(x) for x in n["parts"][:12]))

    fig = go.Figure()
    fig.add_trace(go.Scatter(x=ex, y=ey, mode="lines",
                             line=dict(color="#48566a", width=1),
                             hoverinfo="skip", showlegend=False))
    fig.add_trace(go.Scatter(x=nx, y=ny, mode="markers",
                             marker=dict(size=9, color=nc),
                             text=nt, hoverinfo="text", showlegend=False))
    fig.update_layout(
        margin=dict(l=4, r=4, t=4, b=4),
        paper_bgcolor="rgba(0,0,0,0)", plot_bgcolor="rgba(0,0,0,0)",
        xaxis=dict(visible=False), yaxis=dict(visible=False, autorange="reversed"),
        height=320,
    )
    return fig


class SearchGraphServer:
    """Serves the graph page and its data on its own port."""

    def __init__(self, graph: SearchGraph, port: int = 8081,
                 viewer_url: str = "http://localhost:8080/") -> None:
        self.graph = graph
        self.port = port
        # Where the "back to viewer" button goes, for when the page was opened
        # directly rather than from the viewer.
        self.viewer_url = viewer_url
        self._httpd: ThreadingHTTPServer | None = None

    @property
    def url(self) -> str:
        return f"http://localhost:{self.port}/"

    def start(self, tries: int = 12) -> bool:
        viewer_url = self.viewer_url
        """Bind the first free port at or above the requested one.

        A stale viewer holding the default would otherwise disable the standalone
        view for the rest of the session.
        """
        graph = self.graph

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_a):        # keep the planner's output clean
                pass

            def do_GET(self):                  # noqa: N802
                if self.path.startswith("/graph.json"):
                    body = json.dumps(graph.snapshot()).encode()
                    ctype = "application/json"
                else:
                    body = _PAGE.replace("__VIEWER_URL__", viewer_url).encode()
                    ctype = "text/html; charset=utf-8"
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

        base = self.port
        for offset in range(tries):
            try:
                self._httpd = ThreadingHTTPServer(("127.0.0.1", base + offset), Handler)
                self.port = base + offset
                break
            except OSError:
                continue
        else:
            return False

        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()
        return True


# ---------------------------------------------------------------------------
# The page.  Self-contained: no CDN, so it works with no network.
# Laid out by depth rather than by force, because a search tree has a natural
# left-to-right reading — one column per part removed.
# ---------------------------------------------------------------------------

_PAGE = r"""<!doctype html>
<meta charset="utf-8">
<title>Assembly Search</title>
<style>
  :root{--bg:#12161c;--panel:#1a2029;--ink:#e6eaf0;--dim:#8d97a6;--rule:#2a3340;
        --live:#4da3ff;--dead:#e2564a;--goal:#3fbf7f;--edge:#48566a;}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--ink);
       font:13px/1.5 ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,sans-serif;
       display:flex;height:100vh;overflow:hidden}
  #graph{flex:1;position:relative}
  canvas{display:block;cursor:grab}
  canvas.drag{cursor:grabbing}
  aside{width:310px;border-left:1px solid var(--rule);background:var(--panel);
        padding:14px 16px;overflow:auto}
  h1{font-size:13px;letter-spacing:.08em;text-transform:uppercase;color:var(--dim);
     margin:0 0 10px;font-weight:600}
  h2{font-size:12px;letter-spacing:.06em;text-transform:uppercase;color:var(--dim);
     margin:18px 0 8px;font-weight:600}
  .stat{display:flex;justify-content:space-between;padding:2px 0;
        font-variant-numeric:tabular-nums}
  .stat span:last-child{color:var(--dim)}
  .row{padding:7px 0;border-bottom:1px solid var(--rule)}
  .row:last-child{border-bottom:0}
  .name{font-weight:600}
  .why{color:var(--dim);font-size:12px}
  .pill{display:inline-block;padding:1px 6px;border-radius:9px;font-size:11px;
        background:#26303c;color:var(--dim);margin-left:6px}
  #tip{position:absolute;pointer-events:none;background:#0d1117ee;
       border:1px solid var(--rule);border-radius:5px;padding:7px 9px;max-width:290px;
       font-size:12px;display:none;z-index:5}
  #hint{position:absolute;left:12px;bottom:10px;color:var(--dim);font-size:11px}
  .live{color:var(--live)} .dead{color:var(--dead)} .goal{color:var(--goal)}
  ul{margin:4px 0 0;padding-left:16px} li{margin:1px 0}
  #back{width:100%;margin:0 0 14px;padding:8px 10px;font:inherit;font-weight:600;
        color:var(--ink);background:#26303c;border:1px solid var(--rule);
        border-radius:6px;cursor:pointer;text-align:left}
  #back:hover{background:#2f3b4a;border-color:#3d4a5c}
  #back:active{background:#212a35}
  #back:focus-visible{outline:2px solid var(--live);outline-offset:2px}
</style>
<div id="graph"><canvas id="c"></canvas><div id="tip"></div>
  <div id="hint">drag to pan &middot; scroll to zoom &middot; hover a node for its parts &middot; click to pin</div></div>
<aside>
  <button id="back" title="Return to the 3D viewer">&larr;&nbsp; Back to viewer</button>
  <h1>Assembly search</h1>
  <div class="stat"><span>Status</span><span id="status">waiting…</span></div>
  <div class="stat"><span>States</span><span id="n-nodes">0</span></div>
  <div class="stat"><span>Dead ends</span><span id="n-dead">0</span></div>
  <h2>Hardest to remove</h2>
  <div id="blocked"><div class="why">Nothing rejected yet.</div></div>
  <h2 id="sel-h" style="display:none">Selected state</h2>
  <div id="sel"></div>
</aside>
<script>
const cv=document.getElementById('c'),ctx=cv.getContext('2d'),tip=document.getElementById('tip');
let G={nodes:[],edges:[],parts:{},blocked:[]},pos=new Map(),rows=new Map();
let view={x:70,y:0,k:1},drag=null,hover=null,pinned=null;

function fit(){cv.width=cv.parentElement.clientWidth;cv.height=cv.parentElement.clientHeight;}
addEventListener('resize',()=>{fit();draw();});fit();

const pname=id=>(G.parts[id]&&G.parts[id].name)||('part '+id);

// One column per depth; rows fill downward in the order states first appeared,
// so the picture stays stable as it grows.
function layout(){
  const COLW=120,ROWH=34;
  for(const n of G.nodes){
    if(!rows.has(n.id)){
      const d=n.depth;
      const used=rows.has('d'+d)?rows.get('d'+d):0;
      rows.set('d'+d,used+1); rows.set(n.id,{d:d,r:used});
    }
    const p=rows.get(n.id);
    pos.set(n.id,{x:p.d*COLW,y:p.r*ROWH});
  }
}
const S=p=>({x:p.x*view.k+view.x,y:p.y*view.k+view.y});

function draw(){
  ctx.clearRect(0,0,cv.width,cv.height);
  ctx.lineWidth=1;
  for(const e of G.edges){
    const a=pos.get(e.from),b=pos.get(e.to); if(!a||!b)continue;
    const A=S(a),B=S(b);
    ctx.strokeStyle='#48566a';ctx.beginPath();
    ctx.moveTo(A.x+9,A.y);ctx.bezierCurveTo((A.x+B.x)/2,A.y,(A.x+B.x)/2,B.y,B.x-9,B.y);ctx.stroke();
    if(view.k>0.75){ctx.fillStyle='#7c8899';ctx.font='10px ui-monospace,monospace';
      ctx.fillText(e.part,(A.x+B.x)/2-3,(A.y+B.y)/2-3);}
  }
  for(const n of G.nodes){
    const p=pos.get(n.id); if(!p)continue; const P=S(p);
    if(P.x<-40||P.x>cv.width+40||P.y<-40||P.y>cv.height+40)continue;
    const r=9*Math.min(1.4,view.k);
    ctx.beginPath();ctx.arc(P.x,P.y,r,0,7);
    ctx.fillStyle=n.state==='dead'?'#e2564a':n.state==='goal'?'#3fbf7f':'#39485c';
    ctx.fill();
    if(n.id===G.current){ctx.strokeStyle='#4da3ff';ctx.lineWidth=3;
      ctx.beginPath();ctx.arc(P.x,P.y,r+5,0,7);ctx.stroke();ctx.lineWidth=1;}
    if(n.id===(pinned??hover)){ctx.strokeStyle='#e6eaf0';ctx.lineWidth=2;
      ctx.beginPath();ctx.arc(P.x,P.y,r+3,0,7);ctx.stroke();ctx.lineWidth=1;}
    if(view.k>0.7){ctx.fillStyle='#c3ccd8';ctx.font='10px ui-monospace,monospace';
      ctx.textAlign='center';ctx.fillText(n.id,P.x,P.y+3);ctx.textAlign='left';}
  }
}

function pick(mx,my){
  for(const n of G.nodes){const p=pos.get(n.id);if(!p)continue;const P=S(p);
    if((P.x-mx)**2+(P.y-my)**2<196)return n;}
  return null;
}
cv.addEventListener('mousedown',e=>{drag={x:e.clientX,y:e.clientY,vx:view.x,vy:view.y,moved:0};cv.classList.add('drag');});
addEventListener('mouseup',e=>{
  if(drag&&drag.moved<4){const r=cv.getBoundingClientRect();const n=pick(e.clientX-r.left,e.clientY-r.top);
    pinned=n?n.id:null;side();}
  drag=null;cv.classList.remove('drag');});
cv.addEventListener('mousemove',e=>{
  const r=cv.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top;
  if(drag){drag.moved+=Math.abs(e.movementX)+Math.abs(e.movementY);
    view.x=drag.vx+(e.clientX-drag.x);view.y=drag.vy+(e.clientY-drag.y);draw();return;}
  const n=pick(mx,my); hover=n?n.id:null;
  if(n){tip.style.display='block';tip.style.left=(mx+14)+'px';tip.style.top=(my+12)+'px';
    tip.innerHTML='<b>state '+n.id+'</b> &middot; '+n.parts.length+' parts in place'+
      '<ul>'+n.parts.map(p=>'<li>'+pname(p)+'</li>').join('')+'</ul>';}
  else tip.style.display='none';
  draw();});
cv.addEventListener('wheel',e=>{e.preventDefault();
  const r=cv.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top;
  const f=Math.exp(-e.deltaY*0.0015),k=Math.min(3,Math.max(0.15,view.k*f));
  view.x=mx-(mx-view.x)*(k/view.k);view.y=my-(my-view.y)*(k/view.k);view.k=k;draw();},{passive:false});

function side(){
  document.getElementById('status').innerHTML = G.running
    ? '<span class="live">running</span>' : (G.nodes.some(n=>n.state==='goal')
      ? '<span class="goal">solved</span>' : '<span class="dead">stopped</span>');
  document.getElementById('n-nodes').textContent=(G.counts?G.counts.nodes:0)+(G.truncated?'+':'');
  document.getElementById('n-dead').textContent=G.counts?G.counts.dead:0;

  const b=document.getElementById('blocked');
  b.innerHTML = G.blocked.length ? G.blocked.slice(0,10).map(x=>
    '<div class="row"><div class="name">'+pname(x.part)+'<span class="pill">'+x.count+'&times;</span></div>'+
    '<div class="why">'+x.why+(x.by&&x.by.length?' by '+x.by.map(pname).join(', '):'')+'</div></div>').join('')
    : '<div class="why">Nothing rejected yet.</div>';

  const id=pinned??hover, sel=document.getElementById('sel');
  document.getElementById('sel-h').style.display=id==null?'none':'block';
  if(id==null){sel.innerHTML='';return;}
  const n=G.nodes.find(v=>v.id===id);
  sel.innerHTML=n?('<div class="why">'+n.parts.length+' parts in place</div><ul>'+
    n.parts.map(p=>'<li>'+pname(p)+'</li>').join('')+'</ul>'):'';
}

document.getElementById('back').addEventListener('click',()=>{
  // If this page was opened from the viewer, closing it returns to that tab.
  // Opened directly, there is nothing to fall back to but navigating there.
  if(window.opener && !window.opener.closed){ window.close(); }
  else { location.href='__VIEWER_URL__'; }
});

async function tick(){
  try{
    const r=await fetch('/graph.json',{cache:'no-store'});
    G=await r.json(); layout(); draw(); side();
  }catch(e){}
  setTimeout(tick, G.running?600:2500);
}
tick();
</script>
"""
