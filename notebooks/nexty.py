# next.py
import subprocess, os, time, traceback, uuid
import ipywidgets as widgets
from IPython.display import display, clear_output
import rospy, json
from pymongo import MongoClient
from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologFinish

# --- Konfiguration ---
MONGO_URI     = "mongodb://neemReader:qEWRqc9UdN5TD7No7cjymUA8QEweNz@neem-3.informatik.uni-bremen.de:28015/neems"
MONGO_DB_NAME = "neems"

# --- Status-Codes ---
STATUS_NO_SOLUTION  = 0
STATUS_WRONG_ID     = 1
STATUS_QUERY_FAILED = 2
STATUS_OK           = 3

# --- Globale Variablen ---
current_query_id = None
procs = []

# --- Hilfsfunktionen für ROS / KnowRob starten ---
def start_ros_if_needed():
    global procs
    if not any(p.poll() is None for p in procs):
        procs.append(subprocess.Popen(['roscore'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid))
        time.sleep(2)
        procs.append(subprocess.Popen(['roslaunch','knowrob','openease-test.launch'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid))
        time.sleep(5)

# --- Prolog-Service helper ---
def call_query(q, qid=None):
    if qid is None:
        qid = "q_" + uuid.uuid4().hex[:6]
    srv = rospy.ServiceProxy('/rosprolog/query', PrologQuery)
    return srv(id=qid, query=q), qid

def call_next(qid):
    srv = rospy.ServiceProxy('/rosprolog/next_solution', PrologNextSolution)
    return srv(id=qid)

def call_finish(qid):
    srv = rospy.ServiceProxy('/rosprolog/finish', PrologFinish)
    return srv(id=qid)

# --- UI Widgets ---
neem_dropdown = widgets.Dropdown(options=[], description="NEEM:", layout=widgets.Layout(width='60%'))
connect_btn   = widgets.Button(description="Connect & Load NEEM", button_style='info', layout=widgets.Layout(width='60%'))
query_input   = widgets.Textarea(value="triple(Tsk, rdf:type, dul:'Action').", placeholder="Prolog-Query eingeben ...", layout=widgets.Layout(width='100%', height='80px'))
run_btn       = widgets.Button(description="Run Query", button_style='success', disabled=True)
next_btn      = widgets.Button(description="Next Solution", button_style='warning', disabled=True)
output        = widgets.Output()

# --- Mongo: vorhandene NEEM-IDs holen und Dropdown befüllen ---
def refresh_neem_list():
    try:
        client = MongoClient(MONGO_URI, serverSelectionTimeoutMS=5000)
        db = client[MONGO_DB_NAME]
        cols = db.list_collection_names()
        neem_ids = sorted({c[:-8] for c in cols if c.endswith("_triples")})
        neem_dropdown.options = neem_ids
    except Exception as e:
        print("❌ Fehler beim Verbinden mit MongoDB:", e)

refresh_neem_list()

# --- Button-Callbacks ---
def on_connect(_):
    with output:
        clear_output()
        start_ros_if_needed()
        try:
            if not rospy.core.is_initialized():
                rospy.init_node('neem_gui', anonymous=True)
            rospy.wait_for_service('/rosprolog/query', timeout=10)
            selected = neem_dropdown.value
            if not selected:
                print("⚠️ Keine NEEM ausgewählt.")
                return
            print(f"🔄 Lade NEEM: {selected}")
            _, qid = call_query(f"knowrob_load_neem('{selected}')")
            call_finish(qid)
            print("✅ NEEM geladen:", selected)
            run_btn.disabled = False
        except Exception:
            print("❌ Fehler beim Laden der NEEM:", traceback.format_exc())

def on_run(_):
    global current_query_id
    with output:
        clear_output()
        q = query_input.value.strip()
        if not q:
            print("⚠️ Leere Query.")
            return
        if not q.endswith('.'):
            q += '.'
        try:
            res, qid = call_query(q)
            current_query_id = qid
            print("🔍 Query gestartet:", q)
            next_btn.disabled = False
        except Exception:
            print("❌ Fehler beim Senden der Query:", traceback.format_exc())
            next_btn.disabled = True

def on_next(_):
    global current_query_id
    with output:
        if not current_query_id:
            print("⚠️ Keine laufende Query.")
            return
        try:
            res = call_next(current_query_id)
            st = res.status
            sol_str = res.solution or ""
            # Debug
            # print(f"[DEBUG] status={st}, raw_solution='{sol_str}'")
            # Falls keine textuelle Lösung oder leer
            if st == STATUS_NO_SOLUTION or not sol_str.strip():
                print("✅ Keine weiteren Lösungen.")
                call_finish(current_query_id)
                next_btn.disabled = True
                current_query_id = None
                return
            if st == STATUS_OK:
                sol = json.loads(sol_str)
                print("✅ Lösung:", sol)
            elif st == STATUS_QUERY_FAILED:
                sol = sol_str
                print("❌ Query fehlgeschlagen:", sol)
                call_finish(current_query_id)
                next_btn.disabled = True
                current_query_id = None
            else:
                # evtl. andere Status-Codes
                sol = sol_str
                print(f"ℹ️ Unbekannter Status {st}:", sol)
        except Exception:
            print("❌ Fehler beim Next-Aufruf:", traceback.format_exc())
            next_btn.disabled = True

# --- Buttons mit Callbacks verbinden ---
connect_btn.on_click(on_connect)
run_btn.on_click(on_run)
next_btn.on_click(on_next)

# --- UI anzeigen ---
display(widgets.VBox([
    neem_dropdown,
    connect_btn,
    query_input,
    widgets.HBox([run_btn, next_btn]),
    output
]))
