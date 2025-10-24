import subprocess, os, signal, time, traceback, uuid
import ipywidgets as widgets
from IPython.display import display, clear_output
import rospy, json
from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologFinish


# Definiere Status-Codes
STATUS_NO_SOLUTION = 0
STATUS_WRONG_ID = 1
STATUS_QUERY_FAILED = 2
STATUS_OK = 3


# Prozess-Handles
procs = []
current_query_id = None  # speichert die aktuelle Query-ID

# --- Prozesse starten ---
def start_proc(cmd, name):
    print(f"→ Starte {name}: {' '.join(cmd)}")
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)

def init_ros_system():
    global procs
    if not any(p.poll() is None for p in procs):
        procs = []
        procs.append(start_proc(['roscore'], 'roscore'))
        time.sleep(2)
        procs.append(start_proc(['roslaunch', 'knowrob', 'openease-test.launch'], 'knowrob'))
        time.sleep(5)
        print("✅ ROS & KnowRob gestartet.")

# --- Prolog Service Wrapper ---
def call_query(q, query_id):
    srv = rospy.ServiceProxy('/rosprolog/query', PrologQuery)
    return srv(id=query_id, query=q)

def call_next(query_id):
    srv = rospy.ServiceProxy('/rosprolog/next_solution', PrologNextSolution)
    return srv(id=query_id)

def call_finish(query_id):
    srv = rospy.ServiceProxy('/rosprolog/finish', PrologFinish)
    return srv(id=query_id)

# --- Widgets ---
neem_input = widgets.Text(value='62d5729bb3869a9a9c942f24', description='NEEM-ID:')
connect_btn = widgets.Button(description='Connect NEEM', button_style='info')
query_input = widgets.Textarea(value="triple(Tsk, rdf:type, dul:'Action').", placeholder='Prolog-Query eingeben')
run_btn = widgets.Button(description='Run Query', button_style='success', disabled=True)
next_btn = widgets.Button(description='Next Solution', button_style='warning', disabled=True)
output = widgets.Output()

# --- Handlers ---
def on_connect(_):
    with output:
        clear_output()
        init_ros_system()
        try:
            if not rospy.core.is_initialized():
                rospy.init_node('next_gui', anonymous=True)
            rospy.wait_for_service('/rosprolog/query', timeout=10)
            neem = neem_input.value.strip()
            q = f"knowrob_load_neem('{neem}')"
            query_id = "connect_" + uuid.uuid4().hex[:6]
            call_query(q, query_id)
            call_finish(query_id)
            print(f"✅ NEEM {neem} erfolgreich geladen.")
            run_btn.disabled = False
        except Exception as e:
            print("❌ Fehler beim Laden:", traceback.format_exc())

def on_run(_):
    global current_query_id
    with output:
        clear_output()
        q = query_input.value.strip()
        if not q.endswith('.'):
            q += '.'
        try:
            # Alte Query schließen, falls offen
            if current_query_id:
                call_finish(current_query_id)
            current_query_id = "q_" + uuid.uuid4().hex[:6]
            call_query(q, current_query_id)
            print(f"🔍 Query gestartet (ID={current_query_id}): {q}")
            next_btn.disabled = False
        except Exception as e:
            print("❌ Fehler beim Starten:", traceback.format_exc())

def on_next(_):
    global current_query_id
    with output:
        try:
            if not current_query_id:
                print("⚠️ Keine aktive Query.")
                return
            res = call_next(current_query_id)
            st = res.status
            sol = json.loads(res.solution) if res.solution else {}

            if st == STATUS_OK:
                print(json.dumps(sol, indent=2))
                print("✅ Lösung:", sol)
            elif st == STATUS_NO_SOLUTION:
                print("✅ Keine weiteren Lösungen.")
                call_finish(current_query_id)
                next_btn.disabled = True
                current_query_id = None
            elif st == STATUS_QUERY_FAILED:
                print("❌ Query fehlgeschlagen:", sol)
                call_finish(current_query_id)
                next_btn.disabled = True
                current_query_id = None
            else:
                print(f"ℹ️ Unbekannter Status {st}:", sol)
        except Exception as e:
            print("❌ Fehler beim Next-Aufruf:", e)
            next_btn.disabled = True

# --- Button Events ---
connect_btn.on_click(on_connect)
run_btn.on_click(on_run)
next_btn.on_click(on_next)

# --- UI anzeigen ---
ui = widgets.VBox([
    neem_input,
    connect_btn,
    query_input,
    widgets.HBox([run_btn, next_btn]),
    output
])
display(ui)
