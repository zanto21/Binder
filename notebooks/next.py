import subprocess, os, signal, time, traceback
import ipywidgets as widgets
from IPython.display import display, clear_output
import rospy, json
from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologFinish

# Definiere Status-Codes
STATUS_OK = 0
STATUS_NO_SOLUTION = 1
STATUS_QUERY_FAILED = 2
# evtl. weitere Status-Codes wie:
STATUS_META = 3

# Hilfsfunktion zum Starten eines Prozesses
def start_proc(cmd, name):
    print(f"→ Starte {name}: {' '.join(cmd)}")
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)

# Prozess-Handles global
procs = []

# Starte ROS core und KnowRob (roslaunch)
def init_ros_system():
    global procs
    if not any(p.poll() is None for p in procs):
        procs = []
        procs.append(start_proc(['roscore'], 'roscore'))
        time.sleep(2)  # kurz warten, bis roscore läuft
        procs.append(start_proc(['roslaunch', 'knowrob', 'openease-test.launch'], 'knowrob'))

# Prolog-Query-Funktionen
def call_query(q):
    srv = rospy.ServiceProxy('/rosprolog/query', PrologQuery)
    return srv(id='q1', query=q)

def call_next():
    srv = rospy.ServiceProxy('/rosprolog/next_solution', PrologNextSolution)
    return srv(id='q1')

def call_finish():
    srv = rospy.ServiceProxy('/rosprolog/finish', PrologFinish)
    return srv(id='q1')

# Widgets definieren
neem_input = widgets.Text(value='62d5729bb3869a9a9c942f24', description='NEEM-ID:')
connect_btn = widgets.Button(description='Connect NEEM', button_style='info')
query_input = widgets.Textarea(value="triple(Tsk, rdf:type, dul:'Action').", placeholder='Prolog-Query eingeben')
run_btn = widgets.Button(description='Run Query', button_style='success', disabled=True)
next_btn = widgets.Button(description='Next Solution', button_style='warning', disabled=True)
output = widgets.Output()

# Button-Handler
def on_connect(_):
    with output:
        clear_output()
        init_ros_system()
        try:
            rospy.init_node('next_gui', anonymous=True)
            rospy.wait_for_service('/rosprolog/query', timeout=10)
            call_query(f"knowrob_load_neem('{neem_input.value.strip()}')")
            print("✅ NEEM geladen.")
            run_btn.disabled = False
        except Exception as e:
            print("❌ Fehler beim Laden:", traceback.format_exc())

def on_run(_):
    with output:
        clear_output()
        q = query_input.value.strip()
        try:
            call_query(q)
            print("🔍 Query gestartet:", q)
            next_btn.disabled = False
        except Exception as e:
            print("❌ Fehler beim Starten:", traceback.format_exc())



def on_next(_):
    with output:
        try:
            res = call_next()
            st = res.status
            sol = json.loads(res.solution) if res.solution else {}

            if st == STATUS_OK:
                print("✅ Lösung:", sol)
            elif st == STATUS_META:
                print("✅ Lösung:", sol)
            elif st == STATUS_NO_SOLUTION:
                print("✅ Keine weiteren Lösungen.")
                call_finish()
                next_btn.disabled = True
            elif st == STATUS_QUERY_FAILED:
                print("❌ Query-Fehler:", sol)
                call_finish()
                next_btn.disabled = True
            else:
                print(f"❗ Unbekannter Status {st}:", sol)
                call_finish()
                next_btn.disabled = True

        except Exception as e:
            print("❌ Fehler beim `next`-Aufruf:", e)
            next_btn.disabled = True

# Buttons verbinden
connect_btn.on_click(on_connect)
run_btn.on_click(on_run)
next_btn.on_click(on_next)

# UI anzeigen
ui = widgets.VBox([neem_input, connect_btn, query_input, widgets.HBox([run_btn, next_btn]), output])
display(ui)
