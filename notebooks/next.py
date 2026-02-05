import os, json, uuid, traceback, subprocess, time
import ipywidgets as widgets
from IPython.display import display, clear_output

# --- ROS / KnowRob Imports ---
try:
    import rospy
    from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologFinish
    from json_prolog_msgs.srv import PrologNextSolutionResponse
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

# --- Pfade ---
NEEM_BASE_PATH = "/home/jovyan/work/notebooks/neems"

# --- UI Elemente ---
neem_dropdown = widgets.Dropdown(
    description="NEEM auswählen:", 
    style={'description_width':'initial'},
    layout=widgets.Layout(width="450px")
)
refresh_btn = widgets.Button(description="Aktualisieren", icon="refresh")

query_input = widgets.Textarea(
    value="triple(A, P, O).",
    placeholder="Prolog Query hier eingeben...",
    layout=widgets.Layout(width="100%", height="120px")
)

run_btn = widgets.Button(description="Query ausführen", button_style='success', icon="play")
next_btn = widgets.Button(description="Nächste Lösung", button_style='warning', icon="forward")
connect_btn = widgets.Button(description="1. Diagnose & Connect", button_style="info", icon="wrench", layout=widgets.Layout(width="280px"))
import_btn = widgets.Button(description="2. Full Reset Import", button_style="danger", icon="sync", layout=widgets.Layout(width="280px"))

output = widgets.Output(layout={'border': '1px solid #ddd', 'height': '450px', 'overflow_y': 'scroll'})

# Layout
header = widgets.HTML("<h2>KnowRob NEEM Explorer (Fix: Compound Type Error)</h2>")
ui = widgets.VBox([
    header, 
    widgets.VBox([
        widgets.HBox([neem_dropdown, refresh_btn]),
        widgets.HBox([connect_btn, import_btn]),
        widgets.HTML("<br><b>Prolog Query:</b>"),
        query_input,
        widgets.HBox([run_btn, next_btn]),
        output
    ])
])
display(ui)

# -----------------------------------------------------------------------------
# — Globale Variable für Query-Management
# -----------------------------------------------------------------------------
current_query_id = None

# -----------------------------------------------------------------------------
# — Hilfsfunktionen
# -----------------------------------------------------------------------------

def get_prolog_service(srv_name, srv_type):
    try:
        rospy.wait_for_service(srv_name, timeout=2.0)
        return rospy.ServiceProxy(srv_name, srv_type)
    except: return None

def run_simple_query(query_str):
    srv = get_prolog_service('/rosprolog/query', PrologQuery)
    if not srv: return None
    q_id = "q_" + uuid.uuid4().hex[:4]
    try:
        if not query_str.endswith('.'): query_str += '.'
        srv(id=q_id, query=query_str)
        next_srv = get_prolog_service('/rosprolog/next_solution', PrologNextSolution)
        res = next_srv(id=q_id)
        get_prolog_service('/rosprolog/finish', PrologFinish)(id=q_id)
        return res
    except: return None

def format_term(term):
    if isinstance(term, dict) and 'term' in term:
        val = term['term']
        if isinstance(val, list) and len(val) > 1: return str(val[1])
    elif isinstance(term, list) and len(term) > 1: return str(term[1])
    return str(term)

# -----------------------------------------------------------------------------
# — Callbacks
# -----------------------------------------------------------------------------

def on_import_clicked(_):
    with output:
        clear_output()
        target_neem = neem_dropdown.value
        if not target_neem: 
            print("Bitte zuerst einen NEEM auswählen.")
            return
        
        roslog_path = os.path.join(NEEM_BASE_PATH, target_neem, "data", "triples", "roslog")
        if not os.path.exists(roslog_path):
             roslog_path = os.path.join(NEEM_BASE_PATH, target_neem, "data", "triples")
        
        print(f"Bereinige Datenbank 'neems'...")
        subprocess.call(["mongo", "neems", "--eval", "db.dropDatabase()"])
        
        print(f"Importiere NEEM...")
        try:
            subprocess.check_call(["mongorestore", "--host", "127.0.0.1", "-d", "neems", "--dir", roslog_path])
            subprocess.call(["mongo", "neems", "--eval", "if(db.roslog.exists()){ db.roslog.renameCollection('triples') }"])
            print(f"Import erfolgreich.")
        except Exception as e: print(f"Fehler: {e}")

def on_connect_clicked(_):
    with output:
        clear_output()
        if not rospy.core.is_initialized():
            rospy.init_node('neem_local_ui', anonymous=True)
        
        print("Initialisiere KnowRob & Präfix-Handler...")
        run_simple_query("ensure_loaded(library('knowrob'))")
        run_simple_query("mng_db_name('neems')")
        
        # Namespaces registrieren
        namespaces = {
            'dul':  'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#',
            'soma': 'http://www.ease-crc.org/ont/SOMA.owl#',
            'owl':  'http://www.w3.org/2002/07/owl#',
            'rdf':  'http://www.w3.org/1999/02/22-rdf-syntax-ns#',
            'rdfs': 'http://www.w3.org/2000/01/rdf-schema#',
            'knowrob': 'http://knowrob.org/kb/knowrob.owl#'
        }
        for prefix, uri in namespaces.items():
            run_simple_query(f"rdf_db:rdf_register_ns({prefix}, '{uri}')")
        
        # URI-Helper: Wandelt Präfix-Strukturen sicher in Atome um
        # Wichtig: atom(In) deckt bereits existierende URIs ab.
        # Prefix:Name wird explizit aufgelöst und mit atom_concat zu einem flachen Atom verbunden.
        uri_helper = (
            "assertz(( ensure_uri(In, Out) :- "
            "  (atom(In) -> Out = In ; "
            "  (compound(In), In = Prefix:Name -> "
            "    (rdf_current_ns(Prefix, NS), atom_concat(NS, Name, Out)) ; "
            "    Out = In)) ))"
        )
        run_simple_query("retractall(ensure_uri(_,_))")
        run_simple_query(uri_helper)
        
        # Die Brücke nutzt mng_find. 
        # Das Problem war, dass S_U, P_U oder O_U manchmal noch Compounds waren.
        # Wir erzwingen hier die Auflösung.
        triple_bridge = (
            "assertz(( triple(S,P,O) :- "
            "  (nonvar(S) -> (ensure_uri(S, S_U), S_Q = [s, S_U]) ; S_Q = []), "
            "  (nonvar(P) -> (ensure_uri(P, P_U), P_Q = [p, P_U]) ; P_Q = []), "
            "  (nonvar(O) -> (ensure_uri(O, O_U), O_Q = [o, O_U]) ; O_Q = []), "
            "  append([S_Q, P_Q, O_Q], QueryList), "
            "  mng_client:mng_find(neems, triples, QueryList, Doc), "
            "  mng_client:mng_get_dict('s', Doc, S), "
            "  mng_client:mng_get_dict('p', Doc, P), "
            "  mng_client:mng_get_dict('o', Doc, O) ))"
        )
        
        run_simple_query("retractall(triple(_,_,_))")
        run_simple_query(triple_bridge)
        run_simple_query("retractall(has_type(_,_))")
        run_simple_query("assertz(( has_type(S, T) :- triple(S, rdf:type, T) ))")
        
        print("Verbindung bereit. Präfix-Abfragen sind jetzt stabil.")

def on_run_query(_):
    global current_query_id
    with output:
        clear_output()
        query_str = query_input.value.strip().rstrip('.')
        print(f"Abfrage: {query_str}")
        try:
            if current_query_id:
                get_prolog_service('/rosprolog/finish', PrologFinish)(id=current_query_id)
            current_query_id = "q_" + uuid.uuid4().hex[:4]
            srv = get_prolog_service('/rosprolog/query', PrologQuery)
            if srv:
                srv(id=current_query_id, query=f"{query_str}.")
                on_next_solution(None)
        except Exception as e: print(f" Fehler: {e}")

def on_next_solution(_):
    global current_query_id
    with output:
        if not current_query_id: return
        srv = get_prolog_service('/rosprolog/next_solution', PrologNextSolution)
        res = srv(id=current_query_id)
        if res and res.status == 3:
            sol = json.loads(res.solution)
            if not sol: print("true.")
            else:
                for k, v in sol.items(): print(f"   {k} = {format_term(v)}")
        elif res and res.status == 0:
            print("Keine weiteren Lösungen.")
            current_query_id = None

def refresh_neems(_=None):
    if os.path.exists(NEEM_BASE_PATH):
        dirs = [d for d in os.listdir(NEEM_BASE_PATH) if os.path.isdir(os.path.join(NEEM_BASE_PATH, d))]
        neem_dropdown.options = sorted(dirs)

refresh_btn.on_click(refresh_neems)
connect_btn.on_click(on_connect_clicked)
import_btn.on_click(on_import_clicked)
run_btn.on_click(on_run_query)
next_btn.on_click(on_next_solution)
refresh_neems()