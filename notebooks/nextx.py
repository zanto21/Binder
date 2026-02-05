import os, json, uuid, traceback, subprocess, time
import ipywidgets as widgets
from IPython.display import display, clear_output

# Für BSON-Lesen
try:
    import bson
    from bson.codec_options import CodecOptions
    BSON_OPTIONS = CodecOptions(unicode_decode_error_handler='replace')
except ImportError:
    print("'bson' Modul nicht gefunden. Bitte 'pip install pymongo' ausführen.")

# --- Optional für Server/KnowRob ---
try:
    import rospy
    from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologFinish
except ImportError:
    print("ROS/KnowRob Module nicht gefunden. Server-Modus wird eingeschränkt sein.")

# --- Basis-Pfad zu deinen lokalen NEEMs ---
NEEM_BASE_PATH = "/home/jovyan/work/notebooks/neems"

# --- UI Elemente ---
mode_select = widgets.ToggleButtons(
    options=["Lokale NEEMs (BSON)", "Server/KnowRob"],
    description="Modus:",
    style={'description_width': 'initial'}
)

neem_dropdown = widgets.Dropdown(description="Lokaler NEEM:", style={'description_width':'initial'})
refresh_btn = widgets.Button(description="Refresh NEEM-Liste", button_style="info")
folder_select = widgets.Dropdown(
    options=["annotations", "inferred", "ros_tf", "triples"],
    description="Unterordner:",
    style={'description_width':'initial'},
    layout=widgets.Layout(width="60%")
)

local_query_input = widgets.Textarea(
    value="data['triples.bson'][0] if 'triples.bson' in data else 'Datei nicht gefunden'",
    placeholder="Python-Ausdruck auf 'data' auswerten …",
    layout=widgets.Layout(width="100%", height="90px")
)
local_run_btn = widgets.Button(description="Run Local Query", button_style='success')

server_neem_input = widgets.Text(value="", placeholder="NEEM-ID für Server/KnowRob")
server_connect_btn = widgets.Button(description="Connect Server", button_style="info")
server_query_input = widgets.Textarea(
    placeholder="Prolog-Query eingeben …",
    layout=widgets.Layout(width="100%", height="90px")
)
server_run_btn = widgets.Button(description="Run Remote Query", button_style='success')
server_next_btn = widgets.Button(description="Next Solution", button_style='warning')

output = widgets.Output()

ui = widgets.VBox([
    mode_select,
    widgets.HBox([refresh_btn, neem_dropdown, folder_select]),
    widgets.Label("Lokale Query (Python):"),
    local_query_input,
    local_run_btn,
    widgets.HTML("<hr>"),
    widgets.Label("Server/KnowRob (Prolog):"),
    server_neem_input,
    server_connect_btn,
    server_query_input,
    widgets.HBox([server_run_btn, server_next_btn]),
    output
])
display(ui)


# --- Lokale NEEM (BSON) Funktionen ---

def refresh_local_neem_list(_=None):
    try:
        if not os.path.exists(NEEM_BASE_PATH):
            os.makedirs(NEEM_BASE_PATH, exist_ok=True)
        neem_dirs = sorted([
            d for d in os.listdir(NEEM_BASE_PATH)
            if os.path.isdir(os.path.join(NEEM_BASE_PATH, d))
        ])
        neem_dropdown.options = neem_dirs
    except Exception as e:
        neem_dropdown.options = []
        with output:
            print("Fehler beim Lesen lokaler NEEMs:", e)

refresh_btn.on_click(refresh_local_neem_list)
refresh_local_neem_list()

def load_neem_data(neem_name, subfolder):
    """Lädt BSON und JSON Daten mit robuster Behandlung von Dekodierungsfehlern."""
    data = {}
    base = os.path.join(NEEM_BASE_PATH, neem_name, "data", subfolder)

    if not os.path.isdir(base):
        return data

    for root, dirs, files in os.walk(base):
        for fn in files:
            full_path = os.path.join(root, fn)
            key = fn 

            if fn.endswith(".bson"):
                try:
                    with open(full_path, "rb") as f:
                        content = f.read()
                        if not content: continue
                        
                        docs = []
                        offset = 0
                        # Wir nutzen die robusten CodecOptions
                        while offset < len(content):
                            try:
                                # Größe des BSON-Dokuments lesen
                                doc_size = int.from_bytes(content[offset:offset+4], 'little')
                                if doc_size <= 0 or offset + doc_size > len(content):
                                    break
                                
                                doc_data = content[offset:offset+doc_size]
                                # Dekodieren mit Fehlertoleranz für Sonderzeichen
                                decoded_doc = bson.decode(doc_data, codec_options=BSON_OPTIONS)
                                docs.append(decoded_doc)
                                offset += doc_size
                            except Exception as doc_err:
                                # Falls ein Dokument extrem korrupt ist, überspringen wir 1 Byte und suchen weiter
                                offset += 1 
                        
                        if docs:
                            data[key] = docs
                except Exception as e:
                    with output: print(f"⚠️ Kritischer Fehler bei {fn}: {str(e)}")

            elif fn.endswith(".json"):
                try:
                    with open(full_path, "r", encoding="utf-8", errors="replace") as f:
                        data[key] = json.load(f)
                except Exception as e:
                    with output: print(f"⚠️ Fehler beim Parsen von {fn}: {str(e)}")
    return data

def on_local_run(_):
    with output:
        clear_output()
        if mode_select.value != "Lokale NEEMs (BSON)":
            print("Bitte zuerst den lokalen Modus auswählen.")
            return

        neem = neem_dropdown.value
        subfolder = folder_select.value

        if not neem:
            print("Wähle zuerst einen lokalen NEEM.")
            return

        print(f"Analysiere {neem}/data/{subfolder}...")
        data = load_neem_data(neem, subfolder)
        
        if not data:
            print("Keine lesbaren Dateien gefunden.")
            return

        print(f"Verfügbare Dateien: {list(data.keys())}")

        expr = local_query_input.value.strip()
        if not expr: return

        try:
            # Evaluiere Ausdruck
            result = eval(expr, {"data": data, "json": json, "os": os})
            print("\n ERGEBNIS:")
            pprint_result(result)
        except Exception as e:
            print(f"Fehler bei Query: {e}")

def pprint_result(obj):
    """Formatierte Ausgabe für die Konsole."""
    from pprint import pprint
    if isinstance(obj, list):
        print(f"Liste mit {len(obj)} Einträgen.")
        if len(obj) > 0:
            print("Erster Eintrag:")
            pprint(obj[0])
    else:
        pprint(obj)

local_run_btn.on_click(on_local_run)

# --- Server/KnowRob Funktionen ---

current_query_id = None
procs = []

def start_ros_if_needed():
    global procs
    if not any(p.poll() is None for p in procs):
        try:
            subprocess.check_call(['rosnode', 'list'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except:
            procs.append(subprocess.Popen(['roscore'], preexec_fn=os.setsid))
            time.sleep(3)
        procs.append(subprocess.Popen(['roslaunch','knowrob','knowrob.launch'], preexec_fn=os.setsid))
        time.sleep(5)

def on_server_connect(_):
    with output:
        clear_output()
        if mode_select.value != "Server/KnowRob": return
        start_ros_if_needed()
        try:
            if not rospy.core.is_initialized():
                rospy.init_node('next_gui_server', anonymous=True)
            neem_id = server_neem_input.value.strip()
            sid = uuid.uuid4().hex[:6]
            # Wir nutzen hier remember/1 für neuere KnowRob Versionen
            query = f"remember('{neem_id}')"
            srv = rospy.ServiceProxy('/rosprolog/query', PrologQuery)
            srv(id=sid, query=query)
            print(f"NEEM-Load Befehl gesendet.")
        except Exception as e:
            print("Fehler:", e)

server_connect_btn.on_click(on_server_connect)

def on_server_run(_):
    global current_query_id
    with output:
        clear_output()
        q = server_query_input.value.strip()
        if not q: return
        current_query_id = uuid.uuid4().hex[:6]
        try:
            srv = rospy.ServiceProxy('/rosprolog/query', PrologQuery)
            srv(id=current_query_id, query=q)
            print("Remote Query gestartet.")
        except Exception as e:
            print("Fehler:", e)

server_run_btn.on_click(on_server_run)

def on_server_next(_):
    with output:
        if not current_query_id: return
        try:
            srv = rospy.ServiceProxy('/rosprolog/next_solution', PrologNextSolution)
            res = srv(id=current_query_id)
            if res.solution:
                print("Lösung:", json.loads(res.solution))
            else:
                print("Keine weiteren Lösungen.")
        except Exception as e:
            print("Fehler:", e)

server_next_btn.on_click(on_server_next)