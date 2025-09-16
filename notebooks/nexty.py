# Erforderliche Importe
# Stellen Sie sicher, dass alle diese Bibliotheken in Ihrer Umgebung installiert sind.
# rospy und die json_prolog_msgs müssen in Ihrer ROS-Umgebung verfügbar sein.
import ipywidgets as widgets
from IPython.display import display
from pprint import pprint
import json
import rospy
import string
import random
import subprocess
import os
import time

# WICHTIG: Die folgenden Importe setzen voraus, dass Ihr Jupyter-Notebook in einer
# Umgebung läuft, in der ROS und die entsprechenden Message-Pakete verfügbar sind.
# Wenn Sie diesen Code außerhalb einer Catkin-Workspace ausführen, müssen Sie
# sicherstellen, dass die Python-Pfade korrekt gesetzt sind.
from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologNextSolutionResponse, PrologFinish

# ==============================================================================
# Klasse: KnowRobClient
# Zweck: Kapselt die gesamte Kommunikation mit dem rosprolog-Service.
# Dies macht den Hauptcode sauberer und einfacher zu verwalten.
# ==============================================================================
class KnowRobClient:
    """
    Ein Client zur Interaktion mit einem KnowRob-Prolog-Service über ROS.
    """
    def __init__(self, name_space='rosprolog'):
        self.name_space = name_space
        self.current_query_id = None
        self._is_query_active = False

        # Initialisiert den ROS-Node. Ein eindeutiger Name verhindert Konflikte,
        # wenn mehrere Notebooks gleichzeitig laufen.
        try:
            rospy.init_node('jupyter_knowrob_client', anonymous=True)
            rospy.loginfo("ROS-Node 'jupyter_knowrob_client' initialisiert.")
        except rospy.exceptions.ROSException:
            rospy.logwarn("ROS-Node bereits initialisiert. Fahre fort.")

        # Einrichten der Service-Proxies für die Kommunikation mit rosprolog
        self._query_srv = rospy.ServiceProxy(f'/{self.name_space}/query', PrologQuery)
        self._next_solution_srv = rospy.ServiceProxy(f'/{self.name_space}/next_solution', PrologNextSolution)
        self._finish_srv = rospy.ServiceProxy(f'/{self.name_space}/finish', PrologFinish)

        # Warten, bis die ROS-Services verfügbar sind
        rospy.loginfo("Warte auf rosprolog-Services...")
        self._query_srv.wait_for_service(timeout=10.0)
        self._next_solution_srv.wait_for_service(timeout=10.0)
        self._finish_srv.wait_for_service(timeout=10.0)
        rospy.loginfo("rosprolog-Services sind bereit.")

    def _generate_id(self):
        """Erzeugt eine eindeutige ID für jede Abfrage."""
        timestamp = rospy.Time.now().to_nsec()
        random_str = ''.join(random.choice(string.ascii_lowercase) for _ in range(5))
        return f"jupyter_{timestamp}_{random_str}"

    def start_query(self, query_string):
        """
        Startet eine neue Prolog-Abfrage.
        Beendet jede vorherige, noch laufende Abfrage.
        """
        if self._is_query_active:
            self.finish_query()

        self.current_query_id = self._generate_id()
        self._is_query_active = True
        
        try:
            # Sendet die Abfrage an den Prolog-Service
            self._query_srv(id=self.current_query_id, query=query_string)
            rospy.loginfo(f"Abfrage gestartet mit ID: {self.current_query_id}")
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Fehler beim Starten der Abfrage: {e}")
            self._is_query_active = False
            return False

    def next_solution(self):
        """
        Holt die nächste verfügbare Lösung für die aktive Abfrage.
        Gibt die Lösung als Dictionary zurück, oder None, wenn keine weiteren
        Lösungen existieren oder ein Fehler auftritt.
        """
        if not self._is_query_active:
            rospy.logwarn("Keine aktive Abfrage, um eine Lösung abzurufen.")
            return None

        try:
            response = self._next_solution_srv(id=self.current_query_id)
            
            if response.status == PrologNextSolutionResponse.OK:
                solution = json.loads(response.solution)
                return solution if solution else {"result": "true"}
            
            elif response.status == PrologNextSolutionResponse.NO_SOLUTION:
                rospy.loginfo("Keine weiteren Lösungen gefunden.")
                self.finish_query()
                return None
            
            else: # QUERY_FAILED oder WRONG_ID
                rospy.logerr(f"Fehler bei der Abfrage: {response.solution}")
                self.finish_query()
                return None

        except rospy.ServiceException as e:
            rospy.logerr(f"Service-Fehler beim Abrufen der nächsten Lösung: {e}")
            self.finish_query()
            return None

    def finish_query(self):
        """Informiert den Server, die aktive Abfrage zu beenden."""
        if not self._is_query_active:
            return
            
        try:
            self._finish_srv(id=self.current_query_id)
            rospy.loginfo(f"Abfrage beendet: {self.current_query_id}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Fehler beim Beenden der Abfrage: {e}")
        finally:
            self._is_query_active = False
            self.current_query_id = None


# ==============================================================================
# Funktion: launch_ui
# Zweck: Erstellt und zeigt die ipywidgets-Benutzeroberfläche an.
# ==============================================================================
def launch_ui():
    """
    Startet die Benutzeroberfläche im Jupyter Notebook.
    """
    # Zustandsobjekt, um Client und Prozesse zu speichern
    state = {'client': None, 'roscore_proc': None, 'knowrob_proc': None}

    # --- Widgets für die Benutzeroberfläche ---
    start_ros_button = widgets.Button(description="1. Start roscore & KnowRob", button_style='danger', layout=widgets.Layout(width='100%'))
    connect_client_button = widgets.Button(description="2. Connect to KnowRob", button_style='warning', layout=widgets.Layout(width='100%'))
    
    neem_id_input = widgets.Text(value="62d5729bb3869a9a9c942f24", description="NEEM-ID:", layout=widgets.Layout(width='100%'))
    
    query_input = widgets.Textarea(
        value="triple(Tsk, rdf:type, dul:'Action'), triple(Tsk, soma:hasExecutionState, State)",
        placeholder="Geben Sie Ihre Prolog-Abfrage hier ein.",
        layout=widgets.Layout(width='100%', height='100px')
    )
    run_button = widgets.Button(description="Run Query", button_style='success')
    next_button = widgets.Button(description="Next Solution", button_style='info')
    
    output_area = widgets.Output()

    # Alle Buttons außer dem ersten sind anfangs deaktiviert
    connect_client_button.disabled = True
    run_button.disabled = True
    next_button.disabled = True

    # --- Hilfsfunktion zum Starten von Prozessen ---
    def start_process(command, process_key, log_name):
        with output_area:
            if state[process_key] and state[process_key].poll() is None:
                print(f"ℹ️ {log_name}-Prozess scheint bereits zu laufen.")
                return True
            
            print(f"→ Starte {log_name} mit Befehl: {' '.join(command)}")
            try:
                proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                state[process_key] = proc
                return True
            except FileNotFoundError:
                print(f"❌ Fehler: '{command[0]}' wurde nicht gefunden. Stellen Sie sicher, dass Ihre ROS-Umgebung korrekt eingerichtet ist.")
                return False
            except Exception as e:
                print(f"❌ Ein unerwarteter Fehler ist aufgetreten: {e}")
                return False

    # --- Callback-Funktionen für die Buttons ---
    
    def start_ros_system(_):
        """Startet roscore und den rosprolog-Service über roslaunch."""
        output_area.clear_output()
        
        # Starte roscore
        if not start_process(['roscore'], 'roscore_proc', 'roscore'):
            return
        
        with output_area:
            print("Warte 2 Sekunden, bis roscore initialisiert ist...")
        time.sleep(2)

        # Starte KnowRob
        command = ["roslaunch", "knowrob", "knowrob.launch"]
        if not start_process(command, 'knowrob_proc', 'KnowRob'):
            return
        
        with output_area:
            print("✅ ROS-System gestartet. Bitte warten Sie ca. 10 Sekunden, bevor Sie auf 'Connect' klicken.")
            connect_client_button.disabled = False


    def connect_to_knowrob(_):
        """Erstellt die Client-Instanz und verbindet sich mit KnowRob."""
        with output_area:
            output_area.clear_output()
            print("Versuche, eine Verbindung zu den KnowRob-Services herzustellen...")
            try:
                state['client'] = KnowRobClient()
                print("✅ Erfolgreich mit KnowRob verbunden.")
                # Aktiviere die nächsten Schritte
                run_button.disabled = False
            except Exception as e:
                print(f"❌ Verbindung fehlgeschlagen. Stellen Sie sicher, dass roscore und der KnowRob-Service laufen.\nDetails: {e}")

    def run_query(_):
        """
        Konstruiert eine Abfrage mit dem NEEM-Kontext und führt sie aus.
        """
        if not state['client']: return
        output_area.clear_output()
        
        neem_id = neem_id_input.value.strip()
        # Entferne Leerzeichen und dann einen optionalen Punkt am Ende.
        user_query = query_input.value.strip().rstrip('.')

        if not neem_id:
            with output_area: print("❌ Bitte geben Sie eine NEEM-ID an.")
            return
        if not user_query:
            with output_area: print("❌ Bitte geben Sie eine Abfrage ein.")
            return

        collection_name = f"{neem_id}_triples"
        
        # Fallback auf 'in_graph/2', ein fundamentaleres Prädikat zum Abfragen
        # von benannten Graphen.
        final_query = f"in_graph('{collection_name}', ({user_query}))."

        with output_area:
            print(f"Starte Abfrage in Collection '{collection_name}':")
            print(f"Sende Prolog-Abfrage: {final_query}")
            
            if state['client'].start_query(final_query):
                print("Abfrage erfolgreich gesendet. Klicken Sie auf 'Next Solution'.")
                next_button.disabled = False
            else:
                print("❌ Fehler beim Starten der Abfrage.")
                next_button.disabled = True

    def get_next_solution(_):
        """Ruft die nächste Lösung von KnowRob ab und zeigt sie an."""
        if not state['client']: return
        with output_area:
            solution = state['client'].next_solution()
            if solution is not None:
                print("\n--- Nächste Lösung ---")
                pprint(solution)
            else:
                print("\n✅ Keine weiteren Lösungen oder Abfrage beendet.")
                next_button.disabled = True

    # --- Buttons mit den Callback-Funktionen verbinden ---
    start_ros_button.on_click(start_ros_system)
    connect_client_button.on_click(connect_to_knowrob)
    run_button.on_click(run_query)
    next_button.on_click(get_next_solution)

    # --- UI-Layout zusammenstellen und anzeigen ---
    ui = widgets.VBox([
        widgets.Label("Schritt 1: ROS-System starten (falls noch nicht geschehen)"),
        start_ros_button,
        widgets.HTML("<hr>"),
        widgets.Label("Schritt 2: Mit dem KnowRob-Service verbinden"),
        connect_client_button,
        widgets.HTML("<hr>"),
        widgets.Label("Schritt 3: Abfrage ausführen"),
        widgets.Label("Geben Sie die NEEM-ID und die Prolog-Abfrage an."),
        neem_id_input,
        query_input,
        widgets.HBox([run_button, next_button]),
        widgets.HTML("<hr>"),
        widgets.Label("Ausgabe:"),
        output_area
    ])
    
    display(ui)

# --- Startpunkt ---
# Rufen Sie diese Funktion in einer Zelle Ihres Jupyter-Notebooks auf,
# um die Benutzeroberfläche zu starten.
#
# launch_ui()

