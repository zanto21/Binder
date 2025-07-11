from pymongo import MongoClient
from IPython.display import display
import ipywidgets as widgets
from pprint import pprint

# Verbindung zur MongoDB (NEEM-Server)
client = MongoClient("mongodb://neemReader:qEWRqc9UdN5TD7No7cjymUA8QEweNz@neem-3.informatik.uni-bremen.de:28015/neems")
db = client["neems"]
collection = db["62d5729bb3869a9a9c942f24_triples"]

def launch():
    # UI-Elemente
    description = widgets.HTML(
        value="<b>MongoDB-Query als Dictionary eingeben (z.B. {'p': 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#executesTask'}):</b>",
        layout=widgets.Layout(margin='10px 0px 5px 0px')
    )

    query_input = widgets.Textarea(
        value="{}",
        placeholder="MongoDB-Query z. B. {'p': '...'}",
        description='Query:',
        layout=widgets.Layout(width='100%', height='80px')
    )

    run_button = widgets.Button(description="Run Query", button_style='success')
    next_button = widgets.Button(description="Next Solution", button_style='info', disabled=True)
    output_area = widgets.Output()

    # Interner Iterator
    result_cursor = {'cursor': None}

    def run_query(_):
        output_area.clear_output()
        try:
            mongo_query = eval(query_input.value)
            cursor = collection.find(mongo_query)
            result_cursor['cursor'] = iter(cursor)
            next_button.disabled = False

            with output_area:
                print(f"Abfrage gestartet:\n{mongo_query}")
        except Exception as e:
            with output_area:
                print(f"❌ Fehler: {e}")
            result_cursor['cursor'] = None
            next_button.disabled = True

    def next_solution(_):
        if result_cursor['cursor'] is None:
            return

        try:
            doc = next(result_cursor['cursor'])
            with output_area:
                pprint(doc)
        except StopIteration:
            with output_area:
                print("✅ Keine weiteren Ergebnisse.")
            next_button.disabled = True
        except Exception as e:
            with output_area:
                print(f"❌ Fehler: {e}")
            next_button.disabled = True

    # Button-Events
    run_button.on_click(run_query)
    next_button.on_click(next_solution)

    # Aufbau der UI
    controls = widgets.HBox([run_button, next_button])
    ui = widgets.VBox([description, query_input, controls, output_area])
    display(ui)
