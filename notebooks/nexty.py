from pymongo import MongoClient
from IPython.display import display
import ipywidgets as widgets
from pprint import pprint

def launch():
    # Widgets zur Interaktion
    neem_id_input = widgets.Text(
        value="62d5729bb3869a9a9c942f24",
        description="NEEM-ID:",
        layout=widgets.Layout(width='100%')
    )

    query_input = widgets.Textarea(
        value="{}",
        placeholder="MongoDB-Abfrage (als Dictionary)",
        layout=widgets.Layout(width='100%', height='80px')
    )

    connect_button = widgets.Button(description="Connect NEEM", button_style='primary')
    run_button = widgets.Button(description="Run Query", button_style='success', disabled=True)
    next_button = widgets.Button(description="Next Solution", button_style='info', disabled=True)
    output_area = widgets.Output()

    # Cursor und Collection
    state = {
        'collection': None,
        'cursor': None
    }

    def connect_neem(_):
        output_area.clear_output()
        neem_id = neem_id_input.value.strip()
        try:
            client = MongoClient("mongodb://neemReader:qEWRqc9UdN5TD7No7cjymUA8QEweNz@neem-3.informatik.uni-bremen.de:28015/neems")
            db = client["neems"]
            state['collection'] = db[f"{neem_id}_triples"]
            with output_area:
                print(f"✅ Verbunden mit NEEM-ID: {neem_id}")
            run_button.disabled = False
        except Exception as e:
            with output_area:
                print(f"❌ Verbindung fehlgeschlagen:\n{e}")
            run_button.disabled = True
            next_button.disabled = True

    def run_query(_):
        output_area.clear_output()
        try:
            query_dict = eval(query_input.value)
            state['cursor'] = iter(state['collection'].find(query_dict))
            next_button.disabled = False
            with output_area:
                print(f"Abfrage gestartet:\n{query_dict}")
        except Exception as e:
            with output_area:
                print(f"❌ Fehler beim Starten der Abfrage:\n{e}")
            state['cursor'] = None
            next_button.disabled = True

    def next_solution(_):
        if state['cursor'] is None:
            return
        try:
            doc = next(state['cursor'])
            with output_area:
                pprint(doc)
        except StopIteration:
            with output_area:
                print("✅ Keine weiteren Ergebnisse.")
            next_button.disabled = True
        except Exception as e:
            with output_area:
                print(f"❌ Fehler:\n{e}")
            next_button.disabled = True

    connect_button.on_click(connect_neem)
    run_button.on_click(run_query)
    next_button.on_click(next_solution)

    ui = widgets.VBox([
        neem_id_input,
        connect_button,
        query_input,
        widgets.HBox([run_button, next_button]),
        output_area
    ])
    display(ui)
