from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologFinish
import rospy
import json
from query_utils import create_query

def run_query(code, query_id='q123', namespace='rosprolog'):
    """
    Führt eine Prolog-Query über rosprolog aus und gibt die Lösung(en) zurück.
    """
    rospy.wait_for_service(f'{namespace}/query')
    rospy.wait_for_service(f'{namespace}/next_solution')
    rospy.wait_for_service(f'{namespace}/finish')

    query_srv = rospy.ServiceProxy(f'{namespace}/query', PrologQuery)
    next_srv = rospy.ServiceProxy(f'{namespace}/next_solution', PrologNextSolution)
    finish_srv = rospy.ServiceProxy(f'{namespace}/finish', PrologFinish)

    query = create_query(code, query_id)
    query_srv(id=query_id, query=query)

    while True:
        result = next_srv(id=query_id)
        if result.status == PrologNextSolution._response_class.OK:
            yield json.loads(result.solution)
        else:
            break

    finish_srv(id=query_id)
