def create_query(code, query_id='q123'):
    if ":-" in code:
        return f"cloud_consult_string({query_id},\"{code}\")"
    return code
