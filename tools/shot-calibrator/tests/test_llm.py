from shotlab.llm import _extract_json


def test_extract_json_accepts_fenced_payload():
    parsed = _extract_json(
        """```json
        {"recommended_parameters":{"drag_scale":1.1},"confidence":"low"}
        ```"""
    )
    assert parsed["recommended_parameters"]["drag_scale"] == 1.1
