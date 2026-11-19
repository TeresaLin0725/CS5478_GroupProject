import io
import json
import unittest
from unittest import mock

from frontier_search.policies import transport


class _Response(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def _run(message):
    payload = {'model': 'm', 'usage': {'total_tokens': 3}, 'choices': [{'message': message}]}
    request = {'url': 'https://example.invalid/chat/completions', 'key': 'k',
               'body': {'model': 'm'}, 'timeout': 5}
    out = io.StringIO()
    with mock.patch.object(transport.urllib.request, 'urlopen',
                           return_value=_Response(json.dumps(payload).encode())), \
         mock.patch('sys.stdin', io.StringIO(json.dumps(request))), \
         mock.patch('sys.stdout', out):
        transport.main()
    return json.loads(out.getvalue())


class TransportReasoningTest(unittest.TestCase):
    def test_reasoning_content_is_recorded(self):
        output = _run({'role': 'assistant', 'content': '{"frontier_id":"F1","reason":"r"}',
                       'reasoning_content': 'think step by step'})
        self.assertEqual(output['reasoning_content'], 'think step by step')
        self.assertEqual(output['raw_response'], '{"frontier_id":"F1","reason":"r"}')

    def test_absent_reasoning_adds_no_field(self):
        output = _run({'role': 'assistant', 'content': 'x'})
        self.assertNotIn('reasoning_content', output)
        self.assertNotIn('reasoning', output)


if __name__ == '__main__':
    unittest.main()
