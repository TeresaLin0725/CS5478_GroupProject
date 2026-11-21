import importlib.util
import json
from pathlib import Path
from socket import timeout as SocketTimeout
import sys


ROOT = Path(__file__).resolve().parents[2]


def _load_script():
    path = ROOT / 'scripts' / 'check_llm_connection.py'
    spec = importlib.util.spec_from_file_location('check_llm_connection', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_authoritative_model_is_gemini_35():
    module = _load_script()
    assert module.CANONICAL_LLM_MODEL == 'gemini-3.5-flash'


def test_socket_timeout_is_reported_without_traceback(monkeypatch, tmp_path, capsys):
    module = _load_script()
    output = tmp_path / 'connection.json'
    monkeypatch.setenv('LLM_API_KEY', 'test-secret')
    monkeypatch.setenv('LLM_BASE_URL', 'https://example.invalid/v1beta/openai')
    monkeypatch.setattr(module, 'urlopen', lambda *args, **kwargs: (_ for _ in ()).throw(
        SocketTimeout('connection timed out')))
    monkeypatch.setattr(sys, 'argv', [
        'check_llm_connection.py', '--env-file', str(tmp_path / 'missing.env'),
        '--output', str(output),
    ])

    assert module.main() == 1
    report = json.loads(output.read_text(encoding='utf-8'))
    assert report['live_model_available'] is False
    assert report['transport_error'] == 'TimeoutError'
    assert report['reason'] == 'connection timed out'
    assert 'test-secret' not in output.read_text(encoding='utf-8')
    assert json.loads(capsys.readouterr().out) == report
