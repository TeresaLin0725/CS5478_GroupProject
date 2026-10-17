"""Check the production LLM configuration without exposing credentials."""
import argparse
import json
import os
from pathlib import Path
import sys
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from frontier_search.policies.provider import GeminiCompatibleClient, load_local_env


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--env-file', type=Path, default=ROOT.parent / '.env')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    names = ('LLM_API_KEY', 'LLM_MODEL', 'LLM_BASE_URL')
    inherited = {name: bool(os.environ.get(name)) for name in names}
    load_local_env(args.env_file)
    client = GeminiCompatibleClient()
    model = os.environ.get('LLM_MODEL', 'gemini-2.5-flash')
    report = dict(env_file=str(args.env_file.resolve()), env_file_exists=args.env_file.is_file(),
                  inherited_variables=inherited, endpoint_host=urlsplit(client.base_url).hostname,
                  model=model, live_model_available=False)
    body = dict(model=model, messages=[dict(role='user', content='Reply with OK.')], max_tokens=16)
    request = Request(client.base_url + '/chat/completions',
                      data=json.dumps(body).encode('utf-8'),
                      headers={'Authorization': 'Bearer ' + client.key,
                               'Content-Type': 'application/json'})
    try:
        with urlopen(request, timeout=30) as response:
            payload = json.loads(response.read().decode('utf-8'))
            report.update(http_status=response.status, live_model_available=True,
                          response_present=bool(payload.get('choices')), usage=payload.get('usage'))
    except HTTPError as exc:
        # Keep the provider's reason, but never serialize request headers or keys.
        detail = exc.read().decode('utf-8', errors='replace').replace(client.key, '[REDACTED]')
        report.update(http_status=exc.code, provider_error=detail[:4096])
    except (URLError, TimeoutError) as exc:
        report.update(transport_error=type(exc).__name__,
                      reason=str(exc).replace(client.key, '[REDACTED]')[:1000])
    encoded = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding='utf-8')
    print(encoded)
    return int(not report['live_model_available'])


if __name__ == '__main__':
    raise SystemExit(main())
