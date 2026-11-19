"""Private HTTP worker. Parent enforces an absolute deadline and kills on timeout."""
import json
import sys
import urllib.error
import urllib.request


def main() -> None:
    try:
        request = json.load(sys.stdin)
        http = urllib.request.Request(request['url'],
            data=json.dumps(request['body']).encode('utf-8'),
            headers={'Content-Type': 'application/json',
                     'Authorization': 'Bearer ' + request['key']})
        with urllib.request.urlopen(http, timeout=request['timeout']) as response:
            payload = json.load(response)
        message = payload['choices'][0]['message']
        output = {'raw_response': message['content'],
                  'usage': payload.get('usage'), 'response_model': payload.get('model')}
        # Record-only: keep the provider's chain of thought when it exposes one
        # (DeepSeek: reasoning_content; some gateways: reasoning). Never sent back.
        for field in ('reasoning_content', 'reasoning'):
            if message.get(field):
                output[field] = message[field]
    except urllib.error.HTTPError as exc:
        output = {'error': 'HTTP_' + str(exc.code)}
    except Exception as exc:
        output = {'error': type(exc).__name__}
    # Do not print raw HTTP errors, headers, request objects, or traceback locals.
    print(json.dumps(output))


if __name__ == '__main__':
    main()
