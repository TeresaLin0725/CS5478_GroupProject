"""Keep submitted filenames and explanatory text in English."""
import json
from pathlib import Path
import re
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[2]
CJK = re.compile('[\u3000-\u303f\uff01-\uffef\u3400-\u4dbf\u4e00-\u9fff\uf900-\ufaff\U00020000-\U000323af]')
TEXT_EXTENSIONS = {'.py', '.js', '.html', '.css', '.md', '.txt', '.json', '.jsonl',
                   '.csv', '.toml', '.yaml', '.yml', '.svg', '.ps1', '.cmd'}


def contains_cjk(value):
    if isinstance(value, str):
        return bool(CJK.search(value))
    if isinstance(value, list):
        return any(contains_cjk(item) for item in value)
    if isinstance(value, dict):
        return any(contains_cjk(key) or contains_cjk(item) for key, item in value.items())
    return False


class RepositoryLanguageTests(unittest.TestCase):
    def test_tracked_names_and_text_are_english(self):
        result = subprocess.run(['git', 'ls-files', '-z'], cwd=str(ROOT), check=True,
                                stdout=subprocess.PIPE)
        problems = []
        for name in result.stdout.decode('utf-8').strip('\0').split('\0'):
            path = ROOT/name
            if not path.is_file():
                continue
            if CJK.search(name):
                problems.append(name + ': filename')
            if path.suffix.lower() not in TEXT_EXTENSIONS:
                continue
            content = path.read_text(encoding='utf-8-sig')
            if CJK.search(content):
                problems.append(name + ': text')
            if path.suffix.lower() == '.json' and contains_cjk(json.loads(content)):
                problems.append(name + ': decoded JSON')
            elif path.suffix.lower() == '.jsonl':
                if any(contains_cjk(json.loads(line)) for line in content.splitlines() if line.strip()):
                    problems.append(name + ': decoded JSONL')
        self.assertEqual(problems, [])


if __name__ == '__main__':
    unittest.main()
