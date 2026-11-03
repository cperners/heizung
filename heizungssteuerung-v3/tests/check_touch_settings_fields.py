"""Check that all named web setting inputs also exist in the touch forms."""
from pathlib import Path
import re
root = Path(__file__).resolve().parents[1]
web = (root / "main/web/index.html").read_text()
touch = (root / "main/touch_settings.c").read_text()
expected = set()
for group in re.findall(r'data-fields="([^"]+)"', web):
    expected.update(group.split(","))
expected.update(re.findall(r'<(?:input|select)[^>]*\bname="([a-z_]+)"', web))
expected.discard("mode")  # Existing dedicated public operating-mode page.
actual = set(re.findall(r'F\("([a-z_]+)"', touch))
missing = expected - actual
assert not missing, f"Missing touch settings: {sorted(missing)}"
assert 'current==REG_TIME' in touch and '"periods"' in touch
print(f"PASS: {len(expected)} named web fields covered; separate mode/time pages retained.")
