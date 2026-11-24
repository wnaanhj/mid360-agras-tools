# SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
"""离线检查公开仓库的受控文件、校验值和禁止入包项；不读取仓库外的文件。"""
from pathlib import Path
import hashlib
import json
import re

root = Path(__file__).resolve().parents[1]
manifest = json.loads((root / 'SOURCE_MANIFEST.json').read_text(encoding='utf-8'))
errors = []
expected = {item['path']: item for item in manifest['files']}
ignored_dirs = {'.git', '.venv', 'venv', '__pycache__', 'build', 'test-output'}
for rel, item in expected.items():
    path = root / rel
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
        errors.append('文件缺失或内容变化：' + rel)
for path in root.rglob('*'):
    rel = path.relative_to(root)
    if set(rel.parts) & ignored_dirs:
        continue
    if path.is_symlink():
        errors.append('不允许符号链接：' + rel.as_posix())
        continue
    if not path.is_file():
        continue
    name = rel.as_posix()
    if name != 'SOURCE_MANIFEST.json' and name not in expected:
        errors.append('未登记的文件，请核查后更新清单：' + name)
    if path.suffix.lower() in {'.apk','.aab','.dll','.so','.exe','.jks','.keystore','.pcap','.pcapng','.t100rec','.sqlite','.key','.pem'}:
        errors.append('不应发布的数据/二进制/凭据：' + name)
    if path.stat().st_size > 10 * 1024 * 1024:
        errors.append('超出本源码子集预期大小：' + name)
    if path.suffix in {'.py','.java','.md','.json','.ps1','.txt'} and path.name != 'check_release.py':
        text = path.read_text(encoding='utf-8-sig')
        if re.search(r'(?i)(?:[a-z]:[\\/]Users[\\/]|/home/[^/\s]+/|-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----)', text):
            errors.append('疑似个人路径或私钥标记：' + name)
        if re.search(r'(?i)(?:sk-[a-z0-9]{20,}|gh[pousr]_[a-z0-9]{20,}|AKIA[A-Z0-9]{16})', text):
            errors.append('疑似访问令牌：' + name)
if errors:
    for message in errors:
        print(message)
    raise SystemExit(1)
print(f'PASS: {len(expected)}个受控文件哈希一致；未发现本检查器定义的禁止入包项。')
