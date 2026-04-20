#!/usr/bin/env python3
"""
给 source/_posts/*.md 的 front-matter 自动补充 SEO 字段:
  - description: 从正文前若干段抽取纯文本, 截断到约 DESC_LEN 字
  - keywords:    合并 categories + tags, 用逗号连接

幂等: 已经含有 description 的文章会跳过, 除非 --force.
"""
import argparse
import re
import sys
from pathlib import Path

POSTS_DIR = Path(__file__).parent / "source" / "_posts"
DESC_LEN = 150           # 描述字符数上限
DESC_MIN = 60            # 至少要抽到这么多字才算有效
FM_DELIM = "---"


def split_front_matter(text: str):
    """返回 (front_matter_str_without_delims, body)"""
    if not text.startswith(FM_DELIM):
        return None, text
    end = text.find("\n" + FM_DELIM, len(FM_DELIM))
    if end < 0:
        return None, text
    fm = text[len(FM_DELIM):end].lstrip("\n")
    body = text[end + len(FM_DELIM) + 1:]
    if body.startswith("\n"):
        body = body[1:]
    return fm, body


def parse_fm_list_field(fm: str, field: str) -> list[str]:
    """解析 YAML front-matter 中的列表字段 (tags / categories).

    支持两种写法:
        tags: [a, b, c]
        tags:
          - a
          - b
    """
    m = re.search(rf"^{field}[ \t]*:[ \t]*(.*)$", fm, re.MULTILINE)
    if not m:
        return []
    rest = m.group(1).strip()
    if rest.startswith("[") and rest.endswith("]"):
        return [s.strip().strip('"\'') for s in rest[1:-1].split(",") if s.strip()]
    if rest:
        return [rest.strip('"\'')]
    items = []
    start = m.end()
    for line in fm[start:].splitlines()[1:]:
        if re.match(r"^\s*-\s+", line):
            items.append(re.sub(r"^\s*-\s+", "", line).strip().strip('"\''))
        elif re.match(r"^\S", line):
            break
    return items


def fm_has_field(fm: str, field: str) -> bool:
    return re.search(rf"^{field}[ \t]*:", fm, re.MULTILINE) is not None


def clean_markdown_for_desc(body: str) -> str:
    """把 markdown 正文清洗成一段干净的纯文本, 用来当 description."""
    s = body

    s = re.sub(r"```.*?```", " ", s, flags=re.DOTALL)
    s = re.sub(r"`([^`]+)`", r"\1", s)

    s = re.sub(r"\$\$.*?\$\$", " ", s, flags=re.DOTALL)
    s = re.sub(r"\$[^$\n]+\$", " ", s)

    s = re.sub(r"!\[[^\]]*\]\([^)]*\)", " ", s)

    s = re.sub(r"\[([^\]]+)\]\([^)]*\)", r"\1", s)

    s = re.sub(r"^#+\s*", "", s, flags=re.MULTILINE)

    s = re.sub(r"[*_]{1,3}([^*_]+)[*_]{1,3}", r"\1", s)

    s = re.sub(r"^>\s*", "", s, flags=re.MULTILINE)

    s = re.sub(r"^\s*[-*+]\s+", "", s, flags=re.MULTILINE)
    s = re.sub(r"^\s*\d+\.\s+", "", s, flags=re.MULTILINE)

    s = re.sub(r"<[^>]+>", " ", s)

    s = re.sub(r"\s+", " ", s).strip()
    return s


def make_description(body: str) -> str:
    """从正文生成 ~DESC_LEN 字的 description."""
    text = clean_markdown_for_desc(body)
    if len(text) < DESC_MIN:
        return ""
    if len(text) <= DESC_LEN:
        return text
    cut = text[:DESC_LEN]
    for punct in ["。", "；", "！", "？", ".", "!", "?", "；"]:
        idx = cut.rfind(punct)
        if idx >= DESC_MIN:
            return cut[: idx + 1]
    return cut.rstrip(" ，,") + "..."


def yaml_escape(s: str) -> str:
    return s.replace("\\", "\\\\").replace('"', '\\"')


def inject_fields(fm: str, description: str, keywords: str) -> str:
    lines = fm.rstrip("\n").split("\n")

    if description and not fm_has_field(fm, "description"):
        lines.append(f'description: "{yaml_escape(description)}"')

    if keywords and not fm_has_field(fm, "keywords"):
        lines.append(f'keywords: "{yaml_escape(keywords)}"')

    return "\n".join(lines) + "\n"


def process_file(path: Path, force: bool = False, dry_run: bool = False) -> str:
    text = path.read_text(encoding="utf-8")
    fm, body = split_front_matter(text)
    if fm is None:
        return "SKIP (no front-matter)"

    already_desc = fm_has_field(fm, "description")
    already_kw = fm_has_field(fm, "keywords")

    if already_desc and already_kw and not force:
        return "SKIP (already has both)"

    cats = parse_fm_list_field(fm, "categories")
    tags = parse_fm_list_field(fm, "tags")
    kw_list = []
    for item in cats + tags:
        if item and item not in kw_list:
            kw_list.append(item)
    keywords = ",".join(kw_list) if kw_list else ""

    description = make_description(body)

    if force and already_desc:
        fm = re.sub(r"^description\s*:.*$", "", fm, flags=re.MULTILINE)
    if force and already_kw:
        fm = re.sub(r"^keywords\s*:.*$", "", fm, flags=re.MULTILINE)
    fm = "\n".join(l for l in fm.split("\n") if l.strip() != "")

    new_fm = inject_fields(fm, description, keywords)
    new_text = f"{FM_DELIM}\n{new_fm}{FM_DELIM}\n{body if body.startswith(chr(10)) else chr(10) + body}"

    if dry_run:
        return f"DRY-RUN | desc={len(description)}chars, kw={len(kw_list)}"

    path.write_text(new_text, encoding="utf-8")
    return f"WRITTEN | desc={len(description)}chars, kw={len(kw_list)}"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--force", action="store_true",
                    help="即使文章已有 description/keywords 也重写")
    ap.add_argument("--dry-run", action="store_true",
                    help="只打印不改文件")
    args = ap.parse_args()

    if not POSTS_DIR.is_dir():
        print(f"not found: {POSTS_DIR}", file=sys.stderr)
        sys.exit(1)

    files = sorted(POSTS_DIR.glob("*.md"))
    if not files:
        print("no posts found")
        return

    print(f"{len(files)} posts found in {POSTS_DIR}")
    print("-" * 70)
    for md in files:
        status = process_file(md, force=args.force, dry_run=args.dry_run)
        print(f"{md.name:<50s} {status}")


if __name__ == "__main__":
    main()
