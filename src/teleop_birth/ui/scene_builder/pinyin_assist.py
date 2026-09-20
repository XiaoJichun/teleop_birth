"""场景工坊内置拼音选词（不依赖系统输入法接到 Qt）。"""

from __future__ import annotations

_LEXICON: list[tuple[str, tuple[str, ...]]] = [
    ("快递袋子", ("kuaididaizi", "kddz")),
    ("快递袋", ("kuaidai", "kdai")),
    ("快递盒", ("kuaidihe", "kdh")),
    ("红色小方块", ("hongsexiaofangkuai", "hsxfk", "hongsefangkuai")),
    ("蓝色盘子", ("lansepanzi", "lspz")),
    ("绿色瓶子", ("lvsepingzi", "lvpz", "lusepingzi")),
    ("黄色方块", ("huangsefangkuai", "hsfk")),
    ("白色盘子", ("baisepanzi", "bspz")),
    ("黑色方块", ("heisefangkuai")),
    ("塑料袋", ("suliaodai", "sld")),
    ("购物袋", ("gouwudai", "gwd")),
    ("纸箱", ("zhixiang", "zx")),
    ("纸盒", ("zhihe")),
    ("圆球", ("yuanqiu", "yq")),
    ("小方块", ("xiaofangkuai", "xfk", "fangkuai")),
    ("托盘", ("tuopan", "tp")),
    ("罐头", ("guantou", "gt")),
    ("包裹", ("baoguo", "bg")),
    ("红色盘子", ("hongsepanzi", "hspz")),
    ("蓝色方块", ("lansefangkuai", "lsfk")),
]


def pinyin_candidates(typed: str, *, limit: int = 8) -> list[str]:
    s = (typed or "").strip()
    if not s:
        return []
    if any("\u4e00" <= c <= "\u9fff" for c in s):
        return [s]
    key = "".join(c for c in s.lower() if c.isalnum())
    if not key:
        return []
    out: list[str] = []
    # 全拼/简拼前缀优先
    scored: list[tuple[int, str]] = []
    for word, keys in _LEXICON:
        best = 99
        for k in keys:
            if key == k:
                best = 0
            elif k.startswith(key):
                best = min(best, 1)
            elif key.startswith(k) and len(k) >= 2:
                best = min(best, 2)
            elif key in k:
                best = min(best, 3)
        if best < 99:
            scored.append((best, word))
    scored.sort(key=lambda x: (x[0], len(x[1])))
    for _, word in scored:
        if word not in out:
            out.append(word)
        if len(out) >= limit:
            break
    return out


def commit_first(typed: str) -> str | None:
    """空格确认：返回第一个候选中文，无则 None。"""
    c = pinyin_candidates(typed, limit=1)
    return c[0] if c else None
