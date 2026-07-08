#!/usr/bin/env python3
"""터미네이터 레이아웃(config) 파일을 생성한다. 창 하나를 좌/우로 나누고,
각 쪽을 위아래로 죽 이어붙여서(VPaned 체인) N개의 pane을 한 창에 다 넣는다.
사용자의 실제 ~/.config/terminator/config를 안 건드리고 별도 파일로 만들어서
`terminator -g <이 파일> -l main`으로 띄우는 용도.
"""
import sys
import uuid


def gen(commands: list[tuple[str, str]], out_path: str, layout_name: str = "main"):
    """commands: [(title, shell_command), ...]"""
    n = len(commands)
    half = (n + 1) // 2
    left = commands[:half]
    right = commands[half:]

    lines = ["[global_config]", "  dbus = False", "[layouts]", f"  [[{layout_name}]]"]

    def block(name, indent, **kv):
        lines.append(f"  [[[{name}]]]")
        for k, v in kv.items():
            lines.append(f"    {k} = {v}")

    def terminal_block(name, parent, title, cmd):
        kv = dict(type="Terminal", parent=parent, profile="default",
                  title=f'"{title}"', uuid=uuid.uuid4())
        if cmd:  # 빈 문자열이면 command 자체를 안 써서 그냥 빈 셸로 연다
            # cmd 안에 큰따옴표/작은따옴표가 둘 다 섞여 있어서(예: bash -lc 'echo "..."')
            # 일반 "..."로 감싸면 안쪽 큰따옴표에서 값이 잘림 - configobj의 삼중따옴표로
            # 감싸서 내부에 어떤 따옴표가 있어도 안전하게 (terminator_jenga.config와 동일 방식).
            kv["command"] = f"'''{cmd}'''"
        block(name, 0, **kv)

    def vpaned_chain(prefix, items, parent):
        """items를 위->아래로 균등한 높이로 죽 잇는 VPaned 체인. position(절대 픽셀)
        대신 ratio(0~1 비율)를 씀 - maximised=True로 실제 창 크기가 모니터 해상도로
        꽉 찰 때도 비율은 항상 맞게 재분배되므로 픽셀 계산보다 안전하다."""
        if len(items) == 1:
            title, cmd = items[0]
            term_name = f"{prefix}_t0"
            terminal_block(term_name, parent, title, cmd)
            return term_name

        ratio = round(1 / len(items), 4)
        node_name = f"{prefix}_p"
        block(node_name, 0, type="VPaned", parent=parent, ratio=ratio)
        title0, cmd0 = items[0]
        term_name = f"{prefix}_t0"
        terminal_block(term_name, node_name, title0, cmd0)
        vpaned_chain(prefix + "c", items[1:], node_name)
        return node_name

    block("window0", 0, type="Window", parent='""', size="1400, 900", maximised="True")
    block("root", 0, type="HPaned", parent="window0", ratio=0.5)
    vpaned_chain("l", left, "root")
    vpaned_chain("r", right, "root")

    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    # 사용법: gen_terminator_layout.py out.conf "title1" "cmd1" "title2" "cmd2" ...
    out_path = sys.argv[1]
    rest = sys.argv[2:]
    pairs = list(zip(rest[0::2], rest[1::2]))
    gen(pairs, out_path)
    print(f"레이아웃 생성 완료: {out_path}")
