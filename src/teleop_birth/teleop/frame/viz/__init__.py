"""共用坐标系转换界面：① R_a  ② 腕部姿态 R_t  ③ 握把→夹爪 M。

PyQt 可选；无 PyQt 时由交互式终端 interactive_frame 承担同样流程。
"""

from __future__ import annotations

import sys
from typing import Mapping, Optional

import numpy as np

__all__ = ["run_frame_viz", "FrameVizWindow"]

AXIS_CHOICES = [
    "+x",
    "-x",
    "+y",
    "-y",
    "+z",
    "-z",
    "right",
    "left",
    "forward",
    "back",
    "up",
    "down",
]


def _try_imports():
    try:
        from PyQt6.QtCore import Qt
        from PyQt6.QtWidgets import (
            QApplication,
            QComboBox,
            QFormLayout,
            QGroupBox,
            QHBoxLayout,
            QLabel,
            QLineEdit,
            QMainWindow,
            QMessageBox,
            QPushButton,
            QPlainTextEdit,
            QSlider,
            QTabWidget,
            QVBoxLayout,
            QWidget,
        )
    except ImportError as e:
        raise ImportError(
            "坐标系可视化需要 PyQt6。请安装: uv sync --extra viz  或  pip install 'teleop-birth[viz]'"
        ) from e
    try:
        from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
        from matplotlib.figure import Figure
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError as e:
        raise ImportError("坐标系可视化需要 matplotlib。请安装: uv sync --extra viz") from e
    return {
        "Qt": Qt,
        "QApplication": QApplication,
        "QComboBox": QComboBox,
        "QFormLayout": QFormLayout,
        "QGroupBox": QGroupBox,
        "QHBoxLayout": QHBoxLayout,
        "QLabel": QLabel,
        "QLineEdit": QLineEdit,
        "QMainWindow": QMainWindow,
        "QMessageBox": QMessageBox,
        "QPushButton": QPushButton,
        "QPlainTextEdit": QPlainTextEdit,
        "QSlider": QSlider,
        "QTabWidget": QTabWidget,
        "QVBoxLayout": QVBoxLayout,
        "QWidget": QWidget,
        "FigureCanvasQTAgg": FigureCanvasQTAgg,
        "Figure": Figure,
    }


def run_frame_viz(
    *,
    profile_id: Optional[str] = None,
    mapping: Optional[Mapping[str, str]] = None,
    grip_to_tcp: Optional[str] = None,
    tab: str = "axes",
    block: bool | None = None,
) -> int:
    """启动共用可视化窗口；tab=axes|orient|m。

    若调用方已有运行中的 QApplication（如场景工坊），默认不二次 ``exec``，
    仅 ``show`` 非模态窗口，避免嵌套事件循环导致工坊无响应。
    """
    mods = _try_imports()
    QApplication = mods["QApplication"]
    created_app = QApplication.instance() is None
    app = QApplication.instance() or QApplication(sys.argv)
    try:
        from teleop_birth.ui.qt_fonts import apply_chinese_fonts

        apply_chinese_fonts(app)
    except Exception:
        pass
    win = FrameVizWindow(
        profile_id=profile_id,
        mapping=mapping,
        grip_to_tcp=grip_to_tcp,
        tab=tab,
        _mods=mods,
    )
    win.show()
    do_block = created_app if block is None else bool(block)
    if do_block:
        return int(app.exec())
    return 0


class FrameVizWindow:
    """① R_a + ② R_t + ③ M，同一窗口三个页签。"""

    def __init__(
        self,
        *,
        profile_id: Optional[str] = None,
        mapping: Optional[Mapping[str, str]] = None,
        grip_to_tcp: Optional[str] = None,
        tab: str = "axes",
        _mods=None,
    ) -> None:
        mods = _mods or _try_imports()
        from teleop_birth.teleop.frame.axes import build_ra, openxr_semantic_basis
        from teleop_birth.teleop.frame.grip_tcp import (
            build_M,
            get_grip_tcp_profile,
            list_grip_tcp_profiles,
        )
        from teleop_birth.teleop.frame.profiles import get_arm_profile, list_arm_profiles

        self._build_ra = build_ra
        self._build_M = build_M
        self._xr_basis = openxr_semantic_basis
        self._get_profile = get_arm_profile
        self._get_m_profile = get_grip_tcp_profile
        self._list_m = list_grip_tcp_profiles
        self._QMessageBox = mods["QMessageBox"]
        self._Qt = mods["Qt"]

        QMainWindow = mods["QMainWindow"]
        QWidget = mods["QWidget"]
        QVBoxLayout = mods["QVBoxLayout"]
        QHBoxLayout = mods["QHBoxLayout"]
        QFormLayout = mods["QFormLayout"]
        QGroupBox = mods["QGroupBox"]
        QLabel = mods["QLabel"]
        QComboBox = mods["QComboBox"]
        QLineEdit = mods["QLineEdit"]
        QPushButton = mods["QPushButton"]
        QPlainTextEdit = mods["QPlainTextEdit"]
        QTabWidget = mods["QTabWidget"]
        QSlider = mods["QSlider"]
        Figure = mods["Figure"]
        FigureCanvasQTAgg = mods["FigureCanvasQTAgg"]

        self._win = QMainWindow()
        self._win.setWindowTitle("坐标系转换：R_a  ·  R_t  ·  M")
        self._win.resize(1180, 740)

        root = QWidget()
        self._win.setCentralWidget(root)
        outer = QVBoxLayout(root)
        self.tabs = QTabWidget()
        outer.addWidget(self.tabs)

        # —— Tab ① 轴对齐 ——
        tab_axes = QWidget()
        self.tabs.addTab(tab_axes, "① 轴对齐 R_a")
        lay_a = QHBoxLayout(tab_axes)
        left_a = QVBoxLayout()
        lay_a.addLayout(left_a, stretch=2)

        box = QGroupBox("映射需求（头显语义 → 机械臂房间轴）")
        form = QFormLayout(box)
        self.combo_profile = QComboBox()
        self.combo_profile.addItem("（自定义）", "")
        for p in list_arm_profiles():
            self.combo_profile.addItem(f"{p.id} — {p.title}", p.id)
        form.addRow("臂剖面", self.combo_profile)
        self.cmb_right = QComboBox()
        self.cmb_up = QComboBox()
        self.cmb_fwd = QComboBox()
        for cmb in (self.cmb_right, self.cmb_up, self.cmb_fwd):
            cmb.addItems(AXIS_CHOICES)
            cmb.setEditable(True)
        self.cmb_right.setCurrentText("+x")
        self.cmb_up.setCurrentText("+z")
        self.cmb_fwd.setCurrentText("+y")
        form.addRow("XR 右 (+X) →", self.cmb_right)
        form.addRow("XR 上 (+Y) →", self.cmb_up)
        form.addRow("XR 前 (−Z) →", self.cmb_fwd)
        self.edit_custom = QLineEdit()
        self.edit_custom.setPlaceholderText("可选 3×3：1,0,0; 0,0,-1; 0,1,0")
        form.addRow("矩阵覆盖", self.edit_custom)
        left_a.addWidget(box)

        btns = QHBoxLayout()
        self.btn_apply = QPushButton("生成 R_a 并刷新")
        self.btn_copy = QPushButton("复制 YAML")
        btns.addWidget(self.btn_apply)
        btns.addWidget(self.btn_copy)
        left_a.addLayout(btns)
        self.out = QPlainTextEdit()
        self.out.setReadOnly(True)
        left_a.addWidget(QLabel("R_a 与自检"))
        left_a.addWidget(self.out)
        self.fig = Figure(figsize=(5.2, 4.8))
        self.canvas = FigureCanvasQTAgg(self.fig)
        lay_a.addWidget(self.canvas, stretch=3)

        self.btn_apply.clicked.connect(self._refresh_axes)
        self.btn_copy.clicked.connect(self._copy_yaml_axes)
        self.combo_profile.currentIndexChanged.connect(self._on_profile)

        # —— Tab ② 腕部姿态 ——
        tab_o = QWidget()
        self.tabs.addTab(tab_o, "② 腕部姿态 R_t")
        lay_o = QHBoxLayout(tab_o)
        left_o = QVBoxLayout()
        lay_o.addLayout(left_o, stretch=2)

        box_o = QGroupBox("模拟「按下后」手柄相对转动（度）")
        form_o = QFormLayout(box_o)
        self.combo_m_orient = QComboBox()
        for p in self._list_m():
            self.combo_m_orient.addItem(f"{p.id} — {p.title}", p.id)
        form_o.addRow("M 剖面（与页签③同步）", self.combo_m_orient)
        self.sl_pitch = QSlider(self._Qt.Orientation.Horizontal)
        self.sl_roll = QSlider(self._Qt.Orientation.Horizontal)
        self.sl_twist = QSlider(self._Qt.Orientation.Horizontal)
        for sl in (self.sl_pitch, self.sl_roll, self.sl_twist):
            sl.setRange(-180, 180)
            sl.setValue(0)
            sl.setTickInterval(45)
            sl.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.lbl_pitch = QLabel("0°")
        self.lbl_roll = QLabel("0°")
        self.lbl_twist = QLabel("0°")
        form_o.addRow("绕手柄上(+Y) → 房间竖直转 ψ", self._row(self.sl_pitch, self.lbl_pitch, mods))
        form_o.addRow("绕手柄右(+X) → 侧倾（经 M）", self._row(self.sl_roll, self.lbl_roll, mods))
        form_o.addRow("绕瞄准(≈−Z) → 拧腕（经 M）", self._row(self.sl_twist, self.lbl_twist, mods))
        left_o.addWidget(box_o)

        hint = QLabel(
            "公式：R_t = R_yaw @ R_r0 @ R_tool\n"
            "R_Δ^MJ = R_a (R_x1 R_x0ᵀ) R_aᵀ → ψ；w_y=0；R_tool = Rot(M w)"
        )
        hint.setWordWrap(True)
        left_o.addWidget(hint)

        self.btn_orient = QPushButton("刷新姿态分解")
        self.btn_copy_orient = QPushButton("复制步骤说明")
        row_b = QHBoxLayout()
        row_b.addWidget(self.btn_orient)
        row_b.addWidget(self.btn_copy_orient)
        left_o.addLayout(row_b)

        self.out_orient = QPlainTextEdit()
        self.out_orient.setReadOnly(True)
        left_o.addWidget(self.out_orient)

        self.fig_o = Figure(figsize=(5.2, 4.8))
        self.canvas_o = FigureCanvasQTAgg(self.fig_o)
        lay_o.addWidget(self.canvas_o, stretch=3)

        for sl in (self.sl_pitch, self.sl_roll, self.sl_twist):
            sl.valueChanged.connect(self._on_orient_slider)
        self.btn_orient.clicked.connect(self._refresh_orient)
        self.btn_copy_orient.clicked.connect(self._copy_orient)
        self.combo_m_orient.currentIndexChanged.connect(self._on_m_from_orient)

        # —— Tab ③ 握把→夹爪 M ——
        tab_m = QWidget()
        self.tabs.addTab(tab_m, "③ 握把→夹爪 M")
        lay_m = QHBoxLayout(tab_m)
        left_m = QVBoxLayout()
        lay_m.addLayout(left_m, stretch=2)

        box_m = QGroupBox("映射需求（握把体轴 → 夹爪体轴）")
        form_m = QFormLayout(box_m)
        self.combo_m = QComboBox()
        self.combo_m.addItem("（自定义）", "")
        for p in self._list_m():
            self.combo_m.addItem(f"{p.id} — {p.title}", p.id)
        form_m.addRow("M 剖面", self.combo_m)
        self.cmb_gr = QComboBox()
        self.cmb_gu = QComboBox()
        self.cmb_ga = QComboBox()
        for cmb in (self.cmb_gr, self.cmb_gu, self.cmb_ga):
            cmb.addItems(AXIS_CHOICES)
            cmb.setEditable(True)
        self.cmb_gr.setCurrentText("+y")
        self.cmb_gu.setCurrentText("+x")
        self.cmb_ga.setCurrentText("+z")
        form_m.addRow("握把右 (+X) →", self.cmb_gr)
        form_m.addRow("握把上 (+Y) →", self.cmb_gu)
        form_m.addRow("握把瞄准 (−Z) →", self.cmb_ga)
        self.edit_m = QLineEdit()
        self.edit_m.setPlaceholderText("可选 3×3：0,1,0; 1,0,0; 0,0,-1")
        form_m.addRow("矩阵覆盖", self.edit_m)
        left_m.addWidget(box_m)

        hint_m = QLabel(
            "尖端朝下用 tip_down；尖端朝前用 tip_forward。\n"
            "YAML: mapper.grip_to_tcp / grip_to_tcp_mapping / grip_to_tcp_matrix"
        )
        hint_m.setWordWrap(True)
        left_m.addWidget(hint_m)

        row_m = QHBoxLayout()
        self.btn_m = QPushButton("生成 M 并刷新")
        self.btn_copy_m = QPushButton("复制 YAML")
        row_m.addWidget(self.btn_m)
        row_m.addWidget(self.btn_copy_m)
        left_m.addLayout(row_m)

        self.out_m = QPlainTextEdit()
        self.out_m.setReadOnly(True)
        left_m.addWidget(self.out_m)

        self.fig_m = Figure(figsize=(5.2, 4.8))
        self.canvas_m = FigureCanvasQTAgg(self.fig_m)
        lay_m.addWidget(self.canvas_m, stretch=3)

        self.btn_m.clicked.connect(self._refresh_m)
        self.btn_copy_m.clicked.connect(self._copy_yaml_m)
        self.combo_m.currentIndexChanged.connect(self._on_m_profile)

        # 初始
        if mapping:
            self.cmb_right.setCurrentText(str(mapping.get("right") or mapping.get("xr_right") or "+x"))
            self.cmb_up.setCurrentText(str(mapping.get("up") or mapping.get("xr_up") or "+z"))
            self.cmb_fwd.setCurrentText(
                str(mapping.get("forward") or mapping.get("xr_forward") or "+y")
            )
        if profile_id:
            idx = self.combo_profile.findData(profile_id)
            if idx >= 0:
                self.combo_profile.setCurrentIndex(idx)
                self._on_profile()
        mid = grip_to_tcp or "tip_down"
        for combo in (self.combo_m, self.combo_m_orient):
            idx = combo.findData(mid)
            if idx >= 0:
                combo.setCurrentIndex(idx)
        self._on_m_profile()
        self._refresh_axes()
        self._refresh_orient()
        self._refresh_m()
        t = str(tab).lower()
        if t.startswith("orient") or t == "rt":
            self.tabs.setCurrentIndex(1)
        elif t in ("m", "grip", "tcp"):
            self.tabs.setCurrentIndex(2)

    @staticmethod
    def _row(slider, label, mods):
        w = mods["QWidget"]()
        h = mods["QHBoxLayout"](w)
        h.setContentsMargins(0, 0, 0, 0)
        h.addWidget(slider, stretch=1)
        h.addWidget(label)
        return w

    def show(self) -> None:
        self._win.show()

    def content_widget(self):
        """供场景工坊嵌入：返回中央控件（勿再对同一实例 show/exec）。"""
        return self._win.centralWidget()

    def host_window(self):
        return self._win

    def _on_orient_slider(self) -> None:
        self.lbl_pitch.setText(f"{self.sl_pitch.value()}°")
        self.lbl_roll.setText(f"{self.sl_roll.value()}°")
        self.lbl_twist.setText(f"{self.sl_twist.value()}°")
        self._refresh_orient()

    def _on_profile(self) -> None:
        pid = self.combo_profile.currentData()
        if not pid:
            return
        try:
            p = self._get_profile(str(pid))
        except KeyError:
            return
        if p.xr_align_matrix is not None:
            rows = [",".join(str(x) for x in row) for row in p.xr_align_matrix]
            self.edit_custom.setText("; ".join(rows))
        else:
            self.edit_custom.clear()
            self.cmb_right.setCurrentText(str(p.xr_right) if isinstance(p.xr_right, str) else "+x")
            self.cmb_up.setCurrentText(str(p.xr_up) if isinstance(p.xr_up, str) else "+z")
            self.cmb_fwd.setCurrentText(
                str(p.xr_forward) if isinstance(p.xr_forward, str) else "+y"
            )
        self._refresh_axes()
        self._refresh_orient()

    def _sync_m_combos(self, pid: str | None, *, source) -> None:
        if not pid:
            return
        for combo in (self.combo_m, self.combo_m_orient):
            if combo is source:
                continue
            idx = combo.findData(pid)
            if idx >= 0 and combo.currentIndex() != idx:
                combo.blockSignals(True)
                combo.setCurrentIndex(idx)
                combo.blockSignals(False)

    def _on_m_from_orient(self) -> None:
        pid = self.combo_m_orient.currentData()
        self._sync_m_combos(str(pid) if pid else None, source=self.combo_m_orient)
        if pid:
            self._apply_m_profile(str(pid))
        self._refresh_orient()
        self._refresh_m()

    def _on_m_profile(self) -> None:
        pid = self.combo_m.currentData()
        self._sync_m_combos(str(pid) if pid else None, source=self.combo_m)
        if pid:
            self._apply_m_profile(str(pid))
            idx = self.combo_m_orient.findData(pid)
            if idx >= 0:
                self.combo_m_orient.blockSignals(True)
                self.combo_m_orient.setCurrentIndex(idx)
                self.combo_m_orient.blockSignals(False)
        self._refresh_m()
        self._refresh_orient()

    def _apply_m_profile(self, pid: str) -> None:
        try:
            p = self._get_m_profile(pid)
        except KeyError:
            return
        if p.matrix is not None:
            rows = [",".join(str(x) for x in row) for row in p.matrix]
            self.edit_m.setText("; ".join(rows))
        else:
            self.edit_m.clear()
            self.cmb_gr.setCurrentText(str(p.grip_right) if isinstance(p.grip_right, str) else "+y")
            self.cmb_gu.setCurrentText(str(p.grip_up) if isinstance(p.grip_up, str) else "+x")
            self.cmb_ga.setCurrentText(str(p.grip_aim) if isinstance(p.grip_aim, str) else "+z")

    def _parse_matrix_override(self, edit=None) -> Optional[np.ndarray]:
        text = (edit or self.edit_custom).text().strip()
        if not text:
            return None
        rows = []
        for part in text.replace("|", ";").split(";"):
            part = part.strip()
            if not part:
                continue
            nums = [float(x) for x in part.replace(",", " ").split()]
            if len(nums) != 3:
                raise ValueError(f"矩阵行需要 3 个数: {part}")
            rows.append(nums)
        if len(rows) != 3:
            raise ValueError("需要 3 行构成 3×3")
        return np.asarray(rows, dtype=np.float64)

    def _current_ra(self) -> np.ndarray:
        override = self._parse_matrix_override(self.edit_custom)
        if override is not None:
            return override
        return self._build_ra(
            xr_right_to=self.cmb_right.currentText(),
            xr_up_to=self.cmb_up.currentText(),
            xr_forward_to=self.cmb_fwd.currentText(),
        )

    def _current_M(self) -> np.ndarray:
        override = self._parse_matrix_override(self.edit_m)
        if override is not None:
            return override
        return self._build_M(
            grip_right_to=self.cmb_gr.currentText(),
            grip_up_to=self.cmb_gu.currentText(),
            grip_aim_to=self.cmb_ga.currentText(),
        )

    def _refresh_axes(self) -> None:
        try:
            R = self._current_ra()
        except Exception as exc:  # noqa: BLE001
            self.out.setPlainText(f"错误: {exc}")
            return
        basis = self._xr_basis()
        lines = [
            "R_a =",
            np.array2string(R, precision=4, suppress_small=True),
            "",
            "自检（头显语义 → 房间）:",
        ]
        for name, v in basis.items():
            w = R @ v
            lines.append(
                f"  XR {name:8} {np.array2string(v, precision=0)} → {np.array2string(w, precision=4)}"
            )
        lines.append("")
        lines.append("YAML: mapper.xr_align_matrix / frame.xr_mapping")
        for row in R:
            lines.append(f"  - [{row[0]:.6g}, {row[1]:.6g}, {row[2]:.6g}]")
        self.out.setPlainText("\n".join(lines))
        self._draw_pair_frames(
            self.fig,
            self.canvas,
            left_origin=np.array([-1.2, 0.0, 0.0]),
            right_origin=np.array([1.2, 0.0, 0.0]),
            left_basis=basis,
            map_fn=lambda v: R @ v,
            title="左：OpenXR 语义轴  右：映射后房间轴",
            left_prefix="XR",
            right_prefix="arm",
        )

    def _draw_pair_frames(
        self,
        fig,
        canvas,
        *,
        left_origin,
        right_origin,
        left_basis: dict,
        map_fn,
        title: str,
        left_prefix: str,
        right_prefix: str,
    ) -> None:
        fig.clear()
        ax = fig.add_subplot(111, projection="3d")
        ax.set_title(title)
        colors = {"right": "#c0392b", "up": "#27ae60", "forward": "#2980b9", "aim": "#2980b9"}
        for name, v in left_basis.items():
            c = colors.get(name, "#333333")
            ax.quiver(*left_origin, *v, color=c, length=0.85, normalize=True, linewidth=2)
            ax.text(*(left_origin + v * 0.95), f"{left_prefix}-{name}", color=c)
            w = map_fn(v)
            ax.quiver(*right_origin, *w, color=c, length=0.85, normalize=True, linewidth=2)
            ax.text(*(right_origin + w * 0.95), f"{right_prefix}-{name}", color=c)
        ax.set_xlim(-2.2, 2.2)
        ax.set_ylim(-1.5, 1.5)
        ax.set_zlim(-1.5, 1.5)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        canvas.draw_idle()

    def _copy_yaml_axes(self) -> None:
        try:
            R = self._current_ra()
        except Exception as exc:  # noqa: BLE001
            self._QMessageBox.warning(self._win, "复制失败", str(exc))
            return
        text = "xr_align_matrix:\n" + "\n".join(
            f"  - [{row[0]:.6g}, {row[1]:.6g}, {row[2]:.6g}]" for row in R
        )
        from PyQt6.QtWidgets import QApplication

        QApplication.clipboard().setText(text)
        self._QMessageBox.information(self._win, "已复制", "xr_align_matrix 已复制")

    def _refresh_orient(self) -> None:
        from teleop_birth.scene.plugins.frame import WristOrientPlugin

        try:
            R_a = self._current_ra()
            M = self._current_M()
        except Exception as exc:  # noqa: BLE001
            self.out_orient.setPlainText(f"先修好 R_a / M：{exc}")
            return
        mid = self.combo_m_orient.currentData() or self.combo_m.currentData() or "tip_down"
        res = WristOrientPlugin().map_demo(
            pitch_deg=float(self.sl_pitch.value()),
            roll_deg=float(self.sl_roll.value()),
            yaw_deg=float(self.sl_twist.value()),
            R_a=R_a,
            M=M,
            grip_to_tcp=str(mid),
        )
        lines = list(res.explain_zh())
        lines.append("")
        lines.append(f"ψ = {res.psi:.4f} rad ({np.degrees(res.psi):.1f}°)")
        lines.append(f"w (清零 wy 后) = {np.array2string(res.w, precision=3)}")
        lines.append("R_yaw =")
        lines.append(np.array2string(res.R_yaw, precision=3, suppress_small=True))
        lines.append("R_tool =")
        lines.append(np.array2string(res.R_tool, precision=3, suppress_small=True))
        lines.append("R_t =")
        lines.append(np.array2string(res.R_t, precision=3, suppress_small=True))
        lines.append("")
        lines.append(f"M 剖面={mid}")
        lines.append(np.array2string(res.M, precision=3))
        self.out_orient.setPlainText("\n".join(lines))
        self._draw_orient(res)

    def _draw_orient(self, res) -> None:
        self.fig_o.clear()
        ax = self.fig_o.add_subplot(111, projection="3d")
        ax.set_title("TCP 体轴：按下 R_r0（虚）→ 目标 R_t（实）")
        colors = ["#c0392b", "#27ae60", "#2980b9"]
        labels = ["X", "Y", "Z"]
        for i in range(3):
            v0 = res.R_r0[:, i]
            v1 = res.R_t[:, i]
            ax.quiver(0, 0, 0, *v0, color=colors[i], length=0.7, normalize=True, alpha=0.35)
            ax.quiver(0, 0, 0, *v1, color=colors[i], length=0.95, normalize=True, linewidth=2)
            ax.text(*(v1 * 1.05), f"t-{labels[i]}", color=colors[i])
        ax.quiver(0, 0, 0, 0, 0, 1, color="#888888", length=0.5, normalize=True)
        ax.text(0, 0, 0.55, "room+Z", color="#666666")
        ax.set_xlim(-1.2, 1.2)
        ax.set_ylim(-1.2, 1.2)
        ax.set_zlim(-1.2, 1.2)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        self.canvas_o.draw_idle()

    def _copy_orient(self) -> None:
        from PyQt6.QtWidgets import QApplication

        QApplication.clipboard().setText(self.out_orient.toPlainText())
        self._QMessageBox.information(self._win, "已复制", "姿态分解说明已复制")

    def _refresh_m(self) -> None:
        try:
            M = self._current_M()
        except Exception as exc:  # noqa: BLE001
            self.out_m.setPlainText(f"错误: {exc}")
            return
        basis = {
            "right": np.array([1.0, 0.0, 0.0]),
            "up": np.array([0.0, 1.0, 0.0]),
            "aim": np.array([0.0, 0.0, -1.0]),
        }
        lines = [
            "M =",
            np.array2string(M, precision=4, suppress_small=True),
            "",
            "自检（握把语义 → 夹爪体轴）:",
        ]
        for name, v in basis.items():
            w = M @ v
            lines.append(
                f"  grip {name:6} {np.array2string(v, precision=0)} → {np.array2string(w, precision=4)}"
            )
        lines.append("")
        lines.append("YAML: mapper.grip_to_tcp / grip_to_tcp_matrix")
        for row in M:
            lines.append(f"  - [{row[0]:.6g}, {row[1]:.6g}, {row[2]:.6g}]")
        self.out_m.setPlainText("\n".join(lines))
        self._draw_pair_frames(
            self.fig_m,
            self.canvas_m,
            left_origin=np.array([-1.2, 0.0, 0.0]),
            right_origin=np.array([1.2, 0.0, 0.0]),
            left_basis=basis,
            map_fn=lambda v: M @ v,
            title="左：握把体轴  右：夹爪体轴（经 M）",
            left_prefix="grip",
            right_prefix="tcp",
        )

    def _copy_yaml_m(self) -> None:
        try:
            M = self._current_M()
        except Exception as exc:  # noqa: BLE001
            self._QMessageBox.warning(self._win, "复制失败", str(exc))
            return
        pid = self.combo_m.currentData()
        if pid:
            text = f"grip_to_tcp: {pid}\n"
        else:
            text = "grip_to_tcp_matrix:\n" + "\n".join(
                f"  - [{row[0]:.6g}, {row[1]:.6g}, {row[2]:.6g}]" for row in M
            )
        from PyQt6.QtWidgets import QApplication

        QApplication.clipboard().setText(text)
        self._QMessageBox.information(self._win, "已复制", "grip_to_tcp 配置已复制")
