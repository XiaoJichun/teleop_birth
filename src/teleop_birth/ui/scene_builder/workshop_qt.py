"""场景搭建 PyQt 工坊：物品 / 相机实时画面 / 坐标系可视化 / 保存。"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Optional

import numpy as np

from teleop_birth.ui.scene_builder.draft import SceneDraft, save_scene_draft


def run_scene_workshop(
    project_root: Path,
    draft: SceneDraft,
    preview=None,
) -> Optional[Path]:
    """打开工坊；保存成功返回路径，取消返回 None。"""
    # IME 必须在 QApplication 创建之前配置（含 UTF-8/zh_CN locale）
    try:
        from teleop_birth.ui.qt_ime import configure_qt_ime

        configure_qt_ime(verbose=True)
    except Exception as exc:  # noqa: BLE001
        print(f"  Qt IME 配置跳过: {exc}")

    _require_xcb_runtime()
    ui = _imports()
    try:
        app = ui["QApplication"].instance() or ui["QApplication"]([])
    except Exception as exc:  # noqa: BLE001
        raise ImportError(
            f"无法初始化 Qt 窗口: {exc}\n"
            "请安装: sudo apt install -y libxcb-cursor0 libxkbcommon-x11-0 libegl1 libgl1"
        ) from exc
    try:
        from teleop_birth.ui.qt_ime import apply_qt_locale

        ql = apply_qt_locale(app)
        print(f"  Qt locale: {ql or '?'}")
    except Exception as exc:  # noqa: BLE001
        print(f"  Qt locale 跳过: {exc}")
    try:
        from teleop_birth.ui.qt_fonts import apply_chinese_fonts

        fam = apply_chinese_fonts(app)
        print(f"  PyQt 中文字体: {fam}")
    except Exception as exc:  # noqa: BLE001
        print(f"  警告: 中文字体未就绪（界面可能乱码）: {exc}")
        fam = ""

    # 压制 matplotlib 缺字刷屏（易让工坊像卡死）
    try:
        import logging

        logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)
        logging.getLogger("matplotlib").setLevel(logging.ERROR)
    except Exception:
        pass

    # 无外部 preview 时仍创建离屏会话，保证相机页可用
    if preview is None:
        from teleop_birth.ui.scene_builder.preview import PreviewSession

        preview = PreviewSession(project_root, draft)
        preview.ensure_compiled()

    win = _SceneWorkshopWindow(ui, project_root, draft, preview=preview)
    if fam:
        try:
            from PyQt6.QtGui import QFont

            win.setFont(QFont(fam, 10))
        except Exception:
            pass
    win.show()
    app.exec()
    return win.saved_path


def _require_xcb_runtime() -> None:
    import ctypes.util
    from pathlib import Path as _P

    def _has(name: str) -> bool:
        if ctypes.util.find_library(name):
            return True
        for base in (
            "/usr/lib/x86_64-linux-gnu",
            "/usr/lib/aarch64-linux-gnu",
            "/lib/x86_64-linux-gnu",
            "/usr/lib",
        ):
            for pat in (f"lib{name}.so", f"lib{name}.so.0"):
                if (_P(base) / pat).is_file():
                    return True
        return False

    if not _has("xcb-cursor"):
        raise ImportError(
            "缺少 Qt xcb 系统库 libxcb-cursor\n"
            "请执行: sudo apt install -y libxcb-cursor0 libxkbcommon-x11-0 libegl1 libgl1"
        )


def _imports():
    try:
        from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal
        from PyQt6.QtGui import QImage, QPixmap
        from PyQt6.QtWidgets import (
            QApplication,
            QComboBox,
            QDoubleSpinBox,
            QFormLayout,
            QGroupBox,
            QHBoxLayout,
            QLabel,
            QLineEdit,
            QListWidget,
            QListWidgetItem,
            QFileDialog,
            QMainWindow,
            QMessageBox,
            QPlainTextEdit,
            QPushButton,
            QScrollArea,
            QSlider,
            QSpinBox,
            QSplitter,
            QTabWidget,
            QVBoxLayout,
            QWidget,
        )
    except ImportError as e:
        raise ImportError(
            "场景工坊需要 PyQt6。请: bash start.sh 或 uv sync --extra viz"
        ) from e
    except OSError as e:
        raise ImportError(f"PyQt6 系统依赖缺失: {e}") from e
    return {
        "Qt": Qt,
        "QTimer": QTimer,
        "QThread": QThread,
        "pyqtSignal": pyqtSignal,
        "QImage": QImage,
        "QPixmap": QPixmap,
        "QApplication": QApplication,
        "QComboBox": QComboBox,
        "QDoubleSpinBox": QDoubleSpinBox,
        "QFormLayout": QFormLayout,
        "QGroupBox": QGroupBox,
        "QHBoxLayout": QHBoxLayout,
        "QLabel": QLabel,
        "QLineEdit": QLineEdit,
        "QListWidget": QListWidget,
        "QListWidgetItem": QListWidgetItem,
        "QFileDialog": QFileDialog,
        "QMainWindow": QMainWindow,
        "QMessageBox": QMessageBox,
        "QPlainTextEdit": QPlainTextEdit,
        "QPushButton": QPushButton,
        "QScrollArea": QScrollArea,
        "QSlider": QSlider,
        "QSpinBox": QSpinBox,
        "QSplitter": QSplitter,
        "QTabWidget": QTabWidget,
        "QVBoxLayout": QVBoxLayout,
        "QWidget": QWidget,
    }


def _SceneWorkshopWindow(ui: dict, project_root: Path, draft: SceneDraft, preview=None):
    QMainWindow = ui["QMainWindow"]
    QWidget = ui["QWidget"]
    QVBoxLayout = ui["QVBoxLayout"]
    QHBoxLayout = ui["QHBoxLayout"]
    QTabWidget = ui["QTabWidget"]
    QLabel = ui["QLabel"]
    QPushButton = ui["QPushButton"]
    QPlainTextEdit = ui["QPlainTextEdit"]
    QListWidget = ui["QListWidget"]
    QLineEdit = ui["QLineEdit"]
    QComboBox = ui["QComboBox"]
    QDoubleSpinBox = ui["QDoubleSpinBox"]
    QFormLayout = ui["QFormLayout"]
    QGroupBox = ui["QGroupBox"]
    QMessageBox = ui["QMessageBox"]
    QFileDialog = ui["QFileDialog"]
    QSplitter = ui["QSplitter"]
    QTimer = ui["QTimer"]
    QImage = ui["QImage"]
    QPixmap = ui["QPixmap"]
    Qt = ui["Qt"]

    class SceneWorkshopWindow(QMainWindow):
        def __init__(self) -> None:
            super().__init__()
            self.project_root = project_root
            self.draft = draft
            self.preview = preview
            self.saved_path: Optional[Path] = None
            self._frame_host = None
            self._cam_timer: Any = None
            self.setWindowTitle(
                f"场景工坊 — {draft.mode} — {draft.robot_title} / {draft.gripper_title}"
            )
            self.resize(1100, 780)
            tabs = QTabWidget()
            tabs.addTab(self._tab_props(), "物品（语义/模型库）")
            tabs.addTab(self._tab_camera(), "相机")
            tabs.addTab(self._tab_frames(), "坐标系插件")
            tabs.addTab(self._tab_save(), "保存场景")
            tabs.currentChanged.connect(self._on_tab_changed)
            self.setCentralWidget(tabs)
            self.statusBar().showMessage("就绪：先在「物品」页语义生成，再切到「相机」看画面")

        def _tab_props(self) -> QWidget:
            w = QWidget()
            lay = QVBoxLayout(w)
            tip = (
                "语义模式：空桌面 + 双臂起步；输入描述后点「语义生成并加入」。例：快递袋子、红色小方块。"
                if draft.mode == "semantic"
                else "调用模式：空桌面 + 双臂起步；从模型库拉取物品到列表。"
            )
            lay.addWidget(QLabel(tip))
            self.prop_list = QListWidget()
            self._reload_props()
            lay.addWidget(self.prop_list, stretch=2)

            if draft.mode == "semantic":
                tip = QLabel(
                    "直接输入中文：点绿色中文词条一键填入；或点输入框用系统中文输入法输入汉字；或粘贴。"
                )
                tip.setWordWrap(True)
                lay.addWidget(tip)

                self.sem_edit = QLineEdit()
                self.sem_edit.setPlaceholderText("在此直接输入中文汉字，例如：快递袋子")
                self.sem_edit.setToolTip("系统中文输入法在此框内直接输出汉字")
                try:
                    from teleop_birth.ui.qt_ime import enable_widget_ime

                    enable_widget_ime(self.sem_edit)
                except Exception:
                    pass
                self.sem_edit.focusInEvent = self._wrap_focus_ime(self.sem_edit)  # type: ignore[method-assign]
                self.sem_edit.returnPressed.connect(self._on_semantic_add)
                lay.addWidget(self.sem_edit)

                from teleop_birth.ui.scene_builder.chinese_presets import CHINESE_PRESETS

                lay.addWidget(QLabel("常用中文（点击直接填入，无需打字）："))
                preset_row = QHBoxLayout()
                preset_row2 = QHBoxLayout()
                for i, word in enumerate(CHINESE_PRESETS):
                    b = QPushButton(word)
                    b.setStyleSheet(
                        "padding:6px 10px; background:#1b5e20; color:white; font-weight:600;"
                    )

                    # clicked(bool) 会把 False 传进第一个参数，不能写成 def _fill(w=word)
                    def _fill(_checked: bool = False, *, text: str = word) -> None:
                        self._set_semantic_text(text)
                        self.sem_edit.setFocus()
                        self.statusBar().showMessage(f"已填入中文: {text}", 4000)

                    b.clicked.connect(_fill)
                    (preset_row if i < 10 else preset_row2).addWidget(b)
                lay.addLayout(preset_row)
                lay.addLayout(preset_row2)

                row = QHBoxLayout()
                btn_ime = QPushButton("启用中文输入法")
                btn_ime.setStyleSheet("font-weight:bold; padding:8px;")
                btn_ime.setToolTip("连接系统中文输入法，然后在输入框内直接输入汉字")
                btn_ime.clicked.connect(self._on_switch_chinese_ime)
                row.addWidget(btn_ime)
                btn = QPushButton("语义生成并加入")
                btn.clicked.connect(self._on_semantic_add)
                row.addWidget(btn)
                btn_paste = QPushButton("粘贴剪贴板")
                btn_paste.clicked.connect(self._on_paste_semantic)
                row.addWidget(btn_paste)
                lay.addLayout(row)
            else:
                tip_c = QLabel(
                    "默认模型库：configs/scene/props（可点「更换模型库目录」）。\n"
                    "在下方列表选中物品 →「从模型库拉取」加入左侧场景列表。\n"
                    "拉取后先更新列表与离屏预览；MuJoCo 窗口请再点「刷新预览」（避免卡死）。"
                )
                tip_c.setWordWrap(True)
                lay.addWidget(tip_c)

                self.library_path_label = QLabel("")
                self.library_path_label.setWordWrap(True)
                self.library_path_label.setStyleSheet("color:#555; font-size:11px;")
                lay.addWidget(self.library_path_label)

                lib_btns = QHBoxLayout()
                btn_lib = QPushButton("更换模型库目录…")
                btn_lib.clicked.connect(self._on_change_library)
                lib_btns.addWidget(btn_lib)
                btn_lib_def = QPushButton("恢复默认库")
                btn_lib_def.clicked.connect(self._on_reset_library)
                lib_btns.addWidget(btn_lib_def)
                btn_lib_ref = QPushButton("刷新库列表")
                btn_lib_ref.clicked.connect(self._fill_catalog)
                lib_btns.addWidget(btn_lib_ref)
                lay.addLayout(lib_btns)

                self.catalog_list = QListWidget()
                self.catalog_list.setMinimumHeight(140)
                self.catalog_list.setSelectionMode(
                    QListWidget.SelectionMode.SingleSelection
                )
                self.catalog_list.itemDoubleClicked.connect(
                    lambda _item: self._on_catalog_add()
                )
                lay.addWidget(self.catalog_list)

                # 兼容旧逻辑：保留 combo 但不显示（部分测试/脚本可能引用）
                self.catalog_combo = QComboBox()
                self.catalog_combo.hide()

                row = QHBoxLayout()
                btn = QPushButton("从模型库拉取")
                btn.setStyleSheet(
                    "font-weight:bold; padding:10px; background:#1565c0; color:white;"
                )
                btn.clicked.connect(self._on_catalog_add)
                row.addWidget(btn)
                lay.addLayout(row)
                self._fill_catalog()
                if self.catalog_list.count() == 0:
                    lay.addWidget(
                        QLabel(
                            "（模型库为空：请检查 configs/scene/props/*.yaml "
                            "或 configs/scene/model_library.yaml）"
                        )
                    )

            edit = QGroupBox("选中物品属性（先点左侧列表选中一项）")
            form = QFormLayout(edit)
            self.prop_name_edit = QLineEdit()
            self.sz_x = QDoubleSpinBox()
            self.sz_y = QDoubleSpinBox()
            self.sz_z = QDoubleSpinBox()
            for s in (self.sz_x, self.sz_y, self.sz_z):
                s.setRange(0.001, 2.0)
                s.setDecimals(4)
                s.setSingleStep(0.005)
            self.color = QLineEdit("1,0,0,1")
            self.friction = QLineEdit("1.0,0.005,0.0001")
            form.addRow("名称", self.prop_name_edit)
            form.addRow("尺寸 X (m)", self.sz_x)
            form.addRow("尺寸 Y (m)", self.sz_y)
            form.addRow("尺寸 Z (m)", self.sz_z)
            form.addRow("颜色 RGBA", self.color)
            form.addRow("摩擦", self.friction)
            btn_apply = QPushButton("应用到选中物品并刷新预览")
            btn_apply.clicked.connect(self._on_apply_prop)
            form.addRow(btn_apply)
            btn_del = QPushButton("删除选中物品")
            btn_del.clicked.connect(self._on_delete_prop)
            form.addRow(btn_del)
            lay.addWidget(edit)

            row_prev = QHBoxLayout()
            btn_refresh = QPushButton("刷新 MuJoCo 预览")
            btn_refresh.clicked.connect(self._on_refresh_preview)
            row_prev.addWidget(btn_refresh)
            lay.addLayout(row_prev)
            self.prop_list.currentRowChanged.connect(self._on_prop_selected)
            if self.draft.props:
                self.prop_list.setCurrentRow(0)
            return w

        def _tab_camera(self) -> QWidget:
            w = QWidget()
            split = QSplitter()
            split.setOrientation(Qt.Orientation.Horizontal)
            outer = QVBoxLayout(w)
            outer.addWidget(split)

            left = QWidget()
            lay = QVBoxLayout(left)
            lay.addWidget(QLabel("相机列表（选中后右侧显示实时画面；调 pos/lookat 即时更新）"))
            self.cam_list = QListWidget()
            self._reload_cams()
            if not self.draft.cameras:
                self._seed_default_camera()
            self._reload_cams()
            lay.addWidget(self.cam_list)

            form = QFormLayout()
            self.cam_role = QComboBox()
            self.cam_role.addItems(["third_person（场景固定）", "wrist（随臂近似）"])
            self.cam_name = QLineEdit("third_person")
            self.cam_step = QDoubleSpinBox()
            self.cam_step.setRange(0.001, 0.5)
            self.cam_step.setValue(0.05)
            self.cam_step.setSingleStep(0.01)
            self.cam_x = QDoubleSpinBox()
            self.cam_y = QDoubleSpinBox()
            self.cam_z = QDoubleSpinBox()
            self.look_x = QDoubleSpinBox()
            self.look_y = QDoubleSpinBox()
            self.look_z = QDoubleSpinBox()
            self.cam_fovy = QDoubleSpinBox()
            self.cam_fovy.setRange(10.0, 120.0)
            self.cam_fovy.setValue(45.0)
            for s in (self.cam_x, self.cam_y, self.cam_z, self.look_x, self.look_y, self.look_z):
                s.setRange(-5.0, 5.0)
                s.setDecimals(3)
                s.setSingleStep(0.02)
            self.cam_x.setValue(0.55)
            self.cam_y.setValue(-0.75)
            self.cam_z.setValue(1.15)
            self.look_x.setValue(0.35)
            self.look_y.setValue(0.0)
            self.look_z.setValue(0.45)
            form.addRow("类型", self.cam_role)
            form.addRow("名称", self.cam_name)
            form.addRow("步进 (m)", self.cam_step)
            form.addRow("相机 pos X", self.cam_x)
            form.addRow("相机 pos Y", self.cam_y)
            form.addRow("相机 pos Z", self.cam_z)
            form.addRow("lookat X", self.look_x)
            form.addRow("lookat Y", self.look_y)
            form.addRow("lookat Z", self.look_z)
            form.addRow("fovy (°)", self.cam_fovy)
            lay.addLayout(form)

            row = QHBoxLayout()
            for label, axis, sign in (
                ("X-", 0, -1),
                ("X+", 0, 1),
                ("Y-", 1, -1),
                ("Y+", 1, 1),
                ("Z-", 2, -1),
                ("Z+", 2, 1),
            ):
                b = QPushButton(label)

                def _make(a=axis, s=sign):
                    def _go() -> None:
                        step = float(self.cam_step.value())
                        spins = (self.cam_x, self.cam_y, self.cam_z)
                        spins[a].setValue(spins[a].value() + s * step)
                        self._sync_cam_from_spins(apply=True)
                        self._update_cam_view()

                    return _go

                b.clicked.connect(_make())
                row.addWidget(b)
            lay.addLayout(row)

            row2 = QHBoxLayout()
            btn_add = QPushButton("保存/更新到相机列表")
            btn_add.clicked.connect(self._on_add_camera)
            btn_apply_view = QPushButton("应用到当前选中并刷新画面")
            btn_apply_view.clicked.connect(self._on_apply_cam_pose)
            row2.addWidget(btn_add)
            row2.addWidget(btn_apply_view)
            lay.addLayout(row2)

            for sp in (
                self.cam_x,
                self.cam_y,
                self.cam_z,
                self.look_x,
                self.look_y,
                self.look_z,
                self.cam_fovy,
            ):
                sp.valueChanged.connect(lambda *_: self._on_cam_spin_changed())

            self.cam_list.currentRowChanged.connect(self._on_cam_selected)
            split.addWidget(left)

            right = QWidget()
            rlay = QVBoxLayout(right)
            rlay.addWidget(QLabel("实时相机画面（MuJoCo 离屏渲染）"))
            self.cam_view = QLabel("尚未渲染")
            self.cam_view.setMinimumSize(480, 360)
            self.cam_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.cam_view.setStyleSheet("background:#222; color:#ccc;")
            rlay.addWidget(self.cam_view, stretch=1)
            btn_once = QPushButton("立即刷新画面")
            btn_once.clicked.connect(self._update_cam_view)
            rlay.addWidget(btn_once)
            split.addWidget(right)
            split.setStretchFactor(0, 2)
            split.setStretchFactor(1, 3)

            self._cam_timer = QTimer(self)
            self._cam_timer.setInterval(200)
            self._cam_timer.timeout.connect(self._update_cam_view)
            if self.cam_list.count():
                self.cam_list.setCurrentRow(0)
            return w

        def _tab_frames(self) -> QWidget:
            w = QWidget()
            lay = QVBoxLayout(w)
            lay.addWidget(
                QLabel(
                    "坐标系插件（嵌入）：调节右侧映射后点「生成 R_a 并刷新」可看 3D 坐标轴。"
                )
            )
            try:
                from teleop_birth.teleop.frame.viz import FrameVizWindow

                self._frame_host = FrameVizWindow(
                    profile_id=str((draft.frame or {}).get("arm_profile") or "") or None,
                    tab="axes",
                )
                host = self._frame_host.host_window()
                content = host.takeCentralWidget()
                if content is not None:
                    lay.addWidget(content, stretch=1)
                    host.hide()
                    try:
                        self._frame_host._refresh_axes()
                    except Exception as exc:  # noqa: BLE001
                        lay.addWidget(QLabel(f"初始刷新: {exc}"))
                else:
                    lay.addWidget(QLabel("坐标系控件创建失败"))
            except Exception as exc:  # noqa: BLE001
                lay.addWidget(QLabel(f"坐标系可视化加载失败: {exc}"))
                btn = QPushButton("尝试打开独立窗口")
                btn.clicked.connect(self._on_frame_viz)
                lay.addWidget(btn)
            info = QPlainTextEdit()
            info.setReadOnly(True)
            info.setMaximumHeight(90)
            info.setPlainText(
                f"robot={draft.robot_id}  mount={draft.mount_kind}\n"
                f"desktop_base.arm_id={draft.desktop_base_arm}  table_top_z={draft.table_top_z}\n"
                f"base_xy={draft.base_xy}  frame={draft.frame}\n"
                "MuJoCo：青球=桌面接触 base，品红球=转换后 URDF base，RGB=XYZ 轴"
            )
            lay.addWidget(info)
            btn_base = QPushButton("切换：显示/隐藏场景中的 base 坐标系")
            btn_base.clicked.connect(self._on_toggle_base_frames)
            lay.addWidget(btn_base)
            return w

        def _tab_save(self) -> QWidget:
            w = QWidget()
            lay = QVBoxLayout(w)
            form = QFormLayout()
            self.save_id = QLineEdit(f"{draft.robot_id}_{draft.gripper_id or 'ee'}")
            self.save_title = QLineEdit(f"{draft.robot_title} 场景")
            form.addRow("场景 id", self.save_id)
            form.addRow("标题", self.save_title)
            lay.addLayout(form)
            btn = QPushButton("保存到 configs/scenes/user/")
            btn.clicked.connect(self._on_save)
            lay.addWidget(btn)
            lay.addWidget(QLabel("保存后可在「Pico仿真遥操 / 自动数据采集 / 相机标定」中选择。"))
            lay.addStretch(1)
            return w

        # ── props ──
        def _reload_props(self) -> None:
            self.prop_list.clear()
            for p in self.draft.props:
                zh = (p.get("labels") or {}).get("zh") or ""
                self.prop_list.addItem(
                    f"{p.get('name')}  |  {zh}  |  kind={p.get('kind')}  size={p.get('size')}"
                )

        def _fill_catalog(self) -> None:
            from teleop_birth.scene.props.catalog import PropCatalog, resolve_library_dirs

            cat = PropCatalog.from_project(self.project_root)
            dirs = resolve_library_dirs(self.project_root)
            if hasattr(self, "library_path_label"):
                shown = " + ".join(str(d) for d in dirs) if dirs else "(未配置)"
                self.library_path_label.setText(f"当前模型库: {shown}")

            entries = cat.list_entries()
            if hasattr(self, "catalog_list"):
                self.catalog_list.clear()
                for name, display in entries:
                    item = ui["QListWidgetItem"](display)
                    item.setData(ui["Qt"].ItemDataRole.UserRole, name)
                    self.catalog_list.addItem(item)
                if self.catalog_list.count() > 0:
                    self.catalog_list.setCurrentRow(0)

            if hasattr(self, "catalog_combo"):
                self.catalog_combo.clear()
                for name, display in entries:
                    self.catalog_combo.addItem(display, name)
                if self.catalog_combo.count() > 0:
                    self.catalog_combo.setCurrentIndex(0)

            self.statusBar().showMessage(f"模型库已加载 {len(entries)} 项", 4000)

        def _on_change_library(self) -> None:
            from teleop_birth.scene.props.catalog import (
                resolve_library_dirs,
                save_library_override,
            )

            start = resolve_library_dirs(self.project_root)
            start_dir = str(start[0]) if start else str(self.project_root)
            path = QFileDialog.getExistingDirectory(
                self, "选择模型库目录（内含 *.yaml 道具）", start_dir
            )
            if not path:
                return
            save_library_override(self.project_root, path)
            self._fill_catalog()
            QMessageBox.information(
                self,
                "模型库",
                f"已切换模型库目录：\n{path}\n\n"
                f"（已写入 configs/scene/model_library.yaml → override_root）",
            )

        def _on_reset_library(self) -> None:
            from teleop_birth.scene.props.catalog import save_library_override

            save_library_override(self.project_root, None)
            self._fill_catalog()
            self.statusBar().showMessage("已恢复默认模型库 configs/scene/props", 5000)

        def _unique_prop_name(self, base: str) -> str:
            existing = {str(p.get("name") or "") for p in self.draft.props}
            if base not in existing:
                return base
            i = 1
            while f"{base}_{i}" in existing:
                i += 1
            return f"{base}_{i}"

        def _on_semantic_add(self) -> None:
            text = self._semantic_text().strip()
            if not text:
                QMessageBox.warning(
                    self,
                    "提示",
                    "请输入中文语义描述。\n"
                    "可点绿色中文词条、在输入框用中文输入法输入汉字，或粘贴剪贴板。",
                )
                return
            prop = _semantic_to_prop(text, len(self.draft.props))
            self.draft.props.append(prop)
            self._reload_props()
            self.prop_list.setCurrentRow(len(self.draft.props) - 1)
            self._clear_semantic_text()
            kind_zh = {
                "bag": "快递袋(椭圆软包)",
                "box": "盒子",
                "plate": "盘子",
                "cylinder": "圆柱/瓶",
                "sphere": "球",
            }.get(str(prop["kind"]), str(prop["kind"]))
            self.statusBar().showMessage(
                f"已生成: {prop['name']} ←「{text}」→ {kind_zh}", 10000
            )
            self._on_refresh_preview()
            QMessageBox.information(
                self,
                "已生成",
                f"输入: {text}\n"
                f"内部名(ASCII): {prop['name']}\n"
                f"类型: {kind_zh} (kind={prop['kind']})\n"
                f"尺寸: {prop['size']}\n\n"
                "中文保存在标签里；MuJoCo 使用英文内部名。",
            )

        def _semantic_text(self) -> str:
            w = getattr(self, "sem_edit", None)
            if w is None:
                return ""
            if hasattr(w, "toPlainText"):
                return w.toPlainText()
            return w.text()

        def _clear_semantic_text(self) -> None:
            w = getattr(self, "sem_edit", None)
            if w is None:
                return
            if hasattr(w, "clear"):
                w.clear()

        def _set_semantic_text(self, text: str) -> None:
            w = getattr(self, "sem_edit", None)
            if w is None:
                return
            if hasattr(w, "setPlainText"):
                w.setPlainText(text)
            else:
                w.setText(text)

        def _on_switch_chinese_ime(self) -> None:
            """启用系统中文输入法，使输入框直接输出汉字。"""
            try:
                from teleop_birth.ui.qt_ime import (
                    current_engine,
                    enable_widget_ime,
                    switch_to_chinese_ime,
                )

                ok = switch_to_chinese_ime(verbose=False)
                eng = current_engine()
                if hasattr(self, "sem_edit"):
                    enable_widget_ime(self.sem_edit)
                    self.sem_edit.setFocus()
                if ok or eng == "libpinyin":
                    self.statusBar().showMessage(
                        f"中文输入法已启用（{eng}）— 请在输入框直接输入汉字",
                        8000,
                    )
                else:
                    self.statusBar().showMessage(
                        f"启用未成功（当前={eng or '无'}）。也可点绿色中文词条或粘贴。",
                        8000,
                    )
            except Exception as exc:  # noqa: BLE001
                self.statusBar().showMessage(f"启用中文输入法失败: {exc}", 6000)

        def _wrap_focus_ime(self, edit):
            """获焦时确保中文输入法引擎已就绪。"""
            orig = edit.focusInEvent

            def _fn(event) -> None:
                try:
                    from teleop_birth.ui.qt_ime import switch_to_chinese_ime

                    switch_to_chinese_ime(verbose=False)
                except Exception:
                    pass
                orig(event)

            return _fn

        def _on_paste_semantic(self) -> None:
            clip = ui["QApplication"].clipboard().text()
            if not clip.strip():
                QMessageBox.information(self, "剪贴板", "剪贴板为空。请先复制中文再粘贴。")
                return
            self._set_semantic_text(clip.strip())
            self.statusBar().showMessage(f"已粘贴: {clip.strip()[:40]}", 5000)

        def _on_zenity_chinese(self) -> None:
            QMessageBox.information(
                self,
                "中文输入",
                "请点绿色中文词条，或点「启用中文输入法」后在输入框输入汉字。",
            )

        def _on_terminal_semantic(self) -> None:
            self._on_switch_chinese_ime()

        def _on_catalog_add(self) -> None:
            from teleop_birth.scene.props.catalog import PropCatalog

            name = None
            if hasattr(self, "catalog_list"):
                item = self.catalog_list.currentItem()
                if item is not None:
                    name = item.data(Qt.ItemDataRole.UserRole)
            if not name and hasattr(self, "catalog_combo"):
                name = self.catalog_combo.currentData()
                if not name:
                    text = self.catalog_combo.currentText().strip()
                    if "[" in text and text.endswith("]"):
                        name = text.rsplit("[", 1)[-1].rstrip("]").strip()
                    else:
                        name = text
            if not name:
                QMessageBox.warning(
                    self,
                    "提示",
                    "请先在模型库列表中选中一项。\n"
                    "默认库: configs/scene/props/\n"
                    "也可点「更换模型库目录」指向自定义文件夹。",
                )
                return
            cat = PropCatalog.from_project(self.project_root)
            try:
                prop = cat.get(str(name)).to_dict()
            except KeyError as exc:
                QMessageBox.warning(self, "提示", str(exc))
                return
            uniq = self._unique_prop_name(str(prop.get("name") or name))
            prop["name"] = uniq
            # 错开初始位置，避免叠在一起
            n = len(self.draft.props)
            pos = list(prop.get("pos") or [0.12, 0.38, 0.45])
            while len(pos) < 3:
                pos.append(0.45)
            pos[0] = float(pos[0]) + 0.04 * (n % 5)
            pos[1] = float(pos[1]) + 0.03 * ((n // 5) % 4)
            prop["pos"] = pos
            self.draft.props.append(prop)
            self._reload_props()
            self.prop_list.setCurrentRow(len(self.draft.props) - 1)
            # 离屏同步编译；不立刻重启 MuJoCo viewer（与 Qt 抢 GL 会卡死）
            ok = True
            if self.preview is not None and hasattr(self.preview, "sync_draft"):
                try:
                    ok = bool(self.preview.sync_draft(self.draft))
                except Exception as exc:  # noqa: BLE001
                    ok = False
                    self.statusBar().showMessage(f"离屏编译失败: {exc}", 8000)
            msg = f"已拉取: {uniq}"
            if ok:
                msg += "（已更新离屏预览；MuJoCo 窗口请点「刷新预览」）"
            else:
                err = getattr(self.preview, "_last_error", None) if self.preview else None
                msg += f"（离屏编译失败: {err or 'unknown'}）"
            self.statusBar().showMessage(msg, 8000)
            QTimer.singleShot(200, self._update_cam_view)

        def _on_prop_selected(self, row: int) -> None:
            if row < 0 or row >= len(self.draft.props):
                return
            p = self.draft.props[row]
            self.prop_name_edit.setText(str(p.get("name") or ""))
            size = list(p.get("size") or [0.025, 0.025, 0.025])
            while len(size) < 3:
                size.append(float(size[-1]) if size else 0.025)
            self.sz_x.setValue(float(size[0]))
            self.sz_y.setValue(float(size[1]))
            self.sz_z.setValue(float(size[2]))
            rgba = p.get("rgba") or [0.8, 0.8, 0.8, 1.0]
            self.color.setText(",".join(str(float(x)) for x in rgba))
            fr = p.get("friction") or [1.0, 0.005, 0.0001]
            self.friction.setText(",".join(str(float(x)) for x in fr))

        def _on_apply_prop(self) -> None:
            row = self.prop_list.currentRow()
            if row < 0 or row >= len(self.draft.props):
                QMessageBox.warning(self, "提示", "请先在上方列表中选中一个物品")
                return
            p = self.draft.props[row]
            new_name = self.prop_name_edit.text().strip()
            if new_name:
                # MuJoCo 名必须 ASCII；中文保留在 labels
                safe = "".join(
                    c if c.isascii() and (c.isalnum() or c in "-_") else "_"
                    for c in new_name
                )
                safe = "_".join(x for x in safe.split("_") if x) or f"obj_{row}"
                if safe != new_name:
                    labels = dict(p.get("labels") or {})
                    labels.setdefault("zh", new_name)
                    p["labels"] = labels
                p["name"] = safe[:48]
            kind = str(p.get("kind") or "box").lower()
            if kind in ("plate", "cylinder", "disc", "disk"):
                p["size"] = [self.sz_x.value(), self.sz_y.value()]
            elif kind in ("sphere", "ball"):
                p["size"] = [self.sz_x.value()]
            else:
                # box / bag / ellipsoid：三轴
                p["size"] = [self.sz_x.value(), self.sz_y.value(), self.sz_z.value()]
            try:
                p["rgba"] = [
                    float(x) for x in self.color.text().replace("，", ",").split(",") if x.strip()
                ]
                p["friction"] = [
                    float(x)
                    for x in self.friction.text().replace("，", ",").split(",")
                    if x.strip()
                ]
            except ValueError:
                QMessageBox.warning(self, "错误", "颜色/摩擦格式无效，请用逗号分隔数字")
                return
            if len(p["rgba"]) < 4:
                p["rgba"] = (p["rgba"] + [1.0, 1.0, 1.0, 1.0])[:4]
            self._reload_props()
            self.prop_list.setCurrentRow(row)
            self.statusBar().showMessage(f"已更新属性: {p.get('name')}", 5000)
            self._on_refresh_preview()

        def _on_delete_prop(self) -> None:
            row = self.prop_list.currentRow()
            if row < 0 or row >= len(self.draft.props):
                QMessageBox.warning(self, "提示", "请先选中要删除的物品")
                return
            name = self.draft.props[row].get("name")
            del self.draft.props[row]
            self._reload_props()
            self.statusBar().showMessage(f"已删除: {name}", 5000)
            self._on_refresh_preview()

        # ── camera ──
        def _seed_default_camera(self) -> None:
            self.draft.cameras = [
                {
                    "name": "third_person",
                    "type": "world",
                    "role": "third_person",
                    "camera_asset": "realsense_d455",
                    "pos": [0.55, -0.75, 1.15],
                    "lookat": [0.35, 0.0, 0.45],
                    "fovy_deg": 45.0,
                }
            ]

        def _reload_cams(self) -> None:
            self.cam_list.clear()
            for c in self.draft.cameras:
                self.cam_list.addItem(
                    f"{c.get('name')}  [{c.get('role') or c.get('type')}]  "
                    f"pos={c.get('pos') or c.get('offset_xyz')}"
                )

        def _current_cam_dict(self) -> dict[str, Any]:
            role_txt = self.cam_role.currentText()
            wrist = "wrist" in role_txt
            name = self.cam_name.text().strip() or ("wrist_right" if wrist else "third_person")
            cam: dict[str, Any] = {
                "name": name,
                "type": "site" if wrist else "world",
                "role": "wrist" if wrist else "third_person",
                "camera_asset": "realsense_d405" if wrist else "realsense_d455",
                "fovy_deg": float(self.cam_fovy.value()),
                "lookat": [self.look_x.value(), self.look_y.value(), self.look_z.value()],
            }
            if wrist:
                cam["mount"] = "wrist_camera_bracket"
                cam["site"] = "R_g_pinch"
                cam["offset_xyz"] = [self.cam_x.value(), self.cam_y.value(), self.cam_z.value()]
                cam["pos"] = list(cam["offset_xyz"])
            else:
                cam["pos"] = [self.cam_x.value(), self.cam_y.value(), self.cam_z.value()]
            return cam

        def _on_cam_selected(self, row: int) -> None:
            if row < 0 or row >= len(self.draft.cameras):
                return
            c = self.draft.cameras[row]
            self.cam_name.setText(str(c.get("name") or ""))
            wrist = str(c.get("type") or c.get("role") or "") in ("site", "wrist")
            self.cam_role.setCurrentIndex(1 if wrist else 0)
            pos = list(c.get("pos") or c.get("offset_xyz") or [0.55, -0.75, 1.15])
            look = list(c.get("lookat") or [0.35, 0.0, 0.45])
            while len(pos) < 3:
                pos.append(0.0)
            while len(look) < 3:
                look.append(0.0)
            self.cam_x.setValue(float(pos[0]))
            self.cam_y.setValue(float(pos[1]))
            self.cam_z.setValue(float(pos[2]))
            self.look_x.setValue(float(look[0]))
            self.look_y.setValue(float(look[1]))
            self.look_z.setValue(float(look[2]))
            self.cam_fovy.setValue(float(c.get("fovy_deg") or 45.0))
            self._update_cam_view()

        def _on_cam_spin_changed(self) -> None:
            # 轻量：只刷新画面，写回列表在「应用」时做
            if self.cam_list.currentRow() >= 0:
                self._sync_cam_from_spins(apply=True)
            self._update_cam_view()

        def _sync_cam_from_spins(self, *, apply: bool) -> None:
            if not apply:
                return
            row = self.cam_list.currentRow()
            cam = self._current_cam_dict()
            if row < 0 or row >= len(self.draft.cameras):
                return
            self.draft.cameras[row] = cam

        def _on_apply_cam_pose(self) -> None:
            row = self.cam_list.currentRow()
            cam = self._current_cam_dict()
            if row < 0:
                self.draft.cameras.append(cam)
            else:
                self.draft.cameras[row] = cam
            self._reload_cams()
            idx = next(
                (i for i, c in enumerate(self.draft.cameras) if c.get("name") == cam["name"]),
                len(self.draft.cameras) - 1,
            )
            self.cam_list.setCurrentRow(idx)
            self._update_cam_view()
            self.statusBar().showMessage(f"相机已更新: {cam['name']}", 4000)

        def _on_add_camera(self) -> None:
            cam = self._current_cam_dict()
            name = cam["name"]
            self.draft.cameras = [c for c in self.draft.cameras if c.get("name") != name]
            self.draft.cameras.append(cam)
            self._reload_cams()
            self.cam_list.setCurrentRow(len(self.draft.cameras) - 1)
            self._update_cam_view()
            self.statusBar().showMessage(f"相机已加入列表: {name}", 4000)

        def _update_cam_view(self) -> None:
            if self.preview is None:
                self.cam_view.setText("无预览会话")
                return
            cam = self._current_cam_dict()
            row = self.cam_list.currentRow()
            if 0 <= row < len(self.draft.cameras):
                # 列表项优先
                cam = {**self.draft.cameras[row], **{k: cam[k] for k in ("pos", "lookat", "fovy_deg") if k in cam}}
                cam["pos"] = [self.cam_x.value(), self.cam_y.value(), self.cam_z.value()]
                cam["lookat"] = [self.look_x.value(), self.look_y.value(), self.look_z.value()]
            try:
                rgb = self.preview.render_rgb(cam, width=640, height=480)
            except Exception as exc:  # noqa: BLE001
                self.cam_view.setText(f"渲染失败: {exc}")
                return
            if rgb is None:
                err = getattr(self.preview, "_last_error", None) or "编译失败"
                self.cam_view.setText(f"无画面（{err}）")
                return
            h, w, ch = rgb.shape
            qimg = QImage(rgb.data, w, h, ch * w, QImage.Format.Format_RGB888).copy()
            self.cam_view.setPixmap(
                QPixmap.fromImage(qimg).scaled(
                    self.cam_view.size(),
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                )
            )

        def _on_tab_changed(self, idx: int) -> None:
            # 相机页：开定时刷新；其它页关掉
            if self._cam_timer is None:
                return
            if idx == 1:
                self._cam_timer.start()
                self._update_cam_view()
            else:
                self._cam_timer.stop()

        # ── frames / save / preview ──
        def _on_frame_viz(self) -> None:
            try:
                from teleop_birth.teleop.frame.viz import run_frame_viz

                run_frame_viz(tab="axes", block=False)
            except Exception as exc:  # noqa: BLE001
                QMessageBox.warning(self, "坐标系", str(exc))

        def _on_toggle_base_frames(self) -> None:
            try:
                from teleop_birth.ui.session.base_frame_viz import get_base_frame_overlay

                ov = get_base_frame_overlay()
                on = ov.toggle()
                self.statusBar().showMessage(
                    f"场景 base 坐标系: {'显示' if on else '隐藏'}"
                    "（青=桌面接触 / 品红=转换后 URDF）",
                    6000,
                )
            except Exception as exc:  # noqa: BLE001
                QMessageBox.warning(self, "base", str(exc))

        def _on_refresh_preview(self) -> None:
            if self.preview is None:
                QMessageBox.information(self, "预览", "当前未启动 MuJoCo 预览窗口；离屏渲染仍可用。")
                return
            try:
                self.preview.request_rebuild(self.draft)
                # 给编译一点时间后刷新相机
                QTimer.singleShot(400, self._update_cam_view)
                self.statusBar().showMessage("已请求重建 MuJoCo 预览…", 4000)
            except Exception as exc:  # noqa: BLE001
                QMessageBox.warning(self, "预览", str(exc))

        def _on_save(self) -> None:
            sid = self.save_id.text().strip()
            if not sid:
                QMessageBox.warning(self, "保存", "请填写场景 id")
                return
            path = save_scene_draft(
                self.project_root,
                self.draft,
                scene_id=sid,
                title=self.save_title.text().strip() or None,
            )
            self.saved_path = path
            QMessageBox.information(self, "保存成功", f"已写入\n{path}")
            self.close()

        def closeEvent(self, event) -> None:  # noqa: N802
            if self._cam_timer is not None:
                self._cam_timer.stop()
            super().closeEvent(event)

    return SceneWorkshopWindow()


def _semantic_to_prop(text: str, idx: int) -> dict[str, Any]:
    """中英双语规则语义 → 程序化物品。

    - MuJoCo ``name`` 只用 ASCII（中文写在 labels.zh / labels.en）
    - 「快递袋子」→ kind=bag（椭圆软包，不是硬纸盒）
    """
    raw = (text or "").strip()
    raw = raw.replace("\u3000", " ").replace("，", ",").replace("。", ".")
    t = raw.lower()
    zh = raw
    en = raw if any(c.isascii() and c.isalpha() for c in raw) else ""

    def hit(*keys: str) -> bool:
        return any(k.lower() in t or k in zh for k in keys)

    rgba = [0.78, 0.58, 0.32, 1.0]
    if hit("蓝", "蓝色", "blue"):
        rgba = [0.2, 0.35, 0.9, 1.0]
    elif hit("绿", "绿色", "green"):
        rgba = [0.2, 0.75, 0.3, 1.0]
    elif hit("红", "红色", "red"):
        rgba = [0.9, 0.15, 0.15, 1.0]
    elif hit("黄", "黄色", "yellow"):
        rgba = [0.95, 0.85, 0.15, 1.0]
    elif hit("白", "白色", "white"):
        rgba = [0.92, 0.92, 0.92, 1.0]
    elif hit("黑", "黑色", "black"):
        rgba = [0.15, 0.15, 0.15, 1.0]
    elif hit("棕", "褐", "牛皮", "灰褐", "brown", "kraft", "tan"):
        rgba = [0.72, 0.52, 0.28, 1.0]

    kind = "box"
    size: list[float] = [0.04, 0.03, 0.02]
    ascii_stem = f"obj_{idx}"

    if hit(
        "快递袋",
        "快递袋子",
        "塑料袋",
        "购物袋",
        "信封袋",
        "袋子",
        "袋",
        "软包",
        "bag",
        "mailer",
        "polybag",
        "pouch",
        "sack",
        "courier bag",
        "shipping bag",
    ):
        kind = "bag"
        size = [0.10, 0.07, 0.018]
        ascii_stem = f"bag_{idx}"
        if rgba == [0.78, 0.58, 0.32, 1.0]:
            rgba = [0.70, 0.48, 0.22, 1.0]
    elif hit("盘", "碟子", "托盘", "plate", "dish", "tray"):
        kind = "plate"
        size = [0.06, 0.008]
        ascii_stem = f"plate_{idx}"
    elif hit("瓶", "罐子", "圆筒", "bottle", "can", "cylinder", "jar"):
        kind = "cylinder"
        size = [0.02, 0.05]
        ascii_stem = f"cyl_{idx}"
    elif hit("球", "圆球", "sphere", "ball"):
        kind = "sphere"
        size = [0.025]
        ascii_stem = f"sphere_{idx}"
    elif hit("快递盒", "纸箱", "纸盒", "盒子", "箱子", "方盒", "box", "carton", "chest"):
        kind = "box"
        size = [0.06, 0.045, 0.035]
        ascii_stem = f"box_{idx}"
    elif hit("方块", "小方块", "积木", "立方", "cube", "block", "立方体"):
        kind = "box"
        size = [0.025, 0.025, 0.025]
        ascii_stem = f"cube_{idx}"
    elif hit("快递", "包裹", "parcel", "package", "shipment"):
        kind = "bag"
        size = [0.09, 0.06, 0.02]
        ascii_stem = f"parcel_{idx}"

    z = 0.46 + float(size[-1] if size else 0.02)
    return {
        "name": ascii_stem,
        "kind": kind,
        "rgba": rgba,
        "size": size,
        "mass": 0.06 if kind == "bag" else 0.08,
        "friction": [1.4, 0.005, 0.0001] if kind == "bag" else [1.2, 0.005, 0.0001],
        "freejoint": True,
        "pos": [0.35 + 0.06 * (idx % 3), 0.05 * ((idx % 5) - 2), z],
        "quat_wxyz": [1, 0, 0, 0],
        "labels": {
            "zh": zh,
            "en": en or ascii_stem,
            "source": "semantic",
            "lang": "zh" if any("\u4e00" <= c <= "\u9fff" for c in zh) else "en",
        },
    }
