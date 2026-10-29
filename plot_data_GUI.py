import sys
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from pathlib import Path
import pandas as pd
import numpy as np
import json 
import os

# 嵌入 Matplotlib
import matplotlib
matplotlib.use("TkAgg")
matplotlib.rcParams['axes.unicode_minus'] = False  # 正常显示负号

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import matplotlib.pyplot as plt

class IEEEAdvancedPlotter:
    def __init__(self, root):
        self.root = root
        self.root.title("Data Plotter")
        
        # ==================== DPI 适配与自适应大小计算 ====================
        try:
            dpi = self.root.winfo_fpixels('1i')
            self.scale_factor = dpi / 96.0
        except Exception:
            self.scale_factor = 1.0
            
        self.scale_factor = max(1.0, min(self.scale_factor, 2.5))
        
        init_w = int(1620 * self.scale_factor) 
        init_h = int(900 * self.scale_factor)
        self.root.geometry(f"{init_w}x{init_h}")
        self.root.minsize(int(1200 * self.scale_factor), int(750 * self.scale_factor))

        # 核心数据存储
        self.file_path = None
        self.df = None          
        self.time_data = None   
        self.motion_data = {}   
        self.current_curves = [] 
        
        # 样式与配置映射表
        self.custom_legends = {} 
        self.curve_linestyles = {} 
        self.curve_markers = {}
        self.curve_linewidths = {}  
        self.curve_colors = {} 
        
        # ==================== 每个 Category 独立属性设计 ====================
        self.category_ylabels = {}       # 各自独立的 Y 轴 Label
        self.category_custom_ylims = {}  # 各自独立的 Y 轴限制开关 (bool)
        self.category_ylim_mins = {}     # 各自独立的 Y 轴最小值 (str)
        self.category_ylim_maxs = {}     # 各自独立的 Y 轴最大值 (str)
        self.current_active_category = None  # 当前正处于面板激活编辑态的 Category

        # 初始化经典字体配置与界面组件
        self.setup_styles()
        self.create_widgets()

        # 监听窗口关闭
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_styles(self):
        self.font_size_large = int(11 * self.scale_factor)
        self.font_size_normal = int(10 * self.scale_factor)
        self.font_size_small = int(9 * self.scale_factor)
        self.font_family = "Microsoft YaHei" if sys.platform.startswith("win") else "Arial"

        # 统一配置经典 Tkinter 基础组件字体
        default_font = tk.font.nametofont("TkDefaultFont")
        default_font.configure(size=self.font_size_normal, family=self.font_family)
        
        text_font = tk.font.nametofont("TkTextFont")
        text_font.configure(size=self.font_size_normal, family=self.font_family)
        
        heading_font = tk.font.nametofont("TkHeadingFont")
        heading_font.configure(size=self.font_size_large, weight="bold", family=self.font_family)

        # 仅针对 Treeview 行高做微调，不改变原生风格
        self.style = ttk.Style()
        row_height = int(26 * self.scale_factor)
        self.style.configure("Treeview", rowheight=row_height)

    def create_widgets(self):
        # ==================== 1. TOP CONTROL BAR (顶部控制栏) ====================
        top_frame = ttk.Frame(self.root, padding=f"{int(10*self.scale_factor)} {int(5*self.scale_factor)}")
        top_frame.pack(side=tk.TOP, fill=tk.X)

        self.btn_import = ttk.Button(top_frame, text="Import Data", command=self.import_file)
        self.btn_import.pack(side=tk.LEFT, padx=int(5*self.scale_factor))

        self.lbl_file_path = ttk.Label(top_frame, text="Please import a data file...", foreground="gray")
        self.lbl_file_path.pack(side=tk.LEFT, padx=int(10*self.scale_factor), fill=tk.X, expand=True)

        # X轴数据源选择
        ttk.Label(top_frame, text="X-Axis:").pack(side=tk.LEFT, padx=int(5*self.scale_factor))
        self.combo_x_axis = ttk.Combobox(top_frame, state="readonly", width=int(12*self.scale_factor))
        self.combo_x_axis.pack(side=tk.LEFT, padx=int(5*self.scale_factor))
        self.combo_x_axis.bind("<<ComboboxSelected>>", self.on_x_axis_changed)

        self.var_enable_grid = tk.BooleanVar(value=True)
        self.chk_grid = ttk.Checkbutton(top_frame, text="Grid On", variable=self.var_enable_grid, command=self.draw_curves)
        self.chk_grid.pack(side=tk.LEFT, padx=int(5*self.scale_factor))

        main_pane = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=int(10*self.scale_factor), pady=int(5*self.scale_factor))

        # ---- LEFT PANEL (左侧自适应父容器) ----
        left_parent_outer = ttk.Frame(main_pane)
        main_pane.add(left_parent_outer, weight=2)

        def enforce_left_min_width(event):
            min_w = int(420 * self.scale_factor)
            if event.width < min_w:
                left_parent_outer.config(width=min_w)

        left_parent_outer.bind('<Configure>', enforce_left_min_width)

        left_canvas = tk.Canvas(left_parent_outer, highlightthickness=0)
        left_scrollbar = ttk.Scrollbar(left_parent_outer, orient=tk.VERTICAL, command=left_canvas.yview)
        left_parent = ttk.Frame(left_canvas)
        
        left_parent.bind("<Configure>", lambda e: left_canvas.configure(scrollregion=left_canvas.bbox("all")))
        left_canvas_window = left_canvas.create_window((0, 0), window=left_parent, anchor="nw")
        
        def on_left_canvas_resize(event):
            min_w = int(420 * self.scale_factor)
            target_w = max(min_w, event.width)
            left_canvas.itemconfig(left_canvas_window, width=target_w)
            
        left_canvas.bind('<Configure>', on_left_canvas_resize)
        left_canvas.configure(yscrollcommand=left_scrollbar.set)
        left_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # 2a. Upper-Left: Treeview Data List
        left_frame = ttk.LabelFrame(left_parent, text=" Data Columns List ", padding=int(5*self.scale_factor))
        left_frame.pack(side=tk.TOP, fill=tk.X, pady=int(3*self.scale_factor))

        tree_container = ttk.Frame(left_frame, height=int(200 * self.scale_factor))
        tree_container.pack(fill=tk.X, expand=True)
        tree_container.pack_propagate(False) 

        self.tree = ttk.Treeview(tree_container, columns=("category", "label"), show="tree headings")
        self.tree.heading("#0", text="Categories / Metrics")
        self.tree.column("#0", stretch=tk.YES)
        self.tree.column("category", width=0, stretch=tk.NO)
        self.tree.column("label", width=0, stretch=tk.NO)
        
        tree_scroll = ttk.Scrollbar(tree_container, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=tree_scroll.set)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree.bind("<<TreeviewSelect>>", self.on_tree_select)

        # 2b. Middle-Left: Curve Configurator
        self.legend_frame = ttk.LabelFrame(left_parent, text=" Curve Configurator (LaTeX, Colors & Styles) ", padding=int(5*self.scale_factor))
        self.legend_frame.pack(side=tk.TOP, fill=tk.X, pady=int(5*self.scale_factor))
        
        canvas_height = int(360 * self.scale_factor)
        self.legend_canvas = tk.Canvas(self.legend_frame, height=canvas_height, highlightthickness=0)
        self.legend_scrollbar = ttk.Scrollbar(self.legend_frame, orient=tk.VERTICAL, command=self.legend_canvas.yview)
        self.legend_editor_container = ttk.Frame(self.legend_canvas)
        self.legend_editor_container.bind("<Configure>", lambda e: self.legend_canvas.configure(scrollregion=self.legend_canvas.bbox("all")))
        
        legend_canvas_window = self.legend_canvas.create_window((0, 0), window=self.legend_editor_container, anchor="nw")
        self.legend_canvas.bind('<Configure>', lambda e: self.legend_canvas.itemconfig(legend_canvas_window, width=e.width))
        
        self.legend_canvas.configure(yscrollcommand=self.legend_scrollbar.set)
        self.legend_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.legend_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # 2c. Global Typography & Sizes
        font_settings_frame = ttk.LabelFrame(left_parent, text=" Global Typography & Sizes (IEEE Standards) ", padding=int(8*self.scale_factor))
        font_settings_frame.pack(side=tk.TOP, fill=tk.X, pady=int(5*self.scale_factor))

        fs_row1 = ttk.Frame(font_settings_frame)
        fs_row1.pack(fill=tk.X, pady=int(3*self.scale_factor))
        
        ttk.Label(fs_row1, text="Main Font:").pack(side=tk.LEFT, padx=2)
        self.combo_font_family = ttk.Combobox(fs_row1, values=["Times New Roman", "Arial", "DejaVu Serif", "Courier New"], state="readonly", width=int(12*self.scale_factor))
        self.combo_font_family.set("Times New Roman")
        self.combo_font_family.pack(side=tk.LEFT, padx=5)
        self.combo_font_family.bind("<<ComboboxSelected>>", lambda e: self.draw_curves())

        ttk.Label(fs_row1, text="Math Set:").pack(side=tk.LEFT, padx=(10, 2))
        self.combo_math_font = ttk.Combobox(fs_row1, values=["stix", "stixsans", "cm", "custom", "dejavusans", "dejavuserif"], state="readonly", width=int(10*self.scale_factor))
        self.combo_math_font.set("stix")
        self.combo_math_font.pack(side=tk.LEFT, padx=5)
        self.combo_math_font.bind("<<ComboboxSelected>>", lambda e: self.draw_curves())

        fs_row2 = ttk.Frame(font_settings_frame)
        fs_row2.pack(fill=tk.X, pady=int(3*self.scale_factor))

        ttk.Label(fs_row2, text="Label:").pack(side=tk.LEFT, padx=2)
        self.combo_label_size = ttk.Combobox(fs_row2, values=["8", "9", "10", "11", "12", "14", "16"], state="readonly", width=int(4*self.scale_factor))
        self.combo_label_size.set("14")
        self.combo_label_size.pack(side=tk.LEFT, padx=5)
        self.combo_label_size.bind("<<ComboboxSelected>>", lambda e: self.draw_curves())

        ttk.Label(fs_row2, text="Legend:").pack(side=tk.LEFT, padx=(10, 2))
        self.combo_legend_size = ttk.Combobox(fs_row2, values=["8", "9", "10", "11", "12", "14", "16"], state="readonly", width=int(4*self.scale_factor))
        self.combo_legend_size.set("14")
        self.combo_legend_size.pack(side=tk.LEFT, padx=5)
        self.combo_legend_size.bind("<<ComboboxSelected>>", lambda e: self.draw_curves())

        ttk.Label(fs_row2, text="Title:").pack(side=tk.LEFT, padx=(10, 2))
        self.combo_title_size = ttk.Combobox(fs_row2, values=["8", "9", "10", "11", "12", "14", "16"], state="readonly", width=int(4*self.scale_factor))
        self.combo_title_size.set("14")
        self.combo_title_size.pack(side=tk.LEFT, padx=5)
        self.combo_title_size.bind("<<ComboboxSelected>>", lambda e: self.draw_curves())

        # 2d. Lower-Left: Toolbox
        toolbox_frame = ttk.LabelFrame(left_parent, text=" Aesthetics & Enhancements Toolbox ", padding=int(8*self.scale_factor))
        toolbox_frame.pack(side=tk.TOP, fill=tk.X, pady=int(5*self.scale_factor))

        row2 = ttk.Frame(toolbox_frame)
        row2.pack(fill=tk.X, pady=int(6*self.scale_factor))
        self.var_enable_zoom = tk.BooleanVar(value=False)
        self.chk_zoom = ttk.Checkbutton(row2, text="Enable Inset Zoom Window", variable=self.var_enable_zoom, command=self.draw_curves)
        self.chk_zoom.pack(side=tk.LEFT, padx=int(2*self.scale_factor))
        
        ttk.Label(row2, text="X Range:").pack(side=tk.LEFT, padx=(int(10*self.scale_factor), int(2*self.scale_factor)))
        self.entry_zoom_start = ttk.Entry(row2, width=int(6*self.scale_factor))
        self.entry_zoom_start.insert(0, "0")
        self.entry_zoom_start.pack(side=tk.LEFT, padx=int(2*self.scale_factor))
        ttk.Label(row2, text="to").pack(side=tk.LEFT)
        self.entry_zoom_end = ttk.Entry(row2, width=int(6*self.scale_factor))
        self.entry_zoom_end.insert(0, "10")
        self.entry_zoom_end.pack(side=tk.LEFT, padx=int(2*self.scale_factor))
        
        self.entry_zoom_start.bind("<Return>", lambda e: self.draw_curves())
        self.entry_zoom_end.bind("<Return>", lambda e: self.draw_curves())

        row3 = ttk.Frame(toolbox_frame)
        row3.pack(fill=tk.X, pady=(int(8*self.scale_factor), int(2*self.scale_factor)))
        #ttk.Button(row3, text="Clear Selected", command=self.clear_selection, width=int(14*self.scale_factor)).pack(side=tk.LEFT, padx=int(2*self.scale_factor))
        
        ttk.Button(row3, text="Export Ultra PNG (300DPI)", command=lambda: self.export_figure("png")).pack(side=tk.LEFT, padx=int(2*self.scale_factor))
        ttk.Button(row3, text="Export Vector PDF", command=lambda: self.export_figure("pdf")).pack(side=tk.LEFT, padx=int(2*self.scale_factor))
        # 批量保存功能按钮
        ttk.Button(row3, text="Save All Curves", command=self.batch_save_all_curves).pack(side=tk.RIGHT, padx=int(2*self.scale_factor))
        

        # ---- RIGHT PANEL: Canvas Area (右侧绘图区域) ----
        right_frame = ttk.Frame(main_pane, padding=int(10*self.scale_factor))
        main_pane.add(right_frame, weight=3)

        # 配置 Title、Labels、X范围、每个 Category 独立的 Y 配置区域
        plot_config_panel = ttk.LabelFrame(right_frame, text=" Plot Configurations Panel ", padding=int(8*self.scale_factor))
        plot_config_panel.pack(side=tk.TOP, fill=tk.X, pady=(0, int(10*self.scale_factor)))

        # Panel第一行：Title & Labels配置
        config_row1 = ttk.Frame(plot_config_panel)
        config_row1.pack(fill=tk.X, pady=int(2*self.scale_factor))

        ttk.Label(config_row1, text="Title:").pack(side=tk.LEFT, padx=int(2*self.scale_factor))
        self.entry_title = ttk.Entry(config_row1, width=int(18*self.scale_factor))
        self.entry_title.pack(side=tk.LEFT, padx=int(5*self.scale_factor), fill=tk.X, expand=True)
        self.entry_title.bind("<Return>", lambda e: self.draw_curves()) 

        ttk.Label(config_row1, text="X-Label:").pack(side=tk.LEFT, padx=int(5*self.scale_factor))
        self.entry_x_label = ttk.Entry(config_row1, width=int(15*self.scale_factor))
        self.entry_x_label.pack(side=tk.LEFT, padx=int(5*self.scale_factor), fill=tk.X, expand=True)
        self.entry_x_label.bind("<Return>", lambda e: self.draw_curves()) 

        ttk.Label(config_row1, text="Y-Label:").pack(side=tk.LEFT, padx=int(5*self.scale_factor))
        self.entry_y_label = ttk.Entry(config_row1, width=int(15*self.scale_factor))
        self.entry_y_label.pack(side=tk.LEFT, padx=int(5*self.scale_factor), fill=tk.X, expand=True)
        self.entry_y_label.bind("<Return>", lambda e: self.draw_curves()) 

        self.btn_refresh = ttk.Button(config_row1, text="🔄 Apply Limits & Text", command=self.draw_curves)
        self.btn_refresh.pack(side=tk.RIGHT, padx=int(5*self.scale_factor))

        # Panel第二行：X轴限制 & 增加的Y轴限制功能配置
        config_row2 = ttk.Frame(plot_config_panel)
        config_row2.pack(fill=tk.X, pady=int(4*self.scale_factor))

        # X-Lim 
        self.var_custom_xlim = tk.BooleanVar(value=False)
        self.chk_xlim = ttk.Checkbutton(config_row2, text="Set X-Lim", variable=self.var_custom_xlim, command=self.draw_curves)
        self.chk_xlim.pack(side=tk.LEFT, padx=int(2*self.scale_factor))
        
        ttk.Label(config_row2, text="Min:").pack(side=tk.LEFT, padx=int(2*self.scale_factor))
        self.entry_xlim_min = ttk.Entry(config_row2, width=int(8*self.scale_factor))
        self.entry_xlim_min.insert(0, "0")
        self.entry_xlim_min.pack(side=tk.LEFT, padx=int(5*self.scale_factor))
        self.entry_xlim_min.bind("<Return>", lambda e: self.draw_curves())

        ttk.Label(config_row2, text="Max:").pack(side=tk.LEFT, padx=int(2*self.scale_factor))
        self.entry_xlim_max = ttk.Entry(config_row2, width=int(8*self.scale_factor))
        self.entry_xlim_max.insert(0, "10")
        self.entry_xlim_max.pack(side=tk.LEFT, padx=int(5*self.scale_factor))
        self.entry_xlim_max.bind("<Return>", lambda e: self.draw_curves())

        ttk.Separator(config_row2, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=int(15*self.scale_factor))

        # Y-Lim (现在绑定并作用于各独立的 Category 属性)
        self.var_custom_ylim = tk.BooleanVar(value=False)
        self.chk_ylim = ttk.Checkbutton(config_row2, text="Set Y-Lim", variable=self.var_custom_ylim, command=self.draw_curves)
        self.chk_ylim.pack(side=tk.LEFT, padx=int(2*self.scale_factor))

        ttk.Label(config_row2, text="Min:").pack(side=tk.LEFT, padx=int(2*self.scale_factor))
        self.entry_ylim_min = ttk.Entry(config_row2, width=int(8*self.scale_factor))
        self.entry_ylim_min.insert(0, "-1")
        self.entry_ylim_min.pack(side=tk.LEFT, padx=int(5*self.scale_factor))
        self.entry_ylim_min.bind("<Return>", lambda e: self.draw_curves())

        ttk.Label(config_row2, text="Max:").pack(side=tk.LEFT, padx=int(2*self.scale_factor))
        self.entry_ylim_max = ttk.Entry(config_row2, width=int(8*self.scale_factor))
        self.entry_ylim_max.insert(0, "1")
        self.entry_ylim_max.pack(side=tk.LEFT, padx=int(5*self.scale_factor))
        self.entry_ylim_max.bind("<Return>", lambda e: self.draw_curves())

        # 画布放置在配置 Panel 下方
        self.fig, self.ax = plt.subplots(figsize=(8, 5.5))
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.toolbar = NavigationToolbar2Tk(self.canvas, right_frame)
        self.toolbar.update()
        self.canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.reset_plot_style()

    def get_config_path(self):
        if self.file_path:
            return self.file_path.with_suffix('.json')
        return None

    def auto_save_settings(self):
        config_path = self.get_config_path()
        if not config_path: return

        # 在执行保存前，首先将当前界面输入框内最新的 Y 轴配置同步写回当前的 active category 缓存中
        if self.current_active_category:
            self.category_ylabels[self.current_active_category] = self.entry_y_label.get()
            self.category_custom_ylims[self.current_active_category] = self.var_custom_ylim.get()
            self.category_ylim_mins[self.current_active_category] = self.entry_ylim_min.get()
            self.category_ylim_maxs[self.current_active_category] = self.entry_ylim_max.get()

        selected_nodes = []
        for item in self.tree.selection():
            text = self.tree.item(item, "text")
            values = self.tree.item(item, "values")
            selected_nodes.append({"text": text, "values": list(values) if values else None})

        config_data = {
            "x_axis_selection": self.combo_x_axis.get(),
            "title_text": self.entry_title.get(),
            "x_label": self.entry_x_label.get(),
            "selected_nodes": selected_nodes,
            "custom_legends": self.custom_legends,
            "curve_linestyles": self.curve_linestyles,
            "curve_markers": self.curve_markers,
            "curve_linewidths": self.curve_linewidths,
            "curve_colors": self.curve_colors,
            # 将每个 category 各自独立的 Y 轴属性全部进行 JSON 序列化
            "category_ylabels": self.category_ylabels,
            "category_custom_ylims": self.category_custom_ylims,
            "category_ylim_mins": self.category_ylim_mins,
            "category_ylim_maxs": self.category_ylim_maxs,
            "current_active_category": self.current_active_category,
            "font_family": self.combo_font_family.get(),
            "math_font": self.combo_math_font.get(),
            "label_size": self.combo_label_size.get(),
            "legend_size": self.combo_legend_size.get(),
            "title_size": self.combo_title_size.get(),
            "enable_zoom": self.var_enable_zoom.get(),
            "zoom_start": self.entry_zoom_start.get(),
            "zoom_end": self.entry_zoom_end.get(),
            "enable_grid": self.var_enable_grid.get(),
            "custom_xlim": self.var_custom_xlim.get(),
            "xlim_min": self.entry_xlim_min.get(),
            "xlim_max": self.entry_xlim_max.get()
        }

        try:
            with open(config_path, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=4, ensure_ascii=False)
        except Exception as e:
            print(f"Silent AutoSave Warning: Failed to dump config. {str(e)}")

    def auto_load_settings(self):
        config_path = self.get_config_path()
        if not config_path or not config_path.exists(): 
            return False

        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
            
            if "x_axis_selection" in cfg: self.combo_x_axis.set(cfg["x_axis_selection"])
            if "title_text" in cfg: self.update_entry_text(self.entry_title, cfg["title_text"])
            if "x_label" in cfg: self.update_entry_text(self.entry_x_label, cfg["x_label"])

            if "font_family" in cfg: self.combo_font_family.set(cfg["font_family"])
            if "math_font" in cfg: self.combo_math_font.set(cfg["math_font"])
            if "label_size" in cfg: self.combo_label_size.set(cfg["label_size"])
            if "legend_size" in cfg: self.combo_legend_size.set(cfg["legend_size"])
            if "title_size" in cfg: self.combo_title_size.set(cfg["title_size"])
            
            if "enable_zoom" in cfg: self.var_enable_zoom.set(cfg["enable_zoom"])
            if "zoom_start" in cfg: self.update_entry_text(self.entry_zoom_start, cfg["zoom_start"])
            if "zoom_end" in cfg: self.update_entry_text(self.entry_zoom_end, cfg["zoom_end"])
            if "enable_grid" in cfg: self.var_enable_grid.set(cfg["enable_grid"])
            
            if "custom_xlim" in cfg: self.var_custom_xlim.set(cfg["custom_xlim"])
            if "xlim_min" in cfg: self.update_entry_text(self.entry_xlim_min, cfg["xlim_min"])
            if "xlim_max" in cfg: self.update_entry_text(self.entry_xlim_max, cfg["xlim_max"])

            self.custom_legends = cfg.get("custom_legends", {})
            self.curve_linestyles = cfg.get("curve_linestyles", {})
            self.curve_markers = cfg.get("curve_markers", {})
            self.curve_linewidths = cfg.get("curve_linewidths", {})
            self.curve_colors = cfg.get("curve_colors", {})
            
            # 恢复每个 Category 独立的 Y 配置字典
            self.category_ylabels = cfg.get("category_ylabels", {})
            self.category_custom_ylims = cfg.get("category_custom_ylims", {})
            self.category_ylim_mins = cfg.get("category_ylim_mins", {})
            self.category_ylim_maxs = cfg.get("category_ylim_maxs", {})
            self.current_active_category = cfg.get("current_active_category", None)

            self.tree.selection_remove(self.tree.selection()) 
            saved_nodes = cfg.get("selected_nodes", [])
            
            for node_info in saved_nodes:
                for item in self.tree.get_children():
                    if not node_info["values"] and self.tree.item(item, "text") == node_info["text"]:
                        self.tree.selection_add(item)
                    elif node_info["values"]:
                        for child in self.tree.get_children(item):
                            c_vals = self.tree.item(child, "values")
                            if c_vals == node_info["values"]:
                                self.tree.selection_add(child)

            # 如果有恢复的活跃 Category，刷新其特定的面板显示
            if self.current_active_category:
                self.update_entry_text(self.entry_y_label, self.category_ylabels.get(self.current_active_category, "Amplitude"))
                self.var_custom_ylim.set(self.category_custom_ylims.get(self.current_active_category, False))
                self.update_entry_text(self.entry_ylim_min, self.category_ylim_mins.get(self.current_active_category, "-1"))
                self.update_entry_text(self.entry_ylim_max, self.category_ylim_maxs.get(self.current_active_category, "1"))

            return True
        except Exception as e:
            print(f"Silent AutoLoad Warning: Config file broken or format mismatch. {str(e)}")
            return False

    def import_file(self):
        if self.file_path:
            self.auto_save_settings()

        file_types = [("Data Files", "*.txt *.csv *.tsv *.data"), ("All Files", "*.*")]
        selected_path = filedialog.askopenfilename(title="Select Data File", filetypes=file_types)
        if not selected_path: return

        self.file_path = Path(selected_path)
        self.lbl_file_path.config(text=f"File: {self.file_path.name}", foreground="black")

        try:
            with open(self.file_path, 'r', encoding='utf-8') as f:
                first_line = f.readline()
                delimiter = '\t' if '\t' in first_line else ','

            self.df = pd.read_csv(self.file_path, delimiter=delimiter)
            columns_list = list(self.df.columns)
            self.combo_x_axis['values'] = ["(Row Index)"] + columns_list
            
            time_col = next((col for col in columns_list if 'time' in col.lower()), None)
            if time_col:
                self.combo_x_axis.set(time_col)
                self.update_entry_text(self.entry_x_label, "Time (s)")
            else:
                self.combo_x_axis.set("(Row Index)")
                self.update_entry_text(self.entry_x_label, "Frames")
                
            self.update_entry_text(self.entry_title, "") 

            self.custom_legends.clear()  
            self.curve_linestyles.clear()
            self.curve_markers.clear()
            self.curve_linewidths.clear()  
            self.curve_colors.clear()
            
            # 清空并初始化各 category 的 Y 独立属性
            self.category_ylabels.clear()
            self.category_custom_ylims.clear()
            self.category_ylim_mins.clear()
            self.category_ylim_maxs.clear()
            self.current_active_category = None
            
            self.parse_data_structure()
            
            # 自动基于导入数据区间，初始化每个 category 的专属独立配置与自适应 Y 值域范围
            for cat_name, sub_dict in self.motion_data.items():
                self.category_ylabels[cat_name] = "Amplitude"
                self.category_custom_ylims[cat_name] = False
                
                # 抓取当前大类下的所有子曲线数据，计算一个高合理的初始范围限界
                y_accumulated = []
                for y_arr in sub_dict.values():
                    y_accumulated.extend(y_arr)
                if y_accumulated:
                    ymin, ymax = min(y_accumulated), max(y_accumulated)
                    pad = (ymax - ymin) * 0.1 if ymax != ymin else 1.0
                    self.category_ylim_mins[cat_name] = str(round(ymin - pad, 2))
                    self.category_ylim_maxs[cat_name] = str(round(ymax + pad, 2))
                else:
                    self.category_ylim_mins[cat_name] = "-1"
                    self.category_ylim_maxs[cat_name] = "1"

            self.update_entry_text(self.entry_y_label, "Amplitude")

            loaded = self.auto_load_settings()
            
            if not loaded and self.time_data is not None and len(self.time_data) > 0:
                self.update_entry_text(self.entry_xlim_min, str(round(self.time_data.min(), 2)))
                self.update_entry_text(self.entry_xlim_max, str(round(self.time_data.max(), 2)))

            self.on_tree_select(None)
            
            if loaded:
                self.root.title(f"IEEE Style Plotter - [Profile Loaded: {self.file_path.stem}.json]")

        except Exception as e:
            messagebox.showerror("Parsing Error", f"Failed to parse data!\nDetails: {str(e)}")

    def parse_data_structure(self):
        if self.df is None: return
        selected_x_col = self.combo_x_axis.get()
        
        if selected_x_col == "(Row Index)":
            self.time_data = self.df.index.to_numpy()
        else:
            self.time_data = self.df[selected_x_col].to_numpy()

        self.motion_data = {}
        for col in self.df.columns:
            if col == selected_x_col: continue
            cat_name = col.split('_')[0]
            label = col.split('_')[1] if '_' in col else "Value"
            
            if cat_name not in self.motion_data:
                self.motion_data[cat_name] = {}
            self.motion_data[cat_name][label] = self.df[col].to_numpy()

        for item in self.tree.get_children(): self.tree.delete(item)
        for cat_name, sub_dict in self.motion_data.items():
            parent_node = self.tree.insert("", "end", text=cat_name, open=False)
            for label in sub_dict.keys():
                self.tree.insert(parent_node, "end", text=label, values=(cat_name, label))

    def on_x_axis_changed(self, event):
        if self.df is not None:
            selected_x_col = self.combo_x_axis.get()
            self.update_entry_text(self.entry_x_label, "Index" if selected_x_col == "(Row Index)" else selected_x_col)
            self.parse_data_structure()
            if self.time_data is not None and len(self.time_data) > 0:
                self.update_entry_text(self.entry_xlim_min, str(round(self.time_data.min(), 2)))
                self.update_entry_text(self.entry_xlim_max, str(round(self.time_data.max(), 2)))
            self.draw_curves()

    def clear_selection(self):
        self.tree.selection_remove(self.tree.selection())
        self.current_curves = []
        self.current_active_category = None
        self.reset_plot_style()
        self.update_style_editor()
        self.canvas.draw()
        self.auto_save_settings() 

    def on_tree_select(self, event):
        if self.df is None or not self.motion_data: return
        selected_items = self.tree.selection()
        
        # ==================== 核心切换拦截逻辑 ====================
        if selected_items:
            # 1. 缓存前一个被选中编辑的 Category 参数值，防止修改丢失
            if self.current_active_category:
                self.category_ylabels[self.current_active_category] = self.entry_y_label.get()
                self.category_custom_ylims[self.current_active_category] = self.var_custom_ylim.get()
                self.category_ylim_mins[self.current_active_category] = self.entry_ylim_min.get()
                self.category_ylim_maxs[self.current_active_category] = self.entry_ylim_max.get()

            # 2. 识别并装载当前最新选中的对应 Category 的属性
            first_item = selected_items[0]
            values = self.tree.item(first_item, "values")
            current_cat = values[0] if values else self.tree.item(first_item, "text")
            self.current_active_category = current_cat
            
            # 3. 将其独立属性安全回填刷新至右侧公共面板输入组件中
            if current_cat in self.category_ylabels:
                self.update_entry_text(self.entry_y_label, self.category_ylabels[current_cat])
            if current_cat in self.category_custom_ylims:
                self.var_custom_ylim.set(self.category_custom_ylims[current_cat])
            if current_cat in self.category_ylim_mins:
                self.update_entry_text(self.entry_ylim_min, self.category_ylim_mins[current_cat])
            if current_cat in self.category_ylim_maxs:
                self.update_entry_text(self.entry_ylim_max, self.category_ylim_maxs[current_cat])

        self.current_curves = []
        for item in selected_items:
            values = self.tree.item(item, "values")
            if values:  
                cat_name, label = values[0], values[1]
                if cat_name in self.motion_data and label in self.motion_data[cat_name]:
                    self.current_curves.append((cat_name, label, self.motion_data[cat_name][label]))
            else:  
                parent_text = self.tree.item(item, "text")
                if parent_text in self.motion_data:
                    for label, y_data in self.motion_data[parent_text].items():
                        self.current_curves.append((parent_text, label, y_data))

        self.update_style_editor()
        self.draw_curves()

    def update_style_editor(self):
        for widget in self.legend_editor_container.winfo_children(): widget.destroy()

        if not self.current_curves:
            ttk.Label(self.legend_editor_container, text="No active curves selected", foreground="gray").pack(padx=10, pady=10)
            return

        header = ttk.Frame(self.legend_editor_container)
        header.pack(fill=tk.X, pady=int(3*self.scale_factor))
        header.columnconfigure(0, weight=3) 
        header.columnconfigure(1, weight=3) 
        header.columnconfigure(2, weight=1) 
        header.columnconfigure(3, weight=1) 
        header.columnconfigure(4, weight=1) 
        header.columnconfigure(5, weight=1) 

        ttk.Label(header, text="Source Column", font=(self.font_family, self.font_size_small, "bold")).grid(row=0, column=0, sticky="w", padx=2)
        ttk.Label(header, text="Legend (LaTeX)", font=(self.font_family, self.font_size_small, "bold")).grid(row=0, column=1, sticky="w", padx=2)
        ttk.Label(header, text="Line", font=(self.font_family, self.font_size_small, "bold")).grid(row=0, column=2, sticky="w", padx=2)
        ttk.Label(header, text="Marker", font=(self.font_family, self.font_size_small, "bold")).grid(row=0, column=3, sticky="w", padx=2)
        ttk.Label(header, text="Width", font=(self.font_family, self.font_size_small, "bold")).grid(row=0, column=4, sticky="w", padx=2)
        ttk.Label(header, text="Color", font=(self.font_family, self.font_size_small, "bold")).grid(row=0, column=5, sticky="w", padx=2) 

        linestyles = {"Solid": "-", "Dashed": "--", "Dotted": ":", "Dash-Dot": "-."}
        markers = {"None": "", "Circle": "o", "Square": "s", "Triangle": "^", "Diamond": "d", "Star": "*"}
        linewidths = ["0.5", "1.0", "1.5", "2.0", "2.5", "3.0", "4.0"]
        colors_dict = {"Auto": "auto", "Red": "#ff0000", "Green": "#00ff00", "Blue": "#0000ff", "Black": "#000000", "Yellow": "#ffff00", "RoyalBlue": "#1f77b4", "SafetyOrange": "#ff7f0e", "EmeraldGreen": "#2ca02c", "AcademicRed": "#d62728", "Purple": "#9467bd", "Sienna": "#8c564b", "Orchid": "#e377c2", "DarkGray": "#7f7f7f"}

        for idx, (cat, label, _) in enumerate(self.current_curves):
            row = ttk.Frame(self.legend_editor_container)
            row.pack(fill=tk.X, pady=int(4*self.scale_factor))
            row.columnconfigure(0, weight=3)
            row.columnconfigure(1, weight=3)
            row.columnconfigure(2, weight=1)
            row.columnconfigure(3, weight=1)
            row.columnconfigure(4, weight=1)
            row.columnconfigure(5, weight=1)

            key = f"{cat}_{label}"
            default_txt = f"{cat} [{label}]" if label != "Value" else cat
            
            if key not in self.custom_legends: self.custom_legends[key] = default_txt
            if key not in self.curve_linestyles: self.curve_linestyles[key] = "-"
            if key not in self.curve_markers: self.curve_markers[key] = ""
            if key not in self.curve_linewidths: self.curve_linewidths[key] = "1.5"  
            if key not in self.curve_colors: self.curve_colors[key] = "auto" 

            ttk.Label(row, text=default_txt, anchor="w", font=(self.font_family, self.font_size_small)).grid(row=0, column=0, sticky="w", padx=2)

            ent = ttk.Entry(row, font=(self.font_family, self.font_size_small))
            ent.insert(0, self.custom_legends[key])
            ent.grid(row=0, column=1, sticky="ew", padx=2)
            ent.bind("<Return>", lambda e, c=cat, l=label, w=ent: self.update_and_redraw_legend(c, l, w.get()))

            cb_line = ttk.Combobox(row, values=list(linestyles.keys()), state="readonly", width=int(6*self.scale_factor), font=(self.font_family, self.font_size_small))
            current_line_name = [n for n, s in linestyles.items() if s == self.curve_linestyles[key]][0]
            cb_line.set(current_line_name)
            cb_line.grid(row=0, column=2, sticky="ew", padx=2)
            cb_line.bind("<<ComboboxSelected>>", lambda e, k=key, cb=cb_line: self.update_and_redraw(k, 'line', linestyles[cb.get()]))

            cb_mark = ttk.Combobox(row, values=list(markers.keys()), state="readonly", width=int(6*self.scale_factor), font=(self.font_family, self.font_size_small))
            current_mark_name = [n for n, m in markers.items() if m == self.curve_markers[key]][0]
            cb_mark.set(current_mark_name)
            cb_mark.grid(row=0, column=3, sticky="ew", padx=2)
            cb_mark.bind("<<ComboboxSelected>>", lambda e, k=key, cb=cb_mark: self.update_and_redraw(k, 'marker', markers[cb.get()]))

            cb_width = ttk.Combobox(row, values=linewidths, state="readonly", width=int(4*self.scale_factor), font=(self.font_family, self.font_size_small))
            cb_width.set(self.curve_linewidths[key])
            cb_width.grid(row=0, column=4, sticky="ew", padx=2)
            cb_width.bind("<<ComboboxSelected>>", lambda e, k=key, cb=cb_width: self.update_and_redraw(k, 'width', cb.get()))

            cb_color = ttk.Combobox(row, values=list(colors_dict.keys()), state="readonly", width=int(8*self.scale_factor), font=(self.font_family, self.font_size_small))
            current_color_name = [n for n, c in colors_dict.items() if c == self.curve_colors[key]]
            cb_color.set(current_color_name[0] if current_color_name else "Auto")
            cb_color.grid(row=0, column=5, sticky="ew", padx=2)
            cb_color.bind("<<ComboboxSelected>>", lambda e, k=key, cb=cb_color: self.update_and_redraw(k, 'color', colors_dict[cb.get()]))

    def apply_typography_and_theme(self):
        selected_font = self.combo_font_family.get()
        selected_math_set = self.combo_math_font.get()

        if selected_font == "Times New Roman":
            matplotlib.rcParams['font.family'] = 'serif'
            matplotlib.rcParams['font.serif'] = ['Times New Roman', 'Times', 'DejaVu Serif']
        elif selected_font == "Arial":
            matplotlib.rcParams['font.family'] = 'sans-serif'
            matplotlib.rcParams['font.sans-serif'] = ['Arial', 'Helvetica', 'DejaVu Sans']
        elif selected_font == "DejaVu Serif":
            matplotlib.rcParams['font.family'] = 'serif'
            matplotlib.rcParams['font.serif'] = ['DejaVu Serif']
        else: 
            matplotlib.rcParams['font.family'] = 'monospace'
            matplotlib.rcParams['font.monospace'] = ['Courier New']

        matplotlib.rcParams['mathtext.fontset'] = selected_math_set

    def draw_curves(self):
        self.ax.clear()
        self.apply_typography_and_theme()
        
        # 绘图前，先将右侧面板中最新的 Y 配置数据存回当前活跃的 Category
        if self.current_active_category:
            self.category_ylabels[self.current_active_category] = self.entry_y_label.get()
            self.category_custom_ylims[self.current_active_category] = self.var_custom_ylim.get()
            self.category_ylim_mins[self.current_active_category] = self.entry_ylim_min.get()
            self.category_ylim_maxs[self.current_active_category] = self.entry_ylim_max.get()

        if not self.current_curves:
            self.reset_plot_style()
            self.canvas.draw()
            return
        #self.fig, self.ax = plt.subplots(figsize=(8, 5.5))
        try:
            sz_label = int(self.combo_label_size.get())
            sz_legend = int(self.combo_legend_size.get())
            sz_title = int(self.combo_title_size.get())
        except ValueError:
            sz_label, sz_legend, sz_title = 10, 9, 12

        # 渲染多条选中的数据线
        for cat_name, label, y_data in self.current_curves:
            key = f"{cat_name}_{label}"
            display_legend = self.custom_legends.get(key, f"{cat_name} [{label}]")
            l_style = self.curve_linestyles.get(key, "-")
            m_style = self.curve_markers.get(key, "")
            c_style = self.curve_colors.get(key, "auto") 
            
            try:
                line_w = float(self.curve_linewidths.get(key, "1.5"))
            except ValueError:
                line_w = 1.5
            
            mevery = max(1, len(y_data) // 35) if m_style else None

            plot_kwargs = {
                "label": display_legend,
                "linestyle": l_style,
                "marker": m_style,
                "markevery": mevery,
                "markersize": 6,
                "linewidth": line_w
            }
            if c_style != "auto":
                plot_kwargs["color"] = c_style

            self.ax.plot(self.time_data, y_data, **plot_kwargs)  
        
        # 标题处理
        user_title = self.entry_title.get()
        if user_title: 
            self.ax.set_title(user_title, fontsize=sz_title, pad=10)
        
        # X 轴标签
        self.ax.set_xlabel(self.entry_x_label.get(), fontsize=sz_label)
        
        # ==================== 智能拼合或指定各独占的 Y-Label ====================
        active_cats = list(set([cat for cat, _, _ in self.current_curves]))
        if len(active_cats) == 1:
            y_label_text = self.category_ylabels.get(active_cats[0], "Amplitude")
        elif len(active_cats) > 1:
            # 混合绘制不同的大类时，自动用 / 隔开显示各自的 Y Label
            y_label_text = " / ".join([self.category_ylabels.get(c, "Amplitude") for c in active_cats])
        else:
            y_label_text = self.entry_y_label.get()

        self.ax.set_ylabel(y_label_text, fontsize=sz_label)
        self.ax.tick_params(axis='both', which='major', labelsize=sz_label - 1)

        # IEEE 经典的四周封闭边框线
        self.ax.spines['top'].set_visible(True)
        self.ax.spines['right'].set_visible(True)
        self.ax.spines['top'].set_color('#000000')
        self.ax.spines['right'].set_color('#000000')
        
        self.ax.legend(loc='upper right', fontsize=sz_legend)

        if self.var_enable_grid.get():
            self.ax.grid(True, linestyle='--', alpha=0.4, color='#d3d3d3')
        else:
            self.ax.grid(False)

        # X轴范围限定
        if self.var_custom_xlim.get():
            try:
                self.ax.set_xlim(float(self.entry_xlim_min.get()), float(self.entry_xlim_max.get()))
            except ValueError:
                if self.time_data is not None and len(self.time_data) > 0:
                    self.ax.set_xlim(self.time_data.min(), self.time_data.max())
        else:
            if self.time_data is not None and len(self.time_data) > 0:
                self.ax.set_xlim(self.time_data.min(), self.time_data.max())

        # ==================== 基于独占 Category 范围限定解析 ====================
        # 若只包含单一大类或有明确激活类且该类开启了 Y-Lim 限制，应用其专属限值
        if len(active_cats) == 1 and self.category_custom_ylims.get(active_cats[0], False):
            try:
                ymin = float(self.category_ylim_mins.get(active_cats[0], "-1"))
                ymax = float(self.category_ylim_maxs.get(active_cats[0], "1"))
                self.ax.set_ylim(ymin, ymax)
            except ValueError:
                pass
        elif self.var_custom_ylim.get(): # 多品类混画时默认兼容当前面板上显示的值
            try:
                self.ax.set_ylim(float(self.entry_ylim_min.get()), float(self.entry_ylim_max.get()))
            except ValueError:
                pass

        # 子图画中画 (Inset Zoom)
        if self.var_enable_zoom.get() and len(self.current_curves) > 0:
            try:
                z_start = float(self.entry_zoom_start.get())
                z_end = float(self.entry_zoom_end.get())
                
                ax_inset = self.ax.inset_axes([0.52, 0.12, 0.42, 0.38]) 
                
                for cat_name, label, y_data in self.current_curves:
                    key = f"{cat_name}_{label}"
                    l_style = self.curve_linestyles.get(key, "-")
                    m_style = self.curve_markers.get(key, "")
                    c_style = self.curve_colors.get(key, "auto")
                    try:
                        line_w = float(self.curve_linewidths.get(key, "1.5"))
                    except ValueError:
                        line_w = 1.5
                    mevery = max(1, len(y_data) // 20) if m_style else None
                    
                    inset_kwargs = {"linestyle": l_style, "marker": m_style, "markevery": mevery, "linewidth": line_w}
                    if c_style != "auto": inset_kwargs["color"] = c_style
                    
                    ax_inset.plot(self.time_data, y_data, **inset_kwargs)
                
                ax_inset.set_xlim(z_start, z_end)
                mask = (self.time_data >= z_start) & (self.time_data <= z_end)
                if np.any(mask):
                    y_mins = [np.min(y[mask]) for _, _, y in self.current_curves]
                    y_maxs = [np.max(y[mask]) for _, _, y in self.current_curves]
                    padding = (max(y_maxs) - min(y_mins)) * 0.1 or 1.0
                    ax_inset.set_ylim(min(y_mins) - padding, max(y_maxs) + padding)
                
                if self.var_enable_grid.get():
                    ax_inset.grid(True, linestyle=':', alpha=0.5, color='#d3d3d3')
                else:
                    ax_inset.grid(False)
                    
                ax_inset.set_title("Detailed Zoom", fontsize=sz_legend)
                ax_inset.tick_params(axis='both', which='major', labelsize=max(6, sz_label - 3))
                self.ax.indicate_inset_zoom(ax_inset, edgecolor="black", alpha=0.3)
            except ValueError:
                pass  

        self.fig.tight_layout()
        self.canvas.draw()
        
        self.auto_save_settings()

    def reset_plot_style(self):
        self.ax.text(0.5, 0.5, "Please parse data file and select metrics from list", ha='center', va='center', fontsize=11, color='gray')
        self.ax.set_xlabel(self.entry_x_label.get() if self.entry_x_label.get() else "X Variable", fontsize=10)
        self.ax.set_ylabel(self.entry_y_label.get() if self.entry_y_label.get() else "Y Amplitude", fontsize=10)
        
        user_title = self.entry_title.get() if hasattr(self, 'entry_title') else ""
        if user_title:
            self.ax.set_title(user_title, fontsize=11)
            
        if self.var_enable_grid.get():
            self.ax.grid(True, linestyle='--', alpha=0.3)
        else:
            self.ax.grid(False)
            
        self.fig.tight_layout()

    def export_figure(self, mode="png"):
        if not self.current_curves:
            messagebox.showwarning("Export Void", "There are no curves plotted on canvas to export!")
            return
        #for cat_name, sub_dict in self.current_active_category:
        try:
            sz_label = int(self.combo_label_size.get())
            sz_legend = int(self.combo_legend_size.get())
            sz_title = int(self.combo_title_size.get())
        except ValueError:
            sz_label, sz_legend, sz_title = 10, 9, 12
        tmp_fig, tmp_ax = plt.subplots(figsize=(8, 5.5))
        cat_name = self.current_active_category
        sub_dict = self.motion_data[cat_name]
        for label, y_data in sub_dict.items():
            # 显式建立后台画布（并在绘制下一张图前执行完全清空，避免图像堆叠）
            key = f"{cat_name}_{label}"
            
            display_legend = self.custom_legends.get(key, f"{cat_name} [{label}]")
            l_style = self.curve_linestyles.get(key, "-")
            m_style = self.curve_markers.get(key, "")
            c_style = self.curve_colors.get(key, "auto")
            try:
                line_w = float(self.curve_linewidths.get(key, "1.5"))
            except ValueError:
                line_w = 1.5
            
            mevery = max(1, len(y_data) // 35) if m_style else None
            plot_kwargs = {"label": display_legend, "linestyle": l_style, "marker": m_style, "markevery": mevery, "markersize": 6, "linewidth": line_w}
            if c_style != "auto": plot_kwargs["color"] = c_style

            tmp_ax.plot(self.time_data, y_data, **plot_kwargs)

            # 设置公用 Title 与 X 轴
            user_title = self.entry_title.get()
            #if user_title:
            tmp_ax.set_title(user_title, fontsize=sz_title, pad=10)
            #else:
            #    tmp_ax.set_title(f"Curve of {cat_name} ({label})", fontsize=sz_title, pad=10)
                
            tmp_ax.set_xlabel(self.entry_x_label.get(), fontsize=sz_label)
            
            # ==================== 应用当前大类专属的独立 Y 轴 Label 与限值 ====================
            tmp_ax.set_ylabel(self.category_ylabels.get(cat_name, "Amplitude"), fontsize=sz_label)
            tmp_ax.tick_params(axis='both', which='major', labelsize=sz_label - 1)
            
            tmp_ax.spines['top'].set_visible(True)
            tmp_ax.spines['right'].set_visible(True)
            tmp_ax.spines['top'].set_color('#000000')
            tmp_ax.spines['right'].set_color('#000000')
            tmp_ax.legend(loc='upper right', fontsize=sz_legend)

            if self.var_enable_grid.get():
                tmp_ax.grid(True, linestyle='--', alpha=0.4, color='#d3d3d3')

            if self.var_custom_xlim.get():
                try: tmp_ax.set_xlim(float(self.entry_xlim_min.get()), float(self.entry_xlim_max.get()))
                except ValueError: tmp_ax.set_xlim(self.time_data.min(), self.time_data.max())
            else:
                tmp_ax.set_xlim(self.time_data.min(), self.time_data.max())

            # 使用当前 Category 专有的 Y-Lim 设定
            if self.category_custom_ylims.get(cat_name, False):
                try:
                    ymin = float(self.category_ylim_mins.get(cat_name, "-1"))
                    ymax = float(self.category_ylim_maxs.get(cat_name, "1"))
                    tmp_ax.set_ylim(ymin, ymax)
                except ValueError:
                    pass

        tmp_fig.tight_layout()
            

        if mode == "png":
            save_path = filedialog.asksaveasfilename(defaultextension=".png", filetypes=[("Ultra HQ Images", "*.png")])
            if save_path:
                
                tmp_fig.savefig(save_path, dpi=300, bbox_inches='tight')
                messagebox.showinfo("Export Success", f"Successfully exported 300 DPI high-res curve to:\n{save_path}")
        elif mode == "pdf":
            save_path = filedialog.asksaveasfilename(defaultextension=".pdf", filetypes=[("Vector Graphics Files", "*.pdf")])
            if save_path:
               
                tmp_fig.savefig(save_path, format="pdf", bbox_inches='tight')
                messagebox.showinfo("Export Success", f"Successfully exported publication-ready lossless vector map to:\n{save_path}")

    def batch_save_all_curves(self):
        """ 一键后台遍历并导出数据中所有的单独 curve 图像（完美继承各自 Category 独占的 Y-Label、范围值域配置） """
        if self.df is None or not self.motion_data:
            messagebox.showwarning("Export Void", "No source data imported to process batch generation!")
            return
            
        format_dialog = tk.Toplevel(self.root)
        format_dialog.title("Select Format")
        format_dialog.geometry(f"{int(300*self.scale_factor)}x{int(150*self.scale_factor)}")
        format_dialog.resizable(False, False)
        format_dialog.transient(self.root)
        format_dialog.grab_set()

        ttk.Label(format_dialog, text="Select Export File Format:", font=(self.font_family, self.font_size_normal)).pack(pady=15)
        
        selected_fmt = tk.StringVar(value="png")
        rb_png = ttk.Radiobutton(format_dialog, text="Ultra HQ PNG (300 DPI)", variable=selected_fmt, value="png")
        rb_png.pack(anchor=tk.W, padx=40)
        rb_pdf = ttk.Radiobutton(format_dialog, text="Vector PDF (Lossless)", variable=selected_fmt, value="pdf")
        rb_pdf.pack(anchor=tk.W, padx=40)

        choice = {"confirmed": False}
        def on_confirm():
            choice["confirmed"] = True
            format_dialog.destroy()

        ttk.Button(format_dialog, text="Next", command=on_confirm).pack(pady=10)
        self.root.wait_window(format_dialog)

        if not choice["confirmed"]: return

        target_dir = filedialog.askdirectory(title="Select Output Directory for All Curves")
        if not target_dir: return

        try:
            sz_label = int(self.combo_label_size.get())
            sz_legend = int(self.combo_legend_size.get())
            sz_title = int(self.combo_title_size.get())
        except ValueError:
            sz_label, sz_legend, sz_title = 10, 9, 12

        export_mode = selected_fmt.get()
        self.apply_typography_and_theme()
        
        saved_count = 0
        total_curves = sum(len(sub_dict) for sub_dict in self.motion_data.values())

        # 遍历所有数据大类和小项
        for cat_name, sub_dict in self.motion_data.items():
            tmp_fig, tmp_ax = plt.subplots(figsize=(8, 5.5))
            for label, y_data in sub_dict.items():
                # 显式建立后台画布（并在绘制下一张图前执行完全清空，避免图像堆叠）
                key = f"{cat_name}_{label}"
                
                display_legend = self.custom_legends.get(key, f"{cat_name} [{label}]")
                l_style = self.curve_linestyles.get(key, "-")
                m_style = self.curve_markers.get(key, "")
                c_style = self.curve_colors.get(key, "auto")
                try:
                    line_w = float(self.curve_linewidths.get(key, "1.5"))
                except ValueError:
                    line_w = 1.5
                
                mevery = max(1, len(y_data) // 35) if m_style else None
                plot_kwargs = {"label": display_legend, "linestyle": l_style, "marker": m_style, "markevery": mevery, "markersize": 6, "linewidth": line_w}
                if c_style != "auto": plot_kwargs["color"] = c_style

                tmp_ax.plot(self.time_data, y_data, **plot_kwargs)

                # 设置公用 Title 与 X 轴
                user_title = self.entry_title.get()
                #if user_title:
                tmp_ax.set_title(user_title, fontsize=sz_title, pad=10)
                #else:
                #    tmp_ax.set_title(f"Curve of {cat_name} ({label})", fontsize=sz_title, pad=10)
                    
                tmp_ax.set_xlabel(self.entry_x_label.get(), fontsize=sz_label)
                
                # ==================== 应用当前大类专属的独立 Y 轴 Label 与限值 ====================
                tmp_ax.set_ylabel(self.category_ylabels.get(cat_name, "Amplitude"), fontsize=sz_label)
                tmp_ax.tick_params(axis='both', which='major', labelsize=sz_label - 1)
                
                tmp_ax.spines['top'].set_visible(True)
                tmp_ax.spines['right'].set_visible(True)
                tmp_ax.spines['top'].set_color('#000000')
                tmp_ax.spines['right'].set_color('#000000')
                tmp_ax.legend(loc='upper right', fontsize=sz_legend)

                if self.var_enable_grid.get():
                    tmp_ax.grid(True, linestyle='--', alpha=0.4, color='#d3d3d3')

                if self.var_custom_xlim.get():
                    try: tmp_ax.set_xlim(float(self.entry_xlim_min.get()), float(self.entry_xlim_max.get()))
                    except ValueError: tmp_ax.set_xlim(self.time_data.min(), self.time_data.max())
                else:
                    tmp_ax.set_xlim(self.time_data.min(), self.time_data.max())

                # 使用当前 Category 专有的 Y-Lim 设定
                if self.category_custom_ylims.get(cat_name, False):
                    try:
                        ymin = float(self.category_ylim_mins.get(cat_name, "-1"))
                        ymax = float(self.category_ylim_maxs.get(cat_name, "1"))
                        tmp_ax.set_ylim(ymin, ymax)
                    except ValueError:
                        pass

            tmp_fig.tight_layout()
            
            # 合规化过滤输出名
            safe_filename = f"{cat_name}.{export_mode}".replace("/", "_").replace("\\", "_")
            output_path = Path(target_dir) / safe_filename
            
            if export_mode == "png":
                tmp_fig.savefig(output_path, dpi=300, bbox_inches='tight')
            else:
                tmp_fig.savefig(output_path, format="pdf", bbox_inches='tight')
            
            saved_count += 1
            plt.close(tmp_fig) # 彻底关闭和销毁当前子画布，防止内存泄露

        if messagebox.askyesno("Batch Export Done", f"Successfully generated {saved_count}/{total_curves} files to:\n{target_dir}\n\nWould you like to open the directory now?"):
            os.startfile(target_dir) if sys.platform.startswith("win") else os.system(f'open "{target_dir}"')

    def update_entry_text(self, entry, text):
        entry.delete(0, tk.END)
        entry.insert(0, text)

    def on_closing(self):
        if self.file_path:
            self.auto_save_settings()
        plt.close(self.fig)
        self.root.destroy()

    def update_and_redraw_legend(self, current_cat, current_label, new_value):
        standard_suffix = f"[{current_label}]"
        space_suffix = f" [{current_label}]"
        
        if new_value.endswith(standard_suffix):
            new_prefix = new_value[:-len(standard_suffix)]
        elif new_value.endswith(space_suffix):
            new_prefix = new_value[:-len(space_suffix)]
        else:
            new_prefix = new_value

        for cat, label, _ in self.current_curves:
            if cat == current_cat:
                loop_key = f"{cat}_{label}"
                if label == current_label:
                    self.custom_legends[loop_key] = new_value
                else:
                    old_legend = self.custom_legends.get(loop_key, f"{cat}[{label}]")
                    if f"[{label}]" in old_legend:
                        self.custom_legends[loop_key] = f"{new_prefix}[{label}]"
                    elif f" [{label}]" in old_legend:
                        self.custom_legends[loop_key] = f"{new_prefix} [{label}]"
                    else:
                        suffix = f"[{label}]" if label != "Value" else ""
                        self.custom_legends[loop_key] = f"{new_prefix}{suffix}"

        self.update_style_editor()
        self.draw_curves()

    def update_and_redraw(self, key, config_type, value):
        if config_type == 'line': self.curve_linestyles[key] = value
        elif config_type == 'marker': self.curve_markers[key] = value
        elif config_type == 'width': self.curve_linewidths[key] = value  
        elif config_type == 'color': self.curve_colors[key] = value  
        self.draw_curves()

if __name__ == "__main__":
    root = tk.Tk()
    app = IEEEAdvancedPlotter(root)
    root.mainloop()