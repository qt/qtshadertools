#!/usr/bin/env python3
# Copyright (C) 2026 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR GPL-3.0-only WITH Qt-GPL-exception-1.0

"""Interactive inspector for Qt shader pack (.qsb) files.

Shows everything 'qsb -d' dumps, but navigable: an overview, a tree of the
reflection info, and a per-shader view with the payload rendered as highlighted
source, as a hex dump, or - for the bytecode ones - through an external tool, as
disassembly or as the Vulkan GLSL a SPIR-V module decompiles to. Parsing is done
by qsb_reader (pure Python, no Qt).

    python qsb_inspect.py path/to/shader.qsb
"""

from __future__ import annotations

import argparse
import os
import sys

from rich.markup import escape
from rich.syntax import Syntax
from rich.text import Text

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, ScrollableContainer, VerticalScroll
from textual.css.query import NoMatches
from textual.screen import Screen
from textual.widgets import (
    DataTable, DirectoryTree, Footer, Header, Label, ListItem, ListView, Static,
    TabbedContent, TabPane, Tabs, Tree,
)

import qsb_disasm
import qsb_gerrit
import qsb_git
import qsb_reader
from qsb_reader import (
    QsbFile, ShaderKey, QsbError, TESS_MODES, TESS_PARTITIONINGS, TESS_WINDINGS,
    EXTRA_BUFFER_BINDINGS, BINARY_SOURCES, hexdump, image_flags_str,
    image_format_str, qualifier_flags_str, spirv_header_info,
    variable_type_display,
)

# pygments lexer per QShader::Source, for payloads that are text
LEXERS = {1: "glsl", 2: "hlsl", 4: "cpp", 7: "wgsl"}

HEXDUMP_LIMIT = 64 * 1024

# columns per Left/Right press in the payload pane
HSCROLL_STEP = 8


def _enum_name(table, value: int) -> str:
    return table[value] if 0 <= value < len(table) else str(value)


def _decorations(v: qsb_reader.InOutVariable) -> str:
    """The decorations qsb -d would show for an in/out variable, as markup."""
    bits = []
    if v.location >= 0:
        bits.append(f"location [b]{v.location}[/]")
    if v.binding >= 0:
        bits.append(f"binding [b]{v.binding}[/]")
    if v.descriptor_set >= 0:
        bits.append(f"set [b]{v.descriptor_set}[/]")
    if v.array_dims:
        bits.append("array " + "".join(f"[{d}]" for d in v.array_dims).replace("[", r"\["))
    if v.per_patch:
        bits.append("[i]perPatch[/]")
    if v.image_format:
        bits.append(f"format [b]{image_format_str(v.image_format) or v.image_format}[/]")
    if v.image_flags:
        bits.append(f"flags [b]{image_flags_str(v.image_flags)}[/]")
    return "  ".join(bits)


class ReflectionTree(Tree):
    """The QShaderDescription, as a navigable tree."""

    def __init__(self, qsb: QsbFile):
        super().__init__("Reflection info")
        self.qsb = qsb
        self.show_root = False
        self.guide_depth = 3

    def on_mount(self) -> None:
        self.build()

    def build(self) -> None:
        d = self.qsb.description
        root = self.root
        root.remove_children()

        if d.is_empty():
            root.add_leaf("[dim]no reflection info in this shader pack[/]")
            root.expand_all()
            return

        if d.inputs:
            self._add_inout_section(root, "Inputs", d.inputs)
        if d.outputs:
            self._add_inout_section(root, "Outputs", d.outputs)

        if d.uniform_blocks:
            sec = root.add(f"Uniform blocks [dim]({len(d.uniform_blocks)})[/]", expand=True)
            for b in d.uniform_blocks:
                head = f"[b]{escape(b.block_name)}[/]"
                if b.struct_name:
                    head += f" [dim]struct[/] {escape(b.struct_name)}"
                head += f"  size [b]{b.size}[/]"
                if b.binding >= 0:
                    head += f"  binding [b]{b.binding}[/]"
                if b.descriptor_set >= 0:
                    head += f"  set [b]{b.descriptor_set}[/]"
                node = sec.add(head, expand=True)
                for m in b.members:
                    self._add_block_member(node, m)

        if d.push_constant_blocks:
            sec = root.add(f"Push constant blocks [dim]({len(d.push_constant_blocks)})[/]",
                           expand=True)
            for b in d.push_constant_blocks:
                node = sec.add(f"[b]{escape(b.name)}[/]  size [b]{b.size}[/]", expand=True)
                for m in b.members:
                    self._add_block_member(node, m)

        if d.storage_blocks:
            sec = root.add(f"Storage blocks [dim]({len(d.storage_blocks)})[/]", expand=True)
            for b in d.storage_blocks:
                head = f"[b]{escape(b.block_name)}[/]"
                if b.instance_name:
                    head += f" [dim]instance[/] {escape(b.instance_name)}"
                head += f"  knownSize [b]{b.known_size}[/]"
                if b.binding >= 0:
                    head += f"  binding [b]{b.binding}[/]"
                if b.descriptor_set >= 0:
                    head += f"  set [b]{b.descriptor_set}[/]"
                if b.runtime_array_stride:
                    head += f"  runtimeArrayStride [b]{b.runtime_array_stride}[/]"
                if b.qualifier_flags:
                    head += f"  [b]{qualifier_flags_str(b.qualifier_flags)}[/]"
                node = sec.add(head, expand=True)
                for m in b.members:
                    self._add_block_member(node, m)

        for title, items in (
            ("Combined image samplers", d.combined_image_samplers),
            ("Storage images", d.storage_images),
            ("Separate images", d.separate_images),
            ("Separate samplers", d.separate_samplers),
        ):
            if items:
                self._add_inout_section(root, title, items)

        for title, items in (("Input builtins", d.in_builtins),
                             ("Output builtins", d.out_builtins)):
            if items:
                sec = root.add(f"{title} [dim]({len(items)})[/]", expand=True)
                for v in items:
                    label = f"[b]{escape(v.display_name)}[/]"
                    if v.var_type:
                        label += f"  [cyan]{variable_type_display(v.var_type)}[/]"
                    if v.array_dims:
                        label += "  array " + "".join(f"[{x}]" for x in v.array_dims).replace("[", r"\[")
                    sec.add_leaf(label)

        if any(d.local_size):
            root.add_leaf("Compute local size  [b]{}, {}, {}[/]".format(*d.local_size))

        if d.tess_out_vert_count or d.tess_mode or d.tess_winding or d.tess_partitioning:
            sec = root.add("Tessellation", expand=True)
            if d.tess_out_vert_count:
                sec.add_leaf(f"output vertex count  [b]{d.tess_out_vert_count}[/]")
            if d.tess_mode:
                sec.add_leaf(f"mode  [b]{_enum_name(TESS_MODES, d.tess_mode)}[/]")
            if d.tess_winding:
                sec.add_leaf(f"winding order  [b]{_enum_name(TESS_WINDINGS, d.tess_winding)}[/]")
            if d.tess_partitioning:
                sec.add_leaf("partitioning  [b]{}[/]".format(
                    _enum_name(TESS_PARTITIONINGS, d.tess_partitioning)))

    def _add_inout_section(self, root, title: str, items) -> None:
        sec = root.add(f"{title} [dim]({len(items)})[/]", expand=True)
        for v in items:
            label = f"[cyan]{variable_type_display(v.type)}[/] [b]{escape(v.name)}[/]"
            deco = _decorations(v)
            if deco:
                label += "  " + deco
            if v.struct_members:
                node = sec.add(label)
                for m in v.struct_members:
                    self._add_block_member(node, m)
            else:
                sec.add_leaf(label)

    def _add_block_member(self, parent, m: qsb_reader.BlockVariable) -> None:
        label = f"[cyan]{variable_type_display(m.type)}[/] [b]{escape(m.name)}[/]"
        label += f"  [dim]offset[/] {m.offset}  [dim]size[/] {m.size}"
        if m.array_dims:
            label += "  array " + "".join(f"[{d}]" for d in m.array_dims).replace("[", r"\[")
        if m.array_stride:
            label += f"  [dim]arrayStride[/] {m.array_stride}"
        if m.matrix_stride:
            label += f"  [dim]matrixStride[/] {m.matrix_stride}"
        if m.matrix_row_major:
            label += "  [i]row-major[/]"
        if m.struct_members:
            node = parent.add(label)
            for sm in m.struct_members:
                self._add_block_member(node, sm)
        else:
            parent.add_leaf(label)


class PickerScreen(Screen):
    """Base for the screens that choose a pack to inspect."""


class InspectScreen(Screen):
    """Everything about one shader pack: overview, reflection, shaders, JSON."""

    # what should own the focus on each tab, so that the arrow keys do something
    # useful the moment the tab is shown
    TAB_FOCUS = {
        "tab-overview": "#shader-table",
        "tab-reflection": "ReflectionTree",
        "tab-shaders": "#shader-list",
        "tab-json": "#json-body",
    }

    BINDINGS = [
        # Tab moves between tabs rather than between widgets. Each tab's main
        # widget is focused automatically, so cycling focus had nothing useful to
        # reach except the tab bar - which was only a stop on the way to pressing
        # Right. priority, so it wins over the default app.focus_next.
        Binding("tab", "cycle_tab(1)", "Next tab", priority=True),
        Binding("shift+tab", "cycle_tab(-1)", "Previous tab", show=False,
                priority=True),
        Binding("escape", "back", "Back"),
        Binding("e", "export", "Export payload"),
        Binding("d", "toggle_disassembly", "Disassembly"),
        Binding("c", "toggle_vulkan_glsl", "Vulkan GLSL"),
        Binding("f", "toggle_full_hexdump", "Full hexdump"),
        # The payload pane is not a focus stop, so it is paged from here instead.
        # priority, because the shader list is itself scrollable and would other-
        # wise swallow these once it has more entries than fit; the list is
        # navigated with the arrow keys anyway. check_action keeps them to the
        # Shaders tab, so the other tabs' panes still page themselves.
        Binding("pagedown", "scroll_payload('page-down')", "Scroll payload",
                priority=True),
        Binding("pageup", "scroll_payload('page-up')", "Scroll payload",
                show=False, priority=True),
        Binding("home", "scroll_payload('home')", "Scroll payload",
                show=False, priority=True),
        Binding("end", "scroll_payload('end')", "Scroll payload",
                show=False, priority=True),
        Binding("left", "scroll_payload('left')", "Scroll payload",
                show=False, priority=True),
        Binding("right", "scroll_payload('right')", "Scroll payload",
                show=False, priority=True),
    ]

    # the key that asks for each of qsb_disasm's modes, and the action behind it
    VIEW_ACTIONS = {
        "toggle_disassembly": qsb_disasm.DISASM,
        "toggle_vulkan_glsl": qsb_disasm.CROSS,
    }
    VIEW_KEYS = {qsb_disasm.DISASM: "d", qsb_disasm.CROSS: "c"}

    # actions that only make sense on the Shaders tab, where a shader is selected
    SHADERS_TAB_ACTIONS = frozenset({"export", "toggle_full_hexdump",
                                     "scroll_payload", *VIEW_ACTIONS})

    def __init__(self, qsb: QsbFile, origin: str | None = None):
        super().__init__()
        self.qsb = qsb
        # short tag for where the pack came from, when it was not just a file on
        # disk - a commit's short SHA
        self._origin = origin
        self._keys: list[ShaderKey] = qsb.keys()
        self._selected: ShaderKey | None = self._keys[0] if self._keys else None
        self._full_hexdump = False
        # which qsb_disasm mode a bytecode payload is shown through, "" for the
        # hex dump; one setting for the whole pack, so that stepping through the
        # shader list stays in whichever view was asked for
        self._view = ""
        # keyed by mode and shader: the tools are only run once per payload, and
        # the not-found message is worth keeping too
        self._tool_cache: dict[tuple[str, ShaderKey], qsb_disasm.ToolOutput] = {}

    def compose(self) -> ComposeResult:
        yield Header()
        with TabbedContent(initial="tab-overview"):
            with TabPane("Overview", id="tab-overview"):
                with VerticalScroll(id="overview"):
                    yield Static(self._overview_text(), id="overview-text")
                    yield DataTable(id="shader-table", cursor_type="row", zebra_stripes=True)
            with TabPane("Reflection", id="tab-reflection"):
                yield ReflectionTree(self.qsb)
            with TabPane("Shaders", id="tab-shaders"):
                with Horizontal(id="shader-pane"):
                    yield ListView(
                        *[ListItem(Label(k.short()), id=f"key-{i}")
                          for i, k in enumerate(self._keys)],
                        id="shader-list",
                    )
                    # scrolls both ways: generated MSL in particular has lines
                    # far wider than any terminal
                    with ScrollableContainer(id="shader-body"):
                        yield Static(id="shader-details")
                        yield Static(id="shader-contents")
            with TabPane("JSON", id="tab-json"):
                with VerticalScroll(id="json-body"):
                    yield Static(
                        Syntax(qsb_reader.description_to_json(self.qsb.description),
                               "json", theme="ansi_dark", word_wrap=False),
                        id="json-text",
                    )
        yield Footer()

    def on_mount(self) -> None:
        name = os.path.basename(self.qsb.path)
        self.app.sub_title = f"{name} @ {self._origin}" if self._origin else name

        table = self.query_one("#shader-table", DataTable)
        table.add_columns("#", "Source", "Version", "Variant", "Entry point", "Size", "Extras")
        for i, k in enumerate(self._keys):
            code = self.qsb.shaders[k]
            extras = []
            if self.qsb.bindings.get(k):
                extras.append("native bindings")
            if self.qsb.combined_image_map.get(k):
                extras.append("sampler map")
            info = self.qsb.native_shader_info.get(k)
            if info and (info.flags or info.extra_buffer_bindings):
                extras.append("shader info")
            table.add_row(str(i), k.source_name, k.version_str or "-", k.variant_name,
                          code.entry_point or "-", f"{len(code.shader)} B",
                          ", ".join(extras) or "-")
        if self._selected is not None:
            self._refresh_shader_view()

        # Keep the focus chain to the widgets worth landing on. The overview pane
        # holds only a caption and the table, and the table scrolls its ancestors
        # as its cursor moves. The payload pane would sit between the shader list
        # and the tab bar, so that Tab out of the list went somewhere unobvious
        # instead of back to the tabs; PageUp/PageDown scroll it instead.
        self.query_one("#overview").can_focus = False
        self.query_one("#shader-body").can_focus = False

        self.call_later(self._focus_active_tab)

    # -- focus -------------------------------------------------------------

    def _focus_active_tab(self) -> None:
        """Hand focus to the current tab's primary widget."""
        selector = self.TAB_FOCUS.get(self.query_one(TabbedContent).active)
        if not selector:
            return
        try:
            self.query_one(selector).focus()
        except NoMatches:
            pass  # tab contents not composed yet

    def _came_from_picker(self) -> bool:
        stack = self.app.screen_stack
        return len(stack) > 1 and isinstance(stack[-2], PickerScreen)

    def check_action(self, action: str, parameters: tuple[object, ...]) -> bool | None:
        # These act on the shader selected in the Shaders tab. Offering them on the
        # other tabs would work on whatever that selection happens to be, which
        # need not be the shader the cursor is on there. None leaves the key
        # visible in the footer but greyed out, rather than hiding it.
        if action in self.SHADERS_TAB_ACTIONS:
            try:
                if self.query_one(TabbedContent).active != "tab-shaders":
                    return None
            except NoMatches:
                return None
            # nothing for a tool to chew on in a text payload, and only SPIR-V can
            # be cross-compiled
            if action in self.VIEW_ACTIONS:
                return True if self._can_view(self.VIEW_ACTIONS[action]) else None
            # while a tool's output is up there is no hex dump to lift the limit on
            if action == "toggle_full_hexdump" and self._showing_tool():
                return None
            return True
        # only offered when a file was picked in the browser rather than named on
        # the command line, since otherwise there is nothing to go back to
        if action == "back":
            return True if self._came_from_picker() else None
        return True

    def on_tabbed_content_tab_activated(self, event: TabbedContent.TabActivated) -> None:
        # Queued rather than applied here: when the message arrives the pane being
        # shown may not be laid out yet, and focus() on a widget that is not
        # focusable yet does nothing. call_later rather than call_after_refresh,
        # so that it does not depend on a frame being drawn.
        self.call_later(self._focus_active_tab)
        self.refresh_bindings()  # export is only offered on the Shaders tab

    # -- overview ----------------------------------------------------------

    def _overview_text(self) -> Text:
        q = self.qsb
        ratio = (q.file_size / q.uncompressed_size * 100) if q.uncompressed_size else 0
        rows = [
            ("File", q.path),
            ("Stage", q.stage_name),
            ("QSB_VERSION", str(q.qsb_version)),
            ("Shaders", str(len(q.shaders))),
            ("On disk", f"{q.file_size} bytes"),
            ("Uncompressed", f"{q.uncompressed_size} bytes ({ratio:.0f}% compressed)"),
        ]
        if self._origin:
            # this pack was read out of a commit, not off disk; say so, because
            # the working tree copy may well differ
            rows.insert(1, ("From", self._origin))
        if q.trailing_bytes:
            rows.append(("Unparsed tail", f"{q.trailing_bytes} bytes - format mismatch?"))
        t = Text()
        for i, (k, v) in enumerate(rows):
            if i:
                t.append("\n")
            t.append(f"{k:<14}", style="bold")
            t.append(v, style="red" if k == "Unparsed tail" else "")
        return t

    # -- shader view -------------------------------------------------------

    def on_list_view_highlighted(self, event: ListView.Highlighted) -> None:
        if event.item is not None and event.item.id:
            self._selected = self._keys[int(event.item.id.removeprefix("key-"))]
            self._refresh_shader_view()

    def on_data_table_row_selected(self, event: DataTable.RowSelected) -> None:
        """Picking a row on the Overview tab jumps to that shader."""
        if 0 <= event.cursor_row < len(self._keys):
            self._selected = self._keys[event.cursor_row]
            self.query_one(TabbedContent).active = "tab-shaders"
            self.query_one("#shader-list", ListView).index = event.cursor_row
            self._refresh_shader_view()

    def _refresh_shader_view(self) -> None:
        k = self._selected
        if k is None:
            return
        try:
            details = self.query_one("#shader-details", Static)
            contents = self.query_one("#shader-contents", Static)
        except NoMatches:
            return  # a selection arrived before the panes were composed
        details.update(self._details_text(k))
        contents.update(self._contents_renderable(k))
        # d, c and f apply to the shader now selected, which may not take them
        self.refresh_bindings()

    def _can_view(self, mode: str, k: ShaderKey | None = None) -> bool:
        """Whether this mode's key has anything to offer for this shader."""
        if k is None:
            k = self._selected
        return (k is not None and k.source in BINARY_SOURCES
                and bool(self.qsb.shaders[k].shader)
                and qsb_disasm.can_run(mode, k.source))

    def _showing_tool(self, k: ShaderKey | None = None) -> bool:
        """Whether a tool's output stands in for the hex dump right now."""
        return bool(self._view) and self._can_view(self._view, k)

    def _tool_output(self, mode: str, k: ShaderKey) -> qsb_disasm.ToolOutput:
        """One payload through one tool, running it at most once.

        Synchronous: these tools take a few tens of milliseconds on payloads this
        size, so a worker would buy a spinner nobody would see.
        """
        if (mode, k) not in self._tool_cache:
            self._tool_cache[(mode, k)] = qsb_disasm.run(
                mode, k.source, self.qsb.shaders[k].shader)
        return self._tool_cache[(mode, k)]

    def _details_text(self, k: ShaderKey) -> Text:
        q = self.qsb
        code = q.shaders[k]
        t = Text()
        t.append(f"Shader {self._keys.index(k)}: ", style="dim")
        t.append(str(k), style="bold")
        t.append("\n")
        t.append("Entry point   ", style="bold")
        t.append(code.entry_point or "-")
        t.append("\n")
        t.append("Payload       ", style="bold")
        t.append(f"{len(code.shader)} bytes "
                 f"({'binary' if k.source in BINARY_SOURCES else 'text'})")

        # what the key means to a Qt application loading this pack: the version
        # number is the part that does not speak for itself, so the detail goes
        # on its own line rather than being squeezed into the summary
        use = qsb_reader.runtime_use(k)
        t.append("\n")
        t.append("Runtime use   ", style="bold")
        t.append(use.summary, style="yellow" if use.unused else "")
        if use.detail:
            t.append(f"\n{'':<14}{use.detail}", style="dim")

        entries = qsb_reader.resolve_native_bindings(q, k)
        if entries:
            t.append("\n\nNative resource binding map\n", style="bold")

            # native() falls back to the raw [first, second] for anything it
            # cannot name, so there is no need for a separate raw column
            natives = [e.native(k.source) for e in entries]
            w_native = max(len(s) for s in natives)
            for e, native in zip(entries, natives):
                t.append(f"  binding {e.binding:<3} -> ")
                t.append(f"{native:<{w_native}}",
                         style="dim" if native == "unused" else "bold")
                t.append("   ")
                t.append(e.describe())
                caveat = e.caveat(k.source)
                if caveat:
                    t.append(f"   {caveat}", style="yellow")
                if e.ambiguous:
                    t.append("   several resources share this binding number", style="red")
                t.append("\n")
            t.remove_suffix("\n")

        smap = qsb_reader.resolve_combined_samplers(q, k)
        if smap:
            t.append("\n\nAuto-generated combined image samplers\n", style="bold")
            w_name = max(len(e.combined_sampler_name) for e in smap)
            for e in smap:
                t.append(f"  {e.combined_sampler_name:<{w_name}}", style="bold")
                t.append(f" = {e.describe_texture()} + {e.describe_sampler()}\n")
            t.remove_suffix("\n")

        info = q.native_shader_info.get(k)
        if info and info.flags:
            t.append("\n\nNative shader info flags  ", style="bold")
            t.append(str(info.flags))
        if info and info.extra_buffer_bindings:
            t.append("\n\nNative shader extra buffer bindings\n", style="bold")
            names = {kk: EXTRA_BUFFER_BINDINGS.get(kk, str(kk))
                     for kk in info.extra_buffer_bindings}
            w_name = max(len(n) for n in names.values())
            for kk in sorted(info.extra_buffer_bindings):
                t.append(f"  {names[kk]:<{w_name}} -> ")
                t.append(qsb_reader.extra_buffer_binding_slot(
                             kk, info.extra_buffer_bindings[kk]), style="bold")
                t.append("\n")
            t.remove_suffix("\n")

        if k.source == 0:  # SPIR-V: the header is cheap and worth showing
            h = spirv_header_info(code.shader)
            if h:
                t.append("\n\nSPIR-V header\n", style="bold")
                t.append(f"  version {h['version']}, {h['endian']} endian, "
                         f"id bound {h['id_bound']}, {h['instruction_words']} instruction words\n"
                         f"  generator {h['generator']} (version {h['generator_version']})")
            else:
                t.append("\n\n")
                t.append("SPIR-V magic number missing", style="red")

        t.append("\n\nContents", style="bold")
        # say which view of a bytecode payload this is and what produced it - the
        # tool found is not always the one that was expected - or, on the hex dump,
        # which keys would produce something better for this particular payload
        if self._showing_tool(k):
            label = qsb_disasm.MODE_LABELS[self._view]
            out = self._tool_output(self._view, k)
            if out.ok:
                t.append(f"   {label} from {out.tool}", style="dim")
            else:
                t.append(f"   {label} unavailable "
                         f"({qsb_disasm.tool_name(self._view, k.source)})", style="dim")
        elif k.source in BINARY_SOURCES:
            offers = ", ".join(
                f"{key} for {qsb_disasm.tool_name(mode, k.source)}"
                for mode, key in self.VIEW_KEYS.items() if self._can_view(mode, k))
            t.append(f"   hex dump - {offers}" if offers else "   hex dump",
                     style="dim")
        return t

    def _contents_renderable(self, k: ShaderKey):
        code = self.qsb.shaders[k].shader
        if not code:
            return Text("(empty)", style="dim")
        if self._showing_tool(k):
            out = self._tool_output(self._view, k)
            if not out.ok:
                # the tool is missing or unhappy; the hex dump is one keypress
                # away, so this is a message rather than a failure
                return Text(out.error, style="red")
            return Syntax(out.text, out.lexer, theme="ansi_dark",
                          line_numbers=True, word_wrap=False)
        if k.source in BINARY_SOURCES:
            limit = None if self._full_hexdump else HEXDUMP_LIMIT
            body = hexdump(code, limit=limit)
            return Syntax(body, "text", theme="ansi_dark", word_wrap=False)
        text = code.decode("utf-8", errors="replace")
        return Syntax(text, LEXERS.get(k.source, "text"), theme="ansi_dark",
                      line_numbers=True, word_wrap=False)

    # -- actions -----------------------------------------------------------

    def action_back(self) -> None:
        if self._came_from_picker():
            self.app.pop_screen()

    def action_cycle_tab(self, step: int) -> None:
        """Activate the next or previous tab, wrapping around."""
        try:
            tabs = self.query_one(Tabs)  # TabbedContent's own ContentTabs
        except NoMatches:
            return
        if step > 0:
            tabs.action_next_tab()
        else:
            tabs.action_previous_tab()

    def action_scroll_payload(self, how: str) -> None:
        """Scroll the payload pane, which is deliberately not focusable."""
        try:
            pane = self.query_one("#shader-body", ScrollableContainer)
        except NoMatches:
            return
        # a column at a time would be tedious: generated MSL reaches 360 columns
        step = HSCROLL_STEP
        scroll = {
            "page-up": pane.scroll_page_up,
            "page-down": pane.scroll_page_down,
            "home": pane.scroll_home,
            "end": pane.scroll_end,
            "left": lambda: pane.scroll_relative(x=-step, animate=False),
            "right": lambda: pane.scroll_relative(x=step, animate=False),
        }.get(how)
        if scroll:
            scroll()

    def action_toggle_disassembly(self) -> None:
        self._toggle_view(qsb_disasm.DISASM)

    def action_toggle_vulkan_glsl(self) -> None:
        self._toggle_view(qsb_disasm.CROSS)

    def _toggle_view(self, mode: str) -> None:
        """Show a payload through one of the tools, or back to the hex dump.

        The keys are toggles rather than a cycle: pressing the one already in
        effect goes back to the hex, pressing the other swaps directly to it.
        """
        self._view = "" if self._view == mode else mode
        self._refresh_shader_view()
        # _showing_tool rather than _view: the keys are only offered for a payload
        # their tool can take, but the setting outlives the selection it was made on
        if self._showing_tool() and self._selected is not None:
            out = self._tool_output(self._view, self._selected)
            label = qsb_disasm.MODE_LABELS[self._view]
            # a missing tool is reported in the pane too, at length; this is only
            # so that the keypress visibly did something
            self.notify(f"{label} via {os.path.basename(out.tool)}" if out.ok
                        else f"{qsb_disasm.tool_name(self._view, self._selected.source)} "
                             f"not available",
                        severity="information" if out.ok else "warning")
        else:
            self.notify("Hex dump")
        # the payload changed shape entirely; start from the top of it
        self.action_scroll_payload("home")

    def action_toggle_full_hexdump(self) -> None:
        self._full_hexdump = not self._full_hexdump
        self.notify(f"Hex dump limit {'off' if self._full_hexdump else 'on'}")
        self._refresh_shader_view()

    def action_export(self) -> None:
        k = self._selected
        if k is None:
            self.notify("No shader selected", severity="warning")
            return
        code = self.qsb.shaders[k].shader
        stem = os.path.splitext(os.path.basename(self.qsb.path))[0]
        tag = f"{k.source_name.lower()}{k.version_str.replace(' ', '')}"
        if k.variant:
            tag += f".{k.variant_name.lower()}"
        ext = "bin" if k.source in BINARY_SOURCES else "txt"
        # never clobber: pick the first free name
        out = f"{stem}.{tag}.{ext}"
        n = 1
        while os.path.exists(out):
            out = f"{stem}.{tag}.{n}.{ext}"
            n += 1
        try:
            with open(out, "wb") as fh:
                fh.write(code)
        except OSError as e:
            self.notify(f"Could not write {out}: {e}", severity="error")
            return
        self.notify(f"Wrote {len(code)} bytes to {out}")


class QsbDirectoryTree(DirectoryTree):
    """Directories and .qsb files only, so that the tree stays readable."""

    def filter_paths(self, paths):
        # No sorting here: _load_directory sorts what this returns, directories
        # first and then by name.
        #
        # Dotted names are kept, tempting as it is to treat them as noise: qtbase's
        # CMake integration bakes packs into a directory literally called .qsb
        # inside the build tree, so hiding them would hide the very thing someone
        # browsing a build is looking for. endswith rather than Path.suffix, since
        # a file named just ".qsb" has no suffix as far as pathlib is concerned.
        keep = []
        for p in paths:
            try:
                if p.is_dir() or p.name.lower().endswith(".qsb"):
                    keep.append(p)
            except OSError:
                pass  # unreadable entry, just leave it out
        return keep


class BrowserScreen(PickerScreen):
    """Shown when no file was named: pick a .qsb to inspect."""

    def __init__(self, start_dir: str):
        super().__init__()
        self._start_dir = start_dir

    def compose(self) -> ComposeResult:
        yield Header()
        yield Static("Enter opens a .qsb file or expands a directory",
                     id="browser-hint")
        yield QsbDirectoryTree(self._start_dir, id="browser")
        yield Footer()

    def on_mount(self) -> None:
        self.app.sub_title = self._start_dir
        tree = self.query_one(QsbDirectoryTree)
        # DirectoryTree labels its root with the last component of the path only,
        # which says nothing for ".." and is empty for "." - show where we
        # actually are instead
        tree.root.set_label(self._start_dir)
        tree.focus()

    def on_directory_tree_file_selected(
            self, event: DirectoryTree.FileSelected) -> None:
        try:
            qsb = qsb_reader.load(str(event.path))
        except (OSError, QsbError) as e:
            self.notify(f"{event.path.name}: {e}", severity="error", timeout=8)
            return
        self.app.push_screen(InspectScreen(qsb))


class ChangeScreen(PickerScreen):
    """The .qsb files one change adds or modifies.

    Serves a local commit and a Gerrit change alike: both offer a list of changed
    files and a blob() that reads one as the change leaves it, which is all this
    needs. Neither reads the working tree, so what is shown is the after-patch
    content even when a later commit touched the same file.

    blob() is called on Enter rather than up front. For a commit that is a git
    subprocess; for a Gerrit change it is an HTTP request, which is why opening a
    pack from one takes a moment.
    """

    def __init__(self, change):
        super().__init__()
        self.change = change

    def compose(self) -> ComposeResult:
        yield Header()
        yield Static(self.change.title, id="change-subject")
        yield Static("Enter opens the file as this change leaves it",
                     id="browser-hint")
        yield DataTable(id="change-files", cursor_type="row", zebra_stripes=True)
        yield Footer()

    def on_mount(self) -> None:
        self.app.sub_title = self.change.origin
        table = self.query_one("#change-files", DataTable)
        table.add_columns("Change", "Path")
        for f in self.change.files:
            table.add_row(f.label, f.path)
        table.focus()

    def on_data_table_row_selected(self, event: DataTable.RowSelected) -> None:
        if not 0 <= event.cursor_row < len(self.change.files):
            return
        f = self.change.files[event.cursor_row]
        try:
            qsb = qsb_reader.loads(self.change.blob(f.path), path=f.path)
        except (qsb_git.GitError, qsb_gerrit.GerritError, QsbError) as e:
            self.notify(f"{f.path}: {e}", severity="error", timeout=8)
            return
        self.app.push_screen(InspectScreen(qsb, origin=self.change.origin))


class QsbInspectApp(App):
    TITLE = "qsb inspect"

    CSS = """
    #shader-pane { height: 1fr; }
    #shader-list { width: 34; border-right: solid $panel-lighten-2; }
    #shader-body { padding: 0 1; }
    /* let the payload keep its natural width so it can be scrolled sideways */
    #shader-details, #shader-contents { width: auto; }
    #overview { padding: 1 2; }
    #json-body { padding: 0 1; }
    .section { margin-top: 1; }
    DataTable { height: auto; margin-top: 1; }

    /* The JSON pane is a focus stop, being the whole of its tab; tint it when
       focused so that landing on it with Tab does not look like nothing
       happened. The payload pane is not a focus stop - see on_mount. */
    #json-body:focus { background: $boost; }

    #browser-hint { padding: 1 2 0 2; color: $text-muted; }
    #change-subject { padding: 1 2 0 2; text-style: bold; }
    #change-files { padding: 0 1; }
    #browser { padding: 0 1; }
    """

    # Quit and the theme toggle belong to the app, so that both screens have them
    BINDINGS = [
        Binding("q", "quit", "Quit"),
        Binding("t", "toggle_theme", "Theme"),
    ]

    def __init__(self, qsb: QsbFile | None = None, start_dir: str | None = None,
                 change: qsb_git.Commit | qsb_gerrit.GerritChange | None = None):
        super().__init__()
        # No eased scrolling: when reading code, landing immediately where the key
        # asked beats a smooth glide. Applies to every scroll in the app, the ones
        # the list, tree and table do themselves included. Has to be set here
        # rather than as a class attribute: App.__init__ assigns it from
        # TEXTUAL_ANIMATIONS and would overwrite it.
        self.animation_level = "none"
        self._qsb = qsb
        self._change = change
        # absolute, so that a relative argument like ".." names a real location
        # everywhere it is shown
        self._start_dir = os.path.abspath(start_dir or os.getcwd())

    def on_mount(self) -> None:
        if self._qsb is not None:
            self.push_screen(InspectScreen(self._qsb))
        elif self._change is not None:
            self.push_screen(ChangeScreen(self._change))
        else:
            self.push_screen(BrowserScreen(self._start_dir))

    def action_toggle_theme(self) -> None:
        self.theme = "textual-light" if self.theme == "textual-dark" else "textual-dark"


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Interactively inspect a Qt .qsb shader pack.")
    p.add_argument("file", nargs="?", metavar="path",
                   help="a .qsb file to open, a directory to browse, a git "
                        "revision whose .qsb files to list, or a Gerrit change "
                        "number or URL; omit it to browse the current directory")
    p.add_argument("--json", action="store_true",
                   help="print the reflection info as JSON and exit "
                        "(same content as 'qsb -d' shows)")
    args = p.parse_args(argv)

    if args.file is None or os.path.isdir(args.file):
        if args.json:
            p.error("--json needs a .qsb file")
        QsbInspectApp(start_dir=args.file).run()
        return 0

    if not os.path.isfile(args.file):
        # Not a path at all: a git revision, or a Gerrit change. git goes first
        # because it is local and instant, and because a change number is valid
        # hex - in a big enough repository it could name an object.
        change = qsb_git.describe_commit(args.file, os.getcwd())
        if change is None:
            number = qsb_gerrit.parse_change_ref(args.file)
            if number is None:
                print(f"{args.file}: not a .qsb file, a directory, a git "
                      f"revision, or a Gerrit change", file=sys.stderr)
                return 1
            try:
                change = qsb_gerrit.describe_change(number)
            except qsb_gerrit.GerritError as e:
                print(f"Gerrit change {number}: {e}", file=sys.stderr)
                return 1
        if args.json:
            p.error("--json needs a .qsb file")
        if not change.files:
            print(f"no new or changed .qsb files in {change.origin} "
                  f"({change.subject})")
            return 1
        QsbInspectApp(change=change).run()
        return 0

    try:
        qsb = qsb_reader.load(args.file)
    except (OSError, QsbError) as e:
        print(f"{args.file}: {e}", file=sys.stderr)
        return 1

    if args.json:
        print(qsb_reader.description_to_json(qsb.description))
        return 0

    QsbInspectApp(qsb).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
