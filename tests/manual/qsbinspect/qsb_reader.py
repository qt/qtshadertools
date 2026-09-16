# Copyright (C) 2026 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR GPL-3.0-only WITH Qt-GPL-exception-1.0

"""Pure-Python reader for Qt shader pack (.qsb) files.

Standard library only - no Qt, no third-party modules. Mirrors
QShader::fromSerialized() (qtbase/src/gui/rhi/qshader.cpp) and
QShaderDescriptionPrivate::loadFromStream() (qshaderdescription.cpp).

A .qsb file is qCompress()'d data: a 4-byte big-endian uncompressed size
followed by a zlib stream. Inside is a QDataStream at version Qt_5_10,
big endian, using only int32/uint32/uint8/QByteArray/QString.

The format is Qt private API. QSB_VERSION is read from the stream, so files
written by a newer Qt than this reader knows about are rejected with a clear
message rather than silently misparsed.
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass, field
from typing import NamedTuple

# QShaderPrivate, qshader_p.h
QSB_VERSION = 9
V_WITHOUT_INPUT_OUTPUT_INTERFACE_BLOCKS = 8
V_WITHOUT_EXTENDED_STORAGE_BUFFER_INFO = 7
V_WITHOUT_NATIVE_SHADER_INFO = 6
V_WITHOUT_SEPARATE_IMAGES_AND_SAMPLERS = 5
V_WITHOUT_VAR_ARRAYDIMS = 4
V_WITH_CBOR = 3
V_WITH_BINARY_JSON = 2
V_WITHOUT_BINDINGS = 1

KNOWN_VERSIONS = (4, 5, 6, 7, 8, 9)


class QsbError(Exception):
    pass


# ---------------------------------------------------------------------------
# enum name tables
# ---------------------------------------------------------------------------

# QShader::Stage
STAGES = {
    0: "Vertex",
    1: "TessellationControl",
    2: "TessellationEvaluation",
    3: "Geometry",
    4: "Fragment",
    5: "Compute",
}

# QShader::Source, spelled as qsb -d spells it
SOURCES = {
    0: "SPIR-V",
    1: "GLSL",
    2: "HLSL",
    3: "DXBC",
    4: "MSL",
    5: "DXIL",
    6: "metallib",
    7: "WGSL",
}

# Sources whose payload is binary rather than text
BINARY_SOURCES = frozenset({0, 3, 5, 6})

# Sources that share a native resource binding model. DXBC and DXIL are compiled
# from the HLSL, and metallib from the MSL; qsb's replaceShaderContents() moves
# the binding map over to the bytecode key unchanged, so the numbers mean the
# same thing and want the same terminology.
HLSL_SOURCES = frozenset({2, 3, 5})   # HlslShader, DxbcShader, DxilShader
MSL_SOURCES = frozenset({4, 6})       # MslShader, MetalLibShader

# QShader::Variant
VARIANTS = {
    0: "Standard",
    1: "Batchable",
    2: "UInt16IndexedVertexAsCompute",
    3: "UInt32IndexedVertexAsCompute",
    4: "NonIndexedVertexAsCompute",
    5: "HdrCapableFragment",
    6: "ArgumentBuffer",
}

# QShaderVersion::Flags
GLSL_ES_FLAG = 1 << 0

# QShaderDescription::VariableType -> the spelling in typeTab.
# Index 0 (Unknown) is deliberately empty: typeTab has no entry for Unknown, so
# Qt's typeStr() returns an empty string, and that empty string is what ends up
# in toJson()'s output. Keep it that way so the JSON matches byte for byte.
VARIABLE_TYPES = [""] + (
    "float vec2 vec3 vec4 mat2 mat2x3 mat2x4 mat3 mat3x2 mat3x4 mat4 mat4x2 mat4x3 "
    "int ivec2 ivec3 ivec4 "
    "uint uvec2 uvec3 uvec4 "
    "bool bvec2 bvec3 bvec4 "
    "double dvec2 dvec3 dvec4 dmat2 dmat2x3 dmat2x4 dmat3 dmat3x2 dmat3x4 dmat4 dmat4x2 dmat4x3 "
    "sampler1D sampler2D sampler2DMS sampler3D samplerCube sampler1DArray sampler2DArray "
    "sampler2DMSArray sampler3DArray samplerCubeArray samplerRect samplerBuffer "
    "samplerExternalOES sampler "
    "image1D image2D image2DMS image3D imageCube image1DArray image2DArray image2DMSArray "
    "image3DArray imageCubeArray imageRect imageBuffer "
    "struct half half2 half3 half4"
).split()

# QShaderDescription::ImageFormat, in enum order (imageFormatTab)
IMAGE_FORMATS = (
    "unknown rgba32f rgba16 r32f rgba8 rgba8_snorm rg32f rg16f r11f_g11f_b10f r16f "
    "rgba16 rgb10_a2 rg16 rg8 r16 r8 rgba16_snorm rg16_snorm rg8_snorm r16_snorm r8_snorm "
    "rgba32i rgba16i rgba8i r32i rg32i rg16i rg8i r16i r8i "
    "rgba32ui rgba16ui rgba8ui r32ui rgb10_a2ui rg32ui rg16ui rg8ui r16ui r8ui"
).split()

# QShaderDescription::BuiltinType (sparse - matches SpvBuiltIn)
BUILTIN_TYPES = {
    0: "Position", 1: "PointSize", 3: "ClipDistance", 4: "CullDistance",
    5: "VertexId", 6: "InstanceId", 7: "PrimitiveId", 8: "InvocationId",
    9: "Layer", 10: "ViewportIndex", 11: "TessLevelOuter", 12: "TessLevelInner",
    13: "TessCoord", 14: "PatchVertices", 15: "FragCoord", 16: "PointCoord",
    17: "FrontFacing", 18: "SampleId", 19: "SamplePosition", 20: "SampleMask",
    22: "FragDepth", 24: "NumWorkGroups", 25: "WorkgroupSize", 26: "WorkgroupId",
    27: "LocalInvocationId", 28: "GlobalInvocationId", 29: "LocalInvocationIndex",
    42: "VertexIndex", 43: "InstanceIndex", 4440: "ViewIndex",
}

TESS_MODES = ("unknown", "triangles", "quad", "isoline")
TESS_WINDINGS = ("unknown", "cw", "ccw")
TESS_PARTITIONINGS = ("unknown", "equal_spacing",
                      "fractional_even_spacing", "fractional_odd_spacing")

# QShaderDescription::ImageFlag
IMAGE_FLAG_NAMES = ((1 << 0, "readonly"), (1 << 1, "writeonly"))

# QShaderDescription::QualifierFlag
QUALIFIER_FLAG_NAMES = (
    (1 << 0, "readonly"), (1 << 1, "writeonly"), (1 << 2, "coherent"),
    (1 << 3, "volatile"), (1 << 4, "restrict"),
)

# QShaderPrivate::NativeShaderInfoExtraBufferBindings, named as qsb -d names them
EXTRA_BUFFER_BINDINGS = {
    0: "tessellation(vert)-index-buffer-binding",
    1: "tessellation(vert/tesc)-output-buffer-binding",
    2: "tessellation(tesc)-level-buffer-binding",
    3: "tessellation(tesc)-patch-output-buffer-binding",
    4: "tessellation(tesc)-params-buffer-binding",
    5: "tessellation(tesc)-input-buffer-binding",
    6: "buffer-size-buffer-binding",
    7: "view-mask-buffer-binding",
    8: "argument-buffer-binding",
    9: "push-constant-buffer-binding",
    10: "push-constant-register",
}

# The keys whose value is an HLSL register rather than an MSL buffer argument
# index. The enum lost its Msl prefix when HlslPushConstantBufferBinding joined
# it, so the target is no longer implied by the map itself.
HLSL_EXTRA_BUFFER_BINDINGS = frozenset({10})   # HlslPushConstantBufferBinding


def variable_type_str(t: int) -> str:
    """Exactly what Qt's typeStr() yields, including "" for Unknown/unmapped."""
    return VARIABLE_TYPES[t] if 0 <= t < len(VARIABLE_TYPES) else ""


def variable_type_display(t: int) -> str:
    """Same, but never blank - for the UI, where "" would just look broken."""
    s = variable_type_str(t)
    if s:
        return s
    return "unknown" if t == 0 else f"<type {t}>"


def image_format_str(f: int) -> str:
    return IMAGE_FORMATS[f] if 0 <= f < len(IMAGE_FORMATS) else ""


def _flags_str(value: int, names) -> str:
    hit = [n for bit, n in names if value & bit]
    left = value & ~sum(bit for bit, _ in names)
    if left:
        hit.append(hex(left))
    return "|".join(hit)


def image_flags_str(f: int) -> str:
    return _flags_str(f, IMAGE_FLAG_NAMES)


def qualifier_flags_str(f: int) -> str:
    return _flags_str(f, QUALIFIER_FLAG_NAMES)


# ---------------------------------------------------------------------------
# QDataStream reader
# ---------------------------------------------------------------------------

class DataStream:
    """The subset of QDataStream (version Qt_5_10, big endian) that .qsb uses."""

    def __init__(self, data: bytes):
        self._d = data
        self._p = 0

    @property
    def pos(self) -> int:
        return self._p

    @property
    def remaining(self) -> int:
        return len(self._d) - self._p

    def _take(self, n: int) -> bytes:
        if n < 0 or self._p + n > len(self._d):
            raise QsbError(f"truncated stream: wanted {n} bytes at offset {self._p}, "
                           f"only {self.remaining} left")
        v = self._d[self._p:self._p + n]
        self._p += n
        return v

    def i32(self) -> int:
        return struct.unpack_from(">i", self._take(4))[0]

    def u32(self) -> int:
        return struct.unpack_from(">I", self._take(4))[0]

    def u8(self) -> int:
        return self._take(1)[0]

    def bool_(self) -> bool:
        return self._take(1)[0] != 0

    def qbytearray(self) -> bytes:
        n = self.u32()
        return b"" if n == 0xFFFFFFFF else self._take(n)

    def qstring(self) -> str:
        n = self.u32()  # byte count, not character count
        if n == 0xFFFFFFFF:
            return ""
        return self._take(n).decode("utf-16-be", errors="replace")

    def count(self) -> int:
        """QShaderPrivate::readCount() - reject counts the data cannot back."""
        n = self.i32()
        if n < 0 or n > self.remaining // 4:
            raise QsbError(f"corrupt element count {n} at offset {self._p - 4} "
                           f"({self.remaining} bytes remain)")
        return n


# ---------------------------------------------------------------------------
# data model
# ---------------------------------------------------------------------------

@dataclass(frozen=True, order=True)
class ShaderKey:
    # field order matters: it reproduces operator<(QShaderKey) so that
    # iteration order matches QMap's, i.e. qsb -d's shader numbering
    source: int
    version: int
    flags: int
    variant: int

    @property
    def source_name(self) -> str:
        return SOURCES.get(self.source, f"<source {self.source}>")

    @property
    def version_str(self) -> str:
        s = str(self.version) if self.version else ""
        if self.flags & GLSL_ES_FLAG:
            s += " es"
        return s

    @property
    def variant_name(self) -> str:
        return VARIANTS.get(self.variant, f"<variant {self.variant}>")

    def __str__(self) -> str:
        # matches qsb -d: "GLSL 100 es [Batchable]"
        return f"{self.source_name} {self.version_str} [{self.variant_name}]"

    def short(self) -> str:
        s = f"{self.source_name} {self.version_str}".strip()
        return s if self.variant == 0 else f"{s} ({self.variant_name})"


@dataclass
class BlockVariable:
    name: str = ""
    type: int = 0
    offset: int = 0
    size: int = 0
    array_dims: list[int] = field(default_factory=list)
    array_stride: int = 0
    matrix_stride: int = 0
    matrix_row_major: bool = False
    struct_members: list["BlockVariable"] = field(default_factory=list)


@dataclass
class InOutVariable:
    name: str = ""
    type: int = 0
    location: int = -1
    binding: int = -1
    descriptor_set: int = -1
    image_format: int = 0
    image_flags: int = 0
    array_dims: list[int] = field(default_factory=list)
    per_patch: bool = False
    struct_members: list[BlockVariable] = field(default_factory=list)


@dataclass
class BuiltinVariable:
    type: int = 0
    var_type: int = 0
    array_dims: list[int] = field(default_factory=list)

    @property
    def name(self) -> str:
        # Qt's builtinTypeStr() returns "" for anything not in its table
        return BUILTIN_TYPES.get(self.type, "")

    @property
    def display_name(self) -> str:
        return self.name or f"<builtin {self.type}>"


@dataclass
class UniformBlock:
    block_name: str = ""
    struct_name: str = ""
    size: int = 0
    binding: int = -1
    descriptor_set: int = -1
    members: list[BlockVariable] = field(default_factory=list)


@dataclass
class PushConstantBlock:
    name: str = ""
    size: int = 0
    members: list[BlockVariable] = field(default_factory=list)


@dataclass
class StorageBlock:
    block_name: str = ""
    instance_name: str = ""
    known_size: int = 0
    binding: int = -1
    descriptor_set: int = -1
    members: list[BlockVariable] = field(default_factory=list)
    runtime_array_stride: int = 0
    qualifier_flags: int = 0


@dataclass
class ShaderDescription:
    inputs: list[InOutVariable] = field(default_factory=list)
    outputs: list[InOutVariable] = field(default_factory=list)
    uniform_blocks: list[UniformBlock] = field(default_factory=list)
    push_constant_blocks: list[PushConstantBlock] = field(default_factory=list)
    storage_blocks: list[StorageBlock] = field(default_factory=list)
    combined_image_samplers: list[InOutVariable] = field(default_factory=list)
    storage_images: list[InOutVariable] = field(default_factory=list)
    separate_images: list[InOutVariable] = field(default_factory=list)
    separate_samplers: list[InOutVariable] = field(default_factory=list)
    in_builtins: list[BuiltinVariable] = field(default_factory=list)
    out_builtins: list[BuiltinVariable] = field(default_factory=list)
    local_size: tuple[int, int, int] = (0, 0, 0)
    tess_out_vert_count: int = 0
    tess_mode: int = 0
    tess_winding: int = 0
    tess_partitioning: int = 0

    def is_empty(self) -> bool:
        return not any((self.inputs, self.outputs, self.uniform_blocks,
                        self.push_constant_blocks, self.storage_blocks,
                        self.combined_image_samplers, self.storage_images,
                        self.separate_images, self.separate_samplers,
                        self.in_builtins, self.out_builtins,
                        any(self.local_size), self.tess_out_vert_count,
                        self.tess_mode, self.tess_winding, self.tess_partitioning))


@dataclass
class ShaderCode:
    shader: bytes = b""
    entry_point: str = ""


@dataclass
class NativeShaderInfo:
    flags: int = 0
    extra_buffer_bindings: dict[int, int] = field(default_factory=dict)


@dataclass
class CombinedSamplerMapping:
    combined_sampler_name: str = ""
    texture_binding: int = -1
    sampler_binding: int = -1


@dataclass
class QsbFile:
    path: str = ""
    file_size: int = 0
    uncompressed_size: int = 0
    qsb_version: int = 0
    stage: int = 0
    description: ShaderDescription = field(default_factory=ShaderDescription)
    shaders: dict[ShaderKey, ShaderCode] = field(default_factory=dict)
    bindings: dict[ShaderKey, dict[int, tuple[int, int]]] = field(default_factory=dict)
    combined_image_map: dict[ShaderKey, list[CombinedSamplerMapping]] = field(default_factory=dict)
    native_shader_info: dict[ShaderKey, NativeShaderInfo] = field(default_factory=dict)
    trailing_bytes: int = 0

    @property
    def stage_name(self) -> str:
        return STAGES.get(self.stage, f"<stage {self.stage}>")

    def keys(self) -> list[ShaderKey]:
        """Shader keys in QMap order, i.e. qsb -d's 'Shader N' numbering."""
        return sorted(self.shaders)


# ---------------------------------------------------------------------------
# parsing
# ---------------------------------------------------------------------------

def _read_decorations(ds: DataStream, ver: int, v: InOutVariable) -> None:
    v.location = ds.i32()
    v.binding = ds.i32()
    v.descriptor_set = ds.i32()
    v.image_format = ds.i32()
    v.image_flags = ds.i32()
    if ver > V_WITHOUT_VAR_ARRAYDIMS:
        v.array_dims = [ds.i32() for _ in range(ds.count())]
    if ver > V_WITHOUT_NATIVE_SHADER_INFO:
        v.per_patch = ds.u8() != 0


MAX_STRUCT_NESTING_LEVEL = 64


def _read_block_member(ds: DataStream, ver: int, level: int = 0) -> BlockVariable:
    if level > MAX_STRUCT_NESTING_LEVEL:
        raise QsbError(f"structs nested deeper than {MAX_STRUCT_NESTING_LEVEL} levels")
    v = BlockVariable()
    v.name = ds.qstring()
    v.type = ds.i32()
    v.offset = ds.i32()
    v.size = ds.i32()
    v.array_dims = [ds.i32() for _ in range(ds.count())]
    v.array_stride = ds.i32()
    v.matrix_stride = ds.i32()
    v.matrix_row_major = ds.bool_()
    v.struct_members = [_read_block_member(ds, ver, level + 1) for _ in range(ds.count())]
    return v


def _read_inout(ds: DataStream, ver: int) -> InOutVariable:
    v = InOutVariable()
    v.name = ds.qstring()
    v.type = ds.i32()
    _read_decorations(ds, ver, v)
    if ver > V_WITHOUT_INPUT_OUTPUT_INTERFACE_BLOCKS:
        v.struct_members = [_read_block_member(ds, ver) for _ in range(ds.count())]
    return v


def _read_named_var(ds: DataStream, ver: int) -> InOutVariable:
    """A samplers/images entry: name + type + decorations, no struct members."""
    v = InOutVariable()
    v.name = ds.qstring()
    v.type = ds.i32()
    _read_decorations(ds, ver, v)
    return v


def _read_builtin(ds: DataStream, ver: int) -> BuiltinVariable:
    v = BuiltinVariable()
    v.type = ds.i32()
    if ver > V_WITHOUT_INPUT_OUTPUT_INTERFACE_BLOCKS:
        v.var_type = ds.i32()
        v.array_dims = [ds.i32() for _ in range(ds.count())]
    return v


def _read_description(ds: DataStream, ver: int) -> ShaderDescription:
    d = ShaderDescription()

    d.inputs = [_read_inout(ds, ver) for _ in range(ds.count())]
    d.outputs = [_read_inout(ds, ver) for _ in range(ds.count())]

    for _ in range(ds.count()):
        b = UniformBlock()
        b.block_name = ds.qstring()
        b.struct_name = ds.qstring()
        b.size = ds.i32()
        b.binding = ds.i32()
        b.descriptor_set = ds.i32()
        b.members = [_read_block_member(ds, ver) for _ in range(ds.count())]
        d.uniform_blocks.append(b)

    for _ in range(ds.count()):
        b = PushConstantBlock()
        b.name = ds.qstring()
        b.size = ds.i32()
        b.members = [_read_block_member(ds, ver) for _ in range(ds.count())]
        d.push_constant_blocks.append(b)

    for _ in range(ds.count()):
        b = StorageBlock()
        b.block_name = ds.qstring()
        b.instance_name = ds.qstring()
        b.known_size = ds.i32()
        b.binding = ds.i32()
        b.descriptor_set = ds.i32()
        b.members = [_read_block_member(ds, ver) for _ in range(ds.count())]
        if ver > V_WITHOUT_EXTENDED_STORAGE_BUFFER_INFO:
            b.runtime_array_stride = ds.i32()
            b.qualifier_flags = ds.i32()
        d.storage_blocks.append(b)

    d.combined_image_samplers = [_read_named_var(ds, ver) for _ in range(ds.count())]
    d.storage_images = [_read_named_var(ds, ver) for _ in range(ds.count())]
    d.local_size = (ds.u32(), ds.u32(), ds.u32())

    if ver > V_WITHOUT_SEPARATE_IMAGES_AND_SAMPLERS:
        d.separate_images = [_read_named_var(ds, ver) for _ in range(ds.count())]
        d.separate_samplers = [_read_named_var(ds, ver) for _ in range(ds.count())]

    if ver > V_WITHOUT_NATIVE_SHADER_INFO:
        d.tess_out_vert_count = ds.u32()
        d.tess_mode = ds.u32()
        d.tess_winding = ds.u32()
        d.tess_partitioning = ds.u32()
        d.in_builtins = [_read_builtin(ds, ver) for _ in range(ds.count())]
        d.out_builtins = [_read_builtin(ds, ver) for _ in range(ds.count())]

    return d


def _read_key(ds: DataStream) -> ShaderKey:
    source = ds.i32()
    version = ds.i32()
    flags = ds.i32()
    variant = ds.i32()
    return ShaderKey(source, version, flags, variant)


def loads(blob: bytes, path: str = "") -> QsbFile:
    """Parse the contents of a .qsb file."""
    if len(blob) < 5:
        raise QsbError("file is too short to be a shader pack")

    # qCompress(): 4-byte big-endian uncompressed size, then a zlib stream
    expected = struct.unpack_from(">I", blob)[0]
    try:
        inner = zlib.decompress(blob[4:])
    except zlib.error as e:
        raise QsbError(f"not a shader pack, or corrupt: zlib inflate failed ({e})") from e
    if len(inner) != expected:
        raise QsbError(f"size header says {expected} bytes but inflated to {len(inner)}")

    ds = DataStream(inner)
    f = QsbFile(path=path, file_size=len(blob), uncompressed_size=len(inner))

    f.qsb_version = ver = ds.i32()
    if ver not in KNOWN_VERSIONS:
        if ver in (V_WITH_CBOR, V_WITH_BINARY_JSON, V_WITHOUT_BINDINGS):
            raise QsbError(
                f"qsb version {ver} is too old: its reflection info is CBOR or binary "
                "JSON, which Qt itself no longer loads either")
        raise QsbError(f"unknown qsb version {ver} (this reader knows up to {QSB_VERSION}); "
                       "the file was probably written by a newer Qt")

    f.stage = ds.i32()
    f.description = _read_description(ds, ver)

    for _ in range(ds.count()):
        k = _read_key(ds)
        code = ShaderCode()
        code.shader = ds.qbytearray()
        code.entry_point = ds.qbytearray().decode("utf-8", errors="replace")
        f.shaders[k] = code

    if ver > V_WITHOUT_BINDINGS:
        for _ in range(ds.count()):
            k = _read_key(ds)
            m: dict[int, tuple[int, int]] = {}
            for _ in range(ds.count()):
                binding = ds.i32()
                m[binding] = (ds.i32(), ds.i32())
            f.bindings[k] = m

    if ver > V_WITHOUT_SEPARATE_IMAGES_AND_SAMPLERS:
        for _ in range(ds.count()):
            k = _read_key(ds)
            lst = []
            for _ in range(ds.count()):
                e = CombinedSamplerMapping()
                e.combined_sampler_name = ds.qbytearray().decode("utf-8", errors="replace")
                e.texture_binding = ds.i32()
                e.sampler_binding = ds.i32()
                lst.append(e)
            f.combined_image_map[k] = lst

    if ver > V_WITHOUT_NATIVE_SHADER_INFO:
        for _ in range(ds.count()):
            k = _read_key(ds)
            info = NativeShaderInfo()
            info.flags = ds.i32()
            for _ in range(ds.count()):
                kk = ds.i32()
                info.extra_buffer_bindings[kk] = ds.i32()
            f.native_shader_info[k] = info

    f.trailing_bytes = ds.remaining
    return f


def load(path: str) -> QsbFile:
    with open(path, "rb") as fh:
        return loads(fh.read(), path=path)


# ---------------------------------------------------------------------------
# reflection info as JSON, matching QShaderDescription::toJson()
# ---------------------------------------------------------------------------

def _json_deco(obj: dict, v: InOutVariable) -> None:
    if v.location >= 0:
        obj["location"] = v.location
    if v.binding >= 0:
        obj["binding"] = v.binding
    if v.descriptor_set >= 0:
        obj["set"] = v.descriptor_set
    if v.per_patch:
        obj["perPatch"] = v.per_patch
    if v.image_format != 0:
        obj["imageFormat"] = image_format_str(v.image_format)
    if v.image_flags:
        obj["imageFlags"] = v.image_flags
    if v.array_dims:
        obj["arrayDims"] = list(v.array_dims)


def _json_block_member(v: BlockVariable) -> dict:
    obj: dict = {"name": v.name, "type": variable_type_str(v.type)}
    if v.offset != -1:
        obj["offset"] = v.offset
    obj["size"] = v.size
    if v.array_dims:
        obj["arrayDims"] = list(v.array_dims)
    if v.array_stride:
        obj["arrayStride"] = v.array_stride
    if v.matrix_stride:
        obj["matrixStride"] = v.matrix_stride
    if v.matrix_row_major:
        obj["matrixRowMajor"] = True
    if v.struct_members:
        obj["structMembers"] = [_json_block_member(m) for m in v.struct_members]
    return obj


def _json_inout(v: InOutVariable) -> dict:
    obj: dict = {"name": v.name, "type": variable_type_str(v.type)}
    _json_deco(obj, v)
    if v.struct_members:
        obj["structMembers"] = [_json_block_member(m) for m in v.struct_members]
    return obj


def _json_named_var(v: InOutVariable) -> dict:
    obj: dict = {"name": v.name, "type": variable_type_str(v.type)}
    _json_deco(obj, v)
    return obj


def _json_builtin(v: BuiltinVariable) -> dict:
    obj: dict = {"name": v.name, "type": variable_type_str(v.var_type)}
    if v.array_dims:
        obj["arrayDims"] = list(v.array_dims)
    return obj


def description_to_json_obj(d: ShaderDescription) -> dict:
    """Reproduce QShaderDescription::toJson()'s content (keys are sorted on dump)."""
    root: dict = {}
    if d.inputs:
        root["inputs"] = [_json_inout(v) for v in d.inputs]
    if d.outputs:
        root["outputs"] = [_json_inout(v) for v in d.outputs]

    if d.uniform_blocks:
        arr = []
        for b in d.uniform_blocks:
            o: dict = {"blockName": b.block_name, "structName": b.struct_name, "size": b.size}
            if b.binding >= 0:
                o["binding"] = b.binding
            if b.descriptor_set >= 0:
                o["set"] = b.descriptor_set
            o["members"] = [_json_block_member(m) for m in b.members]
            arr.append(o)
        root["uniformBlocks"] = arr

    if d.push_constant_blocks:
        arr = []
        for b in d.push_constant_blocks:
            o = {"name": b.name, "size": b.size,
                 "members": [_json_block_member(m) for m in b.members]}
            arr.append(o)
        root["pushConstantBlocks"] = arr

    if d.storage_blocks:
        arr = []
        for b in d.storage_blocks:
            o = {"blockName": b.block_name, "instanceName": b.instance_name,
                 "knownSize": b.known_size}
            if b.binding >= 0:
                o["binding"] = b.binding
            if b.descriptor_set >= 0:
                o["set"] = b.descriptor_set
            if b.runtime_array_stride:
                o["runtimeArrayStride"] = b.runtime_array_stride
            if b.qualifier_flags:
                o["qualifierFlags"] = b.qualifier_flags
            o["members"] = [_json_block_member(m) for m in b.members]
            arr.append(o)
        root["storageBlocks"] = arr

    if d.combined_image_samplers:
        root["combinedImageSamplers"] = [_json_named_var(v) for v in d.combined_image_samplers]
    if d.storage_images:
        root["storageImages"] = [_json_named_var(v) for v in d.storage_images]
    if d.in_builtins:
        root["inBuiltins"] = [_json_builtin(v) for v in d.in_builtins]
    if d.out_builtins:
        root["outBuiltins"] = [_json_builtin(v) for v in d.out_builtins]
    if any(d.local_size):
        root["computeLocalSize"] = list(d.local_size)
    if d.tess_out_vert_count:
        root["tessellationOutputVertexCount"] = d.tess_out_vert_count
    if d.tess_mode:
        root["tessellationMode"] = TESS_MODES[d.tess_mode] \
            if d.tess_mode < len(TESS_MODES) else str(d.tess_mode)
    if d.tess_winding:
        root["tessellationWindingOrder"] = TESS_WINDINGS[d.tess_winding] \
            if d.tess_winding < len(TESS_WINDINGS) else str(d.tess_winding)
    if d.tess_partitioning:
        root["tessellationPartitioning"] = TESS_PARTITIONINGS[d.tess_partitioning] \
            if d.tess_partitioning < len(TESS_PARTITIONINGS) else str(d.tess_partitioning)
    if d.separate_images:
        root["separateImages"] = [_json_named_var(v) for v in d.separate_images]
    if d.separate_samplers:
        root["separateSamplers"] = [_json_named_var(v) for v in d.separate_samplers]
    return root


def description_to_json(d: ShaderDescription) -> str:
    import json
    return json.dumps(description_to_json_obj(d), indent=4, sort_keys=True)


# ---------------------------------------------------------------------------
# payload helpers
# ---------------------------------------------------------------------------

SPIRV_MAGIC = 0x07230203

# SPIR-V generator magic numbers, high 16 bits (registered vendors)
SPIRV_GENERATORS = {
    0: "Khronos", 1: "LunarG", 2: "Valve", 3: "Codeplay", 4: "NVIDIA",
    5: "ARM", 6: "Khronos LLVM/SPIR-V translator", 7: "Khronos assembler",
    8: "Khronos glslang", 9: "Qualcomm", 10: "AMD", 11: "Intel", 12: "Imagination",
    13: "Google shaderc", 14: "Google spiregg", 15: "Google rspirv",
    16: "X-LEGEND Mesa-IR/SPIR-V translator", 17: "Khronos SPIRV-Tools assembler",
    18: "Khronos SPIRV-Tools linker", 21: "Google Clspv", 22: "SPIRV-Tools",
    23: "Khronos glslang (HLSL)", 26: "Microsoft DXC",
}


def spirv_header_info(code: bytes) -> dict | None:
    """Decode the 5-word SPIR-V header, or None if this isn't SPIR-V."""
    if len(code) < 20:
        return None
    magic = struct.unpack_from("<I", code)[0]
    endian = "<"
    if magic != SPIRV_MAGIC:
        magic = struct.unpack_from(">I", code)[0]
        endian = ">"
        if magic != SPIRV_MAGIC:
            return None
    _, version, generator, bound, schema = struct.unpack_from(endian + "5I", code)
    gen_vendor = generator >> 16
    return {
        "endian": "little" if endian == "<" else "big",
        "version": f"{(version >> 16) & 0xFF}.{(version >> 8) & 0xFF}",
        "generator": SPIRV_GENERATORS.get(gen_vendor, f"vendor {gen_vendor}"),
        "generator_version": generator & 0xFFFF,
        "id_bound": bound,
        "schema": schema,
        "instruction_words": (len(code) - 20) // 4,
    }


# ---------------------------------------------------------------------------
# native resource binding map, resolved against the reflection info
# ---------------------------------------------------------------------------
#
# The map is binding -> (first, second). What those two numbers mean depends on
# the kind of resource at that binding, which is why it has to be looked up in
# the reflection info to be readable. From qtshadertools' translateToHLSL() /
# translateToMSL(), and the comment in qtbase's qrhid3d11.cpp:
#
#   uniform block          first = cbuffer / buffer index
#   storage block          first = UAV / buffer index
#   storage image          first = UAV / texture index
#   combined image sampler first = texture index, second = sampler index
#   separate image         first = texture index
#   separate sampler       first = *sampler* index   <- not a texture index
#
# A first of -1 means the resource is not actively used by the compiled shader
# (SPIRV-Cross reports no automatic binding for it); the map still carries the
# entry so that it stays complete.

# resource kind -> (human label, HLSL register prefix, MSL argument form)
_RESOURCE_KINDS = {
    "uniform-buffer":         ("uniform buffer",        "b", "buffer"),
    "storage-buffer":         ("storage buffer",        "u", "buffer"),
    "storage-image":          ("storage image",         "u", "texture"),
    "combined-image-sampler": ("combined image sampler", "t", "texture"),
    "separate-image":         ("separate image",        "t", "texture"),
    "separate-sampler":       ("separate sampler",      "s", "sampler"),
}


@dataclass
class ResolvedBinding:
    binding: int
    first: int
    second: int
    kind: str = "unknown"       # a key of _RESOURCE_KINDS, or "unknown"
    name: str = ""
    type: int = 0
    array_dims: list[int] = field(default_factory=list)
    qualifier_flags: int = 0    # storage blocks only
    ambiguous: bool = False     # same binding number used in several sets

    def caveat(self, source: int) -> str:
        """A remark where the mapping is surprising."""
        if (source in HLSL_SOURCES and self.kind == "storage-buffer"
                and self.qualifier_flags & 1):  # QualifierReadOnly
            # translateToHLSL() sets FORCE_STORAGE_BUFFER_AS_UAV, so even a
            # readonly buffer becomes a UAV rather than the default SRV
            return "readonly, but mapped to a UAV"
        return ""

    @property
    def kind_label(self) -> str:
        return _RESOURCE_KINDS[self.kind][0] if self.kind in _RESOURCE_KINDS else "unknown resource"

    @property
    def count(self) -> int:
        """How many consecutive native slots this binding occupies."""
        return self.array_dims[0] if self.array_dims else 1

    @property
    def unused(self) -> bool:
        return self.first < 0

    def describe(self) -> str:
        """'combined image sampler 'tex' (sampler2D)[4]'"""
        s = self.kind_label
        if self.name:
            s += f" '{self.name}'"
        if self.type:
            s += f" ({variable_type_display(self.type)})"
        if self.array_dims:
            s += "".join(f"[{d}]" for d in self.array_dims)
        return s

    def native(self, source: int) -> str:
        """The native binding, in the terminology of the target language."""
        if self.kind not in _RESOURCE_KINDS:
            return f"[{self.first}, {self.second}]"
        _, hlsl_prefix, msl_form = _RESOURCE_KINDS[self.kind]

        if source in HLSL_SOURCES:  # t0, or t0..t3 for an array
            parts = [_hlsl_reg(hlsl_prefix, self.first, self.count)]
            if self.kind == "combined-image-sampler":
                parts.append(_hlsl_reg("s", self.second, self.count))
        elif source in MSL_SOURCES:  # texture(0), or texture(0..3) for an array
            parts = [_msl_slot(msl_form, self.first, self.count)]
            if self.kind == "combined-image-sampler":
                parts.append(_msl_slot("sampler", self.second, self.count))
        else:
            return f"[{self.first}, {self.second}]"

        parts = [p for p in parts if p]
        return " + ".join(parts) if parts else "unused"


def _hlsl_reg(prefix: str, base: int, count: int) -> str:
    """'t3', or 't3..t6' for an array of four."""
    if base < 0:
        return ""
    if count > 1:
        return f"{prefix}{base}..{prefix}{base + count - 1}"
    return f"{prefix}{base}"


def _msl_slot(form: str, base: int, count: int) -> str:
    """'texture(3)', or 'texture(3..6)' for an array of four."""
    if base < 0:
        return ""
    if count > 1:
        return f"{form}({base}..{base + count - 1})"
    return f"{form}({base})"


class _IndexedResource(NamedTuple):
    kind: str
    name: str
    type: int
    array_dims: list[int]
    qualifier_flags: int = 0


def _binding_index(desc: ShaderDescription) -> dict[int, list[_IndexedResource]]:
    """binding number -> the resources declared at it, from the reflection info.

    A list rather than a single entry because binding numbers are only unique
    within a descriptor set, while the native binding map is keyed by binding
    alone. In practice QRhi only uses set 0, so collisions do not occur, but
    they are worth flagging rather than silently picking one.
    """
    index: dict[int, list[_IndexedResource]] = {}

    def add(binding: int, res: _IndexedResource) -> None:
        if binding >= 0:
            index.setdefault(binding, []).append(res)

    for b in desc.uniform_blocks:
        add(b.binding, _IndexedResource("uniform-buffer", b.block_name, 0, []))
    for b in desc.storage_blocks:
        add(b.binding, _IndexedResource("storage-buffer", b.block_name, 0, [],
                                        b.qualifier_flags))
    for v in desc.combined_image_samplers:
        add(v.binding, _IndexedResource("combined-image-sampler", v.name, v.type,
                                        v.array_dims))
    for v in desc.separate_images:
        add(v.binding, _IndexedResource("separate-image", v.name, v.type, v.array_dims))
    for v in desc.separate_samplers:
        add(v.binding, _IndexedResource("separate-sampler", v.name, v.type, v.array_dims))
    for v in desc.storage_images:
        add(v.binding, _IndexedResource("storage-image", v.name, v.type, v.array_dims))
    return index


def resolve_native_bindings(qsb: "QsbFile", key: ShaderKey) -> list[ResolvedBinding]:
    """Pair the native resource binding map with the resources it refers to."""
    raw = qsb.bindings.get(key)
    if not raw:
        return []
    index = _binding_index(qsb.description)
    out = []
    for binding in sorted(raw):
        first, second = raw[binding]
        entry = ResolvedBinding(binding=binding, first=first, second=second)
        candidates = index.get(binding, [])
        if candidates:
            res = candidates[0]
            entry.kind = res.kind
            entry.name = res.name
            entry.type = res.type
            entry.array_dims = list(res.array_dims)
            entry.qualifier_flags = res.qualifier_flags
            entry.ambiguous = len(candidates) > 1
        out.append(entry)
    return out


def msl_buffer_slot(index: int) -> str:
    """A NativeShaderInfo extra buffer binding, in MSL terms.

    Such an entry is a buffer argument index: the Metal backend passes each one
    straight to [encoder setBuffer:... atIndex:value].
    """
    return _msl_slot("buffer", index, 1) if index >= 0 else "unused"


def extra_buffer_binding_slot(kind: int, value: int) -> str:
    """One extraBufferBindings entry, in the terminology of its own target.

    All but the HLSL ones are MSL buffer argument indices; a push constant block
    translated to HLSL gets a constant buffer register instead, which
    translateToHLSL() reserves ahead of the uniform blocks.
    """
    if kind in HLSL_EXTRA_BUFFER_BINDINGS:
        return _hlsl_reg("b", value, 1) or "unused"
    return msl_buffer_slot(value)


@dataclass
class ResolvedCombinedSampler:
    """One entry of the separate-to-combined image sampler mapping list.

    The two numbers are the bindings of the separate texture and sampler that
    SPIRV-Cross combined into combined_sampler_name, so they can be looked up in
    the reflection info to recover which resources those were.
    """
    combined_sampler_name: str
    texture_binding: int
    sampler_binding: int
    texture_name: str = ""
    sampler_name: str = ""

    def _describe(self, binding: int, name: str) -> str:
        if binding < 0:
            return "unresolved binding"
        s = f"binding {binding}"
        if name:
            s += f" '{name}'"
        return s

    def describe_texture(self) -> str:
        return self._describe(self.texture_binding, self.texture_name)

    def describe_sampler(self) -> str:
        return self._describe(self.sampler_binding, self.sampler_name)


def resolve_combined_samplers(qsb: "QsbFile", key: ShaderKey) -> list[ResolvedCombinedSampler]:
    """Name the separate texture and sampler behind each combined sampler."""
    entries = qsb.combined_image_map.get(key)
    if not entries:
        return []
    textures = {v.binding: v.name for v in qsb.description.separate_images if v.binding >= 0}
    samplers = {v.binding: v.name for v in qsb.description.separate_samplers if v.binding >= 0}
    return [
        ResolvedCombinedSampler(
            combined_sampler_name=e.combined_sampler_name,
            texture_binding=e.texture_binding,
            sampler_binding=e.sampler_binding,
            texture_name=textures.get(e.texture_binding, ""),
            sampler_name=samplers.get(e.sampler_binding, ""),
        )
        for e in entries
    ]


# ---------------------------------------------------------------------------
# which QRhi backend uses a given key at run time
# ---------------------------------------------------------------------------
#
# No backend searches the pack: each one looks up a handful of specific keys, in
# a fixed order, and takes the first that is there. So the list of versions a
# backend asks for is the whole story - a key whose version is not on its list is
# dead weight in the pack, however plausible it looks. Transcribed from
# qtbase/src/gui/rhi:
#
#   qrhivulkan.cpp  { SpirvShader, 100, variant }, and nothing else
#   qrhigles2.cpp   QRhiGles2::shaderSource(), the two version lists below
#   qrhid3d11.cpp   compileHlslShaderSource(): { DxbcShader, 50 }, then
#                   { HlslShader, 50 } compiled with D3DCompile
#   qrhid3d12.cpp   compileHlslShaderSource(): DXIL then DXBC at each shader
#                   model from 67 down to 50, then HLSL compiled with dxc
#   qrhimetal.mm    QRhiMetalData::createMetalLib(): metallib, then MSL source,
#                   versions 30, 24, 23, 22, 21, 20, 12
#
# The variant is not part of this: whichever key is looked up, it is looked up
# with the variant the pipeline asked for, so a Batchable key is reached exactly
# when the standard one would be.

# GLSL versions QRhiGles2 asks for. 120 only outside a core profile; note that
# 110 is on neither list, so a GLSL 110 key is never picked up.
GLSL_ES_VERSIONS = (100, 300, 310, 320)
GLSL_VERSIONS = (120, 130, 140, 150, 330, 400, 410, 420, 430, 440, 450, 460)

# The GL version each desktop GLSL version needs. From 330 on the GLSL version
# is the GL version times 100, before that it is not.
_GL_FOR_GLSL = {120: "2.1", 130: "3.0", 140: "3.1", 150: "3.2"}

# The GLSL ES versions are numbered like the ES versions from 3.0 on, but ES 2.0
# shipped GLSL ES 1.00
_GLES_FOR_GLSL = {100: "2.0", 300: "3.0", 310: "3.1", 320: "3.2"}

# MSL versions QRhiMetal asks for, for both metallib and MSL source
METAL_VERSIONS = (12, 20, 21, 22, 23, 24, 30)

# Shader models QRhiD3D12 asks for. QRhiD3D11 asks for 50 alone.
D3D12_SHADER_MODELS = range(50, 68)


def _dotted(v: int) -> str:
    """10 -> "1.0": how shader model and MSL versions are spelled."""
    return f"{v // 10}.{v % 10}"


@dataclass(frozen=True)
class RuntimeUse:
    """Where one shader key ends up when a Qt application loads the pack."""
    summary: str        # the graphics API, or that nothing takes this key
    detail: str = ""    # what the version means, and the fine print
    unused: bool = False  # no backend ever looks this key up


def _unused(detail: str) -> RuntimeUse:
    return RuntimeUse("not used at run time", detail, unused=True)


def runtime_use(key: ShaderKey) -> RuntimeUse:
    """Which graphics API takes this key at run time, and when.

    Says what the version number means in that API's own terms - a GLSL version
    as the OpenGL versions and profiles that accept it, an HLSL one as a shader
    model - since that is the part a key like "GLSL 150" does not say out loud.
    Names APIs rather than QRhi backend classes: the API is what a reader of a
    pack is asking about, and there is one backend per API anyway.
    """
    v = key.version

    if key.source == 0:  # SPIR-V
        if v != 100:
            return _unused(f"only SPIR-V 100 is ever looked up, not {v}")
        # Said the way DXBC and DXIL are, since it is the same story: bytecode the
        # backend hands to the API untouched. Nothing about the 100 - it is Qt's
        # tag for the key, not a SPIR-V version, and the module's real version is
        # decoded from its header further down.
        return RuntimeUse("Vulkan", "SPIR-V bytecode, used as it is")

    if key.source == 1:  # GLSL
        if key.flags & GLSL_ES_FLAG:
            if v not in GLSL_ES_VERSIONS:
                return _unused("only GLSL ES 320, 310, 300 and 100 are ever looked up")
            return RuntimeUse("OpenGL ES",
                              f"GLSL ES {v} - OpenGL ES {_GLES_FOR_GLSL[v]} or newer")
        if v not in GLSL_VERSIONS:
            return _unused("only GLSL 460..330, 150, 140, 130 and 120 are ever "
                           "looked up")
        if v == 120:
            return RuntimeUse("OpenGL",
                              "GLSL 120 - OpenGL 2.1, or any later compatibility "
                              "profile context")
        gl = _GL_FOR_GLSL.get(v) or f"{v // 100}.{v // 10 % 10}"
        return RuntimeUse("OpenGL",
                          f"GLSL {v} - OpenGL {gl} or newer, core or compatibility "
                          "profile")

    if key.source in HLSL_SOURCES:
        sm = f"Shader Model {_dotted(v)}"
        if v not in D3D12_SHADER_MODELS:
            return _unused(f"only Shader Model 6.7 down to 5.0 is ever looked up, "
                           f"not {_dotted(v)}")
        both = "Direct3D 11 and Direct3D 12"
        d3d12 = "Direct3D 12"
        if key.source == 3:  # DXBC, from fxc
            if v == 50:
                return RuntimeUse(both, f"{sm} bytecode, used as it is")
            return RuntimeUse(d3d12, f"{sm} bytecode; D3D11 takes DXBC 5.0 only")
        if key.source == 5:  # DXIL, from dxc
            # DXIL is looked up ahead of DXBC at each shader model, but saying so
            # would be noise: from 6.0 up there is no DXBC to lose to
            return RuntimeUse(d3d12, f"{sm} bytecode, used as it is")
        # HLSL source is the last resort, whatever its shader model: the backends
        # look for bytecode first and compile source only when there is none.
        #
        # Named as the one bytecode form that can exist at this shader model -
        # fxc stops at 5.1 and dxc starts at 6.0, so DXIL 5.0 and DXBC 6.0 are
        # not things. A simplification in one corner: D3D12 scans every model, so
        # bytecode at some *other* model would also win over this source.
        #
        # Which compiler does the compiling is left out on purpose - for 6.0 and
        # up it is dxc, but only if the Qt build has DXC support at all.
        bytecode = "DXBC" if v < 60 else "DXIL"
        return RuntimeUse(both if v == 50 else d3d12,
                          f"{sm} source, compiled at run time when no {bytecode} "
                          f"is present")

    if key.source in MSL_SOURCES:
        if v not in METAL_VERSIONS:
            return _unused(f"only MSL 3.0, 2.4..2.0 and 1.2 are ever looked up, "
                           f"not {_dotted(v)}")
        if key.source == 6:  # metallib
            return RuntimeUse("Metal",
                              f"precompiled MSL {_dotted(v)} library, tried before "
                              f"MSL source")
        return RuntimeUse("Metal",
                          f"MSL {_dotted(v)} source, compiled at run time when no "
                          f"metallib is present")

    if key.source == 7:  # WGSL
        return _unused("WGSL targets WebGPU, and QRhi has no WebGPU backend")

    return _unused(f"source {key.source} is not one this reader knows")


def hexdump(data: bytes, width: int = 16, limit: int | None = None) -> str:
    out = []
    n = len(data) if limit is None else min(len(data), limit)
    for off in range(0, n, width):
        chunk = data[off:off + width]
        hexpart = " ".join(f"{b:02x}" for b in chunk)
        text = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        out.append(f"{off:08x}  {hexpart:<{width * 3 - 1}}  |{text}|")
    if limit is not None and len(data) > limit:
        out.append(f"... {len(data) - limit} more bytes")
    return "\n".join(out)
