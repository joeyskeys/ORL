# ORL editable graph JSON

The editable graph format stores one `orlgraph::GraphModule` in a
deterministic, versioned JSON document. Node definitions are not copied into
the file. A graph node's `definition` field is a stable ID resolved from the
`NodeRegistry` supplied by the application.

## Document shape

```json
{
  "header": {
    "magic": "ORL_GRAPH",
    "format_version": 1,
    "language_version": "orl-0",
    "logical_abi_version": "orlgraph-0",
    "module_id": "character.pose",
    "content_hash": "..."
  },
  "graph": {
    "module_id": "character.pose",
    "version": { "major": 0, "minor": 1, "patch": 0 },
    "language_version": "orl-0",
    "logical_abi_version": "orlgraph-0",
    "inputs": [],
    "outputs": [],
    "resources": [],
    "nodes": [],
    "connections": []
  }
}
```

The `graph` arrays contain the serialized `GraphModule` values:

- `inputs` and `outputs` contain typed interface ports. Their `binding`,
  `semantic`, `coordinate_space`, `domain`, and `shape` metadata is preserved.
- `resources` contain graph-owned buffers and their access declarations.
- `nodes` contain stable instance IDs, node-definition IDs, constant parameter
  values, parameter mappings, provenance, and inline policy.
- `connections` contain source/destination endpoints, conversions, shape
  metadata, provenance, and feedback flags.

Logical types, domains, shapes, constants, and provenance use the same
structured encoding as the existing `.oro` serializer. Numbers are encoded
with explicit kind tags where JSON number precision could otherwise be
ambiguous.

## C++ API

`src/orlgraph/graph_serialization.hpp` provides:

```cpp
GraphJsonSerializationResult serialize_graph_json(const GraphModule&);
GraphJsonDocument deserialize_graph_json(std::string_view);
bool save_graph_json(const std::string&, const GraphModule&,
    std::vector<Diagnostic>* = nullptr);
GraphJsonDocument load_graph_json(const std::string&);
```

The loader checks JSON syntax, format version, language/ABI versions,
duplicate graph IDs, required graph sections, and the canonical content hash.
It intentionally does not resolve node-definition IDs because definitions are
owned by the caller's runtime/editor registry. Call `validate(document.module,
registry)` after loading when registry validation is required.

Serialization is deterministic and content hashes are computed from the
canonical document with an empty `header.content_hash`, then written into the
header.
