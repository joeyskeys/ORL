#version 460

layout(location = 0) flat in uint object_id;
layout(location = 0) out uint out_object_id;

void main() {
    out_object_id = object_id;
}
