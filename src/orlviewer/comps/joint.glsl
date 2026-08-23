// GPU twin of comps/joint.hpp. Keep offsets and 128-byte stride identical.
// Bind this SSBO to the same device buffer ORL receives as Joint joints[].

struct Joint {
    int64_t parent;
    double pad_parent[3];
    double translation[4];
    double rotation[4];
    double scale[4];
};
