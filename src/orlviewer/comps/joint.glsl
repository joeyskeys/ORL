// GPU twin of comps/joint.hpp. Keep offsets and 128-byte stride identical.
// Bind this SSBO to the same device buffer ORL receives as Joint joints[].

struct Joint {
    int64_t parent;
    int64_t selected;
    int64_t pad0;
    int64_t pad1;
    double translation[4];
    double rotation[4];
    double scale[4];
};
