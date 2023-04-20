//
// Created by 水兵 on 2023/3/31.
//

#ifndef SVF_RUSTC_DEMANGLE_H
#define SVF_RUSTC_DEMANGLE_H
#ifdef __cplusplus
extern "C"
{
#endif

    // Demangles symbol given in `mangled` argument into `out` buffer
    //
    // Returns 0 if `mangled` is not Rust symbol or if `out` buffer is too small
    // Returns 1 otherwise
    int rustc_demangle(const char* mangled, char* out, size_t out_size);

#ifdef __cplusplus
}
#endif

#endif // SVF_RUSTC_DEMANGLE_H
