## 2026-05-21 - Optimize pow(x, 1/3) with cbrtf(x) in FTMotion
**Learning:** `pow(x, 1/3)` is commonly used for cube root calculation but is significantly slower than `cbrtf(x)` which is usually hardware accelerated or has a highly optimized native implementation. Replaced `pow` with a new macro `CBRT` wrapping `cbrtf`, which yields roughly a ~25% speedup on x86 for this operation, demonstrating it is faster to use the specialized function.
**Action:** When performing roots, prefer specialized math functions (like `sqrtf`, `cbrtf`) over generic `powf`. Create new macros in `macros.h` to abstract these if missing.

## 2026-05-24 - Avoid dynamic `pow()` in formatting macros
**Learning:** In Marlin's firmware on ESP32, math operations on `double` precision (like standard `pow()`) are extremely slow because the ESP32 only has a single-precision hardware FPU. Software emulation of double precision drastically slows down formatting macros like `INTFLOAT(V,N)`.
**Action:** Always replace dynamic math library calls with `constexpr` recursive templates or functions when the arguments are known at compile time. Explicitly use `10.0f` (single precision) instead of `10` or `10.0` (double precision) to keep floating-point operations hardware-accelerated.
