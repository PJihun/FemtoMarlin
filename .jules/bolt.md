
## $(date +%Y-%m-%d) - Optimize `powf` to `sq` on hot path
**Learning:** `POW(x, 2)` (which calls `powf`) is used in performance-critical motion paths (e.g., `Marlin/src/module/scara.cpp`). `powf` is computationally expensive compared to simple multiplication.
**Action:** Replace `POW(x, 2)` with the `sq(x)` macro for squaring variables on hot paths to save significant clock cycles without side effects (as long as `x` has no side effects).
