// Reuse-leg translation unit for the exact-FP witness: compiled into a
// target that carries generator-expression floating-point options of its
// own and REUSEs the other target's precompiled header. Never called; its
// compilation and the link against probe.cpp are the witness.
int
rigexecReuseProbe(int v)
{
    return v + 1;
}
