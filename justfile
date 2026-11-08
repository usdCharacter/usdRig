set positional-arguments

# Check changed C/C++ files, or pass explicit paths.
format-check *ARGS:
    python3 bin/format_cpp.py "$@"

# Format using the same pinned version and policy as CI.
format *ARGS:
    python3 bin/format_cpp.py --fix "$@"
