# Windows only. rigExec links the math and graph archives privately, and a
# Windows DLL exports only its own objects' symbols, never those of an
# archive it links. A library, tool or test that calls a math kernel or the
# op scheduler directly then has nothing to resolve against. Other platforms
# export the archives' symbols from the shared library, so the private link
# is enough there and stays as it is.
#
# Kept out of the root list: tools/select_affected_tests.py reads that file
# as text and follows INTERFACE links, and this is not a dependency edge for
# test selection.
if (WIN32)
    target_link_libraries(rigExec INTERFACE rigExecGraph rigExecMath)
endif()
