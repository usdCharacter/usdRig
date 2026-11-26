#ifndef RIGEXEC_OPS_AUTHORING_H
#define RIGEXEC_OPS_AUTHORING_H

/// Builders for the schema property an op's port or payload field is
/// authored with (RigExecPropertyInfo).

#include "rigExec/api/registry.h"

#include <cstddef>

namespace rigExecOps::authoring {

inline rigExec::RigExecPropertyInfo
Property(const char *schemaName, const void *fallback,
         const char *doc = nullptr, bool uniform = false)
{
    rigExec::RigExecPropertyInfo info;
    info.schemaName = schemaName;
    info.fallback = fallback;
    info.doc = doc;
    info.uniform = uniform;
    return info;
}

inline rigExec::RigExecPropertyInfo
Uniform(const char *schemaName, const void *fallback, const char *doc = nullptr)
{
    return Property(schemaName, fallback, doc, true);
}

/// A uniform token property; entry i of \p tokens is the token of value i.
template <size_t N>
rigExec::RigExecPropertyInfo
Tokens(const char *schemaName, const void *fallback,
       const char *const (&tokens)[N], const char *doc = nullptr)
{
    rigExec::RigExecPropertyInfo info = Uniform(schemaName, fallback, doc);
    info.tokens = tokens;
    info.tokenCount = N;
    return info;
}

inline rigExec::RigExecPropertyInfo
Relationship(const char *schemaName, const char *doc = nullptr,
             rigExec::RigExecPropertyBinding binding =
                 rigExec::RigExecPropertyBinding::Connection)
{
    rigExec::RigExecPropertyInfo info = Property(schemaName, nullptr, doc);
    info.binding = binding;
    return info;
}

inline rigExec::RigExecPropertyInfo
WithRole(rigExec::RigExecPropertyInfo info, rigExec::RigExecPropertyRole role)
{
    info.role = role;
    return info;
}

inline rigExec::RigExecPropertyInfo
WithType(rigExec::RigExecPropertyInfo info, const char *authoredType)
{
    info.authoredType = authoredType;
    return info;
}

inline rigExec::RigExecPortDesc
Authored(rigExec::RigExecPortDesc port,
         const rigExec::RigExecPropertyInfo &info)
{
    port.authoring = &info;
    return port;
}

inline rigExec::RigExecFieldDesc
Authored(rigExec::RigExecFieldDesc field,
         const rigExec::RigExecPropertyInfo &info)
{
    field.authoring = &info;
    return field;
}

} // namespace rigExecOps::authoring

#endif
