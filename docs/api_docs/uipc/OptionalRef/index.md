---
title: OptionalRef
description: An optional reference class, disable several operators of the boost::optional.
generator: doxide
---


# OptionalRef

**template &lt;typename T&gt; requires(!std::is_reference_v&lt;T&gt;) class OptionalRef : private boost::optional&lt;std::add_lvalue_reference_t&lt;T&gt;&gt;**



An optional reference class, disable several operators of the boost::optional.

- Never allow any assignment or copy operation.
- Apply const propagation.

:material-code-tags: **Template parameter** `T`
:    A const or non-const ValueType.
    


