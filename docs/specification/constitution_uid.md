# Constitution UID

The Constitution UID is a unique identifier for a constitution known by `libuipc`, which is a 64-bit unsigned integer. 

The official constitution UID has a range of $[0, 2^{32}-1]$. The range $[2^{32}, 2^{64}-1]$ is reserved for user-defined constitutions.

Every official constitution will be documented in this specification. A user-defined constitution can apply for an official constitution UID by submitting a pull request to the `libuipc` repository, After code review, the constitution will be added to the official constitution list.

The related documentation of the constitution will be added to the [Constutitions/](./constitutions/index.md) directory.

When applying a constitution to a geometry, the `constitution_uid` attribute of the `meta` attribute of the geometry will be set to the constitution UID. The backend will use this UID to determine the constitution of the geometry.