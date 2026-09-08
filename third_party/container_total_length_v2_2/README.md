# ContainerTotalLength V2.2 SDK

This directory contains the V2.2 algorithm source and a compatibility DLL for
IPC_Station2. The compatibility layer exposes the existing `ctl_*` C API used
by `ContainerTotalLengthService`; the original algorithm package exposes a C++
class API and cannot be loaded directly by the station application.

Enable the SDK with:

```text
-DSCAN_TRACKING_USE_CONTAINER_TOTAL_LENGTH_V22=ON
```

The DLL was built as x64 Release with MSVC v142 and PCL 1.12.0. Runtime PCL
DLLs are bundled beside it. V2.2 is the default SDK for new builds; set the
option to `OFF` to roll back to the existing SDK while comparing field data.
