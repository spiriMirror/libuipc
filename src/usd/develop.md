# USD Support

Now develop USD support for `libuipc` is kind of annoying. Because vcpkg doesn't provide the full USD package. To support custom USD schema, we need to build USD from source and use the tool provided by USD to generate the schema code.

To start, you need to have [Pixar's USD](https://github.com/PixarAnimationStudios/OpenUSD) installed. Say, you can install it under `~/Software/Usd`.

## Generate Schema Code

``` shell
export PATH="$HOME/Software/Usd/bin:$PATH" # to use usdGenSchema
export PYTHONPATH="$HOME/Software/Usd/lib/python:$PYTHONPATH # to use usd python modules
```

```shell
cd ./uipcPhysics
python3 gen_schema.py
``` 

Then you can compile the project as usual.

## Run Usd Related Tests

if you want to run the usd related test, you need to set the `PXR_PLUGINPATH_NAME` environment variable to let usd find the schema info.

```shell
export PXR_PLUGINPATH_NAME="<YourPathToLibuipc>/libuipc/src/usd/uipcPhysics;$PXR_PLUGINPATH_NAME" # let usd correctly register the schema info.
```
