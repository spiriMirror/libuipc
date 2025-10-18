# USD Support

To develop USD related features, you need to have [Pixar's USD](https://github.com/PixarAnimationStudios/OpenUSD) installed. Say, you can install it under `~/Software/Usd`.

## To Compile

``` shell
export PATH="$HOME/Software/Usd/bin:$PATH" # to use usdGenSchema
export PYTHONPATH="$HOME/Software/Usd/lib/python:$PYTHONPATH # to use usd python modules
```

```shell
cd ./uipcPhysics
python3 gen_schema.py
``` 

Then you can compile the project as usual.

## To Run Usd Related Features

```shell
export PXR_PLUGINPATH_NAME="xxx/usd/uipcPhysics/" # to use the custom schema
```
