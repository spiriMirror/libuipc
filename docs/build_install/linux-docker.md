# Build on Linux Docker

## Install Docker

Install Docker following the instructions.

[https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/)

## Clone Libuipc

Clone the repository with the following command:

```shell
git clone https://github.com/spiriMirror/libuipc.git
```

## Install Image

```shell
cd libuipc
docker build -t pub/libuipc:dev -f artifacts/docker/dev.dockerfile .
```

## Run Image

```shell
docker run -it --rm --gpus all pub/libuipc:dev
```

## Check Installation

You can run the `uipc_info.py` to check if the `Pyuipc` is installed correctly.

```shell
cd libuipc/python
python uipc_info.py
```

More samples are at [Pyuipc Samples](https://github.com/spiriMirror/libuipc-samples).