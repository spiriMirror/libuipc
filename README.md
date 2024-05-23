# libuipc
A Library of Unified Incremental Potential Contact.

## Dependencies

| Name                                   | Version | Usage                                               | Import         |
| -------------------------------------- | ------- | --------------------------------------------------- | -------------- |
| [muda](https://github.com/MuGdxy/muda) | -       | improve safety and readability of CUDA programming. | submodule      |
| cuda                                   | >=12.0  | GPU programming                                     | system install |
| eigen3                                 | 3.4.0   | matrix calculation                                  | package        |
| catch2                                 | 3.4.0   | unit tests                                          | package        |
| libigl                                 | 2.5.0   | mesh processing                                     | package        |

## Build

### Linux

We use CMake to build the project.

```bash
git submodule update --init
sudo apt install libeigen3-dev
```


### Windows
We use [vcpkg](https://github.com/microsoft/vcpkg) to manage the libraries we need and use CMake to build the project. The simplest way to let CMake detect vcpkg is to set the system environment variable `CMAKE_TOOLCHAIN_FILE` to `(YOUR_VCPKG_PARENT_FOLDER)/vcpkg/scripts/buildsystems/vcpkg.cmake`

```shell
git submodule update --init
vcpkg install eigen3
```

## Build Document

- Install [mkdocs](https://www.mkdocs.org/)
    ```shell
    pip install mkdocs mkdocs-material mkdocs-literate-nav
    ```
    
- Download [doxide](https://www.doxide.org/installation/), and add the `doxide` binary folder to the system path.

- Run the following command at the root of the project:
    ```shell
    doxide build
    ```
    
- Turn on the local server:
    ```shell
    mkdocs serve
    ```

