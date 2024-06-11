# libuipc
A Modern C++20 Library of Unified Incremental Potential Contact.

## Dependencies

| Name                                   | Version | Usage                                               | Import         |
| -------------------------------------- | ------- | --------------------------------------------------- | -------------- |
| [muda](https://github.com/MuGdxy/muda) | -       | improve safety and readability of CUDA programming. | submodule      |
| cuda                                   | >=12.0  | GPU programming                                     | system install |
| eigen3                                 | 3.4.0   | matrix calculation                                  | package        |
| catch2                                 | 3.4.0   | unit tests                                          | package        |
| libigl                                 | 2.5.0   | mesh processing                                     | package        |
| rapidcsv                               | 8.80    | csv file IO                                         | package        |
| fmt                                    | 10.1.1  | fast string formatting                              | package        |
| cppitertools                           | 2.1#3   | python-like iteration tools                         | package        |
| bgfx                                   | 1.127#1 | cross-platform RHI                                  | package        |
| dylib                                  | 2.2.1   | cross-platform dynamic library loader               | package        |
| benchmark                              | 1.8.3#3 | microbenchmark support library                      | package        |
| nlohmann_json                          | 3.11.2  | json file IO                                        | package        |

## Build

We use [vcpkg](https://github.com/microsoft/vcpkg) to manage the libraries we need and use CMake to build the project. The simplest way to let CMake detect vcpkg is to set the system environment variable `CMAKE_TOOLCHAIN_FILE` to `(YOUR_VCPKG_PARENT_FOLDER)/vcpkg/scripts/buildsystems/vcpkg.cmake`

### Windows & Linux

```shell
git submodule update --init
vcpkg install eigen3 catch2 spdlog libigl fmt cppitertools dylib rapidcsv benchmark nlohmann-json
```
GUI support.
```shell
vcpkg install bgfx
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

