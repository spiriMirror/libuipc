# Header-only library
# Overlay port: vcpkg's pinned SHA512 for the tinygltf v2.9.6 tarball went stale
# when GitHub regenerated the auto-generated archive; this overlay pins the hash
# of the tarball as GitHub currently serves it. Remove once the upstream vcpkg
# tinygltf port is fixed.
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO syoyo/tinygltf
    REF "v${VERSION}"
    SHA512 f736b30a55fcbb3b80bf25240e0ab2f50c57c380e1425efc30ce1b987f9167ea4b56d98f89e34947c3d77f1fb0b895c01e2dbfa10ddf7210c320ec55fd77b700
    HEAD_REF master
)

# Put the licence file where vcpkg expects it
# Copy the tinygltf header files and fix the path to json
vcpkg_replace_string("${SOURCE_PATH}/tiny_gltf.h" "#include \"json.hpp\"" "#include <nlohmann/json.hpp>")
file(INSTALL "${SOURCE_PATH}/tiny_gltf.h" DESTINATION "${CURRENT_PACKAGES_DIR}/include")

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
