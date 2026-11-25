idf_component_register(
    SRCS
        "sqlite3.c"
        "esp32.c"
    INCLUDE_DIRS
        "include"
    PRIV_INCLUDE_DIRS
        "."
        "private_include"
    PRIV_REQUIRES
        spi_flash
        esp_rom
        esp_hw_support
        freertos
)

set_source_files_properties(
    "esp32.c"
    PROPERTIES
        COMPILE_OPTIONS "-Werror;-Wall;-Wextra;-Wformat"
)

set_source_files_properties(
    "sqlite3.c"
    PROPERTIES
        COMPILE_OPTIONS
            "-std=gnu99;-DNDEBUG;-Wno-error;-Wno-unused-parameter;-Wno-unused-function;-Wno-unused-variable;-Wno-unused-but-set-variable;-Wno-unused-label;-Wno-unused-value;-Wno-sign-compare;-Wno-implicit-fallthrough;-Wno-maybe-uninitialized;-Wno-char-subscripts;-Wno-deprecated-declarations;-Wno-old-style-declaration;-Wno-incompatible-pointer-types;-Wno-cast-function-type;-Wno-return-local-addr;-Wno-misleading-indentation;-Wno-dangling-pointer"
)
