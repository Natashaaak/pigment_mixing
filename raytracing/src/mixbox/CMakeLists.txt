# /workspaces/pigment_mixing/raytracing/src/mixbox/CMakeLists.txt

# Vytvoří statickou knihovnu 'mixbox_library' ze zdrojových souborů v tomto adresáři.
add_library(mixbox_library STATIC
    mixbox.cpp
)

# Nastaví, že kdokoliv, kdo se na tuto knihovnu odkáže (nalinkuje ji),
# automaticky získá tento adresář do svých include cest.
# To je potřeba, aby fungovaly #include "mixbox.h" nebo #include "mixbox/mixbox.h".
target_include_directories(mixbox_library PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})