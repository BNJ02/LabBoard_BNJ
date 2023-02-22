# cmake script for generating binary for flashing

add_custom_command(OUTPUT ${PROJECT_NAME}.bin
    DEPENDS ${PROJECT_NAME}
    COMMAND ${CMAKE_OBJCOPY} -Obinary ${PROJECT_NAME} ${PROJECT_NAME}.bin
)

add_custom_target(bin
    DEPENDS ${PROJECT_NAME}.bin
)

add_custom_target(flash
    DEPENDS ${PROJECT_NAME}.bin
	COMMAND st-flash --reset write ${PROJECT_NAME}.bin 0x8000000
)

add_custom_target(qemu-run
    DEPENDS ${PROJECT_NAME}.bin
	COMMAND qemu-system-arm -M nucleo32_f303 -kernel ${PROJECT_NAME}.bin -nographic
)

add_custom_target(qemu-gdb
    DEPENDS ${PROJECT_NAME}.bin
	COMMAND qemu-system-arm -M nucleo32_f303 -kernel ${PROJECT_NAME}.bin -nographic -S -gdb tcp::4242
)
