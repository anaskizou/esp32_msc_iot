# Edit following two lines to set component requirements (see docs)
set(SOURCES main.c)
idf_component_register(
        SRCS ${SOURCES}
        INCLUDE_DIRS
          $ENV{IDF_PATH}/components
		  $ENV{IDF_PATH}/components/nvs_flash/include
		  $ENV{IDF_PATH}/examples/common_components/protocol_examples_common/include
        REQUIRES
          tft
          spiffs
)

#spiffs_create_partition_image(storage ../spiffs_image FLASH_IN_PROJECT)
