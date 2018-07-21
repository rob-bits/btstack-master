#
# Main component makefile.
#
# This Makefile can be left empty. By default, it will take the sources in the 
# src/ directory, compile them and link them into lib(subdirectory_name).a 
# in the build directory. This behaviour is entirely configurable,
# please read the ESP-IDF documents if you need to do this.
#
CFLAGS += -Wno-format


# app depends on compiled gatt db
att_delayed_read_response.o: att_delayed_read_response.h

# rule to compile gatt db
att_delayed_read_response.h: $(COMPONENT_PATH)/att_delayed_read_response.gatt
	$(IDF_PATH)/components/btstack/tool/compile_gatt.py $^ $@

# remove compiled gatt db on clean
COMPONENT_EXTRA_CLEAN = att_delayed_read_response.h
