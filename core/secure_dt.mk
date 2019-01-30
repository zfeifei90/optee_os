#
# Copyright (c) 2015-2018, ARM Limited and Contributors. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#

DTC_FLAGS += -I dts -O dtb
DTC := dtc

define MAKE_PREREQ_DIR
ifneq (${1},${2})
${1} :
	@mkdir -p "${1}"

endif
endef

# Convert device tree source file names to matching blobs
#   $(1) = input dts
define SOURCES_TO_DTBS
        $(notdir $(patsubst %.dts,%.dtb,$(filter %.dts,$(1))))
endef

# MAKE_FDT_DIRS macro creates the prerequisite directories that host the
# FDT binaries
#   $(1) = output directory
#   $(2) = input dts
define MAKE_FDT_DIRS
        $(eval DTBS       := $(addprefix $(1)/,$(call SOURCES_TO_DTBS,$(2))))
        $(eval TEMP_DTB_DIRS := $(sort $(dir ${DTBS})))
        # The $(dir ) function leaves a trailing / on the directory names
        # Rip off the / to match directory names with make rule targets.
        $(eval DTB_DIRS   := $(patsubst %/,%,$(TEMP_DTB_DIRS)))

$(eval $(foreach objd,${DTB_DIRS},$(call MAKE_PREREQ_DIR,${objd},${out-dir})))

fdt_dirs: ${DTB_DIRS}
endef

# MAKE_DTB generate the Flattened device tree binary
#   $(1) = output directory
#   $(2) = input dts
define MAKE_DTB

# List of DTB file(s) to generate, based on DTS file basename list
$(eval DTBOBJ := $(addprefix $(1)/,$(call SOURCES_TO_DTBS,$(2))))
# List of the pre-compiled DTS file(s)
$(eval DTSPRE := $(addprefix $(1)/,$(patsubst %.dts,%.pre.dts,$(notdir $(2)))))
# Dependencies of the pre-compiled DTS file(s) on its source and included files
$(eval DTSDEP := $(patsubst %.dtb,%.o.d,$(DTBOBJ)))
# Dependencies of the DT compilation on its pre-compiled DTS
$(eval DTBDEP := $(patsubst %.dtb,%.d,$(DTBOBJ)))

$(DTBOBJ): $(2) $(filter-out %.d,$(MAKEFILE_LIST)) | fdt_dirs
	@echo "  CPP     $$<"
	$(eval DTBS       := $(addprefix $(1)/,$(call SOURCES_TO_DTBS,$(2))))
	@$(CPP$(sm)) $$(CPPFLAGS) -Icore/include/ -x assembler-with-cpp \
		-E -ffreestanding -MT $(DTBS) -MMD -MF $(DTSDEP) -o $(DTSPRE) $$<
	@echo "  DTC     $$<"
	@$(DTC) $$(DTC_FLAGS) -d $(DTBDEP) -o $$@ $(DTSPRE)

-include $(DTBDEP)
-include $(DTSDEP)
endef

# MAKE_DTBS builds flattened device tree sources
#   $(1) = output directory
#   $(2) = list of flattened device tree source files
define MAKE_DTBS
        $(eval DTBOBJS := $(filter %.dts,$(2)))
        $(eval REMAIN := $(filter-out %.dts,$(2)))
        $(and $(REMAIN),$(error FDT_SOURCES contain non-DTS files: $(REMAIN)))
        $(eval $(foreach obj,$(DTBOBJS),$(call MAKE_DTB,$(1),$(obj))))
        $(eval $(call MAKE_FDT_DIRS,$(1),$(2)))

dtbs: $(DTBS)
all: dtbs
endef

# Generating the DTB from the DTS and integrating into OP-TEE core.
#
# CFG_SECURE_DT provides the DTS base name. It is looked up as file
# name as $(CFG_SECURE_DT).dts from the platform sub directory fdts/.
#
# Build precompiles $(CFG_SECURE_DT).dts to resolve pre compilation features
# (build directive, file inclusion, ...) then generates $(CFG_SECURE_DT).dtb.
#
# If CFG_STATIC_SECURE_DT is enabled, $(CFG_SECURE_DT).dtb content is wrapped
# into C source file builtin_secure_dtb.c that defines the DTB byte array
# CFG_STATIC_SECURE_DT expects.

DTB_FILE_NAME 	:= $(CFG_SECURE_DT).dtb
FDT_SOURCES 	:= $(addprefix $(arch-dir)/fdts/, $(CFG_SECURE_DT).dts)

DTC_FLAGS 	+= -Wno-unit_address_vs_reg

$(eval $(call MAKE_DTBS,$(out-dir)/core/fdts,$(FDT_SOURCES)))

ifeq ($(CFG_STATIC_SECURE_DT),y)
gensrcs-y += builtin_secure_dtb
produce-builtin_secure_dtb = fdts/builtin_secure_dtb.c
depends-builtin_secure_dtb = $(DTBOBJ) scripts/ta_bin_to_c.py
recipe-builtin_secure_dtb = scripts/bin_to_c.py --dtb $(DTBOBJ) \
						--label static_secure_dtb \
						--out $(out-dir)/core/fdts/builtin_secure_dtb.c
cleanfiles += $(out-dir)/core/fdts/builtin_secure_dtb.c
endif
