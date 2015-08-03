# VRBRAIN build is via external build system

ifneq ($(VRBRAIN_ROOT),)
$(error VRBRAIN_ROOT found in config.mk - Please see http://dev.ardupilot.com/wiki/git-submodules/)
endif

ifneq ($(NUTTX_SRC),)
$(error NUTTX_SRC found in config.mk - Please see http://dev.ardupilot.com/wiki/git-submodules/)
endif





# these can be overridden in developer.mk
VRBRAINFIRMWARE_DIRECTORY ?= ../../VRNuttX
VRBRAINNUTTX_DIRECTORY ?= ../../VRNuttX/NuttX


VRBRAIN_ROOT := $(shell cd $(VRBRAINFIRMWARE_DIRECTORY) && pwd)
NUTTX_ROOT := $(shell cd $(VRBRAINNUTTX_DIRECTORY) && pwd)
NUTTX_SRC := $(NUTTX_ROOT)/nuttx/













NUTTX_GIT_VERSION := $(shell cd $(NUTTX_SRC) && git rev-parse HEAD | cut -c1-8)
VRBRAIN_GIT_VERSION   := $(shell cd $(VRBRAIN_ROOT) && git rev-parse HEAD | cut -c1-8)

EXTRAFLAGS += -DNUTTX_GIT_VERSION="\"$(NUTTX_GIT_VERSION)\""
EXTRAFLAGS += -DVRBRAIN_GIT_VERSION="\"$(VRBRAIN_GIT_VERSION)\""




# we have different config files for vrbrain_v45, vrbrain_v51, vrbrain_v52, vrubrain_v51, vrubrain_v52
VRBRAIN_MK_DIR=$(SRCROOT)/$(MK_DIR)/VRBRAIN

VRBRAIN_VB45_CONFIG_FILE=config_vrbrain-v45_APM.mk
VRBRAIN_VB45P_CONFIG_FILE=config_vrbrain-v45P_APM.mk
VRBRAIN_VB50_CONFIG_FILE=config_vrbrain-v50_APM.mk
VRBRAIN_VB51_CONFIG_FILE=config_vrbrain-v51_APM.mk
VRBRAIN_VB51P_CONFIG_FILE=config_vrbrain-v51P_APM.mk
VRBRAIN_VB51PRO_CONFIG_FILE=config_vrbrain-v51Pro_APM.mk
VRBRAIN_VB51PROP_CONFIG_FILE=config_vrbrain-v51ProP_APM.mk
VRBRAIN_VB52_CONFIG_FILE=config_vrbrain-v52_APM.mk
VRBRAIN_VB52P_CONFIG_FILE=config_vrbrain-v52P_APM.mk
VRBRAIN_VB52PRO_CONFIG_FILE=config_vrbrain-v52Pro_APM.mk
VRBRAIN_VB52PROP_CONFIG_FILE=config_vrbrain-v52ProP_APM.mk
VRBRAIN_VU51_CONFIG_FILE=config_vrubrain-v51_APM.mk
VRBRAIN_VU51P_CONFIG_FILE=config_vrubrain-v51P_APM.mk
VRBRAIN_VU52_CONFIG_FILE=config_vrubrain-v52_APM.mk

SKETCHFLAGS=$(SKETCHLIBINCLUDES) -DARDUPILOT_BUILD -DTESTS_MATHLIB_DISABLE -DCONFIG_HAL_BOARD=HAL_BOARD_VRBRAIN -DSKETCHNAME="\\\"$(SKETCH)\\\"" -DSKETCH_MAIN=ArduPilot_main -DAPM_BUILD_DIRECTORY=APM_BUILD_$(SKETCH)

WARNFLAGS = -Werror -Wno-psabi -Wno-packed -Wno-error=double-promotion -Wno-error=unused-variable -Wno-error=reorder -Wno-error=float-equal -Wno-error=pmf-conversions -Wno-error=missing-declarations -Wno-error=unused-function




PYTHONPATH=$(SKETCHBOOK)/mk/VRBRAIN/Tools/genmsg/src:$(SKETCHBOOK)/mk/VRBRAIN/Tools/gencpp/src
export PYTHONPATH

VRBRAIN_MAKE = $(v) ARDUPILOT_BUILD=1 make -C $(SKETCHBOOK) -f $(VRBRAIN_ROOT)/Makefile EXTRADEFINES="$(SKETCHFLAGS) $(WARNFLAGS) "'$(EXTRAFLAGS)' APM_MODULE_DIR=$(SKETCHBOOK) SKETCHBOOK=$(SKETCHBOOK) CCACHE=$(CCACHE) VRBRAIN_ROOT=$(VRBRAIN_ROOT) NUTTX_SRC=$(NUTTX_SRC) MAXOPTIMIZATION="-Os"
VRBRAIN_MAKE_ARCHIVES = make -C $(VRBRAIN_ROOT) NUTTX_SRC=$(NUTTX_SRC) CCACHE=$(CCACHE) archives MAXOPTIMIZATION="-Os"

HASHADDER_FLAGS += --ardupilot "$(SKETCHBOOK)"

ifneq ($(wildcard $(VRBRAIN_ROOT)),)
HASHADDER_FLAGS += --vrbrain "$(VRBRAIN_ROOT)"
endif
ifneq ($(wildcard $(NUTTX_SRC)/..),)
HASHADDER_FLAGS += --nuttx "$(NUTTX_SRC)/.."
endif


.PHONY: module_mk
module_mk:
	$(v) echo "Building $(SKETCHBOOK)/module.mk"
	$(RULEHDR)
	$(v) echo "# Auto-generated file - do not edit" > $(SKETCHBOOK)/module.mk.new
	$(v) echo "MODULE_COMMAND = ArduPilot" >> $(SKETCHBOOK)/module.mk.new
	$(v) echo "SRCS = $(subst $(SKETCHBOOK)/,,$(wildcard $(SRCROOT)/*.cpp)) $(SKETCHLIBSRCSRELATIVE)" >> $(SKETCHBOOK)/module.mk.new
	$(v) echo "MODULE_STACKSIZE = 4096" >> $(SKETCHBOOK)/module.mk.new
	$(v) echo "EXTRACXXFLAGS = -Wframe-larger-than=1300" >> $(SKETCHBOOK)/module.mk.new
	$(v) cmp $(SKETCHBOOK)/module.mk $(SKETCHBOOK)/module.mk.new 2>/dev/null || mv $(SKETCHBOOK)/module.mk.new $(SKETCHBOOK)/module.mk
	$(v) rm -f $(SKETCHBOOK)/module.mk.new

vrbrain-v45: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrbrain-v45.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB45_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VB45_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrbrain-v45_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB45_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrbrain-v45.vrx
	$(v) rm -f $(SKETCH)-vrbrain-v45.hex
	$(v) rm -f $(SKETCH)-vrbrain-v45.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v45_APM.vrx $(SKETCH)-vrbrain-v45.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v45_APM.hex $(SKETCH)-vrbrain-v45.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v45_APM.bin $(SKETCH)-vrbrain-v45.bin
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v45.vrx"

vrbrain-v45P: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrbrain-v45P.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB45P_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VB45P_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrbrain-v45P_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB45P_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrbrain-v45P.vrx
	$(v) rm -f $(SKETCH)-vrbrain-v45P.hex
	$(v) rm -f $(SKETCH)-vrbrain-v45P.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v45P_APM.vrx $(SKETCH)-vrbrain-v45P.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v45P_APM.hex $(SKETCH)-vrbrain-v45P.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v45P_APM.bin $(SKETCH)-vrbrain-v45P.bin
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v45P.vrx"

vrbrain-v51: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrbrain-v51.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB51_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VB51_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrbrain-v51_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB51_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrbrain-v51.vrx
	$(v) rm -f $(SKETCH)-vrbrain-v51.hex
	$(v) rm -f $(SKETCH)-vrbrain-v51.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v51_APM.vrx $(SKETCH)-vrbrain-v51.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v51_APM.hex $(SKETCH)-vrbrain-v51.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v51_APM.bin $(SKETCH)-vrbrain-v51.bin
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v51.vrx"

vrbrain-v51P: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrbrain-v51P.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB51P_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VB51P_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrbrain-v51P_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB51P_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrbrain-v51P.vrx
	$(v) rm -f $(SKETCH)-vrbrain-v51P.hex
	$(v) rm -f $(SKETCH)-vrbrain-v51P.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v51P_APM.vrx $(SKETCH)-vrbrain-v51P.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v51P_APM.hex $(SKETCH)-vrbrain-v51P.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v51P_APM.bin $(SKETCH)-vrbrain-v51P.bin
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v51P.vrx"

vrbrain-v51Pro: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrbrain-v51Pro.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB51PRO_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VB51PRO_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrbrain-v51Pro_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB51PRO_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrbrain-v51Pro.vrx
	$(v) rm -f $(SKETCH)-vrbrain-v51Pro.hex
	$(v) rm -f $(SKETCH)-vrbrain-v51Pro.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v51Pro_APM.vrx $(SKETCH)-vrbrain-v51Pro.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v51Pro_APM.hex $(SKETCH)-vrbrain-v51Pro.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v51Pro_APM.bin $(SKETCH)-vrbrain-v51Pro.bin
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v51Pro.vrx"

vrbrain-v51ProP: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrbrain-v51ProP.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB51PROP_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VB51PROP_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrbrain-v51ProP_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB51PROP_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrbrain-v51ProP.vrx
	$(v) rm -f $(SKETCH)-vrbrain-v51ProP.hex
	$(v) rm -f $(SKETCH)-vrbrain-v51ProP.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v51ProP_APM.vrx $(SKETCH)-vrbrain-v51ProP.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v51ProP_APM.hex $(SKETCH)-vrbrain-v51ProP.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v51ProP_APM.bin $(SKETCH)-vrbrain-v51ProP.bin
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v51ProP.vrx"

vrbrain-v52: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrbrain-v52.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB52_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VB52_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrbrain-v52_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB52_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrbrain-v52.vrx
	$(v) rm -f $(SKETCH)-vrbrain-v52.hex
	$(v) rm -f $(SKETCH)-vrbrain-v52.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v52_APM.vrx $(SKETCH)-vrbrain-v52.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v52_APM.hex $(SKETCH)-vrbrain-v52.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v52_APM.bin $(SKETCH)-vrbrain-v52.bin
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v52.vrx"

vrbrain-v52P: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrbrain-v52P.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB52P_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VB52P_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrbrain-v52P_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB52P_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrbrain-v52P.vrx
	$(v) rm -f $(SKETCH)-vrbrain-v52P.hex
	$(v) rm -f $(SKETCH)-vrbrain-v52P.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v52P_APM.vrx $(SKETCH)-vrbrain-v52P.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v52P_APM.hex $(SKETCH)-vrbrain-v52P.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v52P_APM.bin $(SKETCH)-vrbrain-v52P.bin
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v52P.vrx"

vrbrain-v52Pro: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrbrain-v52Pro.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB52PRO_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VB52PRO_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrbrain-v52Pro_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB52PRO_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrbrain-v52Pro.vrx
	$(v) rm -f $(SKETCH)-vrbrain-v52Pro.hex
	$(v) rm -f $(SKETCH)-vrbrain-v52Pro.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v52Pro_APM.vrx $(SKETCH)-vrbrain-v52Pro.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v52Pro_APM.hex $(SKETCH)-vrbrain-v52Pro.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v52Pro_APM.bin $(SKETCH)-vrbrain-v52Pro.bin
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v52Pro.vrx"

vrbrain-v52ProP: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrbrain-v52ProP.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB52PROP_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VB52PROP_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrbrain-v52ProP_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VB52PROP_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrbrain-v52ProP.vrx
	$(v) rm -f $(SKETCH)-vrbrain-v52ProP.hex
	$(v) rm -f $(SKETCH)-vrbrain-v52ProP.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v52ProP_APM.vrx $(SKETCH)-vrbrain-v52ProP.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v52ProP_APM.hex $(SKETCH)-vrbrain-v52ProP.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v52ProP_APM.bin $(SKETCH)-vrbrain-v52ProP.bin
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v52ProP.vrx"

vrubrain-v51: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrubrain-v51.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VU51_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VU51_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrubrain-v51_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VU51_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrubrain-v51.vrx
	$(v) rm -f $(SKETCH)-vrubrain-v51.hex
	$(v) rm -f $(SKETCH)-vrubrain-v51.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrubrain-v51_APM.vrx $(SKETCH)-vrubrain-v51.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrubrain-v51_APM.hex $(SKETCH)-vrubrain-v51.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrubrain-v51_APM.bin $(SKETCH)-vrubrain-v51.bin
	$(v) echo "MICRO VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrubrain-v51.vrx"

vrubrain-v51P: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrubrain-v51P.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VU51P_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VU51P_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrubrain-v51P_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VU51P_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrubrain-v51P.vrx
	$(v) rm -f $(SKETCH)-vrubrain-v51P.hex
	$(v) rm -f $(SKETCH)-vrubrain-v51P.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrubrain-v51P_APM.vrx $(SKETCH)-vrubrain-v51P.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrubrain-v51P_APM.hex $(SKETCH)-vrubrain-v51P.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrubrain-v51P_APM.bin $(SKETCH)-vrubrain-v51P.bin
	$(v) echo "MICRO VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrubrain-v51P.vrx"

vrubrain-v52: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrubrain-v52.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VU52_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VU52_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrubrain-v52_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_VU52_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrubrain-v52.vrx
	$(v) rm -f $(SKETCH)-vrubrain-v52.hex
	$(v) rm -f $(SKETCH)-vrubrain-v52.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrubrain-v52_APM.vrx $(SKETCH)-vrubrain-v52.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrubrain-v52_APM.hex $(SKETCH)-vrubrain-v52.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrubrain-v52_APM.bin $(SKETCH)-vrubrain-v52.bin
	$(v) echo "MICRO VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrubrain-v52.vrx"

vrbrainStd: vrbrain-v45 vrbrain-v51 vrbrain-v52 vrubrain-v51 vrubrain-v52
vrbrainStdP: vrbrain-v45P vrbrain-v51P vrbrain-v52P vrubrain-v51P
vrbrainPro: vrbrain-v51Pro vrbrain-v52Pro
vrbrainProP: vrbrain-v51ProP vrbrain-v52ProP

vrbrain: vrbrainStd vrbrainStdP vrbrainPro vrbrainProP

#vrbrain-clean: clean vrbrain-archives-clean
vrbrain-clean: clean
	$(v) /bin/rm -rf $(VRBRAIN_ROOT)/Build
	$(v) /bin/rm -rf $(BUILD_ROOT)

vrbrain-archives-clean:
	$(v) /bin/rm -rf $(VRBRAIN_ROOT)/Archives

$(VRBRAIN_ROOT)/Archives/vrbrain-v45.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrbrain-v45"

$(VRBRAIN_ROOT)/Archives/vrbrain-v45P.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrbrain-v45P"

$(VRBRAIN_ROOT)/Archives/vrbrain-v51.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrbrain-v51"

$(VRBRAIN_ROOT)/Archives/vrbrain-v51P.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrbrain-v51P"

$(VRBRAIN_ROOT)/Archives/vrbrain-v51Pro.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrbrain-v51Pro"

$(VRBRAIN_ROOT)/Archives/vrbrain-v51ProP.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrbrain-v51ProP"

$(VRBRAIN_ROOT)/Archives/vrbrain-v52.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrbrain-v52"

$(VRBRAIN_ROOT)/Archives/vrbrain-v52P.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrbrain-v52P"

$(VRBRAIN_ROOT)/Archives/vrbrain-v52Pro.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrbrain-v52Pro"

$(VRBRAIN_ROOT)/Archives/vrbrain-v52ProP.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrbrain-v52ProP"

$(VRBRAIN_ROOT)/Archives/vrubrain-v51.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrubrain-v51"

$(VRBRAIN_ROOT)/Archives/vrubrain-v51P.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrubrain-v51P"

$(VRBRAIN_ROOT)/Archives/vrubrain-v52.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrubrain-v52"

vrbrain-archives:
	$(v) $(VRBRAIN_MAKE_ARCHIVES)

vrbrain-info: module_mk
	@echo "VRBRAINFIRMWARE_DIRECTORY    $(VRBRAINFIRMWARE_DIRECTORY)"
	@echo "VRBRAINNUTTX_DIRECTORY       $(VRBRAINNUTTX_DIRECTORY)"
	@echo "NUTTX_ROOT                   $(NUTTX_ROOT)"
	@echo "VRBRAIN_ROOT                 $(VRBRAIN_ROOT)"
	@echo "NUTTX_SRC                    $(NUTTX_SRC)"
	@echo "SKETCHLIBS                   $(SKETCHLIBS)"
	@echo "SKETCHLIBNAMES               $(SKETCHLIBNAMES)"
	@echo "SKETCHLIBSRCDIRS             $(SKETCHLIBSRCDIRS)"
	@echo "SKETCHLIBSRCS                $(SKETCHLIBSRCS)"
	@echo "SKETCHLIBOBJS                $(SKETCHLIBOBJS)"
	@echo "SKETCHLIBINCLUDES            $(SKETCHLIBINCLUDES)"
	@echo "SKETCHLIBSRCSRELATIVE        $(SKETCHLIBSRCSRELATIVE)"
	@echo "SRCS                         $(subst $(SKETCHBOOK)/,,$(wildcard $(SRCROOT)/*.cpp))"
